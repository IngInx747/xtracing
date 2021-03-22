#pragma once
#ifndef XT_BVH_H
#define XT_BVH_H

#include <stack>
#include <vector>
#include <algorithm>
#include <type_traits>

#include "xt_aabb.h"
#include "xt_ray.h"

////////////////////////////////////////////////////////////////
/// Accelerating Structure :: Bounding Volume Hierarchy
////////////////////////////////////////////////////////////////

struct BvhNode
{
    Aabb bbox;
    int i0 = 0;
    int i1 = 0;
};

/// When representing inner node, i0, i1 = index of left and right
/// child nodes in the node array respectively;
/// When representing leaf node, i0, i1 = beginning index of object
/// in the primitive array and NEGATIVE number of objects.

namespace bounding_volume_hierarchy_node
{

inline int& Left(BvhNode& node)
{ return node.i0; }

inline const int& Left(const BvhNode& node)
{ return node.i0; }

inline int& Right(BvhNode& node)
{ return node.i1; }

inline const int& Right(const BvhNode& node)
{ return node.i1; }

inline int& Offset(BvhNode& node)
{ return node.i0; }

inline const int& Offset(const BvhNode& node)
{ return node.i0; }

inline int& NegLen(BvhNode& node)
{ return node.i1; }

inline const int& NegLen(const BvhNode& node)
{ return node.i1; }

inline int Length(const BvhNode& node)
{ return -node.i1; }

inline bool IsLeaf(const BvhNode& node)
{ return NegLen(node) < 0; }

inline void SetLeaf(BvhNode& node, int objIdx, int objNum)
{
    Offset(node) = objIdx;
    NegLen(node) = -objNum;
}

};


template <class Primitive>
class Bvh
{
public:
    template <class PrimitiveCollide>
    bool Intersect(
        PrimitiveCollide& collide,
        Ray& ray,
        bool is_shadow_ray = false) const;

    template <class PrimitiveBound, class PrimitiveSplit>
    void Build(
        const std::vector<Primitive>& primitives,
        PrimitiveBound& bound,
        PrimitiveSplit& split);

    inline std::vector<BvhNode>& GetNodes() { return mNodes; }
    inline const std::vector<BvhNode>& GetNodes() const { return mNodes; }

    inline std::vector<Primitive>& GetPrimitives() { return mPrimitives; }
    inline const std::vector<Primitive>& GetPrimitives() const { return mPrimitives; }

    inline Aabb GetBoundingBox() const { return mNodes.size() > 0 ? mNodes[0].bbox: Bound(); }
    inline bool IsEmpty() const { return mNodes.empty(); }
    inline void SetNumObjPerNode(int threshold) { mThreshold = threshold; }

protected:
    template <class PrimitiveBound, class PrimitiveSplit>
    void BuildRecursive(
        int beginId,
        int endId,
        int nodeId,
        int depth,
        PrimitiveBound& bound,
        PrimitiveSplit& split);

protected:
    std::vector<Primitive> mPrimitives;
    std::vector<BvhNode> mNodes;
    int mThreshold = 1;
};

/// Required interfaces:
/// 
/// struct PrimitiveBound
/// {
/// 	Aabb operator() (const Primitive& primitive);
///     ...
/// };
/// 
/// struct PrimitiveSplit
/// {
/// 	int operator() (std::vector<Primitive>& primitives, int beginId, int endId);
///     ...
/// };
/// 
/// struct PrimitiveCollide
/// {
/// 	bool operator() (const Primitive& primitive, Ray& ray);
///     ...
/// };
/// 
/// Some built-in implementations are provided.

template <class Primitive>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive>::Build(
    const std::vector<Primitive>& primitives,
    PrimitiveBound& bound,
    PrimitiveSplit& split)
{
    if (primitives.size() == 0) return;
    mPrimitives.clear();
    std::copy(primitives.begin(), primitives.end(), std::back_inserter(mPrimitives));
    mNodes.emplace_back();
    BuildRecursive(0, static_cast<int>(mPrimitives.size()), 0, 0, bound, split);
}

template <class Primitive>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive>::BuildRecursive(
    int beginId,
    int endId,
    int nodeId,
    int depth,
    PrimitiveBound& bound,
    PrimitiveSplit& split)
{
    using namespace bounding_volume_hierarchy_node;

    if ((endId - beginId) <= mThreshold)
    {
        SetLeaf(mNodes[nodeId], beginId, endId - beginId);
        Aabb bbox = Bound();
        for (int i = beginId; i < endId; ++i)
            bbox = Union(bbox, bound(mPrimitives[i]));
        mNodes[nodeId].bbox = bbox;
    }
    else
    {
        // Split primitives into left and right children nodes at splitting index
        int splitId = split(mPrimitives, beginId, endId);

        // Make leaf node if it failed to split primitives into 2 sets
        if (splitId == beginId || splitId == endId)
        {
            SetLeaf(mNodes[nodeId], beginId, endId - beginId);
            Aabb bbox = Bound();
            for (int i = beginId; i < endId; ++i)
                bbox = Union(bbox, bound(mPrimitives[i]));
            mNodes[nodeId].bbox = bbox;
        }
        // Build Bvh recursively after splitting primitives
        else
        {
            mNodes[nodeId].bbox = Bound();

            int left = static_cast<int>(mNodes.size());
            Left(mNodes[nodeId]) = left;
            mNodes.emplace_back();

            BuildRecursive(beginId, splitId, left, depth + 1, bound, split);
            mNodes[nodeId].bbox = Union(mNodes[nodeId].bbox, mNodes[left].bbox);

            int right = static_cast<int>(mNodes.size());
            Right(mNodes[nodeId]) = right;
            mNodes.emplace_back();

            BuildRecursive(splitId, endId, right, depth + 1, bound, split);
            mNodes[nodeId].bbox = Union(mNodes[nodeId].bbox, mNodes[right].bbox);
        }
    }
}

template <class Primitive>
template <class PrimitiveCollide>
inline bool Bvh<Primitive>::Intersect(
    PrimitiveCollide& collide,
    Ray& ray,
    bool is_shadow_ray) const
{
    using namespace bounding_volume_hierarchy_node;

    if (mNodes.empty()) return false;

    int curr = 0;
    bool hit = false;
    std::stack<int> recursive;

    const vec3& org = ray.org;
    const vec3& dir = ray.dir;
    float& dist = ray.dist;

    vec3 invDir = { 1.f / dir.x, 1.f / dir.y, 1.f / dir.z };
    int3 isNeg = { dir.x < 0, dir.y < 0, dir.z < 0 };

    while (true)
    {
        const BvhNode& node = mNodes[curr]; // safe reference

        if (IsIntersectingAabbInv(node.bbox, org, invDir, dist))
        {
            if (IsLeaf(node))
            {
                int beginId = Offset(node);
                int endId = Offset(node) + Length(node);

                for (int i = beginId; i < endId; ++i)
                {
                    if (collide(mPrimitives[i], ray))
                    {
                        if (is_shadow_ray) return true;
                        else hit = true;
                    }
                }

                if (recursive.empty()) break;
                curr = recursive.top();
                recursive.pop();
            }
            else
            {
                int dim = GetMaxExtentDim(node.bbox);
                
                if (isNeg[dim])
                {
                    recursive.push(Left(node));
                    curr = Right(node);
                }
                else
                {
                    recursive.push(Right(node));
                    curr = Left(node);
                }
            }
        }
        else
        {
            if (recursive.empty()) break;
            curr = recursive.top();
            recursive.pop();
        }
    }

    return hit;
}

////////////////////////////////////////////////////////////////
/// Split Methods
////////////////////////////////////////////////////////////////

/// Split Method: MiddlePoint
/// Partition primitives through node's midpoint
template<class Primitive, class PrimitiveBound>
struct MiddlePointSplit
{
    int operator() (std::vector<Primitive>& primitives, int beginId, int endId);

    MiddlePointSplit(PrimitiveBound& bound) : bound(bound) {}

    PrimitiveBound& bound;
};

template<class Primitive, class PrimitiveBound>
inline int MiddlePointSplit<Primitive, PrimitiveBound>::operator() (std::vector<Primitive>& primitives, int beginId, int endId)
{
    auto beginIter = primitives.begin() + beginId;
    auto endIter = primitives.begin() + endId;
    Aabb cbox = Bound(); // centroid bounding box

    for (auto iter = beginIter; iter != endIter; ++iter)
        cbox = Union(cbox, bound(*iter));

    int dim = GetMaxExtentDim(cbox);
    float mid = (cbox.pMin[dim] + cbox.pMax[dim]) * 0.5f;

    auto pIter = std::partition(beginIter, endIter,
        [&](const Primitive& p) { return GetCentroid(bound(p))[dim] < mid; });

    return std::distance(primitives.begin(), pIter);
}

/// Split Method: EqualCounts
/// Partition primitives into equally-sized subsets
template<class Primitive, class PrimitiveBound>
struct EqualCountsSplit
{
    int operator() (std::vector<Primitive>& primitives, int beginId, int endId);

    EqualCountsSplit(PrimitiveBound& bound) : bound(bound) {}

    PrimitiveBound& bound;
};

template<class Primitive, class PrimitiveBound>
inline int EqualCountsSplit<Primitive, PrimitiveBound>::operator() (std::vector<Primitive>& primitives, int beginId, int endId)
{
    auto beginIter = primitives.begin() + beginId;
    auto endIter = primitives.begin() + endId;
    Aabb cbox = Bound(); // centroid bounding box

    for (auto iter = beginIter; iter != endIter; ++iter)
        cbox = Union(cbox, bound(*iter));

    int dim = GetMaxExtentDim(cbox);
    int mid = (beginId + endId) / 2;
    auto midIter = primitives.begin() + mid;

    std::nth_element(beginIter, midIter, endIter,
        [&](const Primitive& a, const Primitive& b)
    { return GetCentroid(bound(a))[dim] < GetCentroid(bound(b))[dim]; });

    return mid;
}

/// Split Method: SAH
/// Partition primitives via surface area heuristic
// [TODO]

#endif