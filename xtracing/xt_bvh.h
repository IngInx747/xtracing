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
        const PrimitiveBound& bound,
        const PrimitiveSplit& split,
        int threshold = 1);

    inline std::vector<BvhNode>& GetNodes() { return mNodes; }
    inline const std::vector<BvhNode>& GetNodes() const { return mNodes; }

    inline std::vector<Primitive>& GetPrimitives() { return mPrimitives; }
    inline const std::vector<Primitive>& GetPrimitives() const { return mPrimitives; }

    inline Aabb GetBoundingBox() const { return mNodes.size() > 0 ? mNodes[0].bbox: Bound(); }
    inline bool IsEmpty() const { return mNodes.empty(); }

protected:
    template <class PrimitiveBound, class PrimitiveSplit>
    void BuildRecursive(
        int beginId,
        int endId,
        int nodeId,
        int depth,
        const PrimitiveBound& bound,
        const PrimitiveSplit& split,
        int threshold);

protected:
    std::vector<Primitive> mPrimitives;
    std::vector<BvhNode> mNodes;
};

/// Bound Program Interfaces:
/// 
/// struct PrimitiveBound
/// {
/// 	Aabb operator() (const Primitive& primitive);
///     ...
/// };
/// 

/// Collide Program Interfaces:
/// 
/// struct PrimitiveCollide
/// {
/// 	bool operator() (const Primitive& primitive, Ray& ray);
///     ...
/// };
/// 

/// Split Program Interfaces:
/// 
/// struct PrimitiveSplit
/// {
/// 	int operator() (std::vector<Primitive>& primitives, int beginId, int endId);
///     ...
/// };
/// 
/// Some built-in implementations are provided.
/// 

template <class Primitive>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive>::Build(
    const std::vector<Primitive>& primitives,
    const PrimitiveBound& bound,
    const PrimitiveSplit& split,
    int threshold)
{
    if (primitives.size() == 0) return;
    mPrimitives.clear();
    std::copy(primitives.begin(), primitives.end(), std::back_inserter(mPrimitives));
    mNodes.emplace_back();
    BuildRecursive(0, static_cast<int>(mPrimitives.size()), 0, 0, bound, split, threshold);
}


template <class Primitive>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive>::BuildRecursive(
    int beginId,
    int endId,
    int nodeId,
    int depth,
    const PrimitiveBound& bound,
    const PrimitiveSplit& split,
    int threshold)
{
    using namespace bounding_volume_hierarchy_node;

	// Split primitives into left and right children nodes at splitting index
	int splitId = -1;
	if ((endId - beginId) > threshold)
		splitId = split(mPrimitives, beginId, endId);

	// Make Bvh leaf node if:
	// 1. #primitive is less than threshold, there is no need to split anymore;
	// 2. split method failed to split primitives into 2 sets(which causes #primitive > threshold).
    // To make #primitive per node strictly less than threshold, one needs a split method that
    // will certainly split successfully, like EqualCount method.
	if (splitId == -1 || splitId == beginId || splitId == endId)
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

        BuildRecursive(beginId, splitId, left, depth + 1, bound, split, threshold);
        mNodes[nodeId].bbox = Union(mNodes[nodeId].bbox, mNodes[left].bbox);

        int right = static_cast<int>(mNodes.size());
        Right(mNodes[nodeId]) = right;
        mNodes.emplace_back();

        BuildRecursive(splitId, endId, right, depth + 1, bound, split, threshold);
        mNodes[nodeId].bbox = Union(mNodes[nodeId].bbox, mNodes[right].bbox);
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
/// Bvh Split Methods
////////////////////////////////////////////////////////////////

enum BvhSplitMethodEnum
{
    EQUAL_COUNT,
    MIDDLE_POINT,
    SAH,
};


/// Split Method: EqualCounts
/// Partition primitives into equally-sized subsets
template<class Primitive, class PrimitiveBound>
struct EqualCountsSplit
{
    int operator() (std::vector<Primitive>& primitives, int beginId, int endId) const;

    EqualCountsSplit(const PrimitiveBound& bound) : bound(bound) {}

    const PrimitiveBound& bound;
};


template<class Primitive, class PrimitiveBound>
inline int EqualCountsSplit<Primitive, PrimitiveBound>::operator() (std::vector<Primitive>& primitives, int beginId, int endId) const
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

/// Split Method: MiddlePoint
/// Partition primitives through node's midpoint
template<class Primitive, class PrimitiveBound>
struct MiddlePointSplit
{
    int operator() (std::vector<Primitive>& primitives, int beginId, int endId) const;

    MiddlePointSplit(const PrimitiveBound& bound) : bound(bound) {}

    const PrimitiveBound& bound;
};


template<class Primitive, class PrimitiveBound>
inline int MiddlePointSplit<Primitive, PrimitiveBound>::operator() (std::vector<Primitive>& primitives, int beginId, int endId) const
{
    auto beginIter = primitives.begin() + beginId;
    auto endIter = primitives.begin() + endId;

    Aabb cbox = Bound(); // centroid bounding box
    for (auto iter = beginIter; iter != endIter; ++iter)
        cbox = Union(cbox, bound(*iter));
    int dim = GetMaxExtentDim(cbox);

    float mid = (cbox.pMin[dim] + cbox.pMax[dim]) * 0.5f;

    auto pIter = std::partition(beginIter, endIter, [&](const Primitive& p)
    { return GetCentroid(bound(p))[dim] < mid; });

    // use EqualCount if split failed
    if (pIter == beginIter || pIter == endIter)
	{
		int mid = (beginId + endId) / 2;
		auto midIter = primitives.begin() + mid;
	
		std::nth_element(beginIter, midIter, endIter,
			[&](const Primitive& a, const Primitive& b)
			{ return GetCentroid(bound(a))[dim] < GetCentroid(bound(b))[dim]; });
	
		return mid;
	}

    return static_cast<int>(std::distance(primitives.begin(), pIter));
}

/// Split Method: SAH
/// Partition primitives via surface area heuristic
template<class Primitive, class PrimitiveBound>
struct SAHSplit
{
    int operator() (std::vector<Primitive>& primitives, int beginId, int endId) const;

    SAHSplit(const PrimitiveBound& bound) : bound(bound) {}

    const PrimitiveBound& bound;
    int nBuckets{ 16 };
};


template<class Primitive, class PrimitiveBound>
int SAHSplit<Primitive, PrimitiveBound>::operator() (std::vector<Primitive>& primitives, int beginId, int endId) const
{
	auto beginIter = primitives.begin() + beginId;
	auto endIter = primitives.begin() + endId;

	Aabb cbox = Bound(); // centroid bounding box
	for (auto iter = beginIter; iter != endIter; ++iter)
		cbox = Union(cbox, bound(*iter));
	int dim = GetMaxExtentDim(cbox);

    std::vector<Aabb> boxes(nBuckets, Bound());
    std::vector<int> counts(nBuckets, 0);
    
	for (auto iter = beginIter; iter != endIter; ++iter)
	{
		vec3 offset = (GetCentroid(bound(*iter)) - cbox.pMin) / GetDiagonal(cbox);
		int b = static_cast<int>(nBuckets * offset[dim]);
		if (b == nBuckets) b = nBuckets - 1;
		boxes[b] = Union(boxes[b], bound(*iter));
		++counts[b];
	}

	// cost of splitting [0,b] and [b+1, nB-1]
	float minCost = kInfP;
	int splitBucketId = 0;

	for (int b = 0; b < nBuckets - 1; ++b)
	{
		Aabb bbox0 = Bound(), bbox1 = Bound();
		int count0{}, count1{};

		for (int i = 0; i <= b; ++i)
		{
			bbox0 = Union(bbox0, boxes[i]);
			count0 += counts[i];
		}

		for (int i = b + 1; i < nBuckets; ++i)
		{
			bbox1 = Union(bbox1, boxes[i]);
			count1 += counts[i];
		}

		//float cost = 0.125f + (GetArea(bbox0) * count0 + GetArea(bbox1) * count1) / GetArea(cbox);
		float cost = GetArea(bbox0) * count0 + GetArea(bbox1) * count1;

		// find bucket id that minimizes SAH metric
		if (minCost > cost)
		{
			minCost = cost;
			splitBucketId = b;
		}
	}

	// split according to the SAH result
	auto pIter = std::partition(beginIter, endIter, [&](const Primitive& p)
	{
		vec3 offset = (GetCentroid(bound(p)) - cbox.pMin) / GetDiagonal(cbox);
		int b = static_cast<int>(nBuckets * offset[dim]);
		if (b == nBuckets) b = nBuckets - 1;
		return b <= splitBucketId;
	});

    // use EqualCount if split failed
	if (pIter == beginIter || pIter == endIter)
	{
		int mid = (beginId + endId) / 2;
		auto midIter = primitives.begin() + mid;
	
		std::nth_element(beginIter, midIter, endIter,
			[&](const Primitive& a, const Primitive& b)
			{ return GetCentroid(bound(a))[dim] < GetCentroid(bound(b))[dim]; });
	
		return mid;
	}

	return static_cast<int>(std::distance(primitives.begin(), pIter));
}

////////////////////////////////////////////////////////////////
/// Bvh Build Options
////////////////////////////////////////////////////////////////

struct BvhBuildOption
{
    // the split method used during building process
    int splitMethod = BvhSplitMethodEnum::SAH;

    // stop splitting if number of primitives each node is less than which
    int threshold = 1;
};


template<class Primitive, class PrimitiveBound>
void BuildBvh(
    Bvh<Primitive>& bvh,
    const std::vector<Primitive>& primitives,
    PrimitiveBound& bound,
    const BvhBuildOption& option)
{
    int threshold = option.threshold;
    
    if (option.splitMethod == BvhSplitMethodEnum::MIDDLE_POINT)
    {
        MiddlePointSplit<Primitive, PrimitiveBound> split(bound);
        bvh.Build<PrimitiveBound, decltype(split)>(primitives, bound, split, threshold);
    }
    else if (option.splitMethod == BvhSplitMethodEnum::SAH)
    {
        SAHSplit<Primitive, PrimitiveBound> split(bound);
        bvh.Build<PrimitiveBound, decltype(split)>(primitives, bound, split, threshold);
    }
    else if (option.splitMethod == BvhSplitMethodEnum::EQUAL_COUNT)
    {
        EqualCountsSplit<Primitive, PrimitiveBound> split(bound);
        bvh.Build<PrimitiveBound, decltype(split)>(primitives, bound, split, threshold);
    }
    else
    {
        EqualCountsSplit<Primitive, PrimitiveBound> split(bound);
        bvh.Build<PrimitiveBound, decltype(split)>(primitives, bound, split, threshold);
    }
}

////////////////////////////////////////////////////////////////
/// Bvh Build Options Global Reference
////////////////////////////////////////////////////////////////

template <typename T = void>
struct GlobalBvhBuildOption
{
    static BvhBuildOption g_option;
};

template <typename T>
BvhBuildOption GlobalBvhBuildOption<T>::g_option;

inline const BvhBuildOption& GetGlobalBvhOption()
{
    return GlobalBvhBuildOption<void>::g_option;
}

inline int GetGlobalBvhBuildOptionSplitMethod()
{
    return GlobalBvhBuildOption<void>::g_option.splitMethod;
}

inline int GetGlobalBvhBuildOptionThreshold()
{
    return GlobalBvhBuildOption<void>::g_option.threshold;
}

inline void SetGlobalBvhOption(const BvhBuildOption& option)
{
    GlobalBvhBuildOption<void>::g_option = option;
}

inline void SetGlobalBvhBuildOptionSplitMethod(int splitMethod)
{
    GlobalBvhBuildOption<void>::g_option.splitMethod = splitMethod;
}

inline void SetGlobalBvhBuildOptionThreshold(int threshold)
{
    GlobalBvhBuildOption<void>::g_option.threshold = threshold;
}

#endif