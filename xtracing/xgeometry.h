////////////////////////////////////////////////////////////////
/// Put: geometry implementation here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef X_GEOMETRY_H
#define X_GEOMETRY_H

#include <stack>
#include <vector>
#include <memory>
#include <algorithm>
#include <type_traits>

#include "xmath.h"

////////////////////////////////////////////////////////////////
/// Axis-Aligned Bounding Box
////////////////////////////////////////////////////////////////

enum AxisEnum
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

struct Aabb
{
    vec3 pMin, pMax;
};

/// Construction of bounding box

inline Aabb Bound()
{
    return { vec3(kInfP), vec3(kInfN) };
}

inline Aabb Bound(const vec3& p)
{
    return { p, p };
}

inline Aabb Bound(const vec3& p1, const vec3& p2)
{
    return {
        vec3(
            std::min(p1.x, p2.x),
            std::min(p1.y, p2.y),
            std::min(p1.z, p2.z)),
        vec3(
            std::max(p1.x, p2.x),
            std::max(p1.y, p2.y),
            std::max(p1.z, p2.z)) };
}

inline Aabb Bound(
    float x0, float y0, float z0,
    float x1, float y1, float z1)
{
    return {
        vec3(
            std::min(x0, x1),
            std::min(y0, y1),
            std::min(z0, z1)),
        vec3(
            std::max(x0, x1),
            std::max(y0, y1),
            std::max(z0, z1)) };
}

inline Aabb Bound(const vec3& p0, const vec3& p1, const vec3& p2)
{
    return {
        vec3(
            std::min(std::min(p0.x, p1.x), p2.x),
            std::min(std::min(p0.y, p1.y), p2.y),
            std::min(std::min(p0.z, p1.z), p2.z)),
        vec3(
            std::max(std::max(p0.x, p1.x), p2.x),
            std::max(std::max(p0.y, p1.y), p2.y),
            std::max(std::max(p0.z, p1.z), p2.z)) };
}

inline Aabb Bound(
    float x0, float y0, float z0,
    float x1, float y1, float z1,
    float x2, float y2, float z2)
{
    return {
        vec3(
            std::min(std::min(x0, x1), x2),
            std::min(std::min(y0, y1), y2),
            std::min(std::min(z0, z1), z2)),
        vec3(
            std::max(std::max(x0, x1), x2),
            std::max(std::max(y0, y1), y2),
            std::max(std::max(z0, z1), z2)) };
}

/// Geometric tests within/between bounding box and other types of primitive

inline bool IsValid(const Aabb& b)
{
    return
        b.pMin.x <= b.pMax.x &&
        b.pMin.y <= b.pMax.y &&
        b.pMin.z <= b.pMax.z;
}

inline bool IsInside(const Aabb& b, const vec3& p)
{
    return
        p.x >= b.pMin.x && p.x <= b.pMax.x &&
        p.y >= b.pMin.y && p.y <= b.pMax.y &&
        p.z >= b.pMin.z && p.z <= b.pMax.z;
}

inline bool IsInside(const Aabb& b, const vec3& p, bool exclusive)
{
    return
        p.x >= b.pMin.x && p.x < b.pMax.x&&
        p.y >= b.pMin.y && p.y < b.pMax.y&&
        p.z >= b.pMin.z && p.z < b.pMax.z;
}

inline bool IsInside(const Aabb& b1, const Aabb& b2)
{
    return
        b1.pMin.x >= b2.pMin.x && b1.pMax.x <= b2.pMax.x &&
        b1.pMin.y >= b2.pMin.y && b1.pMax.y <= b2.pMax.y &&
        b1.pMin.z >= b2.pMin.z && b1.pMax.z <= b2.pMax.z;
}

inline bool IsOverlapping(const Aabb& b1, const Aabb& b2)
{
    return (
        (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x) &&
        (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y) &&
        (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z));
}

inline bool IsIntersectingAabb(const Aabb& b, const vec3& origin, const vec3& direction, float distance)
{
    float t0 = kInfN, t1 = kInfP;
    vec3 invDir = { 1.f / direction.x, 1.f / direction.y, 1.f / direction.z };

    float tx0 = (b.pMin.x - origin.x) * invDir.x;
    float tx1 = (b.pMax.x - origin.x) * invDir.x;

    t0 = std::max(t0, std::min(tx0, tx1));
    t1 = std::min(t1, std::max(tx0, tx1));

    float ty0 = (b.pMin.y - origin.y) * invDir.y;
    float ty1 = (b.pMax.y - origin.y) * invDir.y;

    t0 = std::max(t0, std::min(ty0, ty1));
    t1 = std::min(t1, std::max(ty0, ty1));

    float tz0 = (b.pMin.z - origin.z) * invDir.z;
    float tz1 = (b.pMax.z - origin.z) * invDir.z;

    t0 = std::max(t0, std::min(tz0, tz1));
    t1 = std::min(t1, std::max(tz0, tz1));

    return t1 > 0 && t1 >= t0 && distance > t0;
}

inline bool IsIntersectingAabbInv(const Aabb& b, const vec3& origin, const vec3& invDir, float distance)
{
    float t0 = kInfN, t1 = kInfP;

    float tx0 = (b.pMin.x - origin.x) * invDir.x;
    float tx1 = (b.pMax.x - origin.x) * invDir.x;

    t0 = std::max(t0, std::min(tx0, tx1));
    t1 = std::min(t1, std::max(tx0, tx1));

    float ty0 = (b.pMin.y - origin.y) * invDir.y;
    float ty1 = (b.pMax.y - origin.y) * invDir.y;

    t0 = std::max(t0, std::min(ty0, ty1));
    t1 = std::min(t1, std::max(ty0, ty1));

    float tz0 = (b.pMin.z - origin.z) * invDir.z;
    float tz1 = (b.pMax.z - origin.z) * invDir.z;

    t0 = std::max(t0, std::min(tz0, tz1));
    t1 = std::min(t1, std::max(tz0, tz1));

    return t1 > 0 && t1 >= t0 && distance > t0;
}

/// Geometric traits of bounding box

inline vec3 GetCentroid(const Aabb& b)
{
    return (b.pMax + b.pMin) * 0.5f;
}

inline vec3 GetDiagonal(const Aabb& b)
{
    return b.pMax - b.pMin;
}

inline float GetVolume(const Aabb& b)
{
    vec3 d = GetDiagonal(b);
    return d.x * d.y * d.z;
}

inline float GetArea(const Aabb& b)
{
    vec3 d = GetDiagonal(b);
    return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
}

inline float GetExtentVal(const Aabb& b, int dim)
{
    vec3 d = GetDiagonal(b);
    return (dim == AXIS_X) ? d.x : (dim == AXIS_Y) ? d.y : d.z;
}

inline int GetMaxExtentDim(const Aabb& b)
{
    vec3 d = GetDiagonal(b);
    return (d.x > d.y && d.x > d.z) ? AxisEnum::AXIS_X : (d.y > d.z) ? AxisEnum::AXIS_Y : AxisEnum::AXIS_Z;
}

inline float GetMaxExtentVal(const Aabb& b)
{
    vec3 d = GetDiagonal(b);
    return (d.x > d.y && d.x > d.z) ? d.x : (d.y > d.z) ? d.y : d.z;
}

inline vec3 GetOffset(const Aabb& b, const vec3& p)
{
    vec3 o = p - b.pMin;
    if (b.pMax.x > b.pMin.x) o.x /= b.pMax.x - b.pMin.x;
    if (b.pMax.y > b.pMin.y) o.y /= b.pMax.y - b.pMin.y;
    if (b.pMax.z > b.pMin.z) o.z /= b.pMax.z - b.pMin.z;
    return o;
}

inline vec3 Lerp(const Aabb& b, const vec3& t)
{
    return vec3(
        lerp(t.x, b.pMin.x, b.pMax.x),
        lerp(t.y, b.pMin.y, b.pMax.y),
        lerp(t.z, b.pMin.z, b.pMax.z));
}

/// Operations of bounding boxes

inline Aabb Union(const Aabb& b1, const Aabb& b2)
{
    return {
        vec3(
            std::min(b1.pMin.x, b2.pMin.x),
            std::min(b1.pMin.y, b2.pMin.y),
            std::min(b1.pMin.z, b2.pMin.z)),
        vec3(
            std::max(b1.pMax.x, b2.pMax.x),
            std::max(b1.pMax.y, b2.pMax.y),
            std::max(b1.pMax.z, b2.pMax.z)) };
}

inline Aabb Union(const Aabb& b, const vec3& p)
{
    return {
        vec3(
            std::min(b.pMin.x, p.x),
            std::min(b.pMin.y, p.y),
            std::min(b.pMin.z, p.z)),
        vec3(
            std::max(b.pMax.x, p.x),
            std::max(b.pMax.y, p.y),
            std::max(b.pMax.z, p.z)) };
}

inline Aabb Intersect(const Aabb& b1, const Aabb& b2)
{
    return {
        vec3(
            std::max(b1.pMin.x, b2.pMin.x),
            std::max(b1.pMin.y, b2.pMin.y),
            std::max(b1.pMin.z, b2.pMin.z)),
        vec3(
            std::min(b1.pMax.x, b2.pMax.x),
            std::min(b1.pMax.y, b2.pMax.y),
            std::min(b1.pMax.z, b2.pMax.z)) };
}

inline Aabb Expand(const Aabb& b, float s)
{
    return {
        b.pMin - vec3(s, s, s),
        b.pMax + vec3(s, s, s) };
}

inline Aabb Expand(const Aabb& b, const vec3& s)
{
    return {
        b.pMin - s,
        b.pMax + s };
}

////////////////////////////////////////////////////////////////
/// Transform
////////////////////////////////////////////////////////////////

inline vec3 Object2WorldHomoCoord(const vec3& p, const mat4& transform)
{
    return vec3(transform * vec4(p, 1));
}

inline vec3 Object2WorldNonHomoCoord(const vec3& n, const mat4& transform)
{
    return vec3(transpose(inverse(transform)) * vec4(n, 0));
}

inline Aabb Object2WorldAabb(const Aabb& b, const mat4& transform)
{
    vec3 vs[8];
    
    vs[0] = vec3(transform * vec4{b.pMin.x, b.pMin.y, b.pMin.z, 1});
    vs[1] = vec3(transform * vec4{b.pMin.x, b.pMin.y, b.pMax.z, 1});
    vs[2] = vec3(transform * vec4{b.pMin.x, b.pMax.y, b.pMin.z, 1});
    vs[3] = vec3(transform * vec4{b.pMin.x, b.pMax.y, b.pMax.z, 1});
    vs[4] = vec3(transform * vec4{b.pMax.x, b.pMin.y, b.pMin.z, 1});
    vs[5] = vec3(transform * vec4{b.pMax.x, b.pMin.y, b.pMax.z, 1});
    vs[6] = vec3(transform * vec4{b.pMax.x, b.pMax.y, b.pMin.z, 1});
    vs[7] = vec3(transform * vec4{b.pMax.x, b.pMax.y, b.pMax.z, 1});

    return Union(
        Union(
            Bound(vs[0], vs[1]), Bound(vs[2], vs[3])),
        Union(
            Bound(vs[4], vs[5]), Bound(vs[6], vs[7])));
}

inline vec3 World2ObjectHomoCoord(const vec3& p, const mat4& transform)
{
    return vec3(inverse(transform) * vec4(p, 1));
}

inline vec3 World2ObjectNonHomoCoord(const vec3& n, const mat4& transform)
{
    return vec3(inverse(transform) * vec4(n, 0));
}

////////////////////////////////////////////////////////////////
/// Primitive
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// Primitive :: Triangle
////////////////////////////////////////////////////////////////

inline bool IsIntersectingTriangle(
    const vec3& v0, const vec3& v1, const vec3& v2,
    const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    vec3 v01 = v1 - v0;
    vec3 v02 = v2 - v0;
    vec3 pvc = cross(dir, v02); // T
    float det = dot(v01, pvc); // ((P1, V02, V01))

    if (fabs(det) < kEpsilon) return false;

    float inv = 1 / det;
    vec3 tvc = org - v0; // P0 - V0
    float u = dot(tvc, pvc) * inv; // Eq.3
    if (u < 0.0f || u > 1.0f) return false;

    vec3 qvc = cross(tvc, v01); // S
    float v = dot(dir, qvc) * inv; // Eq.4
    if (v < 0.0f || u + v > 1.0f) return false;

    // distance from ray.origin to hit point
    float t = dot(v02, qvc) * inv; // Eq.5

    // update hit distance
    if (t > ignore && dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

inline bool IsIntersectingTriangleCulling(
    const vec3& v0, const vec3& v1, const vec3& v2,
    const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    vec3 v01 = v1 - v0;
    vec3 v02 = v2 - v0;
    vec3 pvc = cross(dir, v02); // T
    float det = dot(v01, pvc); // ((P1, V02, V01))

    if (det < 0.f) return false;

    float inv = 1 / det;
    vec3 tvc = org - v0; // P0 - V0
    float u = dot(tvc, pvc) * inv; // Eq.3
    if (u < 0.0f || u > 1.0f) return false;

    vec3 qvc = cross(tvc, v01); // S
    float v = dot(dir, qvc) * inv; // Eq.4
    if (v < 0.0f || u + v > 1.0f) return false;

    // distance from ray.origin to hit point
    float t = dot(v02, qvc) * inv; // Eq.5

    // update hit distance
    if (t > ignore && dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}


// Example definition of Triangle struct
// 
// struct Triangle : public IPrimitive
// {
//   vec3 p0, p1, p2;
// };
// 

////////////////////////////////////////////////////////////////
/// Primitive :: Sphere
////////////////////////////////////////////////////////////////

inline bool IsIntersectingSphere(
    const vec3& pos, float radius,
    const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    // solve equation: d^2*t^2 - 2b*t + c = 0 => (d*t)^2 - 2(b/d)(d*t) + c = 0
    vec3 I = pos - org;
    float d = 1.f / length(dir); // 1/d
    float b = dot(I, dir) * d;
    float c = dot(I, I) - radius * radius;
    float disc = b * b - c;

    if (disc < 0.0f) return false;
    disc = sqrtf(disc);

    // distance from ray.origin to hit point
    float t = kInfP;
    if ((b - disc) > 0.0f) t = (b - disc) * d;
    else if ((b + disc) > 0.0f) t = (b + disc) * d;
    else return false;

    // update detecting distance and record hit information
    if (t > ignore && dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

inline bool IsIntersectingUnitSphere(const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    // solve equation: d^2*t^2 - 2b*t + c = 0 => (d*t)^2 - 2(b/d)(d*t) + c = 0
    vec3 I = -org;
    float d = 1.f / length(dir); // 1/d
    float b = dot(I, dir) * d;
    float c = dot(I, I) - 1;
    float disc = b * b - c;

    if (disc < 0.0f) return false;
    disc = sqrtf(disc);

    // distance from ray.origin to hit point
    float t = kInfP;
    if ((b - disc) > 0.0f) t = (b - disc) * d;
    else if ((b + disc) > 0.0f) t = (b + disc) * d;
    else return false;

    // update detecting distance and record hit information
    if (t > ignore && dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

// Example definition of Sphere struct
// 
// struct Sphere : public IPrimitive
// {
//    vec3  center;
//    float radius;
// };
// 
// Position is replaced by transform::translate
// Radius is replaced by transform::scale

////////////////////////////////////////////////////////////////
/// Primitive :: Plane
////////////////////////////////////////////////////////////////

inline bool IsIntersectingInfinitePlane(
    const vec3& pos, const vec3& nor,
    const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    // solve equation
    // check if ray is parallel to plane
    float det = dot(nor, dir);
    if (fabs(det) < kEpsilon) return false;

    // distance from ray.origin to hit point
    float t = dot(nor, pos - org) / det;
    if (t < ignore) return false;

    // update hit distance
    if (dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

inline bool IsIntersectingUnitPlane(int axis, const vec3& org, const vec3& dir, float& dist, float ignore = 0.0f)
{
    int iNor = axis % 3;
    int iTgt = (axis + 1) % 3;
    int iBtg = (axis + 2) % 3;
    vec3 nor{iNor == 0, iNor == 1, iNor == 2}; // normal
    vec3 tgt{iTgt == 0, iTgt == 1, iTgt == 2}; // tangent
    vec3 btg{iBtg == 0, iBtg == 1, iBtg == 2}; // bitangent

    // solve equation
    // check if ray is parallel to plane
    float det = dot(nor, dir);
    if (fabs(det) < kEpsilon) return false;

    // distance from ray.origin to hit point
    float t = -dot(nor, org) / det;
    if (t < ignore) return false;

    // check if ray hits within [-1, 1]tgt * [-1, 1]btg
    vec3 r = org + dir * t;
    float cx = dot(r, tgt);
    float cz = dot(r, btg);
    if (cx < -1 || cx > 1 || cz < -1 || cz > 1) return false;

    // update hit distance
    if (dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

// Example definition of Plane struct
// 
// struct Plane : public IPrimitive
// {
// };
// 
// Position is replaced by transform::translate
// Normal is replaced by transform::rotate

////////////////////////////////////////////////////////////////
/// Ray
////////////////////////////////////////////////////////////////

struct Ray
{
    vec3 org; // origin
    vec3 dir; // direction
    float dist; // hitting distance
    float epsilon; // intersecting-test ignoring distance
};

////////////////////////////////////////////////////////////////
/// Camera
////////////////////////////////////////////////////////////////

inline void ComputeCameraFrame(
    const vec3& eye,    // camera position
    const vec3& lookat, // object position
    const vec3& up,     // up
    float fov_rad,      // field of view
    float aspect_ratio, // width / height
    vec3& U, // right
    vec3& V, // up
    vec3& W, // front
    bool fov_is_vertical)
{
    float ulen, vlen, wlen;
    W = lookat - eye; // Do not normalize W -- it implies focal length

    wlen = length(W);
    U = normalize(cross(W, up));
    V = normalize(cross(U, W));

    if (fov_is_vertical)
    {
        vlen = wlen * tanf(0.5f * fov_rad);
        V *= vlen;
        ulen = vlen * aspect_ratio;
        U *= ulen;
    }
    else
    {
        ulen = wlen * tanf(0.5f * fov_rad);
        U *= ulen;
        vlen = ulen / aspect_ratio;
        V *= vlen;
    }
}

struct CameraFrame
{
    vec3 o; // camera position (origin)
    vec3 u; // camera frame X  (right)
    vec3 v; // camera frame Y  (up)
    vec3 w; // camera frame Z  (front)
};

////////////////////////////////////////////////////////////////
/// Accelerating Structure
////////////////////////////////////////////////////////////////

enum AccelEnum
{
    NONE,
    BVH
};

////////////////////////////////////////////////////////////////
/// Accelerating Structure :: No Accelerating
////////////////////////////////////////////////////////////////

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

inline int& Left(BvhNode& node) { return node.i0; }
inline const int& Left(const BvhNode& node) { return node.i0; }
inline int& Right(BvhNode& node) { return node.i1; }
inline const int& Right(const BvhNode& node) { return node.i1; }
inline int& Offset(BvhNode& node) { return node.i0; }
inline const int& Offset(const BvhNode& node) { return node.i0; }
inline int& NegLen(BvhNode& node) { return node.i1; }
inline const int& NegLen(const BvhNode& node) { return node.i1; }

inline int Length(const BvhNode& node) { return -node.i1; }
inline bool IsLeaf(const BvhNode& node) { return NegLen(node) < 0; }

inline void SetLeaf(BvhNode& node, int objIdx, int objNum)
{
    Offset(node) = objIdx;
    NegLen(node) = -objNum;
}

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

#endif // !X_GEOMETRY_H