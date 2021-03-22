#pragma once
#ifndef XT_AABB_H
#define XT_AABB_H

#include <algorithm>

#include "xt_math.h"

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

inline Aabb TransformBoundingBox(const Aabb& b, const mat4& transform)
{
    vec3 vs[8];

    vs[0] = vec3(transform * vec4{ b.pMin.x, b.pMin.y, b.pMin.z, 1 });
    vs[1] = vec3(transform * vec4{ b.pMin.x, b.pMin.y, b.pMax.z, 1 });
    vs[2] = vec3(transform * vec4{ b.pMin.x, b.pMax.y, b.pMin.z, 1 });
    vs[3] = vec3(transform * vec4{ b.pMin.x, b.pMax.y, b.pMax.z, 1 });
    vs[4] = vec3(transform * vec4{ b.pMax.x, b.pMin.y, b.pMin.z, 1 });
    vs[5] = vec3(transform * vec4{ b.pMax.x, b.pMin.y, b.pMax.z, 1 });
    vs[6] = vec3(transform * vec4{ b.pMax.x, b.pMax.y, b.pMin.z, 1 });
    vs[7] = vec3(transform * vec4{ b.pMax.x, b.pMax.y, b.pMax.z, 1 });

    return Union(
        Union(
            Bound(vs[0], vs[1]), Bound(vs[2], vs[3])),
        Union(
            Bound(vs[4], vs[5]), Bound(vs[6], vs[7])));
}

#endif