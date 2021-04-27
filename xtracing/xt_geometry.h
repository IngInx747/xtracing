////////////////////////////////////////////////////////////////
/// Put: geometry implementation here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef XT_GEOMETRY_H
#define XT_GEOMETRY_H

#include "xt_math.h"

////////////////////////////////////////////////////////////////
/// Primitive
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// Transform
////////////////////////////////////////////////////////////////

//inline vec3 TransformPoint(const vec3& p, const mat4& transform)
//{
//    return vec3(transform * vec4(p, 1));
//}

inline vec3 TransformPoint(const vec3& p, const mat4& transform)
{
    vec4 v = transform * vec4(p, 1);
    return v.w == 1.f ? vec3(v) : vec3(v) / v.w;
}

inline vec3 TransformVector(const vec3& v, const mat4& transform)
{
    return mat3(transform) * v;
}

inline vec3 TransformNormal(const vec3& n, const mat4& transform)
{
    return vec3(transpose(inverse(transform)) * vec4(n, 0));
}

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

inline vec3 GetBarycentricCoordinates(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& hp)
{
	vec3 hpv0 = hp - v0;
	vec3 v0v1 = v1 - v0;
	vec3 v0v2 = v2 - v0;
	float inv = 1.0f / length(cross(v0v1, v0v2));
	float u = length(cross(hpv0, v0v2)) * inv;
	float v = length(cross(v0v1, hpv0)) * inv;
    return vec3{1 - u - v, u, v};
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

inline vec3 GetSphericalCoordinates(const vec3& center, const vec3& ref)
{
	vec3 p = ref - center;
    float rho = length(p); // radial distance
	float phi = std::acosf(p.y / rho); // polar angle
	float psi = std::atan2f(p.z, p.x); // azimuthal angle
    return vec3{rho, phi, psi};
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

#endif // !XT_GEOMETRY_H