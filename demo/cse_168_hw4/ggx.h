#pragma once
#ifndef GGX_H
#define GGX_H

#include <xtracing.h>
#include <algorithm>


// microfacet distribution function
inline float fD(const vec3& h, const vec3& n, float a)
{
    float c = dot(h, n); // cos
    //if (c <= 0.f) return 0.f;
    float t = tanf(acosf(c));
    if (isnan(t)) return 0.f;
    float f = a / (c * c * (a * a + t * t));
    return f * f * k1_Pi;
}


// micro-scale geometry, shadowing-masking function
inline float fG(const vec3& v, const vec3& n, float a)
{
    float c = dot(v, n);
    //if (c <= 0.f) return 0.f;
    float t = tanf(acosf(c));
    if (isnan(t)) return 0.f;
    return 2.f / (1.f + sqrtf(1.f + a * a * t * t));
}


// Fresnel reflection
inline vec3 fF(const vec3& v, const vec3& h, const vec3& Ks)
{
    return Ks + (vec3{1, 1, 1} - Ks) * powf(std::max(1.f - dot(v, h), 0.f), 5.f);
}

#endif