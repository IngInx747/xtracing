#pragma once
#ifndef XT_TEXTURE_H
#define XT_TEXTURE_H

#include "xt_math.h"

#include <vector>

////////////////////////////////////////////////////////////////
/// Texture
////////////////////////////////////////////////////////////////

enum TexFilterEnum
{
    NEAREST,
    LINEAR,
};

enum TexWrapperEnum
{
    REPEAT,
    MIRRORED_REPEAT,
    CLAMP_TO_EDGE,
    CLAMP_TO_BORDER,
};


struct ITexWrapper
{
    virtual float operator() (float x) const = 0;
};


struct RepeatWrapper : public ITexWrapper
{
    inline float operator() (float x) const
    {
        return x - std::floorf(x); // [0.0, 1.0)
    }
};


struct MirrorRepeatWrapper : public ITexWrapper
{
    inline float operator() (float x) const
    {
        return std::fabs(x - std::nearbyint(x * 0.5f) * 2.0f); // distance to the nearest even int
    }
};


struct ClampEdgeWrapper : public ITexWrapper
{
    inline float operator() (float x) const
    {
        return (x > 1.0f) ? 1.0f : (x < 0.0f) ? 0.0f : x;
    }
};


struct ClampBorderWrapper : public ITexWrapper
{
    inline float operator() (float x) const
    {
        return (x > 1.0f || x < 0.0f) ? std::nanf("") : x;
    }
};


template <typename T_Val>
struct ITexFilter
{
    virtual T_Val operator() (const std::vector<T_Val>& texture, const vec2& uv, const int2& dim) const = 0;
};

/* [WIP]
template <typename T_C>
vec3 GetPixel2D(
    const std::vector<T_C>& texture,
    const vec2& uv, const int2& dim,
    int wrapper, int filter)
{
    ITexWrapper* ptw = nullptr;
    ITexFilter* ptf = nullptr;
    int w = dim.x;
    int h = dim.y;
    float x = uv.x * static_cast<float>(w);
    float y = uv.y * static_cast<float>(h);
    ;
}
*/

#endif