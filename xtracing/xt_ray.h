#pragma once
#ifndef XT_RAY_H
#define XT_RAY_H

#include "xt_math.h"

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

#endif