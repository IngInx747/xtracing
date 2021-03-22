#pragma once
#ifndef XT_ACCEL_H
#define XT_ACCEL_H

#include <stack>
#include <vector>
#include <algorithm>
#include <type_traits>

////////////////////////////////////////////////////////////////
/// Accelerating Structure
////////////////////////////////////////////////////////////////

enum AccelEnum
{
    NONE,
    BVH
};

#include "xt_bvh.h"

#endif