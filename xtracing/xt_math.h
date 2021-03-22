////////////////////////////////////////////////////////////////
/// Put: math implementation here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef XT_MATH_H
#define XT_MATH_H

#include <limits>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtc/matrix_transform.hpp>

using vec2 = glm::vec2;
using vec3 = glm::vec3;
using vec4 = glm::vec4;

using int2 = glm::int2;
using int3 = glm::int3;
using int4 = glm::int4;

using dvec2 = glm::dvec2;
using dvec3 = glm::dvec3;
using dvec4 = glm::dvec4;

using mat3 = glm::mat3;
using mat4 = glm::mat4;

using uchar1 = glm::u8vec1;
using uchar3 = glm::u8vec3;
using uchar4 = glm::u8vec4;

using glm::length;
using glm::distance;
using glm::dot;
using glm::cross;
using glm::lerp;
using glm::clamp;
using glm::degrees;
using glm::radians;
using glm::normalize;
using glm::inverse;
using glm::transpose;
using glm::translate;
using glm::scale;
using glm::rotate;
using glm::reflect;

constexpr float kInfP = std::numeric_limits<float>::max();
constexpr float kInfN = std::numeric_limits<float>::lowest();
constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPi_2 = 1.57079632679489661923f;
constexpr float kPi_4 = 0.785398163397448309616f;
constexpr float k1_Pi = 0.318309886183790671538f;
constexpr float k2_Pi = 0.636619772367581343076f;

#endif