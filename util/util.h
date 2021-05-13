////////////////////////////////////////////////////////////////
/// Put: utility functions, etc. here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <xt_math.h>

double When();

uchar4 Vec2Rgba(const vec3& fcolor);

vec3 Rgba2Vec(const uchar4& ucolor);

bool LoadImage(std::vector<uchar4>& pixels, int& width, int& height, int& channel, const char* filename);

void SaveImagePPM(const std::vector<uchar4>& pixels, int width, int height, const char* filename);

void SaveImagePNG(const std::vector<uchar4>& pixels, int width, int height, const char* filename);

void SaveImagePNG(const std::vector<vec3>& pixels, int width, int height, const char* filename);

float GetRandom(float a = 0.f, float b = 1.f, long long seed = 0);

int GetRandom(int a, int b, long long seed = 0);

#endif // !UTIL_H