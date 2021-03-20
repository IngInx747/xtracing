////////////////////////////////////////////////////////////////
/// Put: utility functions, etc. here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include "xmath.h"

double When();
uchar4 Vec2Rgba(const vec3& fcolor);
vec3 Rgba2Vec(const uchar4& ucolor);
bool LoadImage(std::vector<uchar4>& pixels, int& width, int& height, int& channel, const char* filename);
void SaveImagePPM(const std::vector<uchar4>& pixels, int width, int height, const char* filename);
void SaveImagePNG(const std::vector<uchar4>& pixels, int width, int height, const char* filename);
void SaveImagePNG(const std::vector<vec3>& pixels, int width, int height, const char* filename);

#endif // !UTIL_H