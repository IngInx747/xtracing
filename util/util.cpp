
#include "util.h"

#include <fstream>
#include <string>
#include <random>
#include <chrono>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

////////////////////////////////////////////////////////////////
/// Timing
////////////////////////////////////////////////////////////////

double When()
{
    auto current = std::chrono::steady_clock::now().time_since_epoch();
    double ss = (double)std::chrono::duration_cast<std::chrono::seconds>(current).count();
    double ms = std::chrono::duration_cast<std::chrono::milliseconds>(current).count() / 1.0e3;
    double us = std::chrono::duration_cast<std::chrono::microseconds>(current).count() / 1.0e6;
    return ss + ms + us;
}

////////////////////////////////////////////////////////////////
/// Random
////////////////////////////////////////////////////////////////

float GetRandom(float a, float b, long long seed)
{
    thread_local static int init = 1;
    thread_local static std::mt19937_64 rng;

    if (init)
    {
        std::seed_seq ss{uint32_t(seed & 0xffffffff), uint32_t(seed >> 32)};
        rng.seed(ss);
        init = 0;
    }

    std::uniform_real_distribution<float> urd(a, b);
    return urd(rng);
}


int GetRandom(int a, int b, long long seed)
{
    thread_local static int init = 1;
    thread_local static std::mt19937_64 rng;

    if (init)
    {
        std::seed_seq ss{uint32_t(seed & 0xffffffff), uint32_t(seed >> 32)};
        rng.seed(ss);
        init = 0;
    }

    std::uniform_int_distribution<int> urd(a, b);
    return urd(rng);
}

////////////////////////////////////////////////////////////////
/// Pixel
////////////////////////////////////////////////////////////////

// Convert float color to 256-bit
uchar4 Vec2Rgba(const vec3& fcolor)
{
    return {
        static_cast<unsigned char>(255.f * clamp(fcolor.x, 0.0f, 1.0f)),
        static_cast<unsigned char>(255.f * clamp(fcolor.y, 0.0f, 1.0f)),
        static_cast<unsigned char>(255.f * clamp(fcolor.z, 0.0f, 1.0f)),
        static_cast<unsigned char>(255)};
}


// Convert 256-bit color to float color
vec3 Rgba2Vec(const uchar4& ucolor)
{
    return {
        ucolor.r / 255.0f,
        ucolor.g / 255.0f,
        ucolor.b / 255.0f
    };
}


bool LoadImage(std::vector<uchar4>& pixels, int& width, int& height, int& channel, const char* filename)
{
    unsigned char* data = stbi_load(filename, &width, &height, &channel, 0);

    if (!data) return false;

    if (channel != 1 && channel != 3 && channel != 4)
    {
        stbi_image_free(data);
        return false;
    }

    pixels.resize(width * height * sizeof(uchar4));

    if (channel == 1)
    {
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                unsigned char r = data[(i * width + j) * channel];
                pixels[i * width + j].r = r;
                pixels[i * width + j].g = r;
                pixels[i * width + j].b = r;
                pixels[i * width + j].a = 0;
            }
        }
    }
    else if (channel == 3)
    {
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                unsigned char r = data[(i * width + j) * channel + 0];
                unsigned char g = data[(i * width + j) * channel + 1];
                unsigned char b = data[(i * width + j) * channel + 2];
                pixels[i * width + j].r = r;
                pixels[i * width + j].g = g;
                pixels[i * width + j].b = b;
                pixels[i * width + j].a = 0;
            }
        }
    }
    else if (channel == 4)
    {
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                unsigned char r = data[(i * width + j) * channel + 0];
                unsigned char g = data[(i * width + j) * channel + 1];
                unsigned char b = data[(i * width + j) * channel + 2];
                unsigned char a = data[(i * width + j) * channel + 3];
                pixels[i * width + j].r = r;
                pixels[i * width + j].g = g;
                pixels[i * width + j].b = b;
                pixels[i * width + j].a = a;
            }
        }
    }

    stbi_image_free(data);
    return true;
}


void SaveImagePPM(const std::vector<uchar4>& pixels, int width, int height, const char* filename)
{
    std::fstream ofs(filename, std::ios::out | std::ios::binary);

    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (int i = 0; i < width * height; i++)
    {
        unsigned char r = pixels[i].r;
        unsigned char g = pixels[i].g;
        unsigned char b = pixels[i].b;
        ofs << r << g << b;
    }
}


void SaveImagePNG(const std::vector<uchar4>& pixels, int width, int height, const char* filename)
{
    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename, width, height, sizeof(uchar4), (void*)pixels.data(), width * sizeof(uchar4));
}


void SaveImagePNG(const std::vector<vec3>& pixels, int width, int height, const char* filename)
{
    std::vector<uchar4> result(width * height);

    for (int i = 0; i < height; ++i)
        for (int j = 0; j < width; ++j)
            result[i * width + j] = Vec2Rgba(pixels[i * width + j]);

    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename, width, height, sizeof(uchar4), (void*)result.data(), width * sizeof(uchar4));
}
