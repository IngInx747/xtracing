#include "Renderer.h"

vec3 Renderer::RenderPixel(const int2& index, const int2& dim, Scene* scene)
{
    Payload payload;

    // ray 0
    vec2 p = vec2{index.x, index.y} + vec2{0.5f, 0.5f};
    vec2 d = p / vec2{dim.x, dim.y} * 2.0f - 1.0f;
    CameraFrame& cam = scene->cameraFrame;
    vec3 dir = normalize(d.x * cam.u + d.y * cam.v + cam.w);
    vec3 org = cam.o;

    Ray ray{org, dir, kInfP, 0.001f}; // Ray 1 ~ n
    Trace(scene->root.get(), ray, payload, 0, 0, scene->miss.get());

    return payload.radiance;
}

void Renderer::Render(std::vector<vec3>& buffer, Scene* scene)
{
    int width = scene->width;
    int height = scene->height;

    for (int i = 0; i < height; ++i)
    {
        //printf("pixel(%d, :)\n", i);
        for (int j = 0; j < width; ++j)
        {
            //printf("pixel(%d, %d)\n", i, j);
            vec3 pixel = RenderPixel({j, i}, {width, height}, scene);
            buffer[i * width + j] = pixel;
        }
    }
}