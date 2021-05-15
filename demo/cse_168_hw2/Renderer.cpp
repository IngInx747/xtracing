#include "Renderer.h"

vec3 Renderer::RenderPixel(const int2& index, const int2& dim, const Scene& scene)
{
    vec3 result{0, 0, 0};
    Payload payload;
    payload.done = false;
    payload.depth = scene.depth;
    payload.mask = {1, 1, 1};
    float epsilon = 0.001f;

    // ray 0
    vec2 p = vec2{index.x, index.y} + vec2{0.5f, 0.5f};
    vec2 d = p / vec2{dim.x, dim.y} * 2.0f - 1.0f;
    const CameraFrame& cam = scene.cameraFrame;
    vec3 dir = normalize(d.x * cam.u + d.y * cam.v + cam.w);
    vec3 org = cam.o;

    do
    {
        Ray ray{org, dir, kInfP, epsilon}; // Ray 1 ~ n

        Trace(scene.root.get(), ray, payload, 0, false, scene.miss.get());

        result += payload.radiance;

        org = payload.origin;
        dir = payload.direction;
    }
    while (!payload.done && payload.depth > 0);

    return result;
}

void Renderer::Render(std::vector<vec3>& buffer, const Scene& scene)
{
    int width = scene.width;
    int height = scene.height;

    while (currentFrame < numMaxFrame)
    {
        //#pragma omp parallel for schedule(dynamic)
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
        
        ++currentFrame;
    }
}