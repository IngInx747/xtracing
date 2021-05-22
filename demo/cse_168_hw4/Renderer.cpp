#include "Renderer.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "util.h"

vec3 Renderer::RenderPixel(const int2& index, const int2& dim, const Scene& scene)
{
    vec3 result{0, 0, 0};
    int spp = scene.nSamplePerPixel;

#if defined(_OPENMP)
    int tid = omp_get_thread_num();
#else
    int tid = 0;
#endif

    GetRandom(0.f, 1.f, tid); // set seed

    for (int i = 0; i < spp; ++i)
    {
        Payload payload;
        payload.done = false;
        payload.depth = 0;
        payload.weight = vec3{1, 1, 1};
        payload.seed = tid;

        // ray 0
        vec2 p = vec2{index.x, index.y};
        if (i == 0) p += vec2{0.5f, 0.5f};
        else p += vec2{GetRandom(), GetRandom()}; // jaggy

        vec2 d = p / vec2{dim.x, dim.y} * 2.0f - 1.0f;
        const CameraFrame& cam = scene.cameraFrame;
        payload.direction = normalize(d.x * cam.u + d.y * cam.v + cam.w);
        payload.origin = cam.o;

        do
        {
            Ray ray{payload.origin, payload.direction, kInfP, 0.001f}; // Ray 1 ~ n
            Trace(scene.root.get(), ray, payload, 0, false, scene.miss.get());
        }
        while (!payload.done && payload.depth < scene.depth);

        result += payload.radiance;
    }

    return result / static_cast<float>(spp);
}

void Renderer::Render(std::vector<vec3>& buffer, const Scene& scene)
{
    int width = scene.width;
    int height = scene.height;
    float gamma = scene.gamma;

    while (currentFrame < numMaxFrame)
    {
        #pragma omp parallel for// schedule(dynamic)
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

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            vec3 p = buffer[i * width + j];
            if (isnan(p.r) || isnan(p.g) || isnan(p.b))
                printf("(%d, %d)\n", i, j);
        }
    }

    if (gamma > 1.0f)
    {
        float invg = 1.f / gamma;
        vec3 g{invg, invg, invg};

        #pragma omp parallel for
        for (int i = 0; i < height; ++i)
            for (int j = 0; j < width; ++j)
                buffer[i * width + j] = glm::pow(buffer[i * width + j], g);
    }
}