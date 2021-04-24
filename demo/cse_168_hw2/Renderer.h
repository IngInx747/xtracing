#pragma once
#ifndef RENDERER_H
#define RENDERER_H

#include "xtracing.h"
#include "Scene.h"

class Renderer
{
public:
    void Render(std::vector<vec3>& buffer, Scene* scene);
    
    vec3 RenderPixel(const int2& index, const int2& dim, Scene* scene);

    void SetNumFrame(int num)
    { numMaxFrame = num; }

protected:
    int currentFrame = 0;
    int numMaxFrame = 1;
};

#endif