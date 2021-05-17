#pragma once
#ifndef SCENE_H
#define SCENE_H

#include "xtracing.h"
#include "Geometry.h"
#include "Shader.h"
#include "Light.h"


struct Scene
{
    std::string outputFilename{"output.png"};
    unsigned int width = 256;
    unsigned int height = 256;

    CameraFrame cameraFrame;
    int depth = 1;

    std::vector<Material> materials;
    std::vector<mat4> transforms; // for sphere

    std::vector<Triangle> triangles; // temp tri buffers
    std::vector<Sphere> spheres; // temp sphere buffers

    std::vector<DirectionalLight> dlights;
    std::vector<PointLight> plights;
    std::vector<QuadLight> qlights;
    
    std::shared_ptr<IShader> miss;
    std::shared_ptr<SceneNode> root;

    enum Integrator
    {
        DIRECT,
        ANALYTIC_DIRECT,
    } integrator = DIRECT;

    int nSampleQuadLight = 1;
    bool bLightstratify = 0;

    void Build();
};

#endif