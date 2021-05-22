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
        DIRECT, // deprecated since hw3
        PATH_TRACER,
    } integrator = PATH_TRACER;

    int nSampleQuadLight = 1; // number of samples integrating quad light
    bool bLightstratify = 0; // enable straitfy when integrating quad light
    bool bDirectAnalytic = 0; // use analytic direct lighting when integrating quad light

    int nSamplePerPixel = 1; // number of samples per pixel
    bool bUseNEE = 0; // enable Next-Event estimation when path-tracing
    bool bUseRR = 0; // enable Russian Roulette when path-tracing

    enum ImportanceSampling
    {
        HEMISPHERE,
        COSINE,
        BRDF,
    } importanceSampling = BRDF;

    float gamma = 1.f;

    void Build();
};

#endif