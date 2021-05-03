#pragma once
#ifndef SCENE_H
#define SCENE_H

#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <vector>
#include <stack>
#include <iostream>

#include "xtracing.h"


struct Payload : public IPayload
{
    vec3 radiance;
    vec3 mask;
    vec3 origin;
    vec3 direction;
    int  depth;
    bool done;
};


struct ShadowPayload : public IPayload
{
    int visible;
};


struct Material
{
    vec3 Ka; // ambient + emission
    vec3 Kd; // diffuse
    vec3 Ks; // specular
    vec3 Ke; // emission
    float ex; // specular shininess
};


struct Attribute : public IAttribute
{
    vec3 hit;
    vec3 normal;
    vec3 incident;
    int mid;
};


struct Triangle : public IPrimitive
{
    vec3 p0, p1, p2;
    int mid; // material id
};


struct Sphere : public IPrimitive
{
    vec3 p; // center
    float r; // radius
    int mid; // material id
    int tid; // transform matrix id
};


struct PointLight
{
    vec3 pos;
    vec3 color;
    float c0, c1, c2; // attenuation coef
};


struct DirectionalLight
{
    vec3 dir;
    vec3 color;
};


struct Scene
{
    std::string outputFilename{"output.png"};
    unsigned int width = 256;
    unsigned int height = 256;

    CameraFrame cameraFrame;
    int depth = 5;

    std::vector<Material> materials;
    std::vector<mat4> transforms; // for sphere

    std::vector<Triangle> triangles; // temp buffers
    std::vector<Sphere> spheres;

    std::vector<DirectionalLight> dlights;
    std::vector<PointLight> plights;
    
    std::shared_ptr<IShader> miss;
    std::shared_ptr<SceneNode> root;

    void Build();
};

#endif