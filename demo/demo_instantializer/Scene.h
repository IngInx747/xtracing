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


struct Attribute : public IAttribute
{
    vec3 hit;
    vec3 normal;
    vec3 incident;
};


struct Triangle : public IPrimitive
{
    vec3 p0, p1, p2;
    int mid; // material id
};


struct Scene
{
    std::string outputFilename;
    unsigned int width, height;

    CameraFrame cameraFrame;
    int depth = 1;

    std::shared_ptr<SceneNode> root;
    std::shared_ptr<IShader> miss;
};


bool LoadSceneMeshInstance(Scene* scene, const std::string& filename);

#endif