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
};


struct Scene
{
    std::string outputFilename;
    unsigned int width, height;

    CameraFrame cameraFrame;

    std::shared_ptr<SceneNode> root;
    std::shared_ptr<IShader> miss;
};


bool LoadScene(Scene* scene, const std::string& filename);

#endif