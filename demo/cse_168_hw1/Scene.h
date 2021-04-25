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

struct PointLight
{
    vec3 position;
    vec3 color;
    float c0, c1, c2;
    // TODO: define the point light structure
};

struct DirectionalLight
{
    vec3 direction;
    vec3 color;
    // TODO: define the directional light structure
};

struct MyMaterial
{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 emission;
    float shininess;
    // TODO: define the material structure
};

struct MyAttributes
{
    vec3 hit;
    vec3 normal;
    vec3 incident;
    MyMaterial material;
    // TODO: define the attributes structure
};

struct Triangle : public IPrimitive
{
    vec3 p0, p1, p2;
    mat4 transform;
    MyMaterial material;
};

struct Sphere : public IPrimitive
{
    mat4 transform;
    MyMaterial material;
};

struct Scene
{
    // Info about the output image
    std::string outputFilename;
    unsigned int width, height;

    std::string integratorName;

    std::vector<vec3> vertices;

    std::vector<Triangle> triangles;
    std::vector<Sphere> spheres;

    std::vector<DirectionalLight> dlights;
    std::vector<PointLight> plights;

    int depth;

    // TODO: add other variables that you need here
    CameraFrame cameraFrame;

    Scene()
    {
        outputFilename = "raytrace.png";
        integratorName = "raytracer";
    }
};

class SceneLoader
{
private:
    std::stack<mat4> transStack;

    /**
     * Right multiply M to the top matrix of transform stack.
     */
    void rightMultiply(const mat4& M);

    /**
     * Transform a point with the top matrix of transform stack.
     */
    vec3 transformPoint(vec3 v);

    /**
     * Transform an normal with the top matrix of transform stack.
     */
    vec3 transformNormal(vec3 n);

    /**
     * Read values from a stringstream.
     */
    template <class T>
    bool readValues(std::stringstream& s, const int numvals, T* values);

public:
    std::shared_ptr<Scene> load(std::string sceneFilename);
};

#endif