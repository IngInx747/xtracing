#pragma once
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "xtracing.h"


struct Scene;


struct Triangle : public IPrimitive
{
    vec3 p0, p1, p2;
    int mid; // material id
};


struct TriangleBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const;
};


struct TriangleCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const;
};


struct Sphere : public IPrimitive
{
    vec3 p; // center
    float r; // radius
    int mid; // material id
    int tid; // transform matrix id
};


struct SphereBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const;

    Scene* scene;
};


struct SphereCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const;

    Scene* scene;
};


struct Attribute : public IAttribute
{
    vec3 hit;
    vec3 normal;
    vec3 incident;
    int mid;
};

#endif