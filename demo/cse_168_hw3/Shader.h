#pragma once
#ifndef SHADER_H
#define SHADER_H

#include "xtracing.h"


struct Scene;


struct Payload : public IPayload
{
    vec3 radiance;
    vec3 weight;
    vec3 origin;
    vec3 direction;
    int  depth;
    bool done;
    long long seed;
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
    float ex{1}; // specular shininess
    bool ls{0}; // is light source
};


struct SimplePathTracer : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct NEEPathTracer : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct NRPathTracer : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct ShadowShader : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct Miss : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;
    
    vec3 background;
};

#endif