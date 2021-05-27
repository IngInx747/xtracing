#pragma once
#ifndef SHADER_H
#define SHADER_H

#include "xtracing.h"


enum BRDF
{
    PHONG,
    GGX,
};


struct Scene;


struct Payload : public IPayload
{
    vec3 radiance;
    vec3 weight;
    vec3 origin;
    vec3 direction;
    int depth;
    int light;
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
    float ex{1}; // specular shininess
    int brdf{0}; // brdf option
    float rg{0}; // roughness
    int ls{-1}; // id of light source
};


struct PathTracer : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct MISTracer : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const;

    Scene* scene;
};


struct OneStepShader : public IShader
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