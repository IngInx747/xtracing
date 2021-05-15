#pragma once
#ifndef SHADER_H
#define SHADER_H

#include "xtracing.h"


struct Scene;


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
    bool ls = 0; // is light source
};


struct PhongShader : public IShader
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