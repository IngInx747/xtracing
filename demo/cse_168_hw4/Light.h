#pragma once
#ifndef LIGHT_H
#define LIGHT_H

#include "xtracing.h"


struct Attribute;
struct Material;


struct DirectionalLight
{
    vec3 dir;
    vec3 color;
};


struct PointLight
{
    vec3 pos;
    vec3 color;
    float c0, c1, c2; // attenuation coef
};


struct QuadLight
{
    vec3 a;
    vec3 ab;
    vec3 ac;
    vec3 color;
};


bool IsRayOccluded(Ray& ray, SceneNode* root);


vec3 ShadeDirectionalLight(
    const DirectionalLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root);


vec3 ShadePointLight(
    const PointLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root);


vec3 ShadeQuadLightAnalytic(
    const QuadLight& light,
    const Attribute& attrib,
    const Material& material);


vec3 ShadeQuadLightMonteCarlo(
    const QuadLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root,
    int numSample,
    bool stratified,
    int brdf,
    float* pdf);

#endif