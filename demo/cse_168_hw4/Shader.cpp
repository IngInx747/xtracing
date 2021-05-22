#include "Shader.h"

#include "util.h"
#include "Light.h"
#include "Scene.h"
#include "ggx.h"

using std::sinf;
using std::cosf;
using std::tanf;
using std::acosf;
using std::powf;
using std::min;
using std::max;


inline vec3 SampleHemisphere(const vec3& n)
{
    float u0 = GetRandom();
    float u1 = GetRandom();
    float t0 = acosf(u0 * 2.f - 1.f); // theta
    float t1 = u1 * 2.f * kPi; // psi
    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
    if (dot(n, s) < 0) s = -s;
    return s;
}


// cosine weight: diffuse component
inline vec3 SampleCosineDiffuse(const vec3& n)
{
    float u0 = GetRandom();
    float u1 = GetRandom();
    float t0 = acosf(sqrtf(u0)); // theta
    float t1 = u1 * 2.f * kPi; // psi
    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
    vec3 w = normalize(n);
    vec3 u = cross(vec3{0, 1, 0}, w);
    if (length(u) < 0.001f) u = cross(vec3{1, 0, 0}, w);
    u = normalize(u);
    vec3 v = cross(w, u);
    return u * s.x + v * s.y + w * s.z;
}


// cosine weight: specular component
inline vec3 SampleCosineSpecular(const vec3& r, float ex)
{
    float u0 = GetRandom();
    float u1 = GetRandom();
    float t0 = acosf(powf(u0, 1.f / (1.f + ex))); // theta
    float t1 = u1 * 2.f * kPi; // psi
    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
    vec3 w = normalize(r);
    vec3 u = cross(vec3{0, 1, 0}, w);
    if (length(u) < 0.001f) u = cross(vec3{1, 0, 0}, w);
    u = normalize(u);
    vec3 v = cross(w, u);
    return u * s.x + v * s.y + w * s.z;
}


// GGX half vector
inline vec3 SampleCosineHalfVector(const vec3& n, float a)
{
    float u0 = GetRandom();
    float u1 = GetRandom();
    float t0 = atanf(a * sqrtf(u0 / (1.f - u0))); // theta
    float t1 = u1 * 2.f * kPi; // psi
    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
    vec3 w = normalize(n);
    vec3 u = cross(vec3{0, 1, 0}, w);
    if (length(u) < 0.001f) u = cross(vec3{1, 0, 0}, w);
    u = normalize(u);
    vec3 v = cross(w, u);
    return u * s.x + v * s.y + w * s.z;
}


void PathTracer::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    const Material& material = scene->materials[attrib.mid];
    SceneNode* root = scene->root.get();
    const auto& qlights = scene->qlights;
    vec3 result{}; // direct lighting

    const vec3& n = normalize(attrib.normal);
    const vec3& wo = normalize(attrib.incident);
    vec3 r = reflect(wo, n);

    const vec3& Kd = material.Kd;
    const vec3& Ks = material.Ks;
    const vec3& Ke = material.Ke;
    float s = material.ex;
    float a = material.rg;

    if (material.ls >= 0)
    {
        if (payload.depth == 0)
        {
            const auto& light = scene->qlights[material.ls];
            vec3 A = cross(light.ab, light.ac);
            if (dot(n, A) < 0) payload.radiance += Ke;
        }
        payload.done = true;
        return;
    }
    
    for (const auto& light : qlights)
        result += ShadeQuadLightMonteCarlo(light, attrib, material,
            root, scene->nSampleQuadLight, scene->bLightstratify, material.brdf);

    payload.radiance += result * payload.weight;

    if (scene->bUseRR)
    {
        float p = max(max(payload.weight.r, payload.weight.g), payload.weight.b);
        float q = 1.0f - min(p, 1.0f);

        if (GetRandom() < q)
        {
            payload.done = true;
            return;
        }

        payload.weight /= (1 - q); // re-weight path contribution
    }

    if (scene->importanceSampling == Scene::ImportanceSampling::HEMISPHERE)
    {
        vec3 wi = SampleHemisphere(n);
        vec3 f = (Kd + Ks * (s + 2.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
        payload.weight *= f * dot(n, wi) * 2.f * kPi; // brdf * (n, wi) * 2Pi
        payload.direction = wi;
    }
    else if (scene->importanceSampling == Scene::ImportanceSampling::COSINE)
    {
        vec3 wi = SampleCosineDiffuse(n);
        vec3 f = (Kd + Ks * (s + 2.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
        payload.weight *= f * kPi; // brdf * Pi
        payload.direction = wi;
    }
    else if (
        scene->importanceSampling == Scene::ImportanceSampling::BRDF &&
        material.brdf == BRDF::PHONG)
    {
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = ts / (td + ts); // reflectiveness parameter
        if (isnan(t)) t = 0.5f;
        float xi = GetRandom();
        vec3 wi{};
        if (xi <= t) wi = SampleCosineSpecular(r, s);
        else wi = SampleCosineDiffuse(n);
        vec3 f = (Kd + Ks * (s + 2.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
        float pdf = ((1.f - t) * dot(n, wi) + t * (s + 1.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
        payload.weight *= f * max(dot(n, wi), 0.f) / max(pdf, kEpsilon); // brdf * (n, wi) / pdf
        payload.direction = wi;
    }
    else if (
        scene->importanceSampling == Scene::ImportanceSampling::BRDF &&
        material.brdf == BRDF::GGX)
    {
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = max(ts / (td + ts), 0.25f); // reflectiveness parameter
        if (isnan(t)) t = 1.f;
        float xi = GetRandom();
        vec3 wi{}, h{};
        if (xi <= t)
        {
            h = SampleCosineHalfVector(n, a);
            wi = reflect(wo, h);
        }
        else
        {
            wi = SampleCosineDiffuse(n);
            h = normalize(wi - wo);
        }
        //if (dot(n, wi) < 0) payload.done = true;
        float fd = fD(h, n, a);
        float fg = fG(wi, n, a) * fG(-wo, n, a);
        vec3 ff = fF(wi, h, Ks);
        vec3 f = Kd * k1_Pi + ff * fg * fd / (4.f * dot(wi, n) * dot(-wo, n));
        float pdf = (1.f - t) * dot(n, wi) * k1_Pi + 0.25f * t * fd * dot(h, n) / dot(wi, h);
        payload.weight *= f * max(dot(n, wi), 0.f) / max(pdf, kEpsilon); // brdf * (n, wi) / pdf
        payload.direction = wi;
    }
    
    payload.origin = attrib.hit;
    payload.depth = 1; // depth = infinity
}


void ShadowShader::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    ShadowPayload& payload = dynamic_cast<ShadowPayload&>(payload_);
    payload.visible = false;
    TerminateRay(); // actually has no impact
}


void Miss::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    payload.radiance += background;
    payload.done = true;
}
