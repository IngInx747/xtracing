#include "Shader.h"

#include "util.h"
#include "Light.h"
#include "Scene.h"

using std::sinf;
using std::cosf;
using std::acosf;


//vec3 SampleHemisphere(const vec3& n)
//{
//    float u0 = GetRandom();
//    float u1 = GetRandom();
//    float t0 = acosf(u0); // theta
//    float t1 = u1 * 2.f * kPi; // psi
//    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
//    vec3 w = normalize(n);
//    vec3 u = cross(vec3{0, 1, 0}, w);
//    if (length(u) < 0.001f) u = cross(vec3{1, 0, 0}, w);
//    u = normalize(u);
//    vec3 v = cross(w, u);
//    return u * s.x + v * s.y + w * s.z;
//}

vec3 SampleHemisphere(const vec3& n)
{
    float u0 = GetRandom();
    float u1 = GetRandom();
    float t0 = acosf(u0 * 2.f - 1.f); // theta
    float t1 = u1 * 2.f * kPi; // psi
    vec3 s{sinf(t0) * cosf(t1), sinf(t0) * sinf(t1), cosf(t0)};
    if (dot(n, s) < 0) s = -s;
    return s;
}


void SimplePathTracer::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    const Material& material = scene->materials[attrib.mid];
    SceneNode* root = scene->root.get();

    const vec3& n = attrib.normal;
    const vec3& wo = attrib.incident;
    vec3 r = reflect(wo, n);
    vec3 wi = SampleHemisphere(n);

    const vec3& Kd = material.Kd;
    const vec3& Ks = material.Ks;
    const vec3& Ke = material.Ke;
    float s = material.ex;

    //vec3 f = Kd * k1_Pi;
    vec3 f = (Kd + Ks * (s + 2.f) * 0.5f * std::powf(std::max(dot(r, wi), 0.f), s)) * k1_Pi;

    payload.radiance += Ke * payload.weight;
    payload.weight *= f * dot(n, wi) * 2.f * kPi; // brdf * (n, wi) * 2Pi
    payload.origin = attrib.hit;
    payload.direction = wi;
    ++payload.depth;
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
