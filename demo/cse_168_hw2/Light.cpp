#include "Light.h"

#include "Geometry.h"
#include "Shader.h"


bool IsRayOccluded(Ray& ray, SceneNode* root)
{
    ShadowPayload shadowPayload;
    shadowPayload.visible = true;
    Trace(root, ray, shadowPayload, 1, true, nullptr);
    return !shadowPayload.visible;
}


vec3 ShadeDirectionalLight(
    const DirectionalLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root)
{
    vec3 L = normalize(-light.dir);
    float epsilon = 0.001f; // 0.00001f;
    Ray ray{attrib.hit + attrib.normal * epsilon, L, kInfP, 0}; // scene6: 0
    if (IsRayOccluded(ray, root)) return vec3{};
    vec3 N = normalize(attrib.normal);
    vec3 E = normalize(attrib.incident);
    vec3 H = normalize(L - E);
    vec3 I = light.color;
    vec3 D = material.Kd;
    vec3 S = material.Ks;
    float s = material.ex;
    return I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s));
}


vec3 ShadePointLight(
    const PointLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root)
{
    vec3 P = light.pos - attrib.hit;
    vec3 L = normalize(P);
    float r = length(P);
    float epsilon = 0.0001f;// 0.00001f;
    Ray ray{attrib.hit + attrib.normal * epsilon, L, r, 0.01f}; // scene6: 0
    if (IsRayOccluded(ray, root)) return vec3{};
    vec3 N = normalize(attrib.normal);
    vec3 E = normalize(attrib.incident);
    vec3 H = normalize(L - E);
    vec3 I = light.color;
    vec3 D = material.Kd;
    vec3 S = material.Ks;
    float s = material.ex;
    float attenuation = light.c0 + light.c1 * r + light.c2 * r * r;
    return I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s)) / (attenuation);
}


vec3 ShadeQuadLight(
    const QuadLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root)
{
    vec3 a = light.a; // a->b->d->c
    vec3 b = light.a + light.ab;
    vec3 c = light.a + light.ac;
    vec3 d = light.a + light.ab + light.ac;
    vec3 r = attrib.hit;
    vec3 n = attrib.normal;
    vec3 v0 = normalize(a - r);
    vec3 v1 = normalize(b - r);
    vec3 v2 = normalize(d - r);
    vec3 v3 = normalize(c - r);
    vec3 g0 = normalize(cross(v0, v1)) * acosf(dot(v0, v1)); // \\Theta(r) * \\Gamma(r)
    vec3 g1 = normalize(cross(v1, v2)) * acosf(dot(v1, v2)); // \\Theta(r) * \\Gamma(r)
    vec3 g2 = normalize(cross(v2, v3)) * acosf(dot(v2, v3)); // \\Theta(r) * \\Gamma(r)
    vec3 g3 = normalize(cross(v3, v0)) * acosf(dot(v3, v0)); // \\Theta(r) * \\Gamma(r)
    return 0.5f * light.color * material.Kd * k1_Pi * dot(n, g0 + g1 + g2 + g3);
}
