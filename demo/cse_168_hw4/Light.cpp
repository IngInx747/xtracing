#include "Light.h"

#include <functional>

#include "util.h"
#include "Geometry.h"
#include "Shader.h"
#include "ggx.h"

using std::sinf;
using std::cosf;
using std::tanf;
using std::acosf;
using std::powf;
using std::min;
using std::max;


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


vec3 ShadeQuadLightAnalytic(
    const QuadLight& light,
    const Attribute& attrib,
    const Material& material)
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
    vec3 g1 = normalize(cross(v1, v2)) * acosf(dot(v1, v2));
    vec3 g2 = normalize(cross(v2, v3)) * acosf(dot(v2, v3));
    vec3 g3 = normalize(cross(v3, v0)) * acosf(dot(v3, v0));
    return 0.5f * light.color * material.Kd * k1_Pi * std::max(dot(n, g0 + g1 + g2 + g3), 0.0f);
}


vec3 ShadeQuadLightMonteCarlo(
    const QuadLight& light,
    const Attribute& attrib,
    const Material& material,
    SceneNode* root,
    int numSample,
    bool stratified,
    int brdf,
    float* pdf)
{
    vec3 x = attrib.hit;
    vec3 n = attrib.normal;
    vec3 wo = attrib.incident;
    vec3 r = normalize(reflect(wo, n));

    vec3 A = cross(light.ab, light.ac);
    vec3 nl = normalize(A);
    float area = length(A);

    vec3 Kd = material.Kd;
    vec3 Ks = material.Ks;
    float s = material.ex;
    float a = material.rg;

    vec3 sum{};
    int Ns = numSample;

    std::function<vec3(const vec3&)> samplePhong = [&](const vec3& xx)->vec3
    {
        vec3 wi = normalize(xx - x);
        if (dot(wi, n) <= 0.f || dot(wi, nl) <= 0.f) return vec3{};
        float Rs = dot(xx - x, xx - x); // |x - x'|^2
        float R = sqrtf(Rs);

        float ratio_light_src = 0.999f;
        float offset_ignore_self = 0.0001f;
        Ray ray{x, wi, R * ratio_light_src, offset_ignore_self};
        if (IsRayOccluded(ray, root)) return vec3{};

        vec3 F = (Kd + Ks * (s + 2.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
        float G = dot(wi, n) * dot(wi, nl) / Rs;

        if (pdf) *pdf += Rs / max(area * fabs(dot(nl, wi)), kEpsilon);

        return F * G;
    };

    std::function<vec3(const vec3&)> sampleGGX = [&](const vec3& xx)->vec3
    {
        vec3 wi = normalize(xx - x);
        if (dot(wi, n) <= 0.f || dot(wi, nl) <= 0.f) return vec3{};
        float Rs = dot(xx - x, xx - x); // |x - x'|^2
        float R = sqrtf(Rs);

        float ratio_light_src = 0.999f;
        float offset_ignore_self = 0.0001f;
        Ray ray{x, wi, R * ratio_light_src, offset_ignore_self};
        if (IsRayOccluded(ray, root)) return vec3{};

        vec3 h = normalize(wi - wo);
        float fd = fD(h, n, a);
        float fg = fG(wi, n, a) * fG(-wo, n, a);
        vec3 ff = fF(wi, h, Ks);

        vec3 F = Kd * k1_Pi + ff * fg * fd / (4.f * dot(wi, n) * dot(-wo, n));
        float G = dot(wi, n) * dot(wi, nl) / Rs;

        if (pdf) *pdf += Rs / max(area * fabs(dot(nl, wi)), kEpsilon);

        return F * G;
    };

    std::function<vec3(const vec3&)> sample;
    if (brdf == BRDF::GGX) sample = sampleGGX;
    else sample = samplePhong;

    if (stratified)
    {
        int Nc = static_cast<int>(std::ceilf(sqrtf(static_cast<float>(Ns))));
        Ns = Nc * Nc;

        for (int i = 0; i < Nc; ++i)
        {
            for (int j = 0; j < Nc; ++j)
            {
                float ux = (GetRandom() + j) / Nc;
                float uy = (GetRandom() + i) / Nc;
                vec3 xx = light.a + light.ab * ux + light.ac * uy; // x'
                sum += sample(xx);
            }
        }
    }
    else
    {
        for (int i = 0; i < Ns; ++i)
        {
            vec3 xx = light.a + light.ab * GetRandom() + light.ac * GetRandom(); // x'
            sum += sample(xx);
        }
    }

    return sum * light.color * length(A) / static_cast<float>(Ns);
}
