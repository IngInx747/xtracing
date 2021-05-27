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

    const vec3& n = normalize(attrib.normal);
    const vec3& wo = normalize(attrib.incident);
    vec3 r = reflect(wo, n);

    const vec3& Kd = material.Kd;
    const vec3& Ks = material.Ks;
    const vec3& Ke = material.Ke;
    float s = material.ex;
    float a = material.rg;
    
    payload.origin = attrib.hit;

    // Next-Event Estimation
    if (scene->bUseNEE)
    {
        const auto& qlights = scene->qlights;
        vec3 dl{}; // direct lighting

        if (material.ls >= 0)
        {
            if (payload.depth == 0)
            {
                const auto& light = qlights[material.ls];
                vec3 A = cross(light.ab, light.ac);
                if (dot(n, A) < 0) payload.radiance += Ke;
            }
            payload.done = true;
            return;
        }
    
        for (const auto& light : qlights)
            dl += ShadeQuadLightMonteCarlo(light, attrib, material,
                root, scene->nSampleQuadLight, scene->bLightstratify, material.brdf, nullptr);

        payload.radiance += dl * payload.weight;
    }
    else
    {
        if (material.ls >= 0)
        {
            const auto& light = scene->qlights[material.ls];
            vec3 A = cross(light.ab, light.ac);
            if (dot(n, A) < 0) payload.radiance += Ke * payload.weight;
        }
        else payload.radiance += Ke * payload.weight;
    }

    // Russian Roulette ray termination
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
        payload.depth = -1; // depth = infinity
    }
    else
    {
        ++payload.depth;
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
}


// Multiple Importance Sampling
class ISampler
{
public:
    // get the sampled outgoing direction
    virtual vec3 Sample() const = 0;

    // get the sampled BRDF spectrum
    virtual vec3 Eval(const vec3& odir) const = 0;

    // get the probobility of the sampling
    virtual float PDF(const vec3& odir) const = 0;
};


class PhongSampler : public ISampler
{
public:
    PhongSampler(const Attribute& a, const Material& m)
    : attrib(a), material(m)
    {}

    vec3 Eval(const vec3& wi) const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float s = material.ex;
        float a = material.rg;
        vec3 r = reflect(wo, n);
        return (Kd + Ks * (s + 2.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
    }

    float PDF(const vec3& wi) const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float s = material.ex;
        float a = material.rg;
        vec3 r = reflect(wo, n);
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = ts / (td + ts); // reflectiveness parameter
        if (isnan(t)) t = 0.5f;
        return ((1.f - t) * dot(n, wi) + t * (s + 1.f) * 0.5f * powf(max(dot(r, wi), 0.f), s)) * k1_Pi;
    }

    vec3 Sample() const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float s = material.ex;
        float a = material.rg;
        vec3 r = reflect(wo, n);
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = ts / (td + ts); // reflectiveness parameter
        if (isnan(t)) t = 0.5f;

        vec3 wi{};
        float xi = GetRandom();
        if (xi <= t) wi = SampleCosineSpecular(r, s);
        else wi = SampleCosineDiffuse(n);
        return wi;
    }

protected:
    // attributes
    const Attribute& attrib;
    const Material& material;
};


class GGXSampler : public ISampler
{
public:
    GGXSampler(const Attribute& a, const Material& m)
    : attrib(a), material(m)
    {}

    vec3 Eval(const vec3& wi) const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float a = material.rg;
        vec3 h = normalize(wi - wo);
        float fd = fD(h, n, a);
        float fg = fG(wi, n, a) * fG(-wo, n, a);
        vec3 ff = fF(wi, h, Ks);
        return Kd * k1_Pi + ff * fg * fd / (4.f * dot(wi, n) * dot(-wo, n));
    }

    float PDF(const vec3& wi) const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float a = material.rg;
        vec3 h = normalize(wi - wo);
        float fd = fD(h, n, a);
        float fg = fG(wi, n, a) * fG(-wo, n, a);
        vec3 ff = fF(wi, h, Ks);
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = max(ts / (td + ts), 0.25f); // reflectiveness parameter
        if (isnan(t)) t = 1.f;
        return (1.f - t) * dot(n, wi) * k1_Pi + 0.25f * t * fd * dot(h, n) / dot(wi, h);
    }

    vec3 Sample() const
    {
        const vec3& n = attrib.normal;
        const vec3& wo = attrib.incident;
        const vec3& Kd = material.Kd;
        const vec3& Ks = material.Ks;
        float s = material.ex;
        float a = material.rg;
        vec3 r = reflect(wo, n);
        float td = Kd.r + Kd.g + Kd.b;
        float ts = Ks.r + Ks.g + Ks.b;
        float t = max(ts / (td + ts), 0.25f); // reflectiveness parameter
        if (isnan(t)) t = 1.f;
        
        vec3 wi{}, h{};
        float xi = GetRandom();
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
        
        return wi;
    }

protected:
    // attributes
    const Attribute& attrib;
    const Material& material;
};


// Quad Light Next-Event Estimation
class QuadLightNeeSampler : public ISampler
{
public:
    QuadLightNeeSampler(
        const Attribute& a,
        const Material& m,
        const QuadLight& l,
        const ISampler& s)
    : attrib(a), material(m), light(l), sampler(s)
    {}

    // ray-light only
    bool Intersect(const vec3& wi, float& dist) const
    {
        const vec3& x = attrib.hit;
        vec3 v0 = light.a;
        vec3 v1 = light.a + light.ab;
        vec3 v2 = light.a + light.ab + light.ac;
        vec3 v3 = light.a + light.ac;
        //vec3 dx = normalize(light.ab) * 0.01f;
        //vec3 dy = normalize(light.ac) * 0.01f;
        //v0 = v0 - dx - dy;
        //v1 = v1 + dx - dy;
        //v2 = v2 + dx + dy;
        //v3 = v3 - dx + dy;
        if (IsIntersectingTriangle(v0, v1, v2, x, wi, dist, kEpsilon)) return true;
        if (IsIntersectingTriangle(v0, v2, v3, x, wi, dist, kEpsilon)) return true;
        return false;
    }

    // occluded or not-intersecting
    bool Occluded(const vec3& wi, SceneNode* root) const
    {
        const vec3& x = attrib.hit;
        float R = kInfP;
        Intersect(wi, R);
        Ray ray{x, wi, R * 0.999f, 0.0001f};
        return IsRayOccluded(ray, root);
    }

    vec3 Eval(const vec3& wi) const
    {
        //const vec3& x = attrib.hit;
        //const vec3& n = attrib.normal;
        vec3 A = cross(light.ab, light.ac);
        vec3 nl = normalize(A);
        //float area = length(A);
        float R = kInfP;
        if (!Intersect(wi, R)) return {};
        //float G = dot(wi, n) * dot(wi, nl) / (R * R);
        vec3 brdf = sampler.Eval(wi);
        //return brdf * light.color * G * area;
        return brdf;// * dot(wi, nl);// / (R * R);// * area;
    }

    float PDF(const vec3& wi) const
    {
        //const vec3& x = attrib.hit;
        //const vec3& n = attrib.normal;
        vec3 A = cross(light.ab, light.ac);
        vec3 nl = normalize(A);
        float area = length(A);
        float R = kInfP;
        if (!Intersect(wi, R)) return 0.f;
        return (R * R) / (area * fabs(dot(nl, wi)));
    }

    // sample an non-normalized direction from hit point to light source
    vec3 Sample() const
    {
        const vec3& x = attrib.hit;
        vec3 xx = light.a + light.ab * GetRandom() + light.ac * GetRandom(); // x'
        return normalize(xx - x);
    }

protected:
    // attributes
    const Attribute& attrib;
    const Material& material;
    const QuadLight& light;

    // sampler
    const ISampler& sampler;
};


void MISTracer::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    const Material& material = scene->materials[attrib.mid];
    SceneNode* root = scene->root.get();

    vec3 n = normalize(attrib.normal);
    vec3 wo = normalize(attrib.incident);
    vec3 r = reflect(wo, n);
    const vec3& Kd = material.Kd;
    const vec3& Ks = material.Ks;
    const vec3& Ke = material.Ke;
    float s = material.ex;
    float a = material.rg;
    const auto& qlights = scene->qlights;

    PhongSampler phongSampler(attrib, material);
    GGXSampler ggxSampler(attrib, material);
    ISampler* ptrSampler;
    if (material.brdf == BRDF::PHONG) ptrSampler = &phongSampler;
    else if (material.brdf == BRDF::GGX) ptrSampler = &ggxSampler;
    const ISampler& sampler = *ptrSampler;
    
    // Weighted contribution of BRDF importance sampling
    //if (0)
    {
        float sumWeight{};
        vec3 wi = sampler.Sample();

        Payload bisPayload;
        Ray ray{attrib.hit, wi, kInfP, 0.001f};
        Trace(root, ray, bisPayload, 2, false, nullptr);

        float pdf0 = sampler.PDF(wi); // brdf
        vec3 rd = bisPayload.radiance * sampler.Eval(wi) * dot(n, wi) * pdf0;
        sumWeight += pdf0 * pdf0;
#if 0
        float pdf1{}; // nee
        if (bisPayload.light >= 0)
        {
            const auto& light = qlights[bisPayload.light];
            QuadLightNeeSampler qs(attrib, material, light, sampler);
            pdf1 += qs.PDF(wi);
        }
#else
        float pdf1{};
        for (const auto& light : qlights)
        {
            QuadLightNeeSampler qs(attrib, material, light, sampler);
            //if (qs.Occluded(wi, root)) continue;
            pdf1 += qs.PDF(wi);
        }
#endif
        pdf1 /= static_cast<float>(qlights.size());
        sumWeight += pdf1 * pdf1;

        payload.radiance += payload.weight * rd / max(sumWeight, kEpsilon);
    }

    // Weighted contribution of NEE
    //if (0)
    {
        vec3 rd{};

        for (const auto& light : qlights)
        {
            float sumWeight{};
            vec3 nl = normalize(cross(light.ab, light.ac));
            QuadLightNeeSampler qs(attrib, material, light, sampler);
            vec3 wi = qs.Sample();

            if (dot(nl, wi) < 0.f) continue; // no radiance from back of quad light source
            if (qs.Occluded(wi, root)) continue;

            float pdf0 = sampler.PDF(wi); // brdf
            sumWeight += pdf0 * pdf0;
#if 0
            float pdf1 = qs.PDF(wi) / static_cast<float>(qlights.size()); // nee
#else
            float pdf1{};
            for (const auto& other : qlights)
            {
                QuadLightNeeSampler qs1(attrib, material, other, sampler);
                //if (qs1.Occluded(wi, root)) continue;
                pdf1 += qs1.PDF(wi);
            }
            pdf1 /= static_cast<float>(qlights.size());
#endif
            sumWeight += pdf1 * pdf1;

            //vec3 rdNee = light.color * qs.Eval(wi) * dot(n, wi);
            //payload.radiance += payload.weight * rdNee / max(pdf1, kEpsilon);
            rd += light.color * qs.Eval(wi) * max(dot(n, wi), 0.f) * pdf1 / max(sumWeight, kEpsilon);
        }

        payload.radiance += payload.weight * rd / static_cast<float>(qlights.size());
    }

    // Russian Roulette ray termination
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
        payload.depth = -1; // depth = infinity
    }
    else
    {
        ++payload.depth;
    }

    vec3 wi = sampler.Sample();
    payload.origin = attrib.hit;
    payload.direction = wi;
    payload.weight *= sampler.Eval(wi) * max(dot(n, wi), 0.f) / max(sampler.PDF(wi), kEpsilon);
}


void OneStepShader::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    const Material& material = scene->materials[attrib.mid];
    payload.light = material.ls;
    //if (material.ls < 0) return;
    //const auto& light = scene->qlights[material.ls];
    //vec3 nl = normalize(cross(light.ab, light.ac));
    //if (dot(attrib.normal, nl) > 0.f) return;
    payload.radiance += material.Ke;
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
