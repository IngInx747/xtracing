#include "Shader.h"

#include "Light.h"
#include "Scene.h"


void PhongShader::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    const Material& material = scene->materials[attrib.mid];
    SceneNode* root = scene->root.get();
    vec3 result{};

    result += material.Ka + material.Ke; // ambient + emission

    if (material.ls)
    {
        payload.radiance = result * payload.mask;
        payload.done = true;
        return;
    }

    const auto dlights = scene->dlights;
    const auto plights = scene->plights;
    const auto qlights = scene->qlights;

    for (int i = 0; i < dlights.size(); ++i)
    {
        const DirectionalLight& light = dlights[i];
        result += ShadeDirectionalLight(light, attrib, material, root);
    }

    for (int i = 0; i < plights.size(); ++i)
    {
        const PointLight& light = plights[i];
        result += ShadePointLight(light, attrib, material, root);
    }

    for (int i = 0; i < qlights.size(); ++i)
    {
        const QuadLight& light = qlights[i];
        if (scene->integrator == Scene::Integrator::ANALYTIC_DIRECT)
            result += ShadeQuadLightAnalytic(light, attrib, material);
        else if (scene->integrator == Scene::Integrator::DIRECT)
            result += ShadeQuadLightMonteCarlo(light, attrib, material,
                root, scene->nLightSamples, scene->bLightstratify);
    }
    
    payload.radiance = result * payload.mask;
    payload.mask *= material.Ks;

    //if (length(payload.mask) < 0.005f)
    {
        payload.done = true;
    }
    //else
    //{
    //    payload.origin = attrib.hit;
    //    payload.direction = reflect(attrib.incident, attrib.normal);
    //    --payload.depth;
    //}
}


void ShadowShader::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    ShadowPayload& payload = dynamic_cast<ShadowPayload&>(payload_);

    // To enable online visibility check, comment off IsShadowRay() in the colliders
    //const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);
    //const Material& material = scene->materials[attrib.mid];
    //if (!material.ls) payload.visible = false;

    payload.visible = false;
    TerminateRay(); // actually has no impact
}


void Miss::operator() (IPayload& payload_, const IAttribute& attrib_) const
{
    Payload& payload = dynamic_cast<Payload&>(payload_);
    payload.radiance = background;
    payload.done = true;
}
