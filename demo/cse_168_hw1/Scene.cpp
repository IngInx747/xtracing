#include "Scene.h"


struct TriangleBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const
    {
        const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
        return Bound(tri.p0, tri.p1, tri.p2);
    }
};


struct SphereBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const
    {
        const Sphere& sphere = dynamic_cast<const Sphere&>(primitive);
        const mat4& transform = scene->transforms[sphere.tid];
        vec3 p = sphere.p;
        vec3 r{sphere.r, sphere.r, sphere.r};
        return TransformBoundingBox(Bound(p - r, p + r), transform); // obj -> world
    }
    
    Scene* scene;
};


struct TriangleCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const
    {
        const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
        vec3 org = ray.org;
        vec3 dir = ray.dir;

        const vec3& v0 = tri.p0;
        const vec3& v1 = tri.p1;
        const vec3& v2 = tri.p2;
        
        bool ret = IsIntersectingTriangle(v0, v1, v2, org, dir, ray.dist, ray.epsilon);
        if (ret && !IsShadowRay())
        {
            vec3 h = org + dir * ray.dist;
            vec3 n = normalize(cross(v1 - v0, v0 - v2));
            if (dot(n, dir) > 0) n = -n;

            //h = TransformPoint(h, GetWorldTransform());
            //n = normalize(Object2WorldNonHomoCoord(n, GetWorldTransform()));
            //dir = normalize(Object2WorldNonHomoCoord(dir, GetWorldTransform()));

            Attribute attribute;
            attribute.hit = h;
            attribute.normal = n;
            attribute.incident = dir;
            attribute.mid = tri.mid;
            SetMaterialIndex(0);
            SetAttribute<Attribute>(attribute);
        }
        return ret;
    }
};


struct SphereCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const
    {
        const Sphere& sphere = dynamic_cast<const Sphere&>(primitive);
        const mat4& transform = scene->transforms[sphere.tid];
        mat4 invT = inverse(transform);
        vec3 org = TransformPoint(ray.org, invT);
        vec3 dir = TransformVector(ray.dir, invT);

        bool ret = IsIntersectingSphere(sphere.p, sphere.r, org, dir, ray.dist, ray.epsilon);
        if (ret && !IsShadowRay())
        {
            vec3 h = org + dir * ray.dist;
            vec3 n = h - sphere.p;

            h = TransformPoint(h, transform);
            n = normalize(TransformNormal(n, transform));
            dir = normalize(TransformVector(dir, transform));
            //dir = normalize(h - TransformPoint(org, transform));

            Attribute attribute;
            attribute.hit = h;
            attribute.normal = n;
            attribute.incident = dir;
            attribute.mid = sphere.mid;
            SetMaterialIndex(0);
            SetAttribute<Attribute>(attribute);
        }
        return ret;
    }
    
    Scene* scene;
};


struct PhongShader : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        Payload& payload = dynamic_cast<Payload&>(payload_);
        const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);

        vec3 result{};
        const Material& material = scene->materials[attrib.mid];
        //payload.radiance += (attrib.normal + 1.f) * 0.5f;
        //--payload.depth;
        //return;

        result += material.Ka + material.Ke; // ambient + emission

        auto dlights = scene->dlights;
        auto plights = scene->plights;

        for (int i = 0; i < dlights.size(); ++i)
        {
            DirectionalLight light = dlights[i];
            vec3 L = normalize(-light.dir);

            float epsilon = 0.001f;// 0.00001f;
            Ray ray{attrib.hit + attrib.normal * epsilon, L, kInfP, 0}; // scene6: 0
            ShadowPayload shadowPayload;
            shadowPayload.visible = true;
            Trace(scene->root.get(), ray, shadowPayload, 1, true, nullptr);
            if (!shadowPayload.visible) continue;

            vec3 N = normalize(attrib.normal);
            vec3 E = normalize(attrib.incident);
            vec3 H = normalize(L - E);

            vec3 I = light.color;
            vec3 D = material.Kd;
            vec3 S = material.Ks;
            float s = material.ex;

            result += I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s));
        }

        for (int i = 0; i < plights.size(); ++i)
        {
            PointLight light = plights[i];
            vec3 P = light.pos - attrib.hit;
            vec3 L = normalize(P);
            float r = length(P);

            float epsilon = 0.0001f;// 0.00001f;
            Ray ray{attrib.hit + attrib.normal * epsilon, L, r, 0.01f}; // scene6: 0
            ShadowPayload shadowPayload;
            shadowPayload.visible = true;
            Trace(scene->root.get(), ray, shadowPayload, 1, true, nullptr);
            if (!shadowPayload.visible) continue;

            vec3 N = normalize(attrib.normal);
            vec3 E = normalize(attrib.incident);
            vec3 H = normalize(L - E);
            float attenuation = light.c0 + light.c1 * r + light.c2 * r * r;

            vec3 I = light.color;
            vec3 D = material.Kd;
            vec3 S = material.Ks;
            float s = material.ex;

            result += I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s)) / (attenuation);
        }

        payload.radiance = result * payload.mask;
        payload.mask *= material.Ks;
        payload.origin = attrib.hit;
        payload.direction = reflect(attrib.incident, attrib.normal);
        --payload.depth;
    }

    Scene* scene;
};


struct ShadowShader : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        ShadowPayload& payload = dynamic_cast<ShadowPayload&>(payload_);
        payload.visible = false;
        TerminateRay(); // actually has no impact
    }
};


struct Miss : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        Payload& payload = dynamic_cast<Payload&>(payload_);
        payload.radiance = background;
        payload.done = true;
    }

    vec3 background;
};


void Scene::Build()
{
    // setup geometry programs (triangle)
    std::shared_ptr<TriangleBound> triangleBound = std::make_shared<TriangleBound>();
    std::shared_ptr<TriangleCollide> triangleCollide = std::make_shared<TriangleCollide>();
    std::shared_ptr<GeometryProgram> triangleGeometry = std::make_shared<GeometryProgram>();
    triangleGeometry->SetBoundProgram<TriangleBound>(triangleBound);
    triangleGeometry->SetCollideProgram<TriangleCollide>(triangleCollide);

    // setup geometry programs (sphere)
    std::shared_ptr<SphereBound> sphereBound = std::make_shared<SphereBound>();
    sphereBound->scene = this;
    std::shared_ptr<SphereCollide> sphereCollide = std::make_shared<SphereCollide>();
    sphereCollide->scene = this;
    std::shared_ptr<GeometryProgram> sphereGeometry = std::make_shared<GeometryProgram>();
    sphereGeometry->SetBoundProgram<SphereBound>(sphereBound);
    sphereGeometry->SetCollideProgram<SphereCollide>(sphereCollide);

    // setup material programs
    std::shared_ptr<PhongShader> rayClosestHit = std::make_shared<PhongShader>();
    rayClosestHit->scene = this;
    std::shared_ptr<ShadowShader> shadowAnyHit = std::make_shared<ShadowShader>();
    std::shared_ptr<MaterialProgram> materialProgram = std::make_shared<MaterialProgram>();
    materialProgram->SetClosestHitProgram<PhongShader>(0, rayClosestHit); // 0: common ray
    materialProgram->SetAnyHitProgram<ShadowShader>(1, shadowAnyHit); // 1: shadow ray

    // setup context programs
    std::shared_ptr<Miss> bc = std::make_shared<Miss>();
    bc->background = vec3{0, 0, 0};
    miss = bc; // scene miss shader

    // primitive node 0 (triangle)
    std::shared_ptr<PrimitiveNode> p0 = std::make_shared<PrimitiveNode>();
    p0->SetGeometry(triangleGeometry);
    p0->SetMaterial(0, materialProgram);
    p0->SetBuffer<Triangle>(triangles);
    p0->SetAccel(AccelEnum::BVH);

    // primitive node 1 (sphere)
    std::shared_ptr<PrimitiveNode> p1 = std::make_shared<PrimitiveNode>();
    p1->SetGeometry(sphereGeometry);
    p1->SetMaterial(0, materialProgram);
    p1->SetBuffer<Sphere>(spheres);
    p1->SetAccel(AccelEnum::BVH);

    std::shared_ptr<RootNode> rn = std::make_shared<RootNode>();
    rn = std::make_shared<RootNode>();
    rn->SetAccel(AccelEnum::BVH);

    // assemble all nodes
    rn->AppendChild(p1);
    rn->AppendChild(p0);

    // build from root
    rn->UpdateFromLeaves();
    root = rn;

    // clean
    triangles.clear();
    spheres.clear();
}
