#include "Scene.h"

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

template<typename T>
inline T GetPixel2D(const std::vector<T>& texture, const vec2& uv, const int2& dim)
{
    int w = dim.x;
    int h = dim.y;
    RepeatWrapper tw;
    float u = tw(uv.x);
    float v = tw(uv.y);
	int x = static_cast<int>(u * static_cast<float>(w));
	int y = static_cast<int>(v * static_cast<float>(h));
	x %= w;
	y %= h;
	return texture[y * w + x];
}


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
        vec3 p = sphere.p;
        vec3 r{sphere.r, sphere.r, sphere.r};
        return Bound(p - r, p + r);
    }
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
            vec3 n = cross(v1 - v0, v0 - v2);
            if (dot(n, dir) > 0) n = -n;
            vec3 uvw = GetBarycentricCoordinates(v0, v1, v2, h);
            vec2 tc = tri.t0 * uvw.x + tri.t1 * uvw.y + tri.t2 * uvw.z;

            h = TransformPoint(h, GetWorldTransform());
            n = normalize(TransformNormal(n, GetWorldTransform()));
            dir = normalize(TransformVector(dir, GetWorldTransform()));

            Attribute attribute;
            attribute.hit = h;
            attribute.normal = n;
            attribute.incident = dir;
            attribute.texCoord = tc;
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
        vec3 org = ray.org;
        vec3 dir = ray.dir;
        bool ret = false;
        float dist = ray.dist; // backup

        ret = IsIntersectingSphere(sphere.p, sphere.r, org, dir, ray.dist, ray.epsilon);

        if (ret)
        {
            vec3 h = org + dir * ray.dist;
            vec3 sc = GetSphericalCoordinates(sphere.p, h); // rho, phi, psi
            vec2 tc = { sc.y / kPi, sc.z / (kPi * 2) };
            int ignore = GetPixel2D<int>(scene->ignore, tc, int2{ 8, 8 });
            if (ignore == 0)
            {
                ret = false;
                ray.dist = dist; // restore ray status
                org = h + dir * 0.001f;
                bool re2 = IsIntersectingSphere(sphere.p, sphere.r, org, dir, ray.dist, ray.epsilon);
                if (re2)
                {
                    h = org + dir * ray.dist;
                    sc = GetSphericalCoordinates(sphere.p, h); // rho, phi, psi
                    tc = { sc.y / kPi, sc.z / (kPi * 2) };
                    ignore = GetPixel2D<int>(scene->ignore, tc, int2{ 8, 8 });
                    if (ignore != 0) ret = true;
                }
            }
        }

        if (ret && !IsShadowRay())
        {
            vec3 h = org + dir * ray.dist;
            vec3 n = h - sphere.p;
            if (dot(n, dir) > 0) n = -n;
            vec3 sc = GetSphericalCoordinates(sphere.p, h); // rho, phi, psi
            vec2 tc = {sc.y / kPi, sc.z / (kPi * 2)};

            h = TransformPoint(h, GetWorldTransform());
            n = normalize(TransformNormal(n, GetWorldTransform()));
            dir = normalize(TransformVector(dir, GetWorldTransform()));

            Attribute attribute;
            attribute.hit = h;
            attribute.normal = n;
            attribute.incident = dir;
            attribute.texCoord = tc;
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

        //payload.radiance = (attrib.normal * 2.0f) - 1.0f; return;

        result += material.Ka; // ambient + emission

        auto dlights = scene->dlights;
        auto plights = scene->plights;

        for (int i = 0; i < dlights.size(); ++i)
        {
            DirectionalLight light = dlights[i];
            vec3 L = normalize(-light.dir);

            float epsilon = 0.001f;// 0.00001f;
            Ray ray{attrib.hit + attrib.normal * epsilon, L, kInfP, 0};
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

            if (material.tid >= 0) D *= GetPixel2D(scene->chessboard, attrib.texCoord, { 8, 8 });

            result += I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s));
        }

        for (int i = 0; i < plights.size(); ++i)
        {
            PointLight light = plights[i];
            vec3 P = light.pos - attrib.hit;
            vec3 L = normalize(P);
            float r = length(P);

            float epsilon = 0.001f;// 0.00001f;
            Ray ray{attrib.hit + attrib.normal * epsilon, L, r, 0};
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

            if (material.tid >= 0) D *= GetPixel2D(scene->chessboard, attrib.texCoord, { 8, 8 });

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


// used for triangle only(sphere or any primitive with 2 potential intersections does not apply)
struct IgnoreShader : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        Payload& payload = dynamic_cast<Payload&>(payload_);
        const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);

        vec2 uv = attrib.texCoord;
        int ignore = GetPixel2D<int>(scene->ignore, uv, int2{ 8, 8 });

        if (ignore == 0) IgnoreIntersection();
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


static void LoadModel(Scene* scene, std::vector<Triangle>& triangles, std::vector<Sphere>& spheres)
{
    vec3 vs[8] = {
        {-0.5f, -0.5f,  0.5f},
        { 0.5f, -0.5f,  0.5f},
        {-0.5f,  0.5f,  0.5f},
        { 0.5f,  0.5f,  0.5f},
        {-0.5f,  0.5f, -0.5f},
        { 0.5f,  0.5f, -0.5f},
        {-0.5f, -0.5f, -0.5f},
        { 0.5f, -0.5f, -0.5f}
    };

    vec2 ts[4] = {
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, 1.0f},
        {1.0f, 1.0f}
    };

    Material m0{};
    m0.Kd = vec3{ 1, 1, 1 };
    m0.Ks = vec3{ 0, 0, 0 };
    m0.ex = 2.2f;
    scene->materials.push_back(m0);

    Material m1{};
    m1.Kd = vec3{ 1, 0, 0 };
    m1.Ks = vec3{ 0, 0, 0 };
    m1.ex = 2.2f;
    m1.tid = 0;
    scene->materials.push_back(m1);

    Material m2{};
    m2.Kd = vec3{ 0, 0, 1 };
    m2.Ks = vec3{ 0, 0, 0 };
    m2.ex = 2.2f;
    m2.tid = 0;
    scene->materials.push_back(m2);

    Material m3{};
    m3.Kd = vec3{ 1, 1, 1 };
    m3.Ks = vec3{ 0.8f, 0.8f, 0.8f };
    m3.ex = 5.2f;
    scene->materials.push_back(m3);

    // cell
    {
        Triangle tri;
        tri.p0 = vs[2]; tri.p1 = vs[3]; tri.p2 = vs[4];
        tri.t0 = ts[0]; tri.t1 = ts[1]; tri.t2 = ts[2];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    {
        Triangle tri;
        tri.p0 = vs[4]; tri.p1 = vs[3]; tri.p2 = vs[5];
        tri.t0 = ts[2]; tri.t1 = ts[1]; tri.t2 = ts[3];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    // back
    {
        Triangle tri;
        tri.p0 = vs[4]; tri.p1 = vs[5]; tri.p2 = vs[6];
        tri.t0 = ts[3]; tri.t1 = ts[2]; tri.t2 = ts[1];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    {
        Triangle tri;
        tri.p0 = vs[6]; tri.p1 = vs[5]; tri.p2 = vs[7];
        tri.t0 = ts[1]; tri.t1 = ts[2]; tri.t2 = ts[0];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    // floor
    {
        Triangle tri;
        tri.p0 = vs[6]; tri.p1 = vs[7]; tri.p2 = vs[0];
        tri.t0 = ts[0]; tri.t1 = ts[1]; tri.t2 = ts[2];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    {
        Triangle tri;
        tri.p0 = vs[0]; tri.p1 = vs[7]; tri.p2 = vs[1];
        tri.t0 = ts[2]; tri.t1 = ts[1]; tri.t2 = ts[3];
        tri.mid = 0;
        triangles.push_back(tri);
    }

    // right
    {
        Triangle tri;
        tri.p0 = vs[1]; tri.p1 = vs[7]; tri.p2 = vs[3];
        tri.t0 = ts[0]; tri.t1 = ts[1]; tri.t2 = ts[2];
        tri.mid = 2;
        triangles.push_back(tri);
    }

    {
        Triangle tri;
        tri.p0 = vs[3]; tri.p1 = vs[7]; tri.p2 = vs[5];
        tri.t0 = ts[2]; tri.t1 = ts[1]; tri.t2 = ts[3];
        tri.mid = 2;
        triangles.push_back(tri);
    }

    // left
    {
        Triangle tri;
        tri.p0 = vs[6]; tri.p1 = vs[0]; tri.p2 = vs[4];
        tri.t0 = ts[0]; tri.t1 = ts[1]; tri.t2 = ts[2];
        tri.mid = 1;
        triangles.push_back(tri);
    }

    {
        Triangle tri;
        tri.p0 = vs[4]; tri.p1 = vs[0]; tri.p2 = vs[2];
        tri.t0 = ts[2]; tri.t1 = ts[1]; tri.t2 = ts[3];
        tri.mid = 1;
        triangles.push_back(tri);
    }

    {
        Sphere sphere;
        sphere.p = vec3{ -0.25f, -0.25f, 0 }; sphere.r = 0.2f; sphere.mid = 0;
        spheres.push_back(sphere);
    }

    {
        Sphere sphere;
        sphere.p = vec3{  0.25f, -0.25f, 0 }; sphere.r = 0.2f; sphere.mid = 0;
        spheres.push_back(sphere);
    }
}


void LoadLights(Scene* scene)
{
    {
        DirectionalLight dl{ vec3{0, -1, -1}, vec3{ 0.2f } };
        scene->dlights.push_back(dl);
    }

    {
        PointLight pl{ vec3{0, 0.25f, 0}, vec3{ 0.6f }, 1, 0.1f, 0.05f };
        scene->plights.push_back(pl);
    }
}


void LoadTexture(Scene* scene)
{
    auto& ignore = scene->ignore;
    ignore.resize(64);

	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			ignore[i * 8 + j] = ((i + j) & 1) ? 0 : 1;
            
    auto& chessboard = scene->chessboard;
    chessboard.resize(64);

	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			chessboard[i * 8 + j] = ((i + j) & 1) ? vec3{0.2f, 0.2f, 0.2f} : vec3{0.8f, 0.8f, 0.8f};
}


bool LoadScene(Scene* scene)
{
    // declaration
    int width = 800;
    int height = 800;
    int depth = 5;
    std::string outputName = "demo_texture.png";
    std::vector<Triangle> triangles;
    std::vector<Sphere> spheres;

    // setup geometry programs (triangle)
    std::shared_ptr<TriangleBound> triangleBound = std::make_shared<TriangleBound>();
    std::shared_ptr<TriangleCollide> triangleCollide = std::make_shared<TriangleCollide>();
    std::shared_ptr<GeometryProgram> triangleGeometry = std::make_shared<GeometryProgram>();
    triangleGeometry->SetBoundProgram<TriangleBound>(triangleBound);
    triangleGeometry->SetCollideProgram<TriangleCollide>(triangleCollide);

    // setup geometry programs (sphere)
    std::shared_ptr<SphereBound> sphereBound = std::make_shared<SphereBound>();
    std::shared_ptr<SphereCollide> sphereCollide = std::make_shared<SphereCollide>();
    sphereCollide->scene = scene;
    std::shared_ptr<GeometryProgram> sphereGeometry = std::make_shared<GeometryProgram>();
    sphereGeometry->SetBoundProgram<SphereBound>(sphereBound);
    sphereGeometry->SetCollideProgram<SphereCollide>(sphereCollide);

    // setup material programs
    std::shared_ptr<ShadowShader> shadowShader = std::make_shared<ShadowShader>();
    std::shared_ptr<IgnoreShader> ignoreShader = std::make_shared<IgnoreShader>();
    ignoreShader->scene = scene;
    std::shared_ptr<PhongShader> phongShader = std::make_shared<PhongShader>();
    phongShader->scene = scene;

    std::shared_ptr<MaterialProgram> phongProgram = std::make_shared<MaterialProgram>();
    phongProgram->SetClosestHitProgram<PhongShader>(0, phongShader); // 0: common ray
    phongProgram->SetAnyHitProgram<ShadowShader>(1, shadowShader); // 1: shadow ray

    std::shared_ptr<MaterialProgram> checkerProgram = std::make_shared<MaterialProgram>();
    checkerProgram->SetClosestHitProgram<PhongShader>(0, phongShader); // 0: common ray
    //checkerProgram->SetAnyHitProgram<IgnoreShader>(0, ignoreShader); // 0: common ray
    checkerProgram->SetAnyHitProgram<ShadowShader>(1, shadowShader); // 1: shadow ray

    // setup context programs
    std::shared_ptr<Miss> miss = std::make_shared<Miss>();
    miss->background = vec3{0, 0, 0};
    scene->miss = miss;

    // setup model(s)
    LoadModel(scene, triangles, spheres);

    // setup texture(s)
    LoadTexture(scene);

    // setup light source(s)
    LoadLights(scene);

    // primitive node 0 (triangle)
    std::shared_ptr<PrimitiveNode> p0 = std::make_shared<PrimitiveNode>();
    p0->SetGeometry(triangleGeometry);
    p0->SetMaterial(0, phongProgram);
    p0->SetBuffer<Triangle>(triangles);
    p0->SetAccel(AccelEnum::NONE);

    // primitive node 1 (sphere)
    std::shared_ptr<PrimitiveNode> p1 = std::make_shared<PrimitiveNode>();
    p1->SetGeometry(sphereGeometry);
    p1->SetMaterial(0, checkerProgram);
    p1->SetBuffer<Sphere>(spheres);
    p1->SetAccel(AccelEnum::NONE);

    std::shared_ptr<RootNode> root = std::make_shared<RootNode>();
    root->SetAccel(AccelEnum::NONE);

    root->AppendChild(p0);
    root->AppendChild(p1);

    // update all nodes
    root->UpdateFromLeaves();

    // camera
    vec3 eye    = { 0, 0, 2 };
    vec3 lookat = { 0, 0, 0 };
    vec3 up     = { 0, 1, 0 };
    float fov = radians(45.0f);
    float aspect = (float)width / (float)height;
    scene->cameraFrame.o = eye;
    ComputeCameraFrame(
        eye, lookat, up, fov, aspect,
        scene->cameraFrame.u, scene->cameraFrame.v, scene->cameraFrame.w, true);

    // assemble all stuff
    scene->root = root;
    scene->depth = depth;
    scene->width = width;
    scene->height = height;
    scene->outputFilename.clear();
    scene->outputFilename.append(outputName);

    return true;
}