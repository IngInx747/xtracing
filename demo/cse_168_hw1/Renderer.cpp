#include "Renderer.h"

#include <algorithm>

struct TriangleBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const
    {
        const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
        vec3 p0 = vec3(tri.transform * vec4(tri.p0, 1));
        vec3 p1 = vec3(tri.transform * vec4(tri.p1, 1));
        vec3 p2 = vec3(tri.transform * vec4(tri.p2, 1));
        return Bound(p0, p1, p2);
    }
};

struct SphereBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const
    {
        const Sphere& sphere = dynamic_cast<const Sphere&>(primitive);
        Aabb b = Bound({-1, -1, -1}, {1, 1, 1});
        return TransformBoundingBox(b, sphere.transform);
    }
};

struct TriangleCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const
    {
        const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
        vec3 org = World2ObjectHomoCoord(ray.org, tri.transform);
        vec3 dir = World2ObjectNonHomoCoord(ray.dir, tri.transform);

        const vec3& v0 = tri.p0;
        const vec3& v1 = tri.p1;
        const vec3& v2 = tri.p2;
        
        bool ret = IsIntersectingTriangle(v0, v1, v2, org, dir, ray.dist, ray.epsilon);
        if (!isShadowRay && ret)
        {
            vec3 h = ray.org + ray.dir * ray.dist;
            //vec3 h = Object2WorldHomoCoord(org + dir * dist, tri.transform);
            vec3 n = cross(v1 - v0, v0 - v2);
            if (dot(n, dir) > 0) n = -n;
            n = normalize(Object2WorldNonHomoCoord(n, tri.transform));

            attributes.hit = h;
            attributes.normal = n;
            attributes.incident = ray.dir;
            attributes.material = tri.material;
        }
        return ret;
    }

    mutable MyAttributes attributes;
    int isShadowRay{}; // flag bit [to-be-customized]
};

struct SphereCollide : public ICollide
{
    bool operator() (const IPrimitive& primitive, Ray& ray) const
    {
        const Sphere& sphere = dynamic_cast<const Sphere&>(primitive);
        vec3 org = World2ObjectHomoCoord(ray.org, sphere.transform);
        vec3 dir = World2ObjectNonHomoCoord(ray.dir, sphere.transform);

        bool ret = IsIntersectingUnitSphere(org, dir, ray.dist, ray.epsilon);
        if (!isShadowRay && ret)
        {
            vec3 h = ray.org + ray.dir * ray.dist;
            //vec3 h = Object2WorldHomoCoord(org + dir * dist, sphere.transform);
            vec3 n = normalize(Object2WorldNonHomoCoord(org + dir * ray.dist, sphere.transform));

            attributes.hit = h;
            attributes.normal = n;
            attributes.incident = ray.dir;
            attributes.material = sphere.material;
        }
        return ret;
    }

    mutable MyAttributes attributes;
    int isShadowRay{}; // flag bit [to-be-customized]
};


Renderer::Renderer(std::shared_ptr<Scene> scene) : scene(scene)
{
    // Initialize Optix programs
    //initPrograms();

    // Build the scene by constructing an Optix graph
    buildScene();
}

void GenRay(const int2& index, const int2& dim, const CameraFrame& cam, vec3& org, vec3& dir)
{
    vec2 p = vec2{index.x, index.y} + vec2{0.5f, 0.5f};
    vec2 d = p / vec2{dim.x, dim.y} * 2.0f - 1.0f;

    dir = normalize(d.x * cam.u + d.y * cam.v + cam.w);
    org = cam.o;
}

void Renderer::run(bool progressive)
{
    while (currentFrame != numFrames)
    {
        // Render a frame.
//#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                //printf("pixel(%d, %d)\n", i, j);
                generateRay({j, i}, {width, height});
            }
        }
        
        // Only render a frame in progressive mode
        if (progressive) break;
        ++currentFrame;
    }
}

void Renderer::buildScene()
{
    // Record some important info
    numFrames = 1;
    currentFrame = 0;
    width = scene->width;
    height = scene->height;
    resultBuffer.resize(width * height);
    outputFilename = scene->outputFilename;

    TriangleBound triangleBound;
    EqualCountsSplit<Triangle, IBound> triangleSplit(triangleBound);
    mAccelTri.Build<IBound, decltype(triangleSplit)>(
        scene->triangles, triangleBound, triangleSplit);

    SphereBound sphereBound;
    EqualCountsSplit<Sphere, IBound> sphereSplit(sphereBound);
    mAccelSphere.Build<IBound, decltype(sphereSplit)>(
        scene->spheres, sphereBound, sphereSplit);
}

void Renderer::generateRay(const int2& index, const int2& dim)
{
    vec3 result{0, 0, 0};
    Payload payload;
    payload.done = false;
    payload.depth = scene->depth;
    payload.mask = {1, 1, 1};
    float epsilon = 0.001f;

    // generate the 1st ray per pixel
    vec3 org, dir;
    GenRay(index, dim, scene->cameraFrame, org, dir); // Ray 0

    do
    {
        Ray ray{org, dir, kInfP, epsilon}; // Ray 1 ~ n

        rtTrace(ray, payload);

        result += payload.radiance;

        org = payload.origin;
        dir = payload.direction;
    }
    while (!payload.done && payload.depth > 0);

    // Write the result
    resultBuffer[index.y * width + index.x] = result; // <-- overload this
}

void Renderer::rtTrace(Ray& ray, Payload& payload)
{
    bool hit = false;
    MyAttributes attrib;

    TriangleCollide triangleCollide;
    ICollide* tcref = &triangleCollide;
    //if (mAccelTri.ClosestHit<decltype(triangleCollide)>(triangleCollide, ray))
    if (mAccelTri.Intersect<ICollide>(*tcref, ray, false)) // Ok
    {
        hit = true;
        attrib = triangleCollide.attributes;
    }

    SphereCollide sphereCollide;
    if (mAccelSphere.Intersect<ICollide>(sphereCollide, ray, false))
    {
        hit = true;
        attrib = sphereCollide.attributes;
    }

    // deal with payload
    if (hit)
    {
        // go to closestHit program
        closestHit(payload, attrib); // LOOK: payload and attrib could be derived class
    }
    else
    {
        // go to miss program
        miss(payload);
    }
}

void Renderer::rtTrace(Ray& ray, ShadowPayload& shadowPayload)
{
    bool hit = false;

    TriangleCollide triangleCollide;
    triangleCollide.isShadowRay = true;
    if (mAccelTri.Intersect<decltype(triangleCollide)>(triangleCollide, ray, true))
    {
        hit = true;
    }

    SphereCollide sphereCollide;
    sphereCollide.isShadowRay = true;
    if (mAccelSphere.Intersect<decltype(sphereCollide)>(sphereCollide, ray, true))
    {
        hit = true;
    }

    // deal with payload
    if (hit)
    {
        // go to anyHit program
        anyHit(shadowPayload);
    }
}

void Renderer::miss(Payload& payload)
{
    payload.radiance = {0, 0, 0}; // backgroundColor
    payload.done = true;
}

void Renderer::anyHit(ShadowPayload& shadowPayload)
{
    shadowPayload.isVisible = false;
}

void Renderer::closestHit(Payload& payload, MyAttributes& attrib)
{
    vec3 result{};
    const MyMaterial& material = attrib.material;

    result += material.ambient;
    result += material.emission;
    auto dlights = scene->dlights;
    auto plights = scene->plights;

    for (int i = 0; i < dlights.size(); ++i)
    {
        DirectionalLight light = dlights[i];
        vec3 L = normalize(-light.direction);

        float epsilon = 0.001f;// 0.00001f;
        Ray ray{attrib.hit + attrib.normal * epsilon, L, kInfP, 0};
        ShadowPayload shadowPayload;
        shadowPayload.isVisible = true;
        rtTrace(ray, shadowPayload);
        if (!shadowPayload.isVisible) continue;

        vec3 N = normalize(attrib.normal);
        vec3 E = normalize(attrib.incident);
        vec3 H = normalize(L - E);

        vec3 I = light.color;
        vec3 D = material.diffuse;
        vec3 S = material.specular;
        float s = material.shininess;

        result += I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s));
    }

    for (int i = 0; i < plights.size(); ++i)
    {
        PointLight light = plights[i];
        vec3 P = light.position - attrib.hit;
        vec3 L = normalize(P);
        float r = length(P);

        float epsilon = 0.001f;// 0.00001f;
        Ray ray{attrib.hit + attrib.normal * epsilon, L, r, 0};
        ShadowPayload shadowPayload;
        shadowPayload.isVisible = true;
        rtTrace(ray, shadowPayload);
        if (!shadowPayload.isVisible) continue;

        vec3 N = normalize(attrib.normal);
        vec3 E = normalize(attrib.incident);
        vec3 H = normalize(L - E);
        float attenuation = light.c0 + light.c1 * r + light.c2 * r * r;

        vec3 I = light.color;
        vec3 D = material.diffuse;
        vec3 S = material.specular;
        float s = material.shininess;

        result += I * (D * std::max(dot(N, L), 0.f) + S * std::powf(std::max(dot(N, H), 0.f), s)) / (attenuation);
    }

    payload.radiance = result * payload.mask;
    payload.mask *= material.specular;
    payload.origin = attrib.hit;
    payload.direction = reflect(attrib.incident, attrib.normal);
    --payload.depth;
}
