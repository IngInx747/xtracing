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


void SceneLoader::Load(const std::string& filename, Scene* scene)
{
    // setup geometry programs (triangle)
    std::shared_ptr<TriangleBound> triangleBound = std::make_shared<TriangleBound>();
    std::shared_ptr<TriangleCollide> triangleCollide = std::make_shared<TriangleCollide>();
    std::shared_ptr<GeometryProgram> triangleGeometry = std::make_shared<GeometryProgram>();
    triangleGeometry->SetBoundProgram<TriangleBound>(triangleBound);
    triangleGeometry->SetCollideProgram<TriangleCollide>(triangleCollide);

    // setup geometry programs (sphere)
    std::shared_ptr<SphereBound> sphereBound = std::make_shared<SphereBound>();
    sphereBound->scene = scene;
    std::shared_ptr<SphereCollide> sphereCollide = std::make_shared<SphereCollide>();
    sphereCollide->scene = scene;
    std::shared_ptr<GeometryProgram> sphereGeometry = std::make_shared<GeometryProgram>();
    sphereGeometry->SetBoundProgram<SphereBound>(sphereBound);
    sphereGeometry->SetCollideProgram<SphereCollide>(sphereCollide);

    // setup material programs
    std::shared_ptr<PhongShader> rayClosestHit = std::make_shared<PhongShader>();
    rayClosestHit->scene = scene;
    std::shared_ptr<ShadowShader> shadowAnyHit = std::make_shared<ShadowShader>();
    std::shared_ptr<MaterialProgram> materialProgram = std::make_shared<MaterialProgram>();
    materialProgram->SetClosestHitProgram<PhongShader>(0, rayClosestHit); // 0: common ray
    materialProgram->SetAnyHitProgram<ShadowShader>(1, shadowAnyHit); // 1: shadow ray

    // setup context programs
    std::shared_ptr<Miss> miss = std::make_shared<Miss>();
    miss->background = vec3{0, 0, 0};
    scene->miss = miss;

    // scene config default
    scene->width = 256;
    scene->height = 256;
    scene->depth = 5;
    scene->outputFilename = "output.png";

    // load models and light sources
    load(filename, scene);

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

    std::shared_ptr<RootNode> root = std::make_shared<RootNode>();
    root->SetAccel(AccelEnum::BVH);

    // assemble all stuff
    root->AppendChild(p1);
    root->AppendChild(p0);
    root->UpdateFromLeaves();

    scene->root = root;

    // clean
    triangles.clear();
    spheres.clear();
}


template <class T>
bool readValues(std::stringstream& s, const int numvals, T* values)
{
    for (int i = 0; i < numvals; i++)
    {
        s >> values[i];
        if (s.fail())
        {
            std::cout << "Failed reading value " << i << " will skip" << std::endl;
            return false;
        }
    }
    return true;
}


void SceneLoader::load(const std::string& filename, Scene* scene)
{
    // Attempt to open the scene file 
    std::ifstream in(filename);
    if (!in.is_open())
    {
        // Unable to open the file. Check if the filename is correct.
        throw std::runtime_error("Unable to open scene file " + filename);
    }

    std::string str, cmd;

    std::stack<mat4> transStack;
    transStack.push(mat4{1.0f});

    // static var: material
    Material material;
    material.Ka = { 0, 0, 0 };
    material.Kd = { 1, 1, 1 };
    material.Ks = { 0, 0, 0 };
    material.Ke = { 0, 0, 0 };
    material.ex = 0;

    // static var: directional light
    float c0 = 1, c1 = 0, c2 = 0;

    // temp buffers
    std::vector<vec3> vertices;

    // Read a line in the scene file in each iteration
    while (std::getline(in, str))
    {
        // Ruled out comment and blank lines
        if ((str.find_first_not_of(" \t\r\n") == std::string::npos) 
            || (str[0] == '#')) continue;

        // Read a command
        std::stringstream s(str);
        s >> cmd;

        // Some arrays for storing values
        float fvalues[12];
        int ivalues[3];
        std::string svalues[1];

        if (cmd == "size" && readValues(s, 2, fvalues))
        {
            scene->width = (unsigned int)fvalues[0];
            scene->height = (unsigned int)fvalues[1];
        }
        else if (cmd == "output" && readValues(s, 1, svalues))
        {
            scene->outputFilename = svalues[0];
        }
        else if (cmd == "maxdepth" && readValues(s, 1, ivalues))
        {
            scene->depth = ivalues[0];
        }
        else if (cmd == "translate" && readValues(s, 3, fvalues))
        {
            mat4& T = transStack.top();
            T = T * translate(mat4{1.0f}, { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "scale" && readValues(s, 3, fvalues))
        {
            mat4& T = transStack.top();
            T = T * scale(mat4{1.0f}, { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "rotate" && readValues(s, 4, fvalues))
        {
            mat4& T = transStack.top();
            T = T * rotate(mat4{1.f}, radians(fvalues[3]), { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "pushTransform")
        {
            transStack.push(transStack.top());
        }
        else if (cmd == "popTransform")
        {
            if (transStack.size() > 1)
                transStack.pop();
            else
                std::cerr << "[SceneLoader] Command \"popTransform\": Stack ran out of elements\n";
        }
        else if (cmd == "camera" && readValues(s, 10, fvalues))
        {
            vec3 eye    = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 lookat = { fvalues[3], fvalues[4], fvalues[5] };
            vec3 up     = { fvalues[6], fvalues[7], fvalues[8] };
            float fov = radians(fvalues[9]);
            float aspect = (float)scene->width / (float)scene->height;

            scene->cameraFrame.o = eye;

            ComputeCameraFrame(
                eye, lookat, up, fov, aspect,
                scene->cameraFrame.u, scene->cameraFrame.v, scene->cameraFrame.w, true);
        }
        else if (cmd == "vertex" && readValues(s, 3, fvalues))
        {
            vec3 v = { fvalues[0], fvalues[1], fvalues[2] };
            vertices.push_back(v);
        }
        else if (cmd == "tri" && readValues(s, 3, ivalues))
        {
            vec3 p0 = vertices[ivalues[0]];
            vec3 p1 = vertices[ivalues[1]];
            vec3 p2 = vertices[ivalues[2]];

            Triangle tri;
            tri.p0 = TransformPoint(p0, transStack.top());
            tri.p1 = TransformPoint(p1, transStack.top());
            tri.p2 = TransformPoint(p2, transStack.top());

            tri.mid = static_cast<int>(scene->materials.size());
            scene->materials.push_back(material);
            
            triangles.push_back(tri);
        }
        else if (cmd == "sphere" && readValues(s, 4, fvalues))
        {
            vec3 v = { fvalues[0], fvalues[1], fvalues[2] };
            float r = fvalues[3];

            //mat4 replacement{1.0f}; // unit sphere
            //replacement = translate(replacement, v);
            //replacement = scale(replacement, {r, r, r});

            Sphere sphere;
            sphere.p = v;
            sphere.r = r;

            sphere.mid = static_cast<int>(scene->materials.size());
            scene->materials.push_back(material);
            
            sphere.tid = static_cast<int>(scene->transforms.size());
            scene->transforms.push_back(transStack.top());
            
            spheres.push_back(sphere);
        }
        else if (cmd == "ambient" && readValues(s, 3, fvalues))
        {
            material.Ka = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "diffuse" && readValues(s, 3, fvalues))
        {
            material.Kd = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "specular" && readValues(s, 3, fvalues))
        {
            material.Ks = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "emission" && readValues(s, 3, fvalues))
        {
            material.Ke = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "shininess" && readValues(s, 1, fvalues))
        {
            material.ex = fvalues[0];
        }
        else if (cmd == "attenuation" && readValues(s, 3, fvalues))
        {
            c0 = fvalues[0];
            c1 = fvalues[1];
            c2 = fvalues[2];
        }
        else if (cmd == "directional" && readValues(s, 6, fvalues))
        {
            vec3 dir = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 clo = { fvalues[3], fvalues[4], fvalues[5] };

            DirectionalLight light;
            light.dir = -dir; // direction to light source
            light.color = clo;

            scene->dlights.push_back(light);
        }
        else if (cmd == "point" && readValues(s, 6, fvalues))
        {
            vec3 pos = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 clo = { fvalues[3], fvalues[4], fvalues[5] };

            PointLight light;
            light.pos = pos;
            light.color = clo;
            light.c0 = c0;
            light.c1 = c1;
            light.c2 = c2;

            scene->plights.push_back(light);
        }
        // TODO: use the examples above to handle other commands
    }

    in.close();
}