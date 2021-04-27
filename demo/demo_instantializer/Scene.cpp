#include "Scene.h"

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"


struct TriangleBound : public IBound
{
    Aabb operator() (const IPrimitive& primitive) const
    {
        const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
        return Bound(tri.p0, tri.p1, tri.p2);
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
            vec3 h = TransformPoint(org + dir * ray.dist, GetWorldTransform());
            vec3 n = cross(v1 - v0, v0 - v2);
            if (dot(n, dir) > 0) n = -n;
            n = normalize(TransformNormal(n, GetWorldTransform()));
            dir = normalize(TransformVector(dir, GetWorldTransform()));

            Attribute attribute;
            attribute.hit = h;
            attribute.normal = n;
            attribute.incident = dir;
            SetAttribute<Attribute>(attribute);
        }
        return ret;
    }
};


struct TrianglePhong : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        Payload& payload = dynamic_cast<Payload&>(payload_);
        const Attribute& attrib = dynamic_cast<const Attribute&>(attrib_);

        payload.radiance += (attrib.normal + 1.f) * 0.5f;
    }

    Scene* scene;
};


struct Miss : public IShader
{
    void operator() (IPayload& payload_, const IAttribute& attrib_) const
    {
        Payload& payload = dynamic_cast<Payload&>(payload_);
        payload.radiance = background;
    }

    vec3 background;
};


static bool LoadMesh_Implm_TinyObj(const std::string& filename, std::vector<Triangle>& triangles)
{
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./"; // Path to material files

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filename, reader_config))
    {
        if (!reader.Error().empty())
        {
            std::cerr << "[TinyObjReader] " << reader.Error();
        }
        return false;
    }

    if (!reader.Warning().empty())
    {
        std::cout << "[TinyObjReader] " << reader.Warning();
    }
    
    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++)
    {
        // Loop over faces(polygon)
        size_t index_offset = 0;

        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            int fv = shapes[s].mesh.num_face_vertices[f];
            vec3 vertices[3];

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++)
            {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
                tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
                tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
                tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
                tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
                // Optional: vertex colors
                // tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
                // tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
                // tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

                vertices[v] = vec3{vx, vy, vz};
            }
            index_offset += fv;

            // per-face material
            //shapes[s].mesh.material_ids[f];

            Triangle tri{};
            tri.p0 = vertices[0];
            tri.p1 = vertices[1];
            tri.p2 = vertices[2];
            triangles.push_back(tri);
        }
    }

    return true;
}


static void NormalizeMesh(std::vector<Triangle>& triangles)
{
    Aabb bbox = Bound();

    for (const Triangle& tri : triangles)
    {
        Aabb b = Bound(tri.p0, tri.p1, tri.p2);
        bbox = Union(bbox, b);
    }

    vec3 diag = GetDiagonal(bbox);

    for (Triangle& tri : triangles)
    {
        tri.p0 = (tri.p0 - bbox.pMin) / diag * 2.0f - 1.0f;
        tri.p1 = (tri.p1 - bbox.pMin) / diag * 2.0f - 1.0f;
        tri.p2 = (tri.p2 - bbox.pMin) / diag * 2.0f - 1.0f;
    }
}


bool LoadScene(Scene* scene, const std::string& filename)
{
    // declaration
    int width = 600;
    int height = 600;
    //std::string outputName = filename + ".png";
    std::string outputName = "output.png";
    std::vector<Triangle> triangles;

    // setup geometry programs (triangle)
    std::shared_ptr<TriangleBound> triangleBound = std::make_shared<TriangleBound>();
    std::shared_ptr<TriangleCollide> triangleCollide = std::make_shared<TriangleCollide>();
    std::shared_ptr<GeometryProgram> triangleGeometry = std::make_shared<GeometryProgram>();
    triangleGeometry->SetBoundProgram<TriangleBound>(triangleBound);
    triangleGeometry->SetCollideProgram<TriangleCollide>(triangleCollide);

    // setup material programs
    std::shared_ptr<TrianglePhong> rayClosestHit = std::make_shared<TrianglePhong>();
    rayClosestHit->scene = scene;
    std::shared_ptr<MaterialProgram> materialProgram = std::make_shared<MaterialProgram>();
    materialProgram->SetClosestHitProgram<TrianglePhong>(0, rayClosestHit); // 0: common ray

    // setup context programs
    std::shared_ptr<Miss> miss = std::make_shared<Miss>();
    miss->background = vec3{0, 0, 0};
    scene->miss = miss;

    // mesh 0
    triangles.clear();
    if (!LoadMesh_Implm_TinyObj(filename, triangles))
        return false;
    NormalizeMesh(triangles);

    // setup nodes

    SetGlobalBvhBuildOptionSplitMethod(BvhSplitMethodEnum::EQUAL_COUNT);

    // primitive node 0 (triangle)
    std::shared_ptr<PrimitiveNode> p0 = std::make_shared<PrimitiveNode>();
    p0->SetGeometry(triangleGeometry);
    p0->SetMaterial(0, materialProgram); // only one material for triangle
    p0->SetBuffer<Triangle>(triangles);
    p0->SetAccel(AccelEnum::BVH);

    mat4 tm{ 1.0f };

    std::shared_ptr<TransformNode> t0 = std::make_shared<TransformNode>();
    tm = mat4{ 1.0f };
    tm = scale(tm, vec3{ 1, 0.7f, 1 });
    tm = rotate(tm, radians(45.0f), vec3{ 0, 1, 0 });
    t0->SetTransform(tm);
    t0->SetAccel(AccelEnum::NONE);

    SetGlobalBvhBuildOptionSplitMethod(BvhSplitMethodEnum::MIDDLE_POINT);

    std::shared_ptr<TransformNode> t1 = std::make_shared<TransformNode>();
    tm = mat4{ 1.0f };
    //tm = scale(tm, vec3{ 2, 1, 1 });
    tm = rotate(tm, radians(-45.0f), vec3{ 0, 1, 0 });
    t1->SetTransform(tm);
    t1->SetAccel(AccelEnum::BVH);

    std::shared_ptr<InternalNode> g0 = std::make_shared<InternalNode>();
    g0->SetAccel(AccelEnum::BVH);

    std::shared_ptr<RootNode> root = std::make_shared<RootNode>();
    root->SetAccel(AccelEnum::BVH);

    SetGlobalBvhBuildOptionSplitMethod(BvhSplitMethodEnum::EQUAL_COUNT);

    // assemble nodes
    t0->AppendChild(p0);

    int n = 2;
    for (int i = -n; i <= n; ++i)
    {
        for (int j = -n; j <= n; ++j)
        {
            std::shared_ptr<TransformNode> tn = std::make_shared<TransformNode>();
            mat4 m{ 1.0f };
            m = translate(m, vec3{ j, i, 0 } * 2.5f);
            tn->SetTransform(m);
            tn->SetAccel(AccelEnum::NONE);
            tn->AppendChild(t0);
            g0->AppendChild(tn);
        }
    }

    n = 2;
    for (int i = -n; i <= n; ++i)
    {
        std::shared_ptr<TransformNode> tn = std::make_shared<TransformNode>();
        mat4 m{ 1.0f };
        m = translate(m, vec3{ 0, 0, i } * 2.5f);
        tn->SetTransform(m);
        tn->SetAccel(AccelEnum::NONE);
        tn->AppendChild(g0);
        t1->AppendChild(tn);
    }

    root->AppendChild(t1);

    // update all nodes
    root->UpdateFromLeaves();

    // camera
    vec3 eye    = { 0, 10, 20 };
    vec3 lookat = { 0, 0, 0 };
    vec3 up     = { 0, 1, 0 };
    float fov = radians(60.0f);
    float aspect = (float)width / (float)height;
    scene->cameraFrame.o = eye;
    ComputeCameraFrame(
        eye, lookat, up, fov, aspect,
        scene->cameraFrame.u, scene->cameraFrame.v, scene->cameraFrame.w, true);

    // store all stuff
    scene->root = root;
    scene->width = width;
    scene->height = height;
    scene->outputFilename.clear();
    scene->outputFilename.append(outputName);

    return true;
}