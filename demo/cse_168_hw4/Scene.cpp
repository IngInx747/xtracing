#include "Scene.h"


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
    std::shared_ptr<MaterialProgram> materialProgram = std::make_shared<MaterialProgram>();

    std::shared_ptr<ShadowShader> shadowAnyHit = std::make_shared<ShadowShader>();
    shadowAnyHit->scene = this;
    materialProgram->SetAnyHitProgram<ShadowShader>(1, shadowAnyHit); // 1: shadow ray

    if (bUseMIS)
    {
        std::shared_ptr<MISTracer> chs0 = std::make_shared<MISTracer>();
        chs0->scene = this;
        materialProgram->SetClosestHitProgram<MISTracer>(0, chs0); // 0: common ray
        
        std::shared_ptr<OneStepShader> chs2 = std::make_shared<OneStepShader>();
        chs2->scene = this;
        materialProgram->SetClosestHitProgram<OneStepShader>(2, chs2); // 2: sample ray
    }
    else
    {
        std::shared_ptr<PathTracer> rayClosestHit = std::make_shared<PathTracer>();
        rayClosestHit->scene = this;
        materialProgram->SetClosestHitProgram<PathTracer>(0, rayClosestHit); // 0: common ray
    }

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
