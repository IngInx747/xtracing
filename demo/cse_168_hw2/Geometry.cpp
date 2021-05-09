#include "Geometry.h"

#include "Scene.h"


Aabb TriangleBound::operator() (const IPrimitive& primitive) const
{
    const Triangle& tri = dynamic_cast<const Triangle&>(primitive);
    return Bound(tri.p0, tri.p1, tri.p2);
}


Aabb SphereBound::operator() (const IPrimitive& primitive) const
{
    const Sphere& sphere = dynamic_cast<const Sphere&>(primitive);
    const mat4& transform = scene->transforms[sphere.tid];
    vec3 p = sphere.p;
    vec3 r{sphere.r, sphere.r, sphere.r};
    return TransformBoundingBox(Bound(p - r, p + r), transform); // obj -> world
}


bool TriangleCollide::operator() (const IPrimitive& primitive, Ray& ray) const
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


bool SphereCollide::operator() (const IPrimitive& primitive, Ray& ray) const
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
