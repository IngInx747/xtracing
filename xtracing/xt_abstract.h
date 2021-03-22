#pragma once
#ifndef XT_ABSTRACT_H
#define XT_ABSTRACT_H

#include "xt_aabb.h"
#include "xt_ray.h"

////////////////////////////////////////////////////////////////
/// Interfaces
////////////////////////////////////////////////////////////////

class IPrimitive
{
public:
    // placeholding function to be make class polymorphic
    virtual int Type() { return 0; }
};


class IAttribute
{
public:
    // placeholding function to be make class polymorphic
    virtual int Type() { return 0; }
};


class IPayload
{
public:
    // placeholding function to be make class polymorphic
    virtual int Type() { return 0; }
};


class IBound
{
public:
    virtual Aabb operator() (const IPrimitive& primitive) const = 0;
};


class ICollide
{
public:
    virtual bool operator() (const IPrimitive& primitive, Ray& ray) const = 0;

protected:
    // built-in functions

    // transform matrix from object space to world space, accumulated along the scene nodes
    const mat4& GetWorldTransform() const
    { return world_transform; }

    // whether current collider to do the test only, W/O recording intersecting information
    bool IsShadowRay() const
    { return is_shadow_ray; }

    // which material to use(rtReportIntersection(int))
    void SetMaterialIndex(int id) const
    { material_id = id; }

    // store intersecting information
    template <class T_A>
    void SetAttribute(const T_A& attrib) const
    {
        static_assert(std::is_convertible<T_A*, IAttribute*>::value, "T_A must inherit IAttribute as public");
        attribute = std::dynamic_pointer_cast<IAttribute>(std::make_shared<T_A>(attrib));
    }

private:
    //
    void SetWorldTransform(const mat4 transform)
    { world_transform = transform; }

    //
    void SetShadowRay(bool is_shadow_ray_)
    { is_shadow_ray = is_shadow_ray_; }

    //
    int GetMaterialIndex() const
    { return material_id; }

    //
    std::shared_ptr<IAttribute> GetAttribute() const
    { return attribute; }

    // endpoint of intersecting test
    friend class PrimitiveNode;

private:
    // built-in attributes

    // intersecting information
    mutable std::shared_ptr<IAttribute> attribute;

    // which shading program to use
    mutable int material_id = 0;

    // transform accumulated from world to current node
    mat4 world_transform;

    // record attribute or not
    bool is_shadow_ray = false;
};


class IShader
{
public:
    virtual void operator() (IPayload& payload, const IAttribute& attrib) const = 0;

protected:
    // built-in functions

    //
    void TerminateRay() const
    { mIsRayTerminated = true; }

    //
    void IgnoreIntersection() const
    { mIsIntersectionIgnored = true; }

private:
    //
    bool IsRayTerminated() const
    { return mIsRayTerminated; }

    //
    bool IsIntersectionIgnored() const
    { return mIsIntersectionIgnored; }

    // endpoint of intersecting test
    friend class PrimitiveNode;

private:
    // built-in attributes

    //
    mutable bool mIsRayTerminated = false;

    //
    mutable bool mIsIntersectionIgnored = false;
};

#endif