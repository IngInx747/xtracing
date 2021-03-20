////////////////////////////////////////////////////////////////
/// Put: typedef, using, #define, constexpr, etc. here
////////////////////////////////////////////////////////////////

#pragma once
#ifndef X_UTILITY_H
#define X_UTILITY_H

#include <iterator> // std::forward_iterator_tag
#include <cstddef>  // std::ptrdiff_t
#include <cassert>
#include <cstring>
#include <numeric> // std::itoa
#include <unordered_map>

#include "xgeometry.h"

////////////////////////////////////////////////////////////////
/// Forward declaration
////////////////////////////////////////////////////////////////

class Context;
class SceneNode;
class PrimitiveNode;

////////////////////////////////////////////////////////////////
/// Buffer
////////////////////////////////////////////////////////////////

template <class T_Base>
class Buffer
{
protected:
    typedef unsigned char Byte;
    typedef T_Base Base;
    
public:
    struct out_of_range {};
    class iterator;
    
public:
    Buffer()
    {}

    Buffer(size_t stride): mStride(stride)
    {}

    Buffer(size_t stride, size_t size): mStride(stride), mSize(size), mCapacity(size)
    {
        mData = new Byte[stride * size];
    }

    ~Buffer()
    {
        delete[] mData;
        mData = NULL;
    }

    inline size_t capacity() const { return mCapacity; }

    inline size_t size() const { return mSize; }

    inline size_t stride() const { return mStride; }

    inline bool empty() const { return mSize == 0; }

    inline void set_stride(size_t stride) { if (mStride == 1) mStride = stride; }

    template<class T_Derived>
    inline void set_stride() { if (mStride == 1) mStride = sizeof(T_Derived); }

    // Requests a change in capacity
    // reserve() will never decrase the capacity.
    void reserve(size_t newcapacity)
    {
        if (newcapacity <= mCapacity) return;

        Byte* p = new Byte[mStride * newcapacity];

        size_t n = mStride * mSize;

        for (size_t i = 0; i < n; ++i)
            p[i] = mData[i];

        delete[] mData;

        mData = p;
        mCapacity = newcapacity;
    }

    // Changes the Buffer's size.
    // If the newsize is smaller, the last elements will be lost.
    void resize(size_t newsize)
    {
        reserve(newsize);

        size_t m = mStride * mSize;
        size_t n = mStride * newsize;

        for (size_t i = m; i < n; ++i)
            mData[i] = Byte{};

        mSize = newsize;
    }

    // Reduces capcity to fit the size
    void shrink()
    {
        // TODO
    }

    template<class T_Derived>
    void push_back(const T_Derived& elem)
    {
        assert(sizeof(T_Derived) == mStride);

        if (mCapacity == 0)
            reserve(8);
        else if (mSize == mCapacity)
            reserve(2 * mCapacity);

        // Virtual function table along with data needs to be copied
        // Some method like operator= or std::copy won't do the trick
        memcpy(&mData[mStride * mSize], &elem, sizeof(elem));
        
        ++mSize;
    }

    void pop_back()
    {
        --mSize;
    }

    inline T_Base& operator[](size_t n)
    {
        if (n < 0 || mSize <= n) throw out_of_range{};

        T_Base* p = reinterpret_cast<T_Base*>(&mData[mStride * n]);
        return *p;
    }

    inline const T_Base& operator[](size_t n) const
    {
        if (n < 0 || mSize <= n) throw out_of_range{};

        T_Base* p = reinterpret_cast<T_Base*>(&mData[mStride * n]);
        return *p;
    }

    inline T_Base& at(size_t n)
    {
        if (n < 0 || mSize <= n) throw out_of_range{};

        T_Base* p = reinterpret_cast<T_Base*>(&mData[mStride * n]);
        return *p;
    }

    inline const T_Base& at(size_t n) const
    {
        if (n < 0 || mSize <= n) throw out_of_range{};

        T_Base* p = reinterpret_cast<T_Base*>(&mData[mStride * n]);
        return *p;
    }

    template<class T_Derived>
    inline T_Derived& at(size_t n)
    {
        //assert(sizeof(T_Derived) == mStride);

        if (n < 0 || mSize <= n) throw out_of_range{};

        T_Derived* p = reinterpret_cast<T_Derived*>(&mData[mStride * n]);
        return *p;
    }

    template<class T_Derived>
    inline const T_Derived& at(size_t n) const
    {
        //assert(sizeof(T_Derived) == mStride);

        if (n < 0 || mSize <= n) throw out_of_range{};
        
        T_Derived* p = reinterpret_cast<T_Derived*>(&mData[mStride * n]);
        return *p;
    }

    // Removes all elements from the Vector
    // Capacity is not changed.
    void clear()
    {
        mSize = 0;
    }
    
    inline iterator begin()
    {
        return iterator(&mData[0], mStride);
    }
    
    inline const iterator begin() const
    {
        return iterator(&mData[0], mStride);
    }

    inline iterator end()
    {
        return iterator(&mData[mSize * mStride], mStride);
    }

    inline iterator end() const
    {
        return iterator(&mData[mSize * mStride], mStride);
    }
    
protected:
    // meta data
    Byte* mData = NULL;

    // buffer current size
    size_t mSize = 0;

    // buffer capacity
    size_t mCapacity = 0;

    // size of struct(usually a derived class)
    size_t mStride = 1;
};


template<class T_Base>
class Buffer<T_Base>::iterator
{
public:
    iterator()
    {}

    iterator(Byte* p, size_t stride): mCurr(p), mStride(stride)
    {}

    inline iterator& operator++()
    {
        mCurr += mStride;
        return *this;
    }

    inline iterator& operator++(int)
    {
        mCurr += mStride;
        return *this;
    }

    inline iterator& operator--()
    {
        mCurr -= mStride;
        return *this;
    }

    inline iterator& operator--(int)
    {
        mCurr -= mStride;
        return *this;
    }

    inline iterator& operator+=(int n)
    {
        mCurr += mStride * n;
        return *this;
    }

    inline iterator& operator-=(int n)
    {
        mCurr -= mStride * n;
        return *this;
    }

    inline T_Base& operator*()
    {
        return *reinterpret_cast<T_Base*>(mCurr);
    }

    inline T_Base* operator->()
    {
        return reinterpret_cast<T_Base*>(mCurr);
    }

    inline explicit operator T_Base* ()
    {
        return reinterpret_cast<T_Base*>(mCurr);
    }
    
    inline bool operator==(const iterator& iter) const
    {
        return mCurr == iter.mCurr;
    }

    inline bool operator!=(const iterator& iter) const
    {
        return mCurr != iter.mCurr;
    }

protected:
    // meta data
    Byte* mCurr = NULL;

    // size of struct(usually a derived class)
    size_t mStride;

public:
    template<class T_Base>
    friend inline std::ptrdiff_t distance(typename Buffer<T_Base>::iterator first, typename Buffer<T_Base>::iterator last);

    template<class T_Base>
    friend inline void advance(typename Buffer<T_Base>::iterator& iter, size_t n);
};


template<class T_Base>
inline std::ptrdiff_t distance(typename Buffer<T_Base>::iterator first, typename Buffer<T_Base>::iterator last)
{
    return (last.mCurr - first.mCurr) / static_cast<std::ptrdiff_t>(first.mStride);
}

template<class T_Base>
inline void advance(typename Buffer<T_Base>::iterator& iter, size_t n)
{
    iter += n;
}

////////////////////////////////////////////////////////////////
/// Prototype Instantializer
////////////////////////////////////////////////////////////////

template<class T_Base>
class IPrototype
{
public:
    // return a pointer of base class which is a copy of the prototype, null if prototype is not set
    virtual std::shared_ptr<T_Base> Instantialize() const
    { return nullptr; }

    // set a object of derived class as prototype
    void SetPrototype(const std::shared_ptr<T_Base>& prototype)
    { mPrototype = prototype; }

    explicit operator bool() const
    { return mPrototype.operator bool(); }

protected:
    // prototype of object(program)
    std::shared_ptr<T_Base> mPrototype;
};


template<class T_Base, class T_Derived>
class Prototype : public IPrototype<T_Base>
{
    static_assert(std::is_convertible<T_Derived*, T_Base*>::value, "T_Derived must inherit T_Base as public");

public:
    std::shared_ptr<T_Base> Instantialize() const
    {
        if (!mPrototype) return nullptr;
        return std::dynamic_pointer_cast<T_Base>(
            std::make_shared<T_Derived>(
                *std::dynamic_pointer_cast<T_Derived>(mPrototype)));
    }
};

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

////////////////////////////////////////////////////////////////
/// Program collection
////////////////////////////////////////////////////////////////

class GeometryProgram
{
public:
    template<class T_Bound>
    void SetBoundProgram(const std::shared_ptr<IBound>& prog)
    {
        if (!mBound) mBound = std::make_shared<Prototype<IBound, T_Bound>>();
        mBound->SetPrototype(prog);
    }

    template<class T_Collide>
    void SetCollideProgram(const std::shared_ptr<ICollide>& prog)
    {
        if (!mCollide) mCollide = std::make_shared<Prototype<ICollide, T_Collide>>();
        mCollide->SetPrototype(prog);
    }

    // Generate a copy of bounding box program prototype
    std::shared_ptr<IBound> GenBoundProgram() const
    {
        return mBound->Instantialize();
    }

    // Generate a copy of collide program prototype
    std::shared_ptr<ICollide> GenCollideProgram() const
    {
        return mCollide->Instantialize();
    }

protected:
    // bounding box program prototype
    std::shared_ptr<IPrototype<IBound>> mBound;

    // collide program prototype
    std::shared_ptr<IPrototype<ICollide>> mCollide;
};


class MaterialProgram
{
public:
    template<class T_Shader>
    void SetClosestHitProgram(int ray_type, std::shared_ptr<IShader> prog)
    {
        auto iter = mCHShaders.find(ray_type);

        if (iter != mCHShaders.end())
        {
            iter->second->SetPrototype(prog);
        }
        else
        {
            auto proto = std::make_shared<Prototype<IShader, T_Shader>>();
            proto->SetPrototype(prog);
            mCHShaders[ray_type] = proto;
        }
    }

    template<class T_Shader>
    void SetAnyHitProgram(int ray_type, std::shared_ptr<IShader> prog)
    {
        auto iter = mAHShaders.find(ray_type);

        if (iter != mAHShaders.end())
        {
            iter->second->SetPrototype(prog);
        }
        else
        {
            auto proto = std::make_shared<Prototype<IShader, T_Shader>>();
            proto->SetPrototype(prog);
            mAHShaders[ray_type] = proto;
        }
    }

    std::shared_ptr<IShader> GenClosestHitProgram(int ray_type) const
    {
        auto iter = mCHShaders.find(ray_type);
        if (iter == mCHShaders.end()) return nullptr;
        return iter->second->Instantialize();
    }

    std::shared_ptr<IShader> GenAnyHitProgram(int ray_type) const
    {
        auto iter = mAHShaders.find(ray_type);
        if (iter == mAHShaders.end()) return nullptr;
        return iter->second->Instantialize();
    }

protected:
    // closest hit program prototype
    std::unordered_map<int, std::shared_ptr<IPrototype<IShader>>> mCHShaders;

    // any hit program prototype
    std::unordered_map<int, std::shared_ptr<IPrototype<IShader>>> mAHShaders;
};


////////////////////////////////////////////////////////////////
/// Wrapper of Index-Based Primitive Accelerating Structure
////////////////////////////////////////////////////////////////

struct PrimitiveIndexBound
{
    inline Aabb operator() (const int& index) const
    {
        return bound.operator()(buffer.at(index));
    }

    PrimitiveIndexBound(const Buffer<IPrimitive>& buffer, const IBound& bound)
        : buffer(buffer), bound(bound)
    {}

    const Buffer<IPrimitive>& buffer;
    const IBound& bound;
};


struct PrimitiveIndexCollide
{
    inline bool operator() (const int& index, Ray& ray) const
    {
        return collide(buffer.at(index), ray);
    }
    
    PrimitiveIndexCollide(const Buffer<IPrimitive>& buffer, const ICollide& collide)
        : buffer(buffer), collide(collide)
    {}

    const Buffer<IPrimitive>& buffer;
    const ICollide& collide;
};


class IPrimitiveAccel
{
public:
    virtual Aabb GetBoundingBox() const = 0;

    virtual bool Intersect(
        const Buffer<IPrimitive>& buffer,
        const ICollide& collide,
        Ray& ray,
        bool is_shadow_ray = false) const = 0;

    virtual void Build(
        const Buffer<IPrimitive>& buffer,
        const IBound& bound) = 0;
};


class PrimitiveAccelNone : public IPrimitiveAccel
{
public:
    Aabb GetBoundingBox() const { return mBox; }

    bool Intersect(
        const Buffer<IPrimitive>& buffer,
        const ICollide& collide,
        Ray& ray,
        bool is_shadow_ray = false) const
    {
        bool hit = false;

        if (!IsIntersectingAabb(mBox, ray.org, ray.dir, ray.dist))
            return false;

        for (const IPrimitive& p : buffer)
        {
            if (collide(p, ray))
            {
                hit = true;
                if (is_shadow_ray) break;
            }
        }

        return hit;
    }
    
    void Build(
        const Buffer<IPrimitive>& buffer,
        const IBound& bound)
    {
        Aabb bbox = Bound();

        for (const IPrimitive& p : buffer)
            bbox = Union(bbox, bound(p));

        mBox = bbox;
    }

protected:
    Aabb mBox;
};


class PrimitiveAccelBvh : public IPrimitiveAccel
{
public:
    Aabb GetBoundingBox() const
    {
        return mBvh.GetBoundingBox();
    }

    bool Intersect(
        const Buffer<IPrimitive>& buffer,
        const ICollide& collide,
        Ray& ray,
        bool is_shadow_ray = false) const
    {
        PrimitiveIndexCollide indexCollide(buffer, collide);
        return mBvh.Intersect(indexCollide, ray, is_shadow_ray);
    }

    void Build(
        const Buffer<IPrimitive>& buffer,
        const IBound& bound)
    {
        std::vector<int> indices(buffer.size());
        std::iota(std::begin(indices), std::end(indices), 0); // fill with 0 ~ n-1
        PrimitiveIndexBound indexBound(buffer, bound);
        EqualCountsSplit<int, decltype(indexBound)> split(indexBound);
        mBvh.SetNumObjPerNode(1);
        mBvh.Build<PrimitiveIndexBound, decltype(split)>(indices, indexBound, split);
    }

protected:
    // accelerating structure storing indices of primitives
    Bvh<int> mBvh;
};

////////////////////////////////////////////////////////////////
/// Interface of Scene Node
////////////////////////////////////////////////////////////////

struct IntersectInfo
{
    void UpdateClosestHitShader(float dist, std::shared_ptr<IShader> shd, std::shared_ptr<IAttribute> att)
    {
        // atomic operation
        if (min_distance > dist)
        {
            min_distance = dist;
            chs = shd;
            attribute = att;
        }
    }

    void PerformAnyHitShader(std::shared_ptr<IShader> shd, std::shared_ptr<IAttribute> att)
    {
        shd->operator()(*payload, *att);
    }

    bool IsShadowRay()
    { return is_shadow_ray; }

    void SetShadowRay(bool shadow_ray)
    { is_shadow_ray = shadow_ray; }

    int GetRayType()
    { return ray_type; }

    void SetRayType(int type)
    { ray_type = type; }

    void SetPayload(IPayload* ptr)
    { payload = ptr; }

    std::shared_ptr<IShader> GetClosestHitProgram()
    { return chs; }

    std::shared_ptr<IAttribute> GetAttribute()
    { return attribute; }

protected:
    // global receivers
    std::shared_ptr<IAttribute> attribute; // info from collide for shader
    std::shared_ptr<IShader> chs; // closest-hit shader program
    float min_distance = kInfP;

    // for any-hit
    IPayload* payload;

    // global variables
    bool is_shadow_ray;
    int ray_type;
};


struct IntersectArgs
{
    IntersectInfo* GetInfo() const
    { return recv; }

    // controller
    IntersectInfo* recv;

    // accumulated transform matrix
    mat4 transform;
};


class SceneNode
{
public:
    virtual Aabb GetBoundingBox() const = 0;

    virtual bool Intersect(Ray& ray, const IntersectArgs& args) const = 0;

    virtual void UpdateSingleNode() {}

protected:
};

////////////////////////////////////////////////////////////////
/// Wrapper of Scene Node Accelerating Structure
////////////////////////////////////////////////////////////////

struct SceneNodeBound
{
    inline Aabb operator() (const std::shared_ptr<SceneNode>& node) const
    {
        return node->GetBoundingBox();
    }
};


struct SceneNodeCollide
{
    inline bool operator() (const std::shared_ptr<SceneNode>& node, Ray& ray) const
    {
        return node->Intersect(ray, args);
    }

    SceneNodeCollide(const IntersectArgs& args) : args(args)
    {}

    const IntersectArgs& args;
};


class ISceneNodeAccel
{
public:
    virtual Aabb GetBoundingBox() const = 0;

    virtual bool Intersect(
        const std::vector<std::shared_ptr<SceneNode>>& nodes,
        Ray& ray,
        const IntersectArgs& args) const = 0;

    virtual void Build(
        const std::vector<std::shared_ptr<SceneNode>>& nodes) = 0;
};


class SceneNodeAccelNone : public ISceneNodeAccel
{
public:
    Aabb GetBoundingBox() const
    {
        return mBox;
    }

    bool Intersect(
        const std::vector<std::shared_ptr<SceneNode>>& nodes,
        Ray& ray,
        const IntersectArgs& args) const
    {
        bool hit = false;

        if (!IsIntersectingAabb(mBox, ray.org, ray.dir, ray.dist))
            return false;

        for (const std::shared_ptr<SceneNode>& node : nodes)
        {
            if (node->Intersect(ray, args))
            {
                hit = true;
                if (args.GetInfo()->IsShadowRay()) break;
            }
        }

        return hit;
    }
    
    void Build(const std::vector<std::shared_ptr<SceneNode>>& nodes)
    {
        Aabb bbox = Bound();

        for (const std::shared_ptr<SceneNode>& node : nodes)
            bbox = Union(bbox, node->GetBoundingBox());

        mBox = bbox;
    }

protected:
    Aabb mBox;
};


class SceneNodeAccelBvh : public ISceneNodeAccel
{
public:
    Aabb GetBoundingBox() const
    {
        return mBvh.GetBoundingBox();
    }

    bool Intersect(
        const std::vector<std::shared_ptr<SceneNode>>& nodes,
        Ray& ray,
        const IntersectArgs& args) const
    {
        SceneNodeCollide collide(args);
        return mBvh.Intersect(collide, ray, args.GetInfo()->IsShadowRay());
    }

    void Build(const std::vector<std::shared_ptr<SceneNode>>& nodes)
    {
        SceneNodeBound bound;
        EqualCountsSplit<std::shared_ptr<SceneNode>, decltype(bound)> split(bound);
        mBvh.SetNumObjPerNode(1);
        mBvh.Build<SceneNodeBound, decltype(split)>(nodes, bound, split);
    }
    
protected:
    // accelerating structure storing nodes
    Bvh<std::shared_ptr<SceneNode>> mBvh;
};

////////////////////////////////////////////////////////////////
/// Scene Node
////////////////////////////////////////////////////////////////

class LeafNode;
class RootNode;


class LeafNode : public SceneNode
{
public:

    // Keep marking dirty from current node to root node
    // Better call after single or small number of changes
    // Call current node to apply: node->NotifyToRoot()
    void NotifyToRoot();

protected:
    // not for user-end usage
    void AppendParent(RootNode* parent);

    friend class RootNode;

protected:
    // parent scene node
    std::vector<SceneNode*> mParents;
};


class RootNode : public SceneNode
{
public:
    virtual Aabb GetBoundingBox() const
    {
        return mAccel->GetBoundingBox();
    }

    virtual bool Intersect(Ray& ray, const IntersectArgs& args) const
    {
        return mAccel->Intersect(mChildren, ray, args);
    }

    virtual void UpdateSingleNode()
    {
        if (IsDirty()) BuildAccel();
        MarkClean();
    }

    void AppendChild(const std::shared_ptr<LeafNode>& child);

    // UpdateClosestHitShader the nodes of the whole tree from the leaf nodes to root node
    // Better call after large number of changes
    // Call root node to apply: node->UpdateFromLeaves()
    void UpdateFromLeaves();

    void SetAccel(int type)
    {
        if (type == AccelEnum::BVH)
        {
            mAccel = std::make_shared<SceneNodeAccelBvh>();
        }
        else
        {
            mAccel = std::make_shared<SceneNodeAccelNone>();
        }
    }

    void BuildAccel()
    {
        if (!mAccel)
        {
            if (mChildren.size() >= 50) // !?
            {
                mAccel = std::make_shared<SceneNodeAccelBvh>();
            }
            else
            {
                mAccel = std::make_shared<SceneNodeAccelNone>();
            }
        }

        mAccel->Build(mChildren);
    }

    bool IsDirty() const
    { return mIsDirty; }

    void MarkDirty()
    { mIsDirty = true; }
    
    void MarkClean()
    { mIsDirty = false; }

protected:
    // children scene node
    std::vector<std::shared_ptr<SceneNode>> mChildren;

    // node accelerating structure
    std::shared_ptr<ISceneNodeAccel> mAccel;

    //
    bool mIsDirty = true;
};


inline void LeafNode::AppendParent(RootNode* parent)
{
    mParents.push_back(parent);
    parent->MarkDirty(); // redundant op
}


inline void LeafNode::NotifyToRoot()
{
    for (SceneNode* parent : mParents)
    {
        if (!parent) continue;

        RootNode* rnode = dynamic_cast<RootNode*>(parent);

        if (rnode && !rnode->IsDirty())
        {
            rnode->MarkDirty();

            LeafNode* lnode = dynamic_cast<LeafNode*>(parent);

            if (lnode)
            {
                lnode->NotifyToRoot(); // internal node
            }
        }
    }
}


inline void RootNode::AppendChild(const std::shared_ptr<LeafNode>& child)
{
    mChildren.push_back(child);
    child->AppendParent(this);
    MarkDirty(); // primary op
}


inline void RootNode::UpdateFromLeaves()
{
    for (const std::shared_ptr<SceneNode>& child : mChildren)
    {
        if (!child) continue;

        RootNode* rnode = dynamic_cast<RootNode*>(child.get());

        if (rnode && rnode->IsDirty())
        {
            rnode->UpdateFromLeaves(); // internal node
        }
        else
        {
            child->UpdateSingleNode(); // leaf node
        }
    } // update children node recursively

    UpdateSingleNode(); // then update self
}


class InternalNode : public LeafNode, public RootNode
{
public:
    virtual Aabb GetBoundingBox() const
    {
        return RootNode::GetBoundingBox();
    }

    virtual bool Intersect(Ray& ray, const IntersectArgs& args) const
    {
        return RootNode::Intersect(ray, args);
    }

protected:
    // No data stored in super base node
    // No need for virtual inheritance
};

////////////////////////////////////////////////////////////////
/// Scene Node Implementation
////////////////////////////////////////////////////////////////

class TransformNode : public InternalNode
{
public:
    Aabb GetBoundingBox() const
    {
        return Object2WorldAabb(mAccel->GetBoundingBox(), mTransform);
    }

    bool Intersect(Ray& ray, const IntersectArgs& args) const
    {
        bool hit = false;

        vec3 org = World2ObjectHomoCoord(ray.org, mTransform);
        vec3 dir = World2ObjectNonHomoCoord(ray.dir, mTransform);
        Ray  new_ray{org, dir, ray.dist, ray.epsilon}; // apply transform to ray
        mat4 new_transform = args.transform * mTransform; // accumulate transform
        IntersectArgs new_args{args.GetInfo(), new_transform};

        hit = InternalNode::Intersect(new_ray, new_args);
        ray.dist = new_ray.dist; // atomic operation

        return hit;
    }

    const mat4& GetTransform() const
    {
        return mTransform;
    }

    void SetTransform(const mat4& transform)
    {
        mTransform = transform;
    }

protected:
    // transform matrix
    mat4 mTransform;
};


// Collection of primitives and corresponding geometry instance
// Exact usage is to hold a model of great number of triangles
// Leaf Node
class PrimitiveNode : public LeafNode
{
public:
    Aabb GetBoundingBox() const
    {
        return mAccel->GetBoundingBox();
    }

    void UpdateSingleNode()
    {
        if (IsDirty()) BuildAccel();
        MarkClean();
    }

    bool Intersect(Ray& ray, const IntersectArgs& args) const
    {
        bool hit = false;

        std::shared_ptr<ICollide> collide = mGeometry->GenCollideProgram();
        collide->SetWorldTransform(args.transform);
        collide->SetShadowRay(args.GetInfo()->IsShadowRay()); // update built-in variables
        float backupDist = ray.dist;

        hit = mAccel->Intersect(mBuffer, *collide, ray, args.GetInfo()->IsShadowRay());

        // if hit, perform any-hit shader program on each intersection
        if (hit)
        {
            std::shared_ptr<IShader> ahs = mMaterials[collide->GetMaterialIndex()]->GenAnyHitProgram(args.GetInfo()->GetRayType());

            if (ahs)
            {
                args.GetInfo()->PerformAnyHitShader(ahs, collide->GetAttribute());

                // wait for any-hit shader program return

                // handle intersection-ignored condition
                if (ahs->IsIntersectionIgnored())
                {
                    hit = false;
                    ray.dist = backupDist;
                }

                // handle ray-terminated condition
                if (ahs->IsRayTerminated())
                {
                    // single thread
                    args.GetInfo()->SetShadowRay(true);
                    // nothing else need to be done here

                    // [TODO] multithread
                }
            }
        }

        // if hit and any-hit shader does not ignore the hit, report closest-hit shader program to the receiver
        if (hit)
        {
            std::shared_ptr<IShader> chs = mMaterials[collide->GetMaterialIndex()]->GenClosestHitProgram(args.GetInfo()->GetRayType());

            if (chs) // record if current intersection is closer
            {
                args.GetInfo()->UpdateClosestHitShader(ray.dist, chs, collide->GetAttribute());
            }
        }

        return hit;
    }

    void SetGeometry(std::shared_ptr<GeometryProgram> ptr)
    {
        mGeometry = ptr;
    }

    std::shared_ptr<GeometryProgram> GetGeometry() const
    {
        return mGeometry;
    }

    void SetMaterial(int index, std::shared_ptr<MaterialProgram> ptr)
    {
        if (mMaterials.size() <= index)
            mMaterials.resize(index + 1);
        mMaterials[index] = ptr;
    }

    std::shared_ptr<MaterialProgram> GetMaterial(int index) const
    {
        return mMaterials[index];
    }

    template<class Primitive>
    void SetBuffer(const std::vector<Primitive>& primitives)
    {
        static_assert(std::is_convertible<Primitive*, IPrimitive*>::value, "Primitive must inherit IPrimitive as public");

        mBuffer.set_stride(sizeof(Primitive));
        mBuffer.reserve(primitives.size());

        // TODO: replace this part with fill() and back_insertor()
        for (const Primitive& p : primitives)
            mBuffer.push_back<Primitive>(p);

        MarkDirty();
    }

    void SetAccel(int type)
    {
        if (type == AccelEnum::BVH)
        {
            mAccel = std::make_shared<PrimitiveAccelBvh>();
        }
        else
        {
            mAccel = std::make_shared<PrimitiveAccelNone>();
        }
    }

    void BuildAccel()
    {
        if (!mAccel)
        {
            if (mBuffer.size() >= 50) // !?
            {
                mAccel = std::make_shared<PrimitiveAccelBvh>();
            }
            else
            {
                mAccel = std::make_shared<PrimitiveAccelNone>();
            }
        }

        std::shared_ptr<IBound> bound = mGeometry->GenBoundProgram();
        mAccel->Build(mBuffer, *bound);
    }
    
    bool IsDirty() const
    { return mIsDirty; }

    void MarkDirty()
    { mIsDirty = true; }
    
    void MarkClean()
    { mIsDirty = false; }

protected:
    // buffer of primitive structures
    Buffer<IPrimitive> mBuffer;

    // accelerating structure storing indices of primitives
    std::shared_ptr<IPrimitiveAccel> mAccel;

    //
    bool mIsDirty = true;

    // GeometryProgram methods factory
    std::shared_ptr<GeometryProgram> mGeometry;

    // MaterialProgram methods factory
    std::vector<std::shared_ptr<MaterialProgram>> mMaterials;
};

////////////////////////////////////////////////////////////////
/// Context
////////////////////////////////////////////////////////////////

class IRayGenerator
{};


class Context
{
public:

protected:
    //
    std::shared_ptr<IPrototype<IShader>> mMiss;
};

////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////

inline bool Trace(SceneNode* node, Ray& ray, IPayload& payload, int ray_type, bool shadow_ray)
{
    IntersectInfo recv;
    IntersectArgs args{&recv, mat4{1.0f}};

    recv.SetRayType(ray_type);
    recv.SetPayload(&payload);
    recv.SetShadowRay(shadow_ray);

    bool hit = node->Intersect(ray, args);

    if (hit)
    {
        auto chs = recv.GetClosestHitProgram();
        chs->operator()(payload, *recv.GetAttribute());
    }

    return hit;
}

#endif // !X_UTILITY_H