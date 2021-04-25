#pragma once
#ifndef RENDERER_H
#define RENDERER_H

#include "xtracing.h"
#include "Scene.h"

struct Payload
{
    // TODO: add more variable to payload if you need to
    vec3 radiance;
    vec3 mask;
    vec3 origin;
    vec3 direction;
    int depth;
    bool done;
};

struct ShadowPayload
{
    int isVisible;
};


class Renderer
{
public:
    Renderer(std::shared_ptr<Scene> scene);

    /**
     * Build the Graph according to scene.
     */
    void buildScene();

    /**
     * Run the renderer with the constructed graph. The resultBuffer should
     * contain the rendered image as the function terminates. If progressive
     * is true, it will only render one frame; else, it will render all frames.
     */
    void run(bool progressive = true);

    /**
     * Get the rendered result as a 1D array of bytes.
     */
    std::vector<unsigned char> getResult();

    // Some getters
    int getWidth() { return width; }
    int getHeight() { return height; }
    int getNumFrames() { return numFrames; }
    int getCurrentFrame() { return currentFrame; }
    std::string getOutputFilename() { return outputFilename; }
    std::vector<vec3>& getResultBuffer() { return resultBuffer; }

protected:
    /**
     * A helper function for readability. One should initialize all the Optix
     * programs here.
     */
    //void initPrograms();

    /**
     * A helper function to create a program from a .cu file.
     */
    //optix::Program createProgram(const std::string& filename, const std::string& programName);

    /**
     * A helper function to create a buffer and fill it with data.
     */
    //template <class T>
    //optix::Buffer createBuffer(std::vector<T> data);

    void generateRay(const int2& index, const int2& dim);

    void rtTrace(Ray& ray, Payload& payload);
    void rtTrace(Ray& ray, ShadowPayload& payload);

    void miss(Payload& payload);
    void anyHit(ShadowPayload& shadowPayload);
    void closestHit(Payload& payload, MyAttributes& attrib);

    //Context context; // a conext that encapsulates all resources
    //Buffer resultBuffer; // a buffer that stores the rendered image
    //std::unordered_map<std::string, optix::Program> programs; // a map that maps a name to a program
    std::vector<vec3> resultBuffer;
    std::shared_ptr<Scene> scene; // the scene to render
    int numFrames; // number of frames to render (>= 1)
    int currentFrame; // the current frame in [1, numFrames], 0 if rendering hasn't started

    // Some info about the output image
    std::string outputFilename;
    int width, height;

    // Accel
    Bvh<Triangle> mAccelTri;
    Bvh<Sphere> mAccelSphere;
};

#endif