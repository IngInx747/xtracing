#pragma once
#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include "Scene.h"

class SceneLoader
{
public:
    void Load(const std::string& filename, Scene& scene);

private:
    void load(const std::string& filename, Scene& scene);

private:
};

#endif