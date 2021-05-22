#include <iostream>

#include "xtracing.h"
#include "Scene.h"
#include "SceneLoader.h"
#include "Renderer.h"
#include "util.h"

int main(int argc, char** argv)
{
    try
    {
        double dt;

        // Check for scene file 
        if (argc <= 1) throw std::runtime_error("Need 1 argument");

        Scene scene;
        SceneLoader loader;
        Renderer renderer;
        std::vector<vec3> buffer;
        
        // Load the scene
        dt = When();
        loader.Load(argv[1], scene);
        printf("Scene building time = %lf(s)\n", When() - dt);
        buffer.resize(scene.width * scene.height);

        // Render the scene
        dt = When();
        renderer.Render(buffer, scene);
        printf("Rendering time = %lf(s)\n", When() - dt);

        SaveImagePNG(buffer, scene.width, scene.height, scene.outputFilename.c_str());
    }
    catch (const std::exception & ex)
    {
        // Print out the exception for debugging
        std::cerr << ex.what() << std::endl;
    }

    return 0;
}