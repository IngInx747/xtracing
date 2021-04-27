#include <xtracing.h>

#include "Scene.h"
#include "Renderer.h"
#include "util.h"

int main(int argc, char** argv)
{
    try
    {
        double dt;

        // Check for scene file 
        if (argc <= 1) throw std::runtime_error("Need 1 argument");

        std::shared_ptr<Scene> scene = std::make_shared<Scene>();
        SceneLoader loader;
        Renderer renderer;
        std::vector<vec3> buffer;
        
        // Load the scene
        dt = When();
        loader.Load(argv[1], scene.get());
        printf("Scene building time = %lf(s)\n", When() - dt);
        buffer.resize(scene->width * scene->height);

        // Render the scene
        dt = When();
        renderer.Render(buffer, scene.get());
        printf("Rendering time = %lf(s)\n", When() - dt);

        SaveImagePNG(buffer, scene->width, scene->height, scene->outputFilename.c_str());
    }
    catch (const std::exception & ex)
    {
        // Print out the exception for debugging
        std::cerr << ex.what() << std::endl;
    }

    return 0;
}