#include <xtracing.h>

#include "Scene.h"
#include "Renderer.h"
#include "util.h"

int main(int argc, char** argv)
{
    try
    {
        // Check for scene file 
        if (argc == 1) throw
            std::runtime_error("Missing path to a scene file as the first argument.");

        // Load the scene file
        SceneLoader loader;
        std::shared_ptr<Scene> scene = loader.load(argv[1]);
        Renderer renderer(scene);

        double dt = When();
        renderer.run(false);
        printf("Elapsed time = %lf(s)\n", When() - dt);

        SaveImagePNG(renderer.getResultBuffer(), renderer.getWidth(), renderer.getHeight(), renderer.getOutputFilename().c_str());
    }
    catch (const std::exception & ex)
    {
        // Print out the exception for debugging
        std::cerr << ex.what() << std::endl;
    }

    return 0;
}