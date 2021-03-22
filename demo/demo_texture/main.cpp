#include <xtracing.h>

#include "util.h"
#include "Scene.h"
#include "Renderer.h"

int main(int argc, char** argv)
{
    try
    {
        double dt;

        // Load the scene
        std::shared_ptr<Scene> scene = std::make_shared<Scene>();
        dt = When();
        LoadScene(scene.get());
        printf("Scene building time = %lf(s)\n", When() - dt);

        // pixel buffer
        std::vector<vec3> buffer(scene->width * scene->height);

        Renderer renderer;
        renderer.SetNumFrame(1);

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