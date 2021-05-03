#include "SceneLoader.h"

void SceneLoader::Load(const std::string& filename, Scene& scene)
{
    // load models and light sources
    load(filename, scene);

    // build scene using imported data
    scene.Build();
}


template <class T>
bool readValues(std::stringstream& s, const int numvals, T* values)
{
    for (int i = 0; i < numvals; i++)
    {
        s >> values[i];
        if (s.fail())
        {
            std::cout << "Failed reading value " << i << " will skip" << std::endl;
            return false;
        }
    }
    return true;
}


void SceneLoader::load(const std::string& filename, Scene& scene)
{
    // Attempt to open the scene file 
    std::ifstream in(filename);
    if (!in.is_open())
    {
        // Unable to open the file. Check if the filename is correct.
        throw std::runtime_error("Unable to open scene file " + filename);
    }

    std::string str, cmd;

    std::stack<mat4> transStack;
    transStack.push(mat4{1.0f});

    // static var: material
    Material material;
    material.Ka = { 0, 0, 0 };
    material.Kd = { 1, 1, 1 };
    material.Ks = { 0, 0, 0 };
    material.Ke = { 0, 0, 0 };
    material.ex = 0;

    // static var: directional light
    float c0 = 1, c1 = 0, c2 = 0;

    // temp buffers
    std::vector<vec3> vertices;

    // Read a line in the scene file in each iteration
    while (std::getline(in, str))
    {
        // Ruled out comment and blank lines
        if ((str.find_first_not_of(" \t\r\n") == std::string::npos) 
            || (str[0] == '#')) continue;

        // Read a command
        std::stringstream s(str);
        s >> cmd;

        // Some arrays for storing values
        float fvalues[12];
        int ivalues[3];
        std::string svalues[1];

        if (cmd == "size" && readValues(s, 2, fvalues))
        {
            scene.width = (unsigned int)fvalues[0];
            scene.height = (unsigned int)fvalues[1];
        }
        else if (cmd == "output" && readValues(s, 1, svalues))
        {
            scene.outputFilename = svalues[0];
        }
        else if (cmd == "maxdepth" && readValues(s, 1, ivalues))
        {
            scene.depth = ivalues[0];
        }
        else if (cmd == "translate" && readValues(s, 3, fvalues))
        {
            mat4& T = transStack.top();
            T = T * translate(mat4{1.0f}, { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "scale" && readValues(s, 3, fvalues))
        {
            mat4& T = transStack.top();
            T = T * scale(mat4{1.0f}, { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "rotate" && readValues(s, 4, fvalues))
        {
            mat4& T = transStack.top();
            T = T * rotate(mat4{1.f}, radians(fvalues[3]), { fvalues[0], fvalues[1], fvalues[2] });
        }
        else if (cmd == "pushTransform")
        {
            transStack.push(transStack.top());
        }
        else if (cmd == "popTransform")
        {
            if (transStack.size() > 1)
                transStack.pop();
            else
                std::cerr << "[SceneLoader] Command \"popTransform\": Stack ran out of elements\n";
        }
        else if (cmd == "camera" && readValues(s, 10, fvalues))
        {
            vec3 eye    = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 lookat = { fvalues[3], fvalues[4], fvalues[5] };
            vec3 up     = { fvalues[6], fvalues[7], fvalues[8] };
            float fov = radians(fvalues[9]);
            float aspect = (float)scene.width / (float)scene.height;

            scene.cameraFrame.o = eye;

            ComputeCameraFrame(
                eye, lookat, up, fov, aspect,
                scene.cameraFrame.u, scene.cameraFrame.v, scene.cameraFrame.w, true);
        }
        else if (cmd == "vertex" && readValues(s, 3, fvalues))
        {
            vec3 v = { fvalues[0], fvalues[1], fvalues[2] };
            vertices.push_back(v);
        }
        else if (cmd == "tri" && readValues(s, 3, ivalues))
        {
            vec3 p0 = vertices[ivalues[0]];
            vec3 p1 = vertices[ivalues[1]];
            vec3 p2 = vertices[ivalues[2]];

            Triangle tri;
            tri.p0 = TransformPoint(p0, transStack.top());
            tri.p1 = TransformPoint(p1, transStack.top());
            tri.p2 = TransformPoint(p2, transStack.top());

            tri.mid = static_cast<int>(scene.materials.size());
            scene.materials.push_back(material);
            
            scene.triangles.push_back(tri);
        }
        else if (cmd == "sphere" && readValues(s, 4, fvalues))
        {
            vec3 v = { fvalues[0], fvalues[1], fvalues[2] };
            float r = fvalues[3];

            //mat4 replacement{1.0f}; // unit sphere
            //replacement = translate(replacement, v);
            //replacement = scale(replacement, {r, r, r});

            Sphere sphere;
            sphere.p = v;
            sphere.r = r;

            sphere.mid = static_cast<int>(scene.materials.size());
            scene.materials.push_back(material);
            
            sphere.tid = static_cast<int>(scene.transforms.size());
            scene.transforms.push_back(transStack.top());
            
            scene.spheres.push_back(sphere);
        }
        else if (cmd == "ambient" && readValues(s, 3, fvalues))
        {
            material.Ka = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "diffuse" && readValues(s, 3, fvalues))
        {
            material.Kd = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "specular" && readValues(s, 3, fvalues))
        {
            material.Ks = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "emission" && readValues(s, 3, fvalues))
        {
            material.Ke = { fvalues[0], fvalues[1], fvalues[2] };
        }
        else if (cmd == "shininess" && readValues(s, 1, fvalues))
        {
            material.ex = fvalues[0];
        }
        else if (cmd == "attenuation" && readValues(s, 3, fvalues))
        {
            c0 = fvalues[0];
            c1 = fvalues[1];
            c2 = fvalues[2];
        }
        else if (cmd == "directional" && readValues(s, 6, fvalues))
        {
            vec3 dir = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 clo = { fvalues[3], fvalues[4], fvalues[5] };

            DirectionalLight light;
            light.dir = -dir; // direction to light source
            light.color = clo;

            scene.dlights.push_back(light);
        }
        else if (cmd == "point" && readValues(s, 6, fvalues))
        {
            vec3 pos = { fvalues[0], fvalues[1], fvalues[2] };
            vec3 clo = { fvalues[3], fvalues[4], fvalues[5] };

            PointLight light;
            light.pos = pos;
            light.color = clo;
            light.c0 = c0;
            light.c1 = c1;
            light.c2 = c2;

            scene.plights.push_back(light);
        }
        else if (cmd == "integrator" && readValues(s, 1, svalues))
        {
            ;
        }
    }

    in.close();
}