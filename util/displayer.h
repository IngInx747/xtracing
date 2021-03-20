#pragma once
#ifndef DISPLAYER_H
#define DISPLAYER_H

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "xmath.h"

class Window
{
public:
    Window(char* title);
    ~Window();

    // put pixels on the screen
    virtual void draw()
    {}

    // callback that window got resized
    virtual void resize(const int2 &newSize)
    {}

    // callback that keyboard interrupt
    virtual void key(int key, int mods)
    {}
    
    // callback that mouse moving interrupt
    virtual void mouseMotion(const int2 &newPos)
    {}
    
    // callback that mouse click interrupt
    virtual void mouseButton(int button, int action, int mods)
    {}

    // re-render the frame
    virtual void render() 
    {}

    // Opens the actual window, and runs the window's events to completion.
    // Return once the window gets closed.
    void run();

    inline int2 getMousePos() const
    {
        double x, y;
        glfwGetCursorPos(handle, &x, &y);
        return {(int)x, (int)y};
    }
    
public:
    // the glfw window handle
    GLFWwindow *handle { nullptr };
};

#endif // !DISPLAYER_H