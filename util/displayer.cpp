#include "displayer.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "[GLFW][ERROR] %s\n", description);
}
  
Window::~Window()
{
  glfwDestroyWindow(handle);
  glfwTerminate();
}

Window::Window(char* title)
{
    glfwSetErrorCallback(glfw_error_callback);
    // glfwInitHint(GLFW_COCOA_MENUBAR, GLFW_FALSE);
      
    if (!glfwInit()) exit(EXIT_FAILURE);
      
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
      
    handle = glfwCreateWindow(1200, 800, title, NULL, NULL);
    if (!handle)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
      
    glfwSetWindowUserPointer(handle, this);
    glfwMakeContextCurrent(handle);
    glfwSwapInterval( 1 );
}

// callback for a window resizing event
static void glfwindow_reshape_cb(GLFWwindow* window, int width, int height )
{
    Window *gw = static_cast<Window*>(glfwGetWindowUserPointer(window));
    assert(gw);
    gw->resize({width, height});
}

// callback for a key press
static void glfwindow_key_cb(GLFWwindow *window, int key, int scancode, int action, int mods) 
{
    Window *gw = static_cast<Window*>(glfwGetWindowUserPointer(window));
    assert(gw);
    if (action == GLFW_PRESS) gw->key(key, mods);
}

// callback for _moving_ the mouse to a new position
static void glfwindow_mouseMotion_cb(GLFWwindow *window, double x, double y) 
{
    Window *gw = static_cast<Window*>(glfwGetWindowUserPointer(window));
    assert(gw);
    gw->mouseMotion({(int)x, (int)y});
}

// callback for pressing _or_ releasing a mouse button
static void glfwindow_mouseButton_cb(GLFWwindow *window, int button, int action, int mods) 
{
    Window *gw = static_cast<Window*>(glfwGetWindowUserPointer(window));
    assert(gw);
    // double x, y;
    // glfwGetCursorPos(window, &x, &y);
    gw->mouseButton(button, action, mods);
}

void Window::run()
{
    int width, height;
    glfwGetFramebufferSize(handle, &width, &height);
    resize({width, height});
    glfwSetFramebufferSizeCallback(handle, glfwindow_reshape_cb);
    glfwSetMouseButtonCallback(handle, glfwindow_mouseButton_cb);
    glfwSetKeyCallback(handle, glfwindow_key_cb);
    glfwSetCursorPosCallback(handle, glfwindow_mouseMotion_cb);
    
    while (!glfwWindowShouldClose(handle))
    {
        render();
        draw();
          
        glfwSwapBuffers(handle);
        glfwPollEvents();
    }
}