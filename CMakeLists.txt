# cmake_minimum_require must be the first command of the file
cmake_minimum_required(VERSION 3.5.0)

# This is your project statement. You should always list languages;
# Listing the version is nice here since it sets lots of useful variables
project(XTracing VERSION 1.0 LANGUAGES CXX)

# ------------------------------------------------------------------
# glad
# ------------------------------------------------------------------
set(glad_dir "${CMAKE_SOURCE_DIR}/3rdparty/glad")
add_library(glad
    "${glad_dir}/glad.c"
    "${glad_dir}/glad.h"
    "${glad_dir}/khrplatform.h")
target_include_directories(glad PUBLIC "${glad_dir}")

# ------------------------------------------------------------------
# glfw
# ------------------------------------------------------------------
set(glfw_dir "${CMAKE_SOURCE_DIR}/3rdparty/glfw")
set(glfw_include_dir "${glfw_dir}/include")
add_subdirectory("${glfw_dir}")

# ------------------------------------------------------------------
# imgui
# ------------------------------------------------------------------
set(imgui_dir "${CMAKE_SOURCE_DIR}/3rdparty/imgui")
add_library(imgui
    "${imgui_dir}/imgui.cpp"
    "${imgui_dir}/imgui_demo.cpp"
    "${imgui_dir}/imgui_draw.cpp"
    "${imgui_dir}/imgui_widgets.cpp"
    "${imgui_dir}/imgui_impl_glfw.cpp"
    "${imgui_dir}/imgui_impl_opengl3.cpp"
    "${imgui_dir}/imconfig.h"
    "${imgui_dir}/imgui.h"
    "${imgui_dir}/imgui_internal.h"
    "${imgui_dir}/imstb_rectpack.h"
    "${imgui_dir}/imstb_textedit.h"
    "${imgui_dir}/imstb_truetype.h"
    "${imgui_dir}/imgui_impl_glfw.h"
    "${imgui_dir}/imgui_impl_opengl3.h")
target_include_directories(imgui PUBLIC "${imgui_dir}")
target_include_directories(imgui PUBLIC "${glad_dir}")
target_include_directories(imgui PUBLIC "${glfw_include_dir}")
add_dependencies(imgui glad)
add_dependencies(imgui glfw)

# ------------------------------------------------------------------
# assimp
# ------------------------------------------------------------------
#set(assimp_dir "${CMAKE_SOURCE_DIR}/3rdparty/assimp")
#include_directories("${assimp_dir}/include")
#add_subdirectory("${assimp_dir}")

# ------------------------------------------------------------------
# glm
# ------------------------------------------------------------------
set(glm_dir "${CMAKE_SOURCE_DIR}/3rdparty/glm")
add_library(glm INTERFACE)
target_include_directories(glm INTERFACE "${glm_dir}")

# ------------------------------------------------------------------
# stb
# ------------------------------------------------------------------
set(stb_dir "${CMAKE_SOURCE_DIR}/3rdparty/stb")
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE "${stb_dir}")

# ------------------------------------------------------------------
# tiny_obj_loader
# ------------------------------------------------------------------
set(tiny_obj_loader_dir "${CMAKE_SOURCE_DIR}/3rdparty/tiny_obj_loader")
add_library(tiny_obj_loader INTERFACE)
target_include_directories(tiny_obj_loader INTERFACE "${tiny_obj_loader_dir}")

# ------------------------------------------------------------------
# xtracing
# ------------------------------------------------------------------
set(xtracing_dir "${CMAKE_SOURCE_DIR}/xtracing")
add_library(xtracing INTERFACE)
target_link_libraries(xtracing INTERFACE glm)
target_include_directories(xtracing INTERFACE "${xtracing_dir}")

# ------------------------------------------------------------------
# 
# ------------------------------------------------------------------
#include_directories("${CMAKE_SOURCE_DIR}/3rdparty")
#include_directories("${CMAKE_SOURCE_DIR}/util")

add_subdirectory(util)
add_subdirectory(demo/demo_instantializer)
add_subdirectory(demo/demo_phong)
add_subdirectory(demo/demo_texture)
add_subdirectory(demo/cse_168_hw1)
add_subdirectory(demo/cse_168_hw2)
