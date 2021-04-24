# cmake_minimum_require must be the first command of the file
cmake_minimum_required(VERSION 3.5.0)

# This is your project statement. You should always list languages;
# Listing the version is nice here since it sets lots of useful variables
project(XTracing VERSION 1.0 LANGUAGES CXX)

# ------------------------------------------------------------------
# glad
# ------------------------------------------------------------------
set(glad_dir "${CMAKE_SOURCE_DIR}/3rdparty/glad")
include_directories("${glad_dir}")
add_library(glad
    "${glad_dir}/glad.c"
    "${glad_dir}/glad.h"
    "${glad_dir}/khrplatform.h")

# ------------------------------------------------------------------
# glfw
# ------------------------------------------------------------------
set(glfw_dir "${CMAKE_SOURCE_DIR}/3rdparty/glfw")
include_directories("${glfw_dir}/include")
add_subdirectory("${glfw_dir}")

# ------------------------------------------------------------------
# imgui
# ------------------------------------------------------------------
set(imgui_dir "${CMAKE_SOURCE_DIR}/3rdparty/imgui")
include_directories("${imgui_dir}")
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
add_dependencies(imgui glad)

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
include_directories("${glm_dir}")

# ------------------------------------------------------------------
# stb
# ------------------------------------------------------------------
set(stb_dir "${CMAKE_SOURCE_DIR}/3rdparty/stb")
include_directories("${stb_dir}")

# ------------------------------------------------------------------
# tiny_obj_loader
# ------------------------------------------------------------------
set(tiny_obj_loader_dir "${CMAKE_SOURCE_DIR}/3rdparty/tiny_obj_loader")
include_directories("${tiny_obj_loader_dir}")

# ------------------------------------------------------------------
# 
# ------------------------------------------------------------------
include_directories("${CMAKE_SOURCE_DIR}/3rdparty")
include_directories("${CMAKE_SOURCE_DIR}/xtracing")
include_directories("${CMAKE_SOURCE_DIR}/util")

add_subdirectory(util)
add_subdirectory(demo/demo_instantializer)
add_subdirectory(demo/demo_phong)
add_subdirectory(demo/demo_texture)
#add_subdirectory(demo/cse_168_hw1)
add_subdirectory(demo/cse_168_hw2)
