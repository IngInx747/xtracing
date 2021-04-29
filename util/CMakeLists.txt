add_library(util
    util.cpp
    util.h)
target_link_libraries(util
    stb
    xtracing)
target_include_directories(util PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

add_library(displayer
    displayer.cpp
    displayer.h)
target_link_libraries(displayer
    util
    glad
    glfw
    imgui)
target_include_directories(displayer PUBLIC "${CMAKE_CURRENT_LIST_DIR}")