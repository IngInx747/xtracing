add_library(util
    util.cpp
    util.h)

add_library(displayer
    displayer.cpp
    displayer.h)

include_directories("${CMAKE_CURRENT_LIST_DIR}")

target_link_libraries(displayer
    util
    glad
    glfw
    imgui)