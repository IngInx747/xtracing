add_executable(demo_texture
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)

include_directories("${CMAKE_CURRENT_LIST_DIR}")

add_dependencies(demo_texture util)
target_link_libraries(demo_texture util)