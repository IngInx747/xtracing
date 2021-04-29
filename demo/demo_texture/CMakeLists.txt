add_executable(demo_texture
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)
add_dependencies(demo_texture util)
target_link_libraries(demo_texture util)
target_include_directories(demo_texture PRIVATE "${CMAKE_CURRENT_LIST_DIR}")