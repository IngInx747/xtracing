add_executable(demo_phong
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)
add_dependencies(demo_phong util)
target_link_libraries(demo_phong util)
target_include_directories(demo_phong PRIVATE "${CMAKE_CURRENT_LIST_DIR}")