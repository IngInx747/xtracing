add_executable(cse_168_hw2
    main.cpp
    Geometry.cpp
    Geometry.h
    Light.cpp
    Light.h
    Shader.cpp
    Shader.h
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h
    SceneLoader.cpp
    SceneLoader.h)
add_dependencies(cse_168_hw2 util)
target_link_libraries(cse_168_hw2 util)
target_include_directories(cse_168_hw2 PRIVATE "${CMAKE_CURRENT_LIST_DIR}")