add_executable(demo_instantializer
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)

include_directories("${CMAKE_CURRENT_LIST_DIR}")

add_dependencies(demo_instantializer util)
target_link_libraries(demo_instantializer util)