add_executable(demo_instantializer
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)
add_dependencies(demo_instantializer util)
target_link_libraries(demo_instantializer util)
target_link_libraries(demo_instantializer tiny_obj_loader)
target_include_directories(demo_instantializer PRIVATE "${CMAKE_CURRENT_LIST_DIR}")