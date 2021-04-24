add_executable(cse_168_hw2
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)

include_directories("${CMAKE_CURRENT_LIST_DIR}")

add_dependencies(cse_168_hw2 util)
target_link_libraries(cse_168_hw2 util)