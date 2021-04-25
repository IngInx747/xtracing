add_executable(cse_168_hw1
    main.cpp
    Renderer.cpp
    Renderer.h
    Scene.cpp
    Scene.h)

include_directories("${CMAKE_CURRENT_LIST_DIR}")

add_dependencies(cse_168_hw1 util)
target_link_libraries(cse_168_hw1 util)