cmake_minimum_required(VERSION 3.16)
project(2DPoseGraph VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${PROJECT_SOURCE_DIR}/utils)
add_subdirectory(${PROJECT_SOURCE_DIR}/pose_graph)
add_subdirectory(${PROJECT_SOURCE_DIR}/main_exe)