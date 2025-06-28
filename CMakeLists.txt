cmake_minimum_required(VERSION 3.10.0)
project(CubeDetect VERSION 0.1.0 LANGUAGES C CXX)

add_executable(CubeDetect main.cpp)

configure_file(
${CMAKE_SOURCE_DIR}/assets/cube.jpg
${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/cube.jpg
COPYONLY
)
