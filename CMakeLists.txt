cmake_minimum_required(VERSION 3.10.0)
project(CubeDetect VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(CubeDetect main.cpp)
target_link_libraries(CubeDetect ${OpenCV_LIBS})


configure_file(
${CMAKE_SOURCE_DIR}/assets/cube.jpg
${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/cube.jpg
COPYONLY
)
