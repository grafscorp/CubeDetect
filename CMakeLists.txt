cmake_minimum_required(VERSION 3.10.0)
project(CubeDetect VERSION 0.1.0 LANGUAGES C CXX)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

find_package(OpenCV REQUIRED)

add_subdirectory(cubedetect)

add_executable(CubeDetect main.cpp)

target_include_directories(CubeDetect PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/cubedetect/include)
target_link_libraries(CubeDetect PRIVATE 
${OpenCV_LIBS}
CubeDetectLib)








