cmake_minimum_required(VERSION 3.10.0)
project(CubeDetectLib VERSION 0.1.0 LANGUAGES C CXX)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(sources 
    src/cubedetect.cpp
)
set(headers
    include/cubedetect.hpp
    )

add_library(CubeDetectLib STATIC ${headers} ${sources} )

target_include_directories(CubeDetectLib PUBLIC
 include
)   


target_link_libraries(CubeDetectLib PUBLIC ${OpenCV_LIBS})

install(TARGETS CubeDetectLib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)