cmake_minimum_required(VERSION 3.10)
project(HermesFaceDetection)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# openCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

# imported target
add_library(facedetection SHARED IMPORTED)
set_target_properties(facedetection PROPERTIES
    IMPORTED_LOCATION
        ${CMAKE_SOURCE_DIR}/extern/libfacedetection/lib/libfacedetection.so
    INTERFACE_INCLUDE_DIRECTORIES
        ${CMAKE_SOURCE_DIR}/extern/libfacedetection/include/
)

# executable to build
add_executable(facedetect src/facedetect.cpp)

# dynamic libraries
target_link_libraries(facedetect PRIVATE ${OpenCV_LIBS})
target_link_libraries(facedetect PRIVATE facedetection)
set_target_properties(facedetect PROPERTIES
    BUILD_RPATH
        ${CMAKE_SOURCE_DIR}/extern/libfacedetection/lib
)

# 7. Optimization for Pi 5
target_compile_options(facedetect PRIVATE -O3 -D_ENABLE_NEON)