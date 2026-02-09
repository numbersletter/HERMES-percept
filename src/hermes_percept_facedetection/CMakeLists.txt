cmake_minimum_required(VERSION 3.10)
project(hermes_percept_facedetection)

# Standard ROS 2 / Ament setup
find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

# Handle the external library (the library name is facedetection)
add_library(facedetection SHARED IMPORTED)
set_target_properties(facedetection PROPERTIES
    IMPORTED_LOCATION
        "${PROJECT_SOURCE_DIR}/extern/libfacedetection/lib/libfacedetection.so.v0.0.3"
    INTERFACE_INCLUDE_DIRECTORIES
        "${PROJECT_SOURCE_DIR}/extern/libfacedetection/include/"
)

# Non-ROS executable (Optional standalone test)
add_executable(facedetect src/facedetect.cpp)
target_include_directories(facedetect PRIVATE 
    src
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(facedetect PRIVATE 
    facedetection 
    ${OpenCV_LIBS})
target_compile_options(facedetect PRIVATE -O3 -D_ENABLE_NEON)
set_target_properties(facedetect PROPERTIES BUILD_RPATH "${PROJECT_SOURCE_DIR}/extern/libfacedetection/lib")

# ROS related stuff below
add_executable(facedetect_node src/facedetect_node.cpp)

# Include directories for the node
target_include_directories(facedetect_node PRIVATE 
    src
    ${OpenCV_INCLUDE_DIRS}
)

# Use ament_target_dependencies for ROS packages (libraries)
ament_target_dependencies(facedetect_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
)

# Non-ROS library linking
target_link_libraries(facedetect_node 
    ${OpenCV_LIBS}
    facedetection
)

# Optimization and RPATH
target_compile_options(facedetect_node PRIVATE -O3 -D_ENABLE_NEON)
set_target_properties(facedetect_node PROPERTIES 
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/extern/libfacedetection/lib"
    INSTALL_RPATH "$ORIGIN/../extern/libfacedetection/lib"
)

# Install rules
install(TARGETS
    facedetect
    facedetect_node
    DESTINATION lib/${PROJECT_NAME}
)

# Ament Package Finalize
ament_package()
