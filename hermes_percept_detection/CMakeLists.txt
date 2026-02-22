cmake_minimum_required(VERSION 3.10)
project(hermes_percept_detection)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

option(ENABLE_NEON "Enable NEON SIMD optimizations (ARM only)" OFF)

# libfacedetection built from source
add_library(facedetection STATIC
    src/facedetectcnn.cpp
    src/facedetectcnn-data.cpp
    src/facedetectcnn-model.cpp
)
target_include_directories(facedetection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(facedetection PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(ENABLE_NEON)
    target_compile_options(facedetection PRIVATE -O3 -D_ENABLE_NEON)
else()
    target_compile_options(facedetection PRIVATE -O3)
endif()

# --- Image Detection Component (shared library, loadable into a component container) ---
add_library(image_detection_component SHARED
    src/image_detection_component.cpp
)
target_include_directories(image_detection_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_detection_component
    rclcpp
    rclcpp_components
    image_transport
    sensor_msgs
    cv_bridge
    std_msgs
)
target_link_libraries(image_detection_component
    ${OpenCV_LIBS}
    facedetection
)
rclcpp_components_register_nodes(image_detection_component
    "hermes_percept::ImageDetectionComponent")

# --- Standalone node executable ---
add_executable(image_detection_node src/image_detection_node.cpp)
target_include_directories(image_detection_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_detection_node
    rclcpp
    image_transport
    sensor_msgs
    cv_bridge
    std_msgs
)
target_link_libraries(image_detection_node image_detection_component ${OpenCV_LIBS})

# Install
install(TARGETS
    image_detection_component
    image_detection_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
