cmake_minimum_required(VERSION 3.10)
project(hermes_percept_publisher)

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

# --- Image Publisher Component (shared library, loadable into a component container) ---
add_library(image_publisher_component SHARED
    src/image_publisher_component.cpp
)
target_include_directories(image_publisher_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_publisher_component
    rclcpp
    rclcpp_components
    image_transport
    sensor_msgs
    cv_bridge
    std_msgs
)
target_link_libraries(image_publisher_component ${OpenCV_LIBS})
target_compile_options(image_publisher_component PRIVATE -O3)
rclcpp_components_register_nodes(image_publisher_component
    "hermes_percept::ImagePublisherComponent")

# --- Standalone node executable ---
add_executable(image_publisher_node src/image_publisher_node.cpp)
target_include_directories(image_publisher_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_publisher_node
    rclcpp
    image_transport
    sensor_msgs
    cv_bridge
    std_msgs
)
target_link_libraries(image_publisher_node image_publisher_component ${OpenCV_LIBS})

# --- Test viewer executable ---
add_executable(test_viewer test/test_viewer.cpp)
target_include_directories(test_viewer PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(test_viewer
    rclcpp
    image_transport
    sensor_msgs
    cv_bridge
)
target_link_libraries(test_viewer ${OpenCV_LIBS})

# Install
install(TARGETS
    image_publisher_component
    image_publisher_node
    test_viewer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
