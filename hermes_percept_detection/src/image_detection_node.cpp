#include "rclcpp/rclcpp.hpp"
#include "hermes_percept_detection/image_detection_component.hpp"

// Standalone executable that spins a single ImageDetectionComponent.
//
// Example:
//   ./image_detection_node --ros-args --remap camera/image_raw:=/my_cam/image_raw
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<hermes_percept::ImageDetectionComponent>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
