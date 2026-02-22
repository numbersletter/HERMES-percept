#include "rclcpp/rclcpp.hpp"
#include "hermes_percept_publisher/image_publisher_component.hpp"

// Standalone executable that spins a single ImagePublisherComponent.
//
// All node parameters can be supplied from the command line via --ros-args, e.g.:
//   ./image_publisher_node --ros-args -p source:=0 -p publish_rate:=30.0
//                                     -p flip_horizontal:=true
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<hermes_percept::ImagePublisherComponent>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
