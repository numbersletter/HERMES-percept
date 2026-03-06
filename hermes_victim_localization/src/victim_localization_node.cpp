#include "rclcpp/rclcpp.hpp"
#include "hermes_victim_localization/victim_localizer_component.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<hermes_percept::VictimLocalizer>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
