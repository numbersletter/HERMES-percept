#pragma once

#include <cstdlib>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "std_msgs/msg/header.hpp"

namespace hermes_percept {

class ImageDetectionComponent : public rclcpp::Node {
public:
    explicit ImageDetectionComponent(const rclcpp::NodeOptions & options);
    ~ImageDetectionComponent();

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
    void publish_image(const cv::Mat & frame);

    unsigned char * pBuffer_;
    image_transport::Publisher publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Time last_publish_time_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr bbox_publisher_;

};

}  // namespace hermes_percept
