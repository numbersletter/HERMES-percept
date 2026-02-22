#pragma once

#include <chrono>
#include <string>
#include <opencv4/opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace hermes_percept {

class ImagePublisherComponent : public rclcpp::Node {
public:
    explicit ImagePublisherComponent(const rclcpp::NodeOptions & options);
    ~ImagePublisherComponent();

private:
    // Computes needs_flip_ and flip_code_ from the current flip flags.
    void compute_flip_params();

    // Opens (or re-opens) the capture source and pre-computes flip params.
    void init();

    // Recreates the timer using the current publish_rate_.
    void reset_timer();

    // Timer callback: captures one frame and publishes via image_transport.
    void work();

    // Parameter-change callback.
    rcl_interfaces::msg::SetParametersResult on_params_changed(
        const std::vector<rclcpp::Parameter> & params);

    std::string source_;
    double publish_rate_;
    bool flip_horizontal_;
    bool flip_vertical_;
    bool retry_on_failure_;

    // Pre-computed flip values (set by compute_flip_params(), used in work()).
    // needs_flip_ gates use of flip_code_, so flip_code_ is only meaningful
    // when needs_flip_ is true.
    bool needs_flip_{false};
    int  flip_code_{0};

    // True when the source is a video file (not a camera or still image).
    // Used in work() to rewind the stream when it reaches the end.
    bool is_video_{false};

    cv::VideoCapture cap_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;
    int consecutive_failures_{0};
};

}  // namespace hermes_percept
