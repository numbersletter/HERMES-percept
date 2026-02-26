/**
 * test_viewer — subscribes to camera/image_raw published by
 * ImagePublisherComponent and displays every frame in an OpenCV window.
 *
 * Usage (two terminals):
 *   Terminal 1:  libcamerify ros2 run hermes_percept_publisher image_publisher_node
 *   Terminal 2:  ros2 run hermes_percept_publisher test_viewer
 *
 * Press 'q' or ESC in the window to quit.
 */

#include <chrono>
#include <opencv4/opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"

class TestViewer : public rclcpp::Node
{
public:
    TestViewer()
    : Node("test_viewer"), frame_count_(0)
    {
        // Subscribe via image_transport so compressed transport works too.
        sub_ = image_transport::create_subscription(
            this, "camera/image_raw",
            std::bind(&TestViewer::on_image, this, std::placeholders::_1),
            "compressed/40",
            rmw_qos_profile_sensor_data);

        last_fps_time_ = std::chrono::steady_clock::now();

        cv::namedWindow("HERMES Camera Test", cv::WINDOW_AUTOSIZE);

        RCLCPP_INFO(get_logger(),
            "TestViewer ready — waiting for images on 'camera/image_raw'. "
            "Press 'q' or ESC in the window to quit.");
    }

    ~TestViewer()
    {
        cv::destroyAllWindows();
    }

private:
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("HERMES Camera Test", cv_ptr->image);

        // Print FPS every second
        ++frame_count_;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time_).count();
        if (elapsed >= 1.0) {
            RCLCPP_INFO(get_logger(), "Receiving %.1f FPS (%dx%d)",
                frame_count_ / elapsed,
                cv_ptr->image.cols, cv_ptr->image.rows);
            frame_count_ = 0;
            last_fps_time_ = now;
        }

        // Check for quit key — 'q' or ESC
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            RCLCPP_INFO(get_logger(), "Quit key pressed — shutting down.");
            rclcpp::shutdown();
        }
    }

    image_transport::Subscriber sub_;
    int frame_count_;
    std::chrono::steady_clock::time_point last_fps_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestViewer>());
    rclcpp::shutdown();
    return 0;
}