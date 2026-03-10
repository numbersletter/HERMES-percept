#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "image_transport/image_transport.hpp"
#include <vector>
#include <cmath>


namespace hermes_percept {
    class VictimLocalizer : public rclcpp::Node {
    public:
        explicit VictimLocalizer(const rclcpp::NodeOptions & options);
        ~VictimLocalizer();
    private:

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::msg::PointStamped cam_point;
        geometry_msgs::msg::PointStamped victim_point;

        std::vector<geometry_msgs::msg::Point> found_victims_;
        const double PROXIMITY_THRESHOLD = 0.25;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        sensor_msgs::msg::LaserScan::SharedPtr recent_scan_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        sensor_msgs::msg::CameraInfo::SharedPtr recent_cam_info_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr bbox_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_publisher_;

        double distance_between_victims(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

        void on_victim_detect(const geometry_msgs::msg::Point::SharedPtr bbox_center);

    };
}