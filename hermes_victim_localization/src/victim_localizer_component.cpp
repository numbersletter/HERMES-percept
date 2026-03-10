#include <stdexcept>
#include "hermes_victim_localization/victim_localizer_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace hermes_percept {
VictimLocalizer::~VictimLocalizer() = default;

VictimLocalizer::VictimLocalizer(const rclcpp::NodeOptions & options)
: Node("victim_localization", options),
tf_buffer_(this->get_clock()),
tf_listener_(tf_buffer_)
{
    //subscribe to scan and camera_info topics 
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){
            this->recent_scan_ = msg;
        });

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            this->recent_cam_info_ = msg;
        });
    
    bbox_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/bbox_center", 10,
        std::bind(&VictimLocalizer::on_victim_detect, this, std::placeholders::_1));

    //create a publisher for result of victim localization
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/victim_point", 10);
}


double VictimLocalizer::distance_between_victims(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2){
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}


void VictimLocalizer::on_victim_detect(const geometry_msgs::msg::Point::SharedPtr bbox_center){
    if (!recent_cam_info_ || !recent_scan_) return;
    
    //calculate bearing angle
    double fx = recent_cam_info_ -> k[0];
    double cx = recent_cam_info_ -> k[2];

    double bearing_angle = atan2(bbox_center->x - cx, fx);

    //lidar scan index for distance calculation
    int scan_index = (bearing_angle - recent_scan_->angle_min) / recent_scan_->angle_increment;
    if(scan_index < 0 || scan_index >= (int)recent_scan_->ranges.size()) return; // if scan index within valid range

    float lidar_distance = recent_scan_->ranges[scan_index]; //get distance away from lidar using proper scan range
    if(lidar_distance < recent_scan_->range_min || lidar_distance >= recent_scan_->range_max) return; //check if valid distance value

    //get current timestamp and set correct frame id
    cam_point.header.stamp = this->get_clock()->now();
    cam_point.header.frame_id = "camera"; 

    //convert angle to coordinates using camera pose
    cam_point.point.x = lidar_distance * sin(-bearing_angle);
    cam_point.point.z = lidar_distance * cos(bearing_angle);
    cam_point.point.y = 0; 

    //use tf to transform to map frame
    try{
        auto victim_point = tf_buffer_.transform(cam_point, "map");
        auto current_point = victim_point.point;

        bool repeat = false;
        for(const auto & confirmed_victim : found_victims_){ //check if victim already identified and located
            if(distance_between_victims(current_point, confirmed_victim) < PROXIMITY_THRESHOLD){
                repeat = true;
                break;
            }
        }

        if(!repeat){ //if not a repeat victim, publish location and add to seen victims
            RCLCPP_INFO(this->get_logger(),"New victim at: x%.2f, y%.2f", current_point.x, current_point.y);

            found_victims_.push_back(current_point);
            pose_publisher_->publish(victim_point); //publish for map 
        }
    }catch(tf2::TransformException & ex){
        RCLCPP_WARN(this->get_logger(), "failed transform %s", ex.what());
    }

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(hermes_percept::VictimLocalizer)