#include "hermes_percept_detection/image_detection_component.hpp"
#include "facedetectcnn/facedetectcnn.h"
#include "rclcpp_components/register_node_macro.hpp"

// Memory buffer size for detection results (bytes)
#define DETECT_BUFFER_SIZE 0x20000

namespace hermes_percept {

ImageDetectionComponent::ImageDetectionComponent(const rclcpp::NodeOptions & options)
: Node("image_detection", options),
  last_publish_time_(this->get_clock()->now())
{
    pBuffer_ = static_cast<unsigned char *>(malloc(DETECT_BUFFER_SIZE));

    // image_transport publisher: advertises face_img (raw + compressed)
    publisher_ = image_transport::create_publisher(this, "face_img");

    bbox_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/bbox_center", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&ImageDetectionComponent::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Image Detection Component Active.");
}

ImageDetectionComponent::~ImageDetectionComponent()
{
    if (pBuffer_) {
        free(pBuffer_);
    }
}

void ImageDetectionComponent::on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat & frame = cv_ptr->image;

    // Run face detection inference
    int * results = facedetect_cnn(
        pBuffer_,
        reinterpret_cast<unsigned char *>(frame.ptr(0)),
        frame.cols, frame.rows, static_cast<int>(frame.step));

    bool face_detected = false;
    int num_faces = (results ? *results : 0);

    for (int i = 0; i < num_faces; i++) {
        short * p = reinterpret_cast<short *>(results + 1) + 142 * i;
        int confidence = p[0];

        if (confidence >= 80) {
            face_detected = true;
            int x = p[1], y = p[2], w = p[3], h = p[4];

            // Draw bounding box
            cv::rectangle(frame, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 3);

            //get center of bounding box for victim pose
            auto bbox_center = geometry_msgs::msg::Point();
            bbox_center.x = x + (w / 2.0); // center x
            bbox_center.y = y + (h / 2.0); // center y
            bbox_center.z = 0.0;

            bbox_publisher_->publish(bbox_center);

            // Draw landmarks
            for (int j = 0; j < 5; j++) {
                cv::circle(frame, cv::Point(p[5 + j * 2], p[5 + j * 2 + 1]), 2, cv::Scalar(0, 255, 0), 2);
            }

            // Draw label
            std::string label = "Face (" + std::to_string(confidence) + ")";
            cv::putText(frame, label, cv::Point(x + 10, y + 20),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
        }
    }

    // Throttled publishing at 1 Hz when a face is detected
    if (face_detected) {
        auto now = this->get_clock()->now();
        if ((now - last_publish_time_).seconds() >= 1.0) {
            publish_image(frame);
            last_publish_time_ = now;
            RCLCPP_INFO(this->get_logger(), "Published face detection image");
        }
    }
}

void ImageDetectionComponent::publish_image(const cv::Mat & frame)
{
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "camera";
    publisher_.publish(*msg);
}

}  // namespace hermes_percept

RCLCPP_COMPONENTS_REGISTER_NODE(hermes_percept::ImageDetectionComponent)
