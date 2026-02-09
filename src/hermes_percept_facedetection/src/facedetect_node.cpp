#include <iostream>
#include <vector>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>
#include "facedetectcnn.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

// memory buffer for detection results (bytes)
#define DETECT_BUFFER_SIZE 0x20000

using namespace std::chrono_literals;

class FaceDetectNode : public rclcpp::Node {
public:
    FaceDetectNode() : Node("face_detect_node"), last_publish_time_(this->get_clock()->now()) {
        // gstreamer pipeline
        std::string pipeline = 
            "libcamerasrc af-mode=2 ae-exposure-mode=1 awb-enable=true ! "
            "video/x-raw,width=640,height=480,format=NV12,framerate=30/1 ! "
            "videoconvert ! "
            "tee name=t "
            "t. ! queue ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! mpegtsmux ! tcpserversink host=0.0.0.0 port=5000 "
            "t. ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1";

        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open video stream.");
            throw std::runtime_error("Could not open video stream");
        }

        // Buffer for detection results
        pBuffer_ = (unsigned char *)malloc(DETECT_BUFFER_SIZE);

        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("face_img", 10);

        // Timer to process frames (approx 30 FPS)
        timer_ = this->create_wall_timer(33ms, std::bind(&FaceDetectNode::process_frame, this));
        
        RCLCPP_INFO(this->get_logger(), "Face Detection Node Active.");
    }

    ~FaceDetectNode() {
        if (pBuffer_) free(pBuffer_);
        cap_.release();
    }

private:
    cv::VideoCapture cap_;
    unsigned char * pBuffer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_publish_time_;
    
    void process_frame() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame");
            return;
        }

        // 1. Inference
        int* results = facedetect_cnn(pBuffer_, (unsigned char*)(frame.ptr(0)), frame.cols, frame.rows, (int)frame.step);

        bool face_detected = false;
        int num_faces = (results ? *results : 0);

        for (int i = 0; i < num_faces; i++) {
            short* p = ((short*)(results + 1)) + 142 * i;
            int confidence = p[0];

            if (confidence >= 85) {
                face_detected = true;
                int x = p[1], y = p[2], w = p[3], h = p[4];
                
                // Draw Bounding Box
                cv::rectangle(frame, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 3);

                // Draw Landmarks
                for (int j = 0; j < 5; j++) {
                    cv::circle(frame, cv::Point(p[5 + j*2], p[5 + j*2 + 1]), 2, cv::Scalar(0, 255, 0), 2);
                }

                // Draw Label
                std::string label = "Face (" + std::to_string(confidence) + ")";
                cv::putText(frame, label, cv::Point(x + 10, y + 20), 
                            cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
            }
        }

        // Throttled Publishing (1Hz)
        if (face_detected) {
            auto now = this->get_clock()->now();
            if ((now - last_publish_time_).seconds() >= 1.0) {
                publish_image(frame);
                last_publish_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Published face detection image");
            }
        }

        // Optional debugging if needed 
        cv::imshow("Node Debug", frame);
        cv::waitKey(1);
    }

    void publish_image(const cv::Mat& frame) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FaceDetectNode>());
    } catch (const std::exception& e) {
        std::cerr << "Node terminated: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
