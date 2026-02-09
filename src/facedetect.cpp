#include <iostream>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include "facedetectcnn.h"

// memory buffer for detection results (bytes)
#define DETECT_BUFFER_SIZE 0x20000

int main() {
    // gstreamer pipeline from the python prototype
    std::string pipeline = 
        "libcamerasrc af-mode=2 ae-exposure-mode=1 awb-enable=true ! "
        "video/x-raw,width=640,height=480,format=NV12,framerate=30/1 ! "
        "videoconvert ! "
        "tee name=t "
        "t. ! queue ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! mpegtsmux ! tcpserversink host=0.0.0.0 port=5000 "
        "t. ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1";

    // launch stream
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video stream." << std::endl;
        return -1;
    }

    // buffer to store results of inference
    unsigned char * pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
    // used for storing raw frames
    cv::Mat frame;

    std::cout << "C++ Detection Active. Starting loop..." << std::endl;

    while (true) {
        if (!cap.read(frame)) break;

        // 2. Inference
        // Passing raw frame data directly for maximum efficiency
        int* results = facedetect_cnn(pBuffer, (unsigned char*)(frame.ptr(0)), frame.cols, frame.rows, (int)frame.step);

        // 3. Visualize results on imshow
        for (int i = 0; i < (results ? *results : 0); i++) {
            short* p = ((short*)(results + 1)) + 142 * i;
            int confidence = p[0];

            if (confidence >= 85) { // Matching your 0.9 confidence threshold
                int x = p[1], y = p[2], w = p[3], h = p[4];
                
                // Draw Bounding Box (Red, like your Python TEXT_COLOR)
                cv::rectangle(frame, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 3);

                // Draw Landmarks (Green circles)
                for (int j = 0; j < 5; j++) {
                    cv::circle(frame, cv::Point(p[5 + j*2], p[5 + j*2 + 1]), 2, cv::Scalar(0, 255, 0), 2);
                }

                // Draw Label and Score
                std::string label = "Face (" + std::to_string(confidence) + ")";
                cv::putText(frame, label, cv::Point(x + 10, y + 20), 
                            cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
            }
        }
        cv::imshow("Pi 5 Face Detection (C++)", frame);
        if (cv::waitKey(1) == 'q') break;
    }

    // on 'q'
    free(pBuffer);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}