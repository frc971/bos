#pragma once
#include "src/camera/cv_camera.h"
#include <iostream>
namespace camera {


// Asks Users for input and selects camera based on that.
CVCamera SelectCamera();
    std::cout << "Which input do you want to use; usb or mipi: ";
    string camType;
    std::cin >> camType;

    if (camType == "usb") {
        std::cout << "Which camera number; 1-4: ";
        int camNum;
        std::cin >> camNum;

        switch (camNum) {
            case 1:
                return std::make_unique<camera::CVCamera>(cv::VideoCapture("/dev/video0"));

            case 2:
                return std::make_unique<camera::CVCamera>(cv::VideoCapture("/dev/video2"));

            case 3:
                return std::make_unique<camera::CVCamera>(cv::VideoCapture("/dev/video4"));

            case 4:
                return std::make_unique<camera::CVCamera>(cv::VideoCapture("/dev/video6"));

            default:
                std::cout << "INVALID INPUT: must be a number from 1 to 4.";
                SelectCamera();
        }
        

    } else if (camType == "mipi") {

    } else {
        std::cout << "INVALID INPUT: must be lowercase";
        SelectCamera();

    }
}
