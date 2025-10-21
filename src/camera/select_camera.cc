#include "src/camera/select_camera.h"
#include "cv_camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace camera {

    /*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
    */
    CVCamera SelectCamera() {
        std::cout << "Which input do you want to use; usb or mipi; use 1 or 2: ";
        int camType; // Which camera type to use
        std::cin >> camType;

        if (camType == 1) {
            std::cout << "Which camera number; 1-4: ";
            int camNumUSB; // Which USB camera to to use
            std::cin >> camNumUSB;

            switch (camNumUSB) {
                case 1:
                    return camera::CVCamera(cv::VideoCapture("/dev/video0"));

                case 2:
                    return camera::CVCamera(cv::VideoCapture("/dev/video2"));

                case 3:
                    return camera::CVCamera(cv::VideoCapture("/dev/video4"));

                case 4:
                    return camera::CVCamera(cv::VideoCapture("/dev/video6"));

                default:
                    std::cout << "INVALID INPUT: must be a number from 1 to 4.";
                    return SelectCamera();
            }
            

        } else if (camType == 2) {
            std::cout << "Which camera number; 1-2: ";
            int camNumMIPI; // Which MIPI camera to use
            std::cin >> camNumMIPI;

            switch (camNumMIPI) {
                case 1:
                    return camera::CVCamera(cv::VideoCapture(camera::gstreamer1_30fps));
                
                case 2:
                    return camera::CVCamera(cv::VideoCapture(camera::gstreamer1_30fps));

                default:
                    std::cout << "INVALID INPUT: must be between 1 and 2";
            }
        } else {
            std::cout << "INVALID INPUT: 1 or 2";
            return SelectCamera();
        }
    }
}
