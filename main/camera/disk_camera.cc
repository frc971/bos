#include "disk_camera.h"
#include <opencv2/opencv.hpp>
namespace fs = std::filesystem;

namespace camera {

DiskCamera::DiskCamera(std::string path_to_img_dir) 
    : path_to_img_dir(path_to_img_dir) {}

cv::Mat DiskCamera::GetLatestFrame() {
    int max_num = -1;
    std::string latest_path;

    for (const auto& entry : fs::directory_iterator(path_to_img_dir)) {
        if (entry.path().extension() == ".png") {
            std::string stem = entry.path().stem().string();

            // ensure file nname is purely digits
            if (!stem.empty() && std::all_of(stem.begin(), stem.end(), ::isdigit)) {
                int num = std::stoi(stem);
                if (num > max_num) {
                    max_num = num;
                    latest_path = entry.path().string();
                }
            }
        }
    }
    return cv::imread(latest_path, cv::IMREAD_COLOR);
}

} // namespace camera