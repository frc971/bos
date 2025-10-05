#pragma once
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>

namespace camera {

class DiskCamera {
    private:
        std::string path_to_img_dir;
    

    public:
        DiskCamera(std::string path_to_img_dir);
        cv::Mat GetLatestFrame();
};

} // namespace camera