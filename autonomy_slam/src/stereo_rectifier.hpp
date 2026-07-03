#ifndef AUTONOMY_SLAM_STEREO_RECTIFIER_HPP_
#define AUTONOMY_SLAM_STEREO_RECTIFIER_HPP_

#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/config.hpp"

#include <opencv2/core/mat.hpp>
#include <yaml-cpp/yaml.h>

namespace autonomy_slam {

// Uses /usr/local OpenCV 5.x (same ABI as autonomy::autonomy).
class StereoRectifier {
public:
    StereoRectifier(const std::shared_ptr<autonomy::localization::atlas::config>& cfg,
                    autonomy::localization::atlas::camera::base* camera);

    void rectify(const cv::Mat& in_left, const cv::Mat& in_right,
                 cv::Mat& out_left, cv::Mat& out_right) const;

private:
    static cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec);

    cv::Mat undist_map_x_l_;
    cv::Mat undist_map_y_l_;
    cv::Mat undist_map_x_r_;
    cv::Mat undist_map_y_r_;
};

}  // namespace autonomy_slam

#endif  // AUTONOMY_SLAM_STEREO_RECTIFIER_HPP_
