#include "stereo_rectifier.hpp"

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"

#include <opencv2/imgproc.hpp>
#include <cstring>
#include <stdexcept>

namespace atlas = autonomy::localization::atlas;

namespace autonomy_slam {

StereoRectifier::StereoRectifier(const std::shared_ptr<atlas::config>& cfg,
                                 atlas::camera::base* camera) {
    if (camera->setup_type_ != atlas::camera::setup_type_t::Stereo) {
        throw std::runtime_error("StereoRectifier requires camera setup 'stereo'");
    }
    if (camera->model_type_ != atlas::camera::model_type_t::Perspective) {
        throw std::runtime_error("StereoRectifier requires camera model 'perspective'");
    }

    const auto& yaml_node = atlas::util::yaml_optional_ref(cfg->yaml_node_, "StereoRectifier");
    const cv::Size img_size(camera->cols_, camera->rows_);

    const auto K_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["K_left"].as<std::vector<double>>());
    const auto K_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["K_right"].as<std::vector<double>>());
    const auto R_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["R_left"].as<std::vector<double>>());
    const auto R_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["R_right"].as<std::vector<double>>());

    const auto D_l_vec = yaml_node["D_left"].as<std::vector<double>>();
    const auto D_r_vec = yaml_node["D_right"].as<std::vector<double>>();
    const auto D_l = parse_vector_as_mat(cv::Size(1, static_cast<int>(D_l_vec.size())), D_l_vec);
    const auto D_r = parse_vector_as_mat(cv::Size(1, static_cast<int>(D_r_vec.size())), D_r_vec);

    const auto& K_rect = static_cast<atlas::camera::perspective*>(camera)->cv_cam_matrix_;

    cv::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
}

void StereoRectifier::rectify(const cv::Mat& in_left, const cv::Mat& in_right,
                              cv::Mat& out_left, cv::Mat& out_right) const {
    cv::remap(in_left, out_left, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
    cv::remap(in_right, out_right, undist_map_x_r_, undist_map_y_r_, cv::INTER_LINEAR);
}

cv::Mat StereoRectifier::parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec) {
    const auto expected = static_cast<size_t>(shape.width * shape.height);
    if (vec.size() != expected) {
        throw std::runtime_error(
            "StereoRectifier: expected " + std::to_string(expected) + " values, got "
            + std::to_string(vec.size()));
    }
    cv::Mat mat(shape.height, shape.width, CV_64F);
    std::memcpy(mat.ptr<double>(), vec.data(), expected * sizeof(double));
    return mat;
}

}  // namespace autonomy_slam
