#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"
#include <autonomy_slam.hpp>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <filesystem>

#include "slam_cli.hpp"

namespace autonomy_slam {

class System : public rclcpp::Node {
public:
    System(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    System(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~System();

    std::shared_ptr<autonomy_slam::system> slam_ros_;
    std::shared_ptr<atlas::system> slam_;
    std::shared_ptr<atlas::config> cfg_;
    std::string map_db_path_out_;
};

System::System(
    const rclcpp::NodeOptions& options)
    : System("", options) {}

System::System(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
    : Node("run_slam", name_space, options) {
    std::string vocab_file_path = declare_parameter("vocab_file_path", "");
    std::string setting_file_path = declare_parameter("setting_file_path", "");
    std::string log_level = declare_parameter("log_level", "info");
    std::string map_db_path_in = declare_parameter("map_db_path_in", "");
    map_db_path_out_ = declare_parameter("map_db_path_out", "");
    bool disable_mapping = declare_parameter("disable_mapping", false);
    bool temporal_mapping = declare_parameter("temporal_mapping", false);

    if (vocab_file_path.empty() || setting_file_path.empty()) {
        RCLCPP_FATAL(get_logger(), "Invalid parameter");
        return;
    }

    autonomy_slam::cli::setup_glog(log_level);

    try {
        cfg_ = std::make_shared<atlas::config>(setting_file_path);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), e.what());
        return;
    }

    slam_ = std::make_shared<atlas::system>(cfg_, vocab_file_path);
    bool need_initialize = true;
    if (!map_db_path_in.empty()) {
        need_initialize = false;
        const auto path = std::filesystem::path(map_db_path_in);
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path.string());
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                slam_->load_map_database((path.parent_path() / map_path).string());
            }
        }
        else {
            slam_->load_map_database(path.string());
        }
    }
    slam_->startup(need_initialize);
    if (disable_mapping) {
        slam_->disable_mapping_module();
    }
    else if (temporal_mapping) {
        slam_->enable_temporal_mapping();
        slam_->disable_loop_detector();
    }

    if (slam_->get_camera()->setup_type_ == atlas::camera::setup_type_t::Monocular) {
        slam_ros_ = std::make_shared<autonomy_slam::mono>(slam_, this, "");
    }
    else {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid setup type: " << slam_->get_camera()->get_setup_type_string());
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Visualization: use rviz2 (ros2 launch autonomy_slam rviz.launch.py)");
}

System::~System() {
    slam_->shutdown();

    if (!map_db_path_out_.empty()) {
        slam_->save_map_database(map_db_path_out_);
    }
}

} // namespace autonomy_slam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autonomy_slam::System)
