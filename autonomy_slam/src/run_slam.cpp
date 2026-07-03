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

#include <rclcpp/executors/multi_threaded_executor.hpp>

void tracking(const std::shared_ptr<autonomy_slam::system>& slam_ros,
              const std::shared_ptr<rclcpp::Node>& ros_node,
              const std::string& eval_log_dir,
              const std::string& map_db_path) {
    auto& SLAM = slam_ros->slam_;

    RCLCPP_INFO(
        slam_ros->node_->get_logger(),
        "Visualization: use rviz2 (ros2 launch autonomy_slam rviz.launch.py)");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(ros_node);
    executor.spin();

    SLAM->shutdown();

    auto& track_times = slam_ros->track_times_;
    if (!eval_log_dir.empty()) {
        SLAM->save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        SLAM->save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        SLAM->save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        RCLCPP_DEBUG(slam_ros->node_->get_logger(), "Median tracking time: %f [s] ", track_times.at(track_times.size() / 2));
        RCLCPP_DEBUG(slam_ros->node_->get_logger(), "Mean tracking time: %f [s] ", total_track_time / track_times.size());
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto cli_argv = autonomy_slam::cli::filter_ros_arguments(argc, argv);
    const int cli_argc = static_cast<int>(cli_argv.size());

    autonomy_slam::cli::RunSlamOptions opts;
    if (!autonomy_slam::cli::parse_run_slam_options(cli_argc, cli_argv.data(), opts)) {
        autonomy_slam::cli::print_run_slam_usage(cli_argv.empty() ? argv[0] : cli_argv[0]);
        return EXIT_FAILURE;
    }
    if (opts.show_help) {
        autonomy_slam::cli::print_run_slam_usage(cli_argv.empty() ? argv[0] : cli_argv[0]);
        return EXIT_FAILURE;
    }

    autonomy_slam::cli::setup_glog(opts.log_level);

    std::shared_ptr<atlas::config> cfg;
    try {
        cfg = std::make_shared<atlas::config>(opts.setting_file_path);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    auto slam = std::make_shared<atlas::system>(cfg, opts.vocab_file_path);
    bool need_initialize = true;
    if (!opts.map_db_path_in.empty()) {
        need_initialize = false;
        const auto path = std::filesystem::path(opts.map_db_path_in);
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path.string());
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                slam->load_map_database((path.parent_path() / map_path).string());
            }
        }
        else {
            slam->load_map_database(path.string());
        }
    }
    slam->startup(need_initialize);
    if (opts.disable_mapping) {
        slam->disable_mapping_module();
    }
    else if (opts.temporal_mapping) {
        slam->enable_temporal_mapping();
        slam->disable_loop_detector();
    }

    auto node = std::make_shared<rclcpp::Node>("run_slam");
    std::shared_ptr<autonomy_slam::system> slam_ros;
    if (slam->get_camera()->setup_type_ == atlas::camera::setup_type_t::Monocular) {
        slam_ros = std::make_shared<autonomy_slam::mono>(slam, node.get(), opts.mask_img_path);
    }
    else if (slam->get_camera()->setup_type_ == atlas::camera::setup_type_t::Stereo) {
        auto rectifier = opts.rectify
            ? std::make_shared<autonomy_slam::StereoRectifier>(cfg, slam->get_camera())
            : nullptr;
        slam_ros = std::make_shared<autonomy_slam::stereo>(slam, node.get(), opts.mask_img_path, rectifier);
    }
    else if (slam->get_camera()->setup_type_ == atlas::camera::setup_type_t::RGBD) {
        slam_ros = std::make_shared<autonomy_slam::rgbd>(slam, node.get(), opts.mask_img_path);
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

    tracking(slam_ros, node, opts.eval_log_dir, opts.map_db_path_out);

    return EXIT_SUCCESS;
}
