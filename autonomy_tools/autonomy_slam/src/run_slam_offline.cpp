#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"
#include <autonomy_slam.hpp>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <filesystem>

#include "slam_cli.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

void tracking(const std::shared_ptr<autonomy_slam::system>& slam_ros,
              const std::string& eval_log_dir,
              const std::string& map_db_path,
              const std::string& bag_path,
              const std::string& camera_topic,
              const std::string& left_topic,
              const std::string& right_topic,
              const std::string& color_topic,
              const std::string& depth_topic,
              const std::string& bag_storage_id,
              const double start_offset,
              const bool no_sleep) {
    auto& SLAM = slam_ros->slam_;

    RCLCPP_INFO(
        slam_ros->node_->get_logger(),
        "Visualization: use rviz2 (ros2 launch autonomy_slam rviz.launch.py)");

    RCLCPP_INFO_STREAM(slam_ros->node_->get_logger(), "Open " << bag_path << "(storage_id=" << bag_storage_id << ")");
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = bag_storage_id;
    rosbag2_cpp::Reader reader;
    reader.open(storage_options);
    auto metadata = reader.get_metadata();
    auto starting_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             metadata.starting_time.time_since_epoch())
                             .count()
                         + start_offset * 1e9;
    reader.seek(starting_time);
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que;
    double track_time;
    if (slam_ros->slam_->get_camera()->setup_type_ == atlas::camera::setup_type_t::Monocular) {
        auto mono = std::static_pointer_cast<autonomy_slam::mono>(slam_ros);
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == camera_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            mono->callback(que.front());
            const auto tp_2 = std::chrono::steady_clock::now();
            track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            if (!no_sleep && que.size() == 2) {
                const auto wait_time = (rclcpp::Time(que.back()->header.stamp) - rclcpp::Time(que.front()->header.stamp)).seconds() + track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }
            que.pop();
        }
    }
    else if (slam_ros->slam_->get_camera()->setup_type_ == atlas::camera::setup_type_t::Stereo) {
        auto stereo = std::static_pointer_cast<autonomy_slam::stereo>(slam_ros);
        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que_right;
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == left_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
                if (bag_message->topic_name == right_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que_right.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            stereo->left_sf_.cb(que.front());
            if (que_right.size() > 0) {
                stereo->right_sf_.cb(que_right.front());
                que_right.pop();
            }
            const auto tp_2 = std::chrono::steady_clock::now();
            track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            if (!no_sleep && que.size() == 2) {
                const auto wait_time = (rclcpp::Time(que.back()->header.stamp) - rclcpp::Time(que.front()->header.stamp)).seconds() + track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }
            que.pop();
        }
    }
    else if (slam_ros->slam_->get_camera()->setup_type_ == atlas::camera::setup_type_t::RGBD) {
        auto rgbd = std::static_pointer_cast<autonomy_slam::rgbd>(slam_ros);
        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que_depth;
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == color_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
                if (bag_message->topic_name == depth_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que_depth.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            rgbd->color_sf_.cb(que.front());
            if (que_depth.size() > 0) {
                rgbd->depth_sf_.cb(que_depth.front());
                que_depth.pop();
            }
            const auto tp_2 = std::chrono::steady_clock::now();
            track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            if (!no_sleep && que.size() == 2) {
                const auto wait_time = (rclcpp::Time(que.back()->header.stamp) - rclcpp::Time(que.front()->header.stamp)).seconds() + track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }
            que.pop();
        }
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam_ros->slam_->get_camera()->get_setup_type_string());
    }

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

    autonomy_slam::cli::RunSlamOfflineOptions opts;
    if (!autonomy_slam::cli::parse_run_slam_offline_options(cli_argc, cli_argv.data(), opts)) {
        autonomy_slam::cli::print_run_slam_offline_usage(cli_argv.empty() ? argv[0] : cli_argv[0]);
        return EXIT_FAILURE;
    }
    if (opts.show_help) {
        autonomy_slam::cli::print_run_slam_offline_usage(cli_argv.empty() ? argv[0] : cli_argv[0]);
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

    tracking(
        slam_ros,
        opts.eval_log_dir,
        opts.map_db_path_out,
        opts.bag_path,
        opts.camera_topic,
        opts.left_topic,
        opts.right_topic,
        opts.color_topic,
        opts.depth_topic,
        opts.bag_storage_id,
        opts.start_offset,
        opts.no_sleep);

    return EXIT_SUCCESS;
}
