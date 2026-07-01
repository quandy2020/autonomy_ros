#ifndef AUTONOMY_SLAM_SLAM_CLI_HPP_
#define AUTONOMY_SLAM_SLAM_CLI_HPP_

#include <string>
#include <vector>

namespace autonomy_slam {
namespace cli {

struct RunSlamOptions {
    bool show_help = false;
    std::string vocab_file_path;
    std::string setting_file_path;
    std::string mask_img_path;
    std::string log_level = "info";
    std::string eval_log_dir;
    std::string map_db_path_in;
    std::string map_db_path_out;
    bool disable_mapping = false;
    bool temporal_mapping = false;
    bool rectify = false;
};

struct RunSlamOfflineOptions : RunSlamOptions {
    std::string bag_path;
    std::string camera_topic = "camera/image_raw";
    std::string left_topic = "camera/left/image_raw";
    std::string right_topic = "camera/right/image_raw";
    std::string color_topic = "camera/color/image_raw";
    std::string depth_topic = "camera/depth/image_raw";
    std::string bag_storage_id = "sqlite3";
    double start_offset = 0.0;
    bool no_sleep = false;
};

struct RunVideoSlamOptions : RunSlamOptions {
    std::string video_path;
    int frame_skip = 1;
    bool no_sleep = false;
};

void setup_glog(const std::string& log_level);

// Strip --ros-args and ROS-specific flags before getopt parsing.
std::vector<char*> filter_ros_arguments(int argc, char* argv[]);

bool parse_run_slam_options(int argc, char* argv[], RunSlamOptions& opts);
bool parse_run_slam_offline_options(int argc, char* argv[], RunSlamOfflineOptions& opts);
bool parse_run_video_slam_options(int argc, char* argv[], RunVideoSlamOptions& opts);

void print_run_slam_usage(const char* argv0);
void print_run_slam_offline_usage(const char* argv0);
void print_run_video_slam_usage(const char* argv0);

}  // namespace cli
}  // namespace autonomy_slam

#endif  // AUTONOMY_SLAM_SLAM_CLI_HPP_
