#include "slam_cli.hpp"

#include <getopt.h>
#include <glog/logging.h>

#include <cstdlib>
#include <iostream>
#include <vector>

namespace autonomy_slam {
namespace cli {

void setup_glog(const std::string& log_level) {
    google::InitGoogleLogging("autonomy_slam");
    FLAGS_logtostderr = 1;
    if (log_level == "warn" || log_level == "warning") {
        FLAGS_minloglevel = google::WARNING;
    }
    else if (log_level == "error" || log_level == "err") {
        FLAGS_minloglevel = google::ERROR;
    }
    else {
        FLAGS_minloglevel = google::INFO;
    }
}

namespace {

bool parse_common_options(int argc, char* argv[], RunSlamOptions& opts, const char* short_opts,
                          const option* long_opts) {
    optind = 1;
    int c = 0;
    while ((c = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1) {
        switch (c) {
        case 'h':
            opts.show_help = true;
            return true;
        case 'v':
            opts.vocab_file_path = optarg;
            break;
        case 'c':
            opts.setting_file_path = optarg;
            break;
        case 'i':
            opts.map_db_path_in = optarg;
            break;
        case 'o':
            opts.map_db_path_out = optarg;
            break;
        case 'r':
            opts.rectify = true;
            break;
        case 1000:
            opts.mask_img_path = optarg;
            break;
        case 1001:
            opts.log_level = optarg;
            break;
        case 1002:
            opts.eval_log_dir = optarg;
            break;
        case 1003:
            opts.disable_mapping = true;
            break;
        case 1004:
            opts.temporal_mapping = true;
            break;
        default:
            return false;
        }
    }
    return true;
}

const option kRunSlamLongOpts[] = {
    {"help", no_argument, nullptr, 'h'},
    {"vocab", required_argument, nullptr, 'v'},
    {"config", required_argument, nullptr, 'c'},
    {"mask", required_argument, nullptr, 1000},
    {"log-level", required_argument, nullptr, 1001},
    {"eval-log-dir", required_argument, nullptr, 1002},
    {"map-db-in", required_argument, nullptr, 'i'},
    {"map-db-out", required_argument, nullptr, 'o'},
    {"disable-mapping", no_argument, nullptr, 1003},
    {"temporal-mapping", no_argument, nullptr, 1004},
    {"rectify", no_argument, nullptr, 'r'},
    {nullptr, 0, nullptr, 0},
};

const option kRunSlamOfflineLongOpts[] = {
    {"help", no_argument, nullptr, 'h'},
    {"bag", required_argument, nullptr, 'b'},
    {"camera", required_argument, nullptr, 1006},
    {"left", required_argument, nullptr, 1007},
    {"right", required_argument, nullptr, 1008},
    {"color", required_argument, nullptr, 1009},
    {"depth", required_argument, nullptr, 1010},
    {"storage-id", required_argument, nullptr, 1011},
    {"start-offset", required_argument, nullptr, 1012},
    {"no-sleep", no_argument, nullptr, 1013},
    {"vocab", required_argument, nullptr, 'v'},
    {"config", required_argument, nullptr, 'c'},
    {"mask", required_argument, nullptr, 1000},
    {"log-level", required_argument, nullptr, 1001},
    {"eval-log-dir", required_argument, nullptr, 1002},
    {"map-db-in", required_argument, nullptr, 'i'},
    {"map-db-out", required_argument, nullptr, 'o'},
    {"disable-mapping", no_argument, nullptr, 1003},
    {"temporal-mapping", no_argument, nullptr, 1004},
    {"rectify", no_argument, nullptr, 'r'},
    {nullptr, 0, nullptr, 0},
};

}  // namespace

void print_run_slam_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " -v/--vocab VOCAB -c/--config CONFIG [options]\n"
        << "  -h, --help              Show help\n"
        << "  -v, --vocab PATH        Vocabulary file\n"
        << "  -c, --config PATH       Config YAML\n"
        << "      --mask PATH         Mask image\n"
        << "      --log-level LEVEL   trace|debug|info|warn|error (default: info)\n"
        << "      --eval-log-dir DIR  Trajectory output directory\n"
        << "  -i, --map-db-in PATH    Load map database\n"
        << "  -o, --map-db-out PATH   Save map database\n"
        << "      --disable-mapping   Disable mapping module\n"
        << "      --temporal-mapping  Enable temporal mapping\n"
        << "  -r, --rectify           Rectify stereo images\n"
        << "\nVisualization: ros2 launch autonomy_slam rviz.launch.py\n";
}

void print_run_slam_offline_usage(const char* argv0) {
    print_run_slam_usage(argv0);
    std::cerr
        << "Offline options:\n"
        << "  -b, --bag PATH          Rosbag2 directory\n"
        << "      --camera TOPIC      Monocular image topic (default: camera/image_raw)\n"
        << "      --left TOPIC        Left image topic\n"
        << "      --right TOPIC       Right image topic\n"
        << "      --color TOPIC       RGBD color topic\n"
        << "      --depth TOPIC       RGBD depth topic\n"
        << "      --storage-id ID     Rosbag2 storage id (default: sqlite3)\n"
        << "      --start-offset SEC  Bag start offset\n"
        << "      --no-sleep          Do not wait for real-time playback\n";
}

bool parse_run_slam_options(int argc, char* argv[], RunSlamOptions& opts) {
    if (!parse_common_options(argc, argv, opts, "hv:c:i:o:r:", kRunSlamLongOpts)) {
        return false;
    }
    if (opts.show_help) {
        return true;
    }
    return !opts.vocab_file_path.empty() && !opts.setting_file_path.empty();
}

bool parse_run_slam_offline_options(int argc, char* argv[], RunSlamOfflineOptions& opts) {
    optind = 1;
    int c = 0;
    while ((c = getopt_long(argc, argv, "hb:v:c:i:o:r:", kRunSlamOfflineLongOpts, nullptr)) != -1) {
        switch (c) {
        case 'h':
            opts.show_help = true;
            return true;
        case 'b':
            opts.bag_path = optarg;
            break;
        case 'v':
            opts.vocab_file_path = optarg;
            break;
        case 'c':
            opts.setting_file_path = optarg;
            break;
        case 'i':
            opts.map_db_path_in = optarg;
            break;
        case 'o':
            opts.map_db_path_out = optarg;
            break;
        case 'r':
            opts.rectify = true;
            break;
        case 1000:
            opts.mask_img_path = optarg;
            break;
        case 1001:
            opts.log_level = optarg;
            break;
        case 1002:
            opts.eval_log_dir = optarg;
            break;
        case 1003:
            opts.disable_mapping = true;
            break;
        case 1004:
            opts.temporal_mapping = true;
            break;
        case 1006:
            opts.camera_topic = optarg;
            break;
        case 1007:
            opts.left_topic = optarg;
            break;
        case 1008:
            opts.right_topic = optarg;
            break;
        case 1009:
            opts.color_topic = optarg;
            break;
        case 1010:
            opts.depth_topic = optarg;
            break;
        case 1011:
            opts.bag_storage_id = optarg;
            break;
        case 1012:
            opts.start_offset = std::stod(optarg);
            break;
        case 1013:
            opts.no_sleep = true;
            break;
        default:
            return false;
        }
    }
    if (opts.show_help) {
        return true;
    }
    return !opts.vocab_file_path.empty() && !opts.setting_file_path.empty();
}

const option kRunVideoSlamLongOpts[] = {
    {"help", no_argument, nullptr, 'h'},
    {"vocab", required_argument, nullptr, 'v'},
    {"video", required_argument, nullptr, 'm'},
    {"config", required_argument, nullptr, 'c'},
    {"mask", required_argument, nullptr, 1000},
    {"frame-skip", required_argument, nullptr, 1005},
    {"no-sleep", no_argument, nullptr, 1013},
    {"log-level", required_argument, nullptr, 1001},
    {"eval-log-dir", required_argument, nullptr, 1002},
    {"map-db-in", required_argument, nullptr, 'i'},
    {"map-db-out", required_argument, nullptr, 'o'},
    {"disable-mapping", no_argument, nullptr, 1003},
    {"temporal-mapping", no_argument, nullptr, 1004},
    {nullptr, 0, nullptr, 0},
};

void print_run_video_slam_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " -v/--vocab VOCAB -m/--video VIDEO -c/--config CONFIG [options]\n"
        << "  -h, --help              Show help\n"
        << "  -v, --vocab PATH        Vocabulary file (orb_vocab.fbow)\n"
        << "  -m, --video PATH        Video file path\n"
        << "  -c, --config PATH       Config YAML\n"
        << "      --mask PATH         Mask image\n"
        << "      --frame-skip N      Frame interval (default: 1)\n"
        << "      --no-sleep          Process frames as fast as possible\n"
        << "      --log-level LEVEL   trace|debug|info|warn|error (default: info)\n"
        << "      --eval-log-dir DIR  Trajectory output directory\n"
        << "  -i, --map-db-in PATH    Load map database\n"
        << "  -o, --map-db-out PATH   Save map database\n"
        << "      --disable-mapping   Disable mapping module (localization)\n"
        << "      --temporal-mapping  Temporal mapping localization\n"
        << "\nSee: https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html\n";
}

bool parse_run_video_slam_options(int argc, char* argv[], RunVideoSlamOptions& opts) {
    optind = 1;
    int c = 0;
    while ((c = getopt_long(argc, argv, "hm:v:c:i:o:", kRunVideoSlamLongOpts, nullptr)) != -1) {
        switch (c) {
        case 'h':
            opts.show_help = true;
            return true;
        case 'm':
            opts.video_path = optarg;
            break;
        case 'v':
            opts.vocab_file_path = optarg;
            break;
        case 'c':
            opts.setting_file_path = optarg;
            break;
        case 'i':
            opts.map_db_path_in = optarg;
            break;
        case 'o':
            opts.map_db_path_out = optarg;
            break;
        case 1000:
            opts.mask_img_path = optarg;
            break;
        case 1001:
            opts.log_level = optarg;
            break;
        case 1002:
            opts.eval_log_dir = optarg;
            break;
        case 1003:
            opts.disable_mapping = true;
            break;
        case 1004:
            opts.temporal_mapping = true;
            break;
        case 1005:
            opts.frame_skip = std::max(1, std::stoi(optarg));
            break;
        case 1013:
            opts.no_sleep = true;
            break;
        default:
            return false;
        }
    }
    if (opts.show_help) {
        return true;
    }
    return !opts.vocab_file_path.empty() && !opts.setting_file_path.empty()
           && !opts.video_path.empty();
}

}  // namespace cli
}  // namespace autonomy_slam
