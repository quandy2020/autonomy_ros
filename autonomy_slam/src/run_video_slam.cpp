#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "slam_cli.hpp"

namespace atlas = autonomy::localization::atlas;

namespace {

void load_map_if_needed(const std::shared_ptr<atlas::system>& slam,
                        const std::string& map_db_path_in) {
    if (map_db_path_in.empty()) {
        return;
    }
    const auto path = std::filesystem::path(map_db_path_in);
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

void save_outputs(const std::shared_ptr<atlas::system>& slam,
                  const std::string& eval_log_dir,
                  const std::string& map_db_path_out,
                  const std::vector<double>& track_times) {
    slam->shutdown();

    if (!eval_log_dir.empty()) {
        slam->save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        slam->save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
        for (const auto t : track_times) {
            ofs << t << '\n';
        }
    }

    if (!map_db_path_out.empty()) {
        slam->save_map_database(map_db_path_out);
    }

    if (!track_times.empty()) {
        std::vector<double> sorted = track_times;
        std::sort(sorted.begin(), sorted.end());
        const auto total = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        std::cout << "median tracking time: " << sorted.at(sorted.size() / 2) << "[s]\n";
        std::cout << "mean tracking time: " << total / sorted.size() << "[s]\n";
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    autonomy_slam::cli::RunVideoSlamOptions opts;
    if (!autonomy_slam::cli::parse_run_video_slam_options(argc, argv, opts)) {
        autonomy_slam::cli::print_run_video_slam_usage(argv[0]);
        return EXIT_FAILURE;
    }
    if (opts.show_help) {
        autonomy_slam::cli::print_run_video_slam_usage(argv[0]);
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
    load_map_if_needed(slam, opts.map_db_path_in);

    const bool need_initialize = opts.map_db_path_in.empty();
    slam->startup(need_initialize);
    if (opts.disable_mapping) {
        slam->disable_mapping_module();
    }
    else if (opts.temporal_mapping) {
        slam->enable_temporal_mapping();
        slam->disable_loop_detector();
    }

    if (slam->get_camera()->setup_type_ != atlas::camera::setup_type_t::Monocular) {
        std::cerr << "run_video_slam currently supports monocular config only\n";
        return EXIT_FAILURE;
    }

    cv::VideoCapture cap(opts.video_path);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open video: " << opts.video_path << std::endl;
        return EXIT_FAILURE;
    }

    const cv::Mat mask = opts.mask_img_path.empty()
        ? cv::Mat{}
        : cv::imread(opts.mask_img_path, cv::IMREAD_GRAYSCALE);

    const double fps = slam->get_camera()->fps_;
    const double frame_period = (fps > 0.0) ? (1.0 / fps) : (1.0 / 30.0);
    std::vector<double> track_times;
    double timestamp = 0.0;

    cv::Mat frame;
    while (cap.isOpened()) {
        for (int skip = 0; skip < opts.frame_skip; ++skip) {
            if (!cap.read(frame) || frame.empty()) {
                save_outputs(slam, opts.eval_log_dir, opts.map_db_path_out, track_times);
                return EXIT_SUCCESS;
            }
        }

        const auto t1 = std::chrono::steady_clock::now();
        slam->feed_monocular_frame(frame, timestamp, mask);
        const auto t2 = std::chrono::steady_clock::now();
        const double track_time =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        track_times.push_back(track_time);

        if (!opts.no_sleep) {
            const double wait = frame_period * opts.frame_skip - track_time;
            if (wait > 0.0) {
                std::this_thread::sleep_for(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::duration<double>(wait)));
            }
        }

        timestamp += frame_period * opts.frame_skip;
    }

    save_outputs(slam, opts.eval_log_dir, opts.map_db_path_out, track_times);
    return EXIT_SUCCESS;
}
