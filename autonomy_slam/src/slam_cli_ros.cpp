#include "slam_cli.hpp"

#include <rclcpp/utilities.hpp>

#include <string>
#include <vector>

namespace autonomy_slam {
namespace cli {

std::vector<char*> filter_ros_arguments(int argc, char* argv[]) {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    static thread_local std::vector<std::string> storage;
    static thread_local std::vector<char*> pointers;
    storage.assign(args.begin(), args.end());
    pointers.clear();
    pointers.reserve(storage.size());
    for (auto& arg : storage) {
        pointers.push_back(arg.data());
    }
    return pointers;
}

}  // namespace cli
}  // namespace autonomy_slam
