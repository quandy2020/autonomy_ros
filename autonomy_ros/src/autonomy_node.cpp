#include "autonomy_ros/autonomy_node.hpp"

namespace autonomy_ros
{

AutonomyNode::AutonomyNode()
: Node("autonomy_node")
{
  declare_parameter<bool>("use_sim_time", false);
  declare_parameter<bool>("enable_map", enable_map_);
  declare_parameter<bool>("enable_planner", enable_planner_);
  declare_parameter<bool>("enable_controller", enable_controller_);
  declare_parameter<bool>("enable_visualization", enable_visualization_);
  declare_parameter<bool>("enable_command", enable_command_);

  enable_map_ = get_parameter("enable_map").as_bool();
  enable_planner_ = get_parameter("enable_planner").as_bool();
  enable_controller_ = get_parameter("enable_controller").as_bool();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  enable_command_ = get_parameter("enable_command").as_bool();

  task_manager_ = std::make_unique<task::TaskManager>(*this);
  task_manager_->start();

  if (enable_map_) {
    map_manager_ = std::make_unique<map::MapManager>(*this);
    map_manager_->start();
  }

  if (enable_planner_) {
    if (!map_manager_) {
      map_manager_ = std::make_unique<map::MapManager>(*this);
      map_manager_->start();
    }
    planner_ = std::make_unique<planner::Planner>(*this, *map_manager_);
    planner_->start();
  }

  if (enable_controller_) {
    controller_ = std::make_unique<controller::Controller>(*this);
    controller_->start();
  }

  if (enable_visualization_) {
    visualizer_ = std::make_unique<visualization::Visualizer>(*this);
    visualizer_->start();
  }

  if (enable_command_) {
    if (!planner_ || !controller_) {
      RCLCPP_WARN(
        get_logger(),
        "enable_command requires enable_planner and enable_controller; command disabled");
    } else {
      command_interface_ = std::make_unique<command::CommandInterface>(
        *this, *task_manager_, *planner_, *controller_);
      command_interface_->start();
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "autonomy_ros: map=%s planner=%s controller=%s visualization=%s command=%s",
    enable_map_ ? "on" : "off",
    enable_planner_ ? "on" : "off",
    enable_controller_ ? "on" : "off",
    enable_visualization_ ? "on" : "off",
    (enable_command_ && command_interface_) ? "on" : "off");
}

}  // namespace autonomy_ros
