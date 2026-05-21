#include "autonomy_ros/visualization/visualizer.hpp"

namespace autonomy_ros::visualization
{

Visualizer::Visualizer(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter("visualization.frame_id", frame_id_);
  frame_id_ = node_.get_parameter("visualization.frame_id").as_string();
}

void Visualizer::start()
{
  marker_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization/markers", 10);
  path_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
    "plan", 10, std::bind(&Visualizer::onPath, this, std::placeholders::_1));
  goal_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10, std::bind(&Visualizer::onGoal, this, std::placeholders::_1));
  RCLCPP_INFO(node_.get_logger(), "[visualization] ready (frame=%s)", frame_id_.c_str());
}

void Visualizer::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  latest_path_ = *msg;
  publishMarkers();
}

void Visualizer::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_goal_ = *msg;
  publishMarkers();
}

void Visualizer::publishMarkers()
{
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = node_.now();

  if (latest_path_ && latest_path_->poses.size() >= 2) {
    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = frame_id_;
    line.ns = "plan";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.03;
    line.color.r = 0.1f;
    line.color.g = 0.8f;
    line.color.b = 0.2f;
    line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;
    for (const auto & ps : latest_path_->poses) {
      line.points.push_back(ps.pose.position);
    }
    array.markers.push_back(line);
  }

  if (latest_goal_) {
    visualization_msgs::msg::Marker goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = frame_id_;
    goal.ns = "goal";
    goal.id = 1;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.action = visualization_msgs::msg::Marker::ADD;
    goal.pose = latest_goal_->pose;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.2;
    goal.color.r = 1.0f;
    goal.color.g = 0.2f;
    goal.color.b = 0.1f;
    goal.color.a = 1.0f;
    array.markers.push_back(goal);
  }

  if (!array.markers.empty()) {
    marker_pub_->publish(array);
  }
}

}  // namespace autonomy_ros::visualization
