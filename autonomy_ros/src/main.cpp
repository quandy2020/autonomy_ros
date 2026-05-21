#include <memory>

#include "autonomy_ros/autonomy_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_ros::AutonomyNode>());
  rclcpp::shutdown();
  return 0;
}
