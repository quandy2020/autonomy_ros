#ifndef ROS2_PEDESTRIAN_SIMULATOR_H
#define ROS2_PEDESTRIAN_SIMULATOR_H

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <pedsim_msgs/msg/obstacle_array.hpp>
#include <pedsim_msgs/msg/gaussian.hpp>

class Prediction;
class PedestrianSimulator;

class ROSPedestrianSimulator : public rclcpp::Node
{
public:
    ROSPedestrianSimulator();

    void ResetCallback(std_msgs::msg::Empty::SharedPtr msg);
    void ResetToStartCallback(std_msgs::msg::Empty::SharedPtr msg);
    void VehicleVelocityCallback(geometry_msgs::msg::Twist::SharedPtr msg); /* For pretending that the vehicle is moving! */

    /** @brief Shift the origin to the origin of the reference path */
    void OriginCallback(nav_msgs::msg::Path::SharedPtr msg);
    void RobotStateCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void startCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                       std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void pauseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                       std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void unpauseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res);

    void SettingNCallback(std_msgs::msg::Int32::SharedPtr msg);
    void SettingdtCallback(std_msgs::msg::Float32::SharedPtr msg);
    void SettingHzCallback(std_msgs::msg::Float32::SharedPtr msg);

    void beginSimulation();

    void Loop();

    void publishPedestrianTransforms(const std::vector<Prediction> &pedestrians);

private:
    std::unique_ptr<PedestrianSimulator> _simulator;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    void InitializePublishersAndSubscribers();

    derived_object_msgs::msg::ObjectArray PredictionsToObjectArray(const std::vector<Prediction> &predictions);
    pedsim_msgs::msg::ObstacleArray PredictionsToObstacleArray(const std::vector<Prediction> &predictions);

    bool set_N_{false}, set_dt_{false}, set_hz_{false};
    bool paused_{false};
    bool publish_pedestrian_tf_{false};

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<derived_object_msgs::msg::ObjectArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<pedsim_msgs::msg::ObstacleArray>::SharedPtr obstacle_trajectory_prediction_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_server;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_server_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr unpause_server_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_to_start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_origin_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr setting_N_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setting_dt_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setting_hz_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_sub_;
};

#endif // ROS2_PEDESTRIAN_SIMULATOR_H
