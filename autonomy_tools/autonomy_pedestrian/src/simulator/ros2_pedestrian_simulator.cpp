
#include <pedestrian_simulator/ros2_pedestrian_simulator.h>

#include <pedestrian_simulator/pedestrian_simulator.h>

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/configuration.h>

#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>
#include <ros_tools/logging.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

ROSPedestrianSimulator::ROSPedestrianSimulator()
    : rclcpp::Node("pedestrian_simulator")
{
    STATIC_NODE_POINTER.init(this);

    declare_parameter<bool>("auto_start", true);
    declare_parameter<bool>("publish_pedestrian_tf", false);
    publish_pedestrian_tf_ = get_parameter("publish_pedestrian_tf").as_bool();
    if (publish_pedestrian_tf_)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    InitializePublishersAndSubscribers();

    VISUALS.init(this);
    _simulator = std::make_unique<PedestrianSimulator>();

    if (get_parameter("auto_start").as_bool()) {
        start_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (start_timer_) {
                    start_timer_->cancel();
                    start_timer_.reset();
                }
                beginSimulation();
            });
    }
}

void ROSPedestrianSimulator::InitializePublishersAndSubscribers()
{

    obstacle_pub_ = this->create_publisher<derived_object_msgs::msg::ObjectArray>(
        "/pedestrian_simulator/pedestrians", 1);
    obstacle_trajectory_prediction_pub_ = this->create_publisher<pedsim_msgs::msg::ObstacleArray>(
        "/pedestrian_simulator/trajectory_predictions", 1);

    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/pedestrian_simulator/reset", 1,
        std::bind(&ROSPedestrianSimulator::ResetCallback, this, std::placeholders::_1));
    reset_to_start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/pedestrian_simulator/reset_to_start", 1,
        std::bind(&ROSPedestrianSimulator::ResetToStartCallback, this, std::placeholders::_1));
    robot_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_state", 1,
        std::bind(&ROSPedestrianSimulator::RobotStateCallback, this, std::placeholders::_1));
    setting_N_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/pedestrian_simulator/horizon", 1,
        std::bind(&ROSPedestrianSimulator::SettingNCallback, this, std::placeholders::_1));
    setting_dt_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/pedestrian_simulator/integrator_step", 1,
        std::bind(&ROSPedestrianSimulator::SettingdtCallback, this, std::placeholders::_1));
    setting_hz_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/pedestrian_simulator/clock_frequency", 1,
        std::bind(&ROSPedestrianSimulator::SettingHzCallback, this, std::placeholders::_1));

    if (CONFIG.use_path_origin_)
    {
        path_origin_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/roadmap/reference", 1,
            std::bind(&ROSPedestrianSimulator::OriginCallback, this, std::placeholders::_1));
    }

    start_server = this->create_service<std_srvs::srv::Empty>(
        "/pedestrian_simulator/start",
        std::bind(&ROSPedestrianSimulator::startCallback, this, std::placeholders::_1, std::placeholders::_2));
    pause_server_ = this->create_service<std_srvs::srv::Empty>(
        "/pedestrian_simulator/pause_simulation",
        std::bind(&ROSPedestrianSimulator::pauseCallback, this, std::placeholders::_1, std::placeholders::_2));
    unpause_server_ = this->create_service<std_srvs::srv::Empty>(
        "/pedestrian_simulator/unpause_simulation",
        std::bind(&ROSPedestrianSimulator::unpauseCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void ROSPedestrianSimulator::startCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                           std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    beginSimulation();
}

void ROSPedestrianSimulator::pauseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                           std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    paused_ = true;
    LOG_INFO("Pedestrian Simulator: paused");
}

void ROSPedestrianSimulator::unpauseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                             std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    paused_ = false;
    LOG_INFO("Pedestrian Simulator: unpaused");
}

void ROSPedestrianSimulator::beginSimulation()
{
    if (timer_) {
        return;
    }

    if (CONFIG.debug_output_)
        LOG_WARN("Pedestrian Simulator: Start requested.");

    const bool ready_n = set_N_ || CONFIG.horizon_N_ > 0;
    const bool ready_dt = set_dt_ || CONFIG.prediction_step_ > 0.0;
    const bool ready_hz = set_hz_ || CONFIG.update_frequency_ > 0.0;

    if (!ready_n) {
        LOG_INFO("Pedestrian Simulator: Horizon not set yet, ignoring start");
        return;
    }
    if (!ready_dt) {
        LOG_INFO("Pedestrian Simulator: prediction step not set yet, ignoring start");
        return;
    }
    if (!ready_hz) {
        LOG_INFO("Pedestrian Simulator: Update frequency not set yet, ignoring start");
        return;
    }

    LOG_INFO("Pedestrian Simulator [horizon = " << CONFIG.horizon_N_ << ", dt = "
                                                << CONFIG.prediction_step_ << ", Hz = "
                                                << CONFIG.update_frequency_ << "]");

    timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration::from_seconds(1.0 / CONFIG.update_frequency_),
        std::bind(&ROSPedestrianSimulator::Loop, this));
}

void ROSPedestrianSimulator::ResetCallback(std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    // LOG_INFO("PedestrianSimulator: Reset callback");
    _simulator->Reset();
}

void ROSPedestrianSimulator::ResetToStartCallback(std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    _simulator->ResetToStart();
}
void ROSPedestrianSimulator::RobotStateCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const double yaw = RosTools::quaternionToAngle(msg->pose.orientation);
    _simulator->SetRobotState(RobotState(
        Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y),
        yaw,
        0.));
}

void ROSPedestrianSimulator::VehicleVelocityCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
    (void)msg;
    // _simulator->vehicle_frame_(0) += msg.linear.x * CONFIG.delta_t_;
    // _simulator->vehicle_frame_(1) += msg.linear.y * CONFIG.delta_t_;
}

void ROSPedestrianSimulator::OriginCallback(nav_msgs::msg::Path::SharedPtr msg)
{
    (void)msg;
    // double angle = std::atan2(msg.poses[1].pose.position.y - msg.poses[0].pose.position.y, msg.poses[1].pose.position.x - msg.poses[0].pose.position.x);

    // // Update if the path is new
    // if (origin_.position.x != msg.poses[0].pose.position.x || origin_.position.y != msg.poses[0].pose.position.y || RosTools::rotationMatrixFromHeading(-angle) != CONFIG.origin_R_)
    // {
    //     // We save the rotation of the origin to also move in the direction of the origin frame
    //     CONFIG.origin_R_ = RosTools::rotationMatrixFromHeading(-angle);

    //     for (auto &ped : pedestrians_) // Shift the peds start and goal to the origin
    //     {
    //         ped->start_.UndoTransform(origin_);
    //         ped->start_.Transform(msg.poses[0].pose, angle);

    //         ped->goal_.UndoTransform(origin_);
    //         ped->goal_.Transform(msg.poses[0].pose, angle);

    //         ped->Reset();
    //     }
    //     origin_ = msg.poses[0].pose;
    // }
}

void ROSPedestrianSimulator::SettingNCallback(std_msgs::msg::Int32::SharedPtr msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: horizon = " << (int)msg->data;);

    CONFIG.horizon_N_ = (int)msg->data;
    set_N_ = true;
}

void ROSPedestrianSimulator::SettingdtCallback(std_msgs::msg::Float32::SharedPtr msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: integrator_step = " << (double)msg->data;);

    CONFIG.prediction_step_ = (double)msg->data;
    set_dt_ = true;
}

void ROSPedestrianSimulator::SettingHzCallback(std_msgs::msg::Float32::SharedPtr msg)
{
    if (CONFIG.debug_output_)
        LOG_INFO("Pedestrian Simulator: clock_frequency = " << (double)msg->data;);

    CONFIG.update_frequency_ = (double)msg->data;
    CONFIG.delta_t_ = 1.0 / CONFIG.update_frequency_;

    set_hz_ = true;
}

void ROSPedestrianSimulator::Loop()
{
    if (paused_)
        return;

    _simulator->Loop();

    std::vector<Prediction> obstacles = _simulator->GetPedestrians();
    derived_object_msgs::msg::ObjectArray obstacles_msg = PredictionsToObjectArray(obstacles);
    obstacle_pub_->publish(obstacles_msg);

    std::vector<Prediction> predictions = _simulator->GetPredictions();
    pedsim_msgs::msg::ObstacleArray predictions_msg = PredictionsToObstacleArray(predictions);
    obstacle_trajectory_prediction_pub_->publish(predictions_msg);

    if (publish_pedestrian_tf_)
        publishPedestrianTransforms(obstacles);
}

void ROSPedestrianSimulator::publishPedestrianTransforms(const std::vector<Prediction> &pedestrians)
{
    if (!tf_broadcaster_)
        return;

    const auto stamp = this->get_clock()->now();
    for (const auto &pedestrian : pedestrians)
    {
        if (pedestrian.pos.empty())
            continue;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "pedestrian_" + std::to_string(pedestrian.id);
        transform.transform.translation.x = pedestrian.pos[0](0);
        transform.transform.translation.y = pedestrian.pos[0](1);
        transform.transform.translation.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, pedestrian.angle.empty() ? 0.0 : pedestrian.angle[0]);
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(transform);
    }
}

pedsim_msgs::msg::ObstacleArray ROSPedestrianSimulator::PredictionsToObstacleArray(const std::vector<Prediction> &predictions)
{
    pedsim_msgs::msg::ObstacleArray obstacle_array_msg;

    for (auto &prediction : predictions)
    {
        obstacle_array_msg.obstacles.emplace_back();
        obstacle_array_msg.obstacles.back().id = prediction.id;
        obstacle_array_msg.obstacles.back().gaussians.emplace_back();
        obstacle_array_msg.obstacles.back().pose.position.x = prediction.pos[0](0);
        obstacle_array_msg.obstacles.back().pose.position.y = prediction.pos[0](1);
        obstacle_array_msg.obstacles.back().pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);

        auto &gaussian = obstacle_array_msg.obstacles.back().gaussians.back();

        for (size_t i = 1; i < prediction.pos.size(); i++) // The first is the current position
        {
            gaussian.mean.poses.emplace_back();
            gaussian.mean.poses.back().pose.position.x = prediction.pos[i](0);
            gaussian.mean.poses.back().pose.position.y = prediction.pos[i](1);
            gaussian.mean.poses.back().pose.position.z = 0.;
            gaussian.mean.poses.back().pose.orientation = RosTools::angleToQuaternion(prediction.angle[i]);

            gaussian.major_semiaxis.push_back(prediction.major_axis[i]);
            gaussian.minor_semiaxis.push_back(prediction.minor_axis[i]);
        }

        obstacle_array_msg.obstacles.back().probabilities.push_back(1.);
    }
    obstacle_array_msg.header.stamp = this->get_clock()->now();
    obstacle_array_msg.header.frame_id = "map";
    return obstacle_array_msg;
}

derived_object_msgs::msg::ObjectArray ROSPedestrianSimulator::PredictionsToObjectArray(const std::vector<Prediction> &predictions)
{
    derived_object_msgs::msg::ObjectArray ped_array_msg;

    for (auto &prediction : predictions)
    {
        derived_object_msgs::msg::Object ped_msg;
        ped_msg.id = prediction.id;
        ped_msg.pose.position.x = prediction.pos[0](0);
        ped_msg.pose.position.y = prediction.pos[0](1);
        ped_msg.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);
        const double vx = prediction.vel.empty() ? 0.0 : prediction.vel[0](0);
        const double vy = prediction.vel.empty() ? 0.0 : prediction.vel[0](1);
        ped_msg.twist.linear.x = std::isfinite(vx) ? vx : 0.0;
        ped_msg.twist.linear.y = std::isfinite(vy) ? vy : 0.0;
        ped_msg.twist.linear.z = 0.;

        ped_array_msg.objects.push_back(ped_msg);
    }
    ped_array_msg.header.stamp = this->get_clock()->now();
    ped_array_msg.header.frame_id = "map";

    return ped_array_msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto sim = std::make_shared<ROSPedestrianSimulator>();

    rclcpp::spin(sim);

    return 0;
}
