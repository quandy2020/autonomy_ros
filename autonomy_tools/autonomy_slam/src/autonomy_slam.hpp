#ifndef AUTONOMY_SLAM_HPP_
#define AUTONOMY_SLAM_HPP_

#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "stereo_rectifier.hpp"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <mutex>
#include <set>

namespace atlas = autonomy::localization::atlas;

namespace autonomy::localization::atlas::data {
class landmark;
}

namespace autonomy_slam {
class system {
public:
    system(const std::shared_ptr<atlas::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp,
                      bool append_trajectory = true);
    double next_slam_timestamp(const rclcpp::Time& msg_stamp);
    rclcpp::Time to_publish_stamp(double slam_timestamp, const rclcpp::Time& msg_stamp);
    void handle_tracking_result(const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc,
                                const rclcpp::Time& msg_stamp);
    void notify_frame_processed();
    void publish_keyframes(const rclcpp::Time& stamp);
    void publish_keyframe_frustums(const rclcpp::Time& stamp);
    void publish_pose_graph(const rclcpp::Time& stamp);
    void publish_current_camera_frustum(const rclcpp::Time& stamp, const Eigen::Matrix4d& cam_pose_wc);
    void publish_tracking_image(const rclcpp::Time& stamp);
    void publish_map_points(const rclcpp::Time& stamp);
    void publish_feature_matches(const rclcpp::Time& stamp, const Eigen::Matrix4d& cam_pose_wc);
    void publish_slam_visualizations(const rclcpp::Time& stamp,
                                     const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc);
    void finish_frame_visualization(const rclcpp::Time& stamp,
                                    const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc);
    void on_visualization_timer();
    bool publish_keyframes_if_changed(const rclcpp::Time& stamp);
    void setParams();
    void update_map_frame_rotation();
    std::shared_ptr<atlas::system> slam_;
    std::shared_ptr<atlas::config> cfg_;
    rclcpp::Node* node_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> trajectory_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_2d_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> tracking_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_points_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> local_map_points_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> feature_match_marker_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> keyframe_frustums_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pose_graph_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> current_camera_frustum_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> frame_processed_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string robot_base_frame_;
    std::string camera_frame_;
    std::string camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    // If true, publish tf from map_frame to odom_frame
    bool publish_tf_;

    // If true, publish keyframes
    bool publish_keyframes_;

    bool publish_tracking_image_;
    bool publish_map_points_;
    bool publish_local_map_points_;
    bool publish_feature_matches_;
    bool publish_pose_graph_;
    int map_points_skip_;
    double camera_frustum_scale_;
    double camera_frustum_aspect_;
    double keyframe_frustum_scale_;
    double keyframe_frustum_aspect_;
    double current_camera_frustum_scale_;
    double current_camera_frustum_aspect_;
    double landmark_point_size_;

    // Publish pose's timestamp in the future
    double transform_tolerance_;

    // If true, odom_frame is fixed on the xy-plane of map_frame. This is useful when working with 2D navigation modules.
    bool odom2d_;

    std::string encoding_;

    bool use_message_timestamp_;
    int frame_skip_;
    double slam_frame_timestamp_;
    double slam_frame_period_;
    bool publish_trajectory_;
    bool trajectory_keyframes_only_;
    int trajectory_stride_;
    int keyframe_frustum_skip_;
    bool publish_last_pose_when_lost_;
    bool apply_ros_map_conversion_;

    // Use RELIABLE QoS for image subscription (required for video backpressure sync).
    bool image_qos_reliable_;
    bool map_had_keyframes_;
    bool has_last_cam_pose_;
    Eigen::Matrix4d last_cam_pose_wc_;
    nav_msgs::msg::Path trajectory_path_;
    std::set<unsigned int> trajectory_keyframe_ids_;
    int trajectory_frame_counter_;

    double visualization_hz_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::Time pending_viz_stamp_;
    std::shared_ptr<Eigen::Matrix4d> pending_cam_pose_wc_;
    bool pending_viz_;
    std::size_t last_published_keyframe_count_;
    std::mutex viz_mutex_;

private:
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void fill_map_point_cloud(const std::vector<std::shared_ptr<atlas::data::landmark>>& landmarks,
                              const std::set<std::shared_ptr<atlas::data::landmark>>* highlight,
                              sensor_msgs::msg::PointCloud2& cloud) const;

    Eigen::AngleAxisd rot_ros_to_cv_map_frame_;
};

class mono : public system {
public:
    mono(const std::shared_ptr<atlas::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(sensor_msgs::msg::Image::UniquePtr msg);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> raw_image_sub_;
};

template<class M, class NodeType = rclcpp::Node>
class ModifiedSubscriber : public message_filters::Subscriber<M> {
public:
    ModifiedSubscriber(typename message_filters::Subscriber<M>::NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(NodeType* node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(
        typename message_filters::Subscriber<M>::NodePtr node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    ModifiedSubscriber(
        NodeType* node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    void cb(const typename message_filters::Subscriber<M>::MConstPtr& msg) {
        this->signalMessage(msg);
    }
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<atlas::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path,
           const std::shared_ptr<StereoRectifier>& rectifier);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<StereoRectifier> rectifier_;
    ModifiedSubscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<atlas::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    ModifiedSubscriber<sensor_msgs::msg::Image> color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

} // namespace autonomy_slam

#endif // AUTONOMY_SLAM_HPP_
