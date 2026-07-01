#include <autonomy_slam.hpp>
#include "autonomy/localization/atlas/publish/map_publisher.hpp"
#include "autonomy/localization/atlas/publish/frame_publisher.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/graph_node.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace {
Eigen::Affine3d project_to_xy_plane(const Eigen::Affine3d& affine) {
    Eigen::Matrix4d mat = affine.matrix();
    mat(2, 3) = 0.0;
    Eigen::Translation<double, 3> trans(mat.col(3).head<3>());
    double rx = mat(0, 0);
    double ry = mat(1, 0);
    double yaw = std::atan2(ry, rx);
    return trans * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

Eigen::Vector3d cv_map_point_to_ros(const Eigen::AngleAxisd& rot_cv_to_ros, const atlas::Vec3_t& pos_cv) {
    return rot_cv_to_ros * Eigen::Vector3d(pos_cv(0), pos_cv(1), pos_cv(2));
}

Eigen::Affine3d cam_pose_wc_to_ros(const Eigen::Matrix4d& cam_pose_wc,
                                   const Eigen::AngleAxisd& rot_ros_to_cv) {
    const Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    const Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    const Eigen::Affine3d map_to_camera_cv(trans * rot);
    return rot_ros_to_cv * map_to_camera_cv * rot_ros_to_cv.inverse();
}

geometry_msgs::msg::Point to_point_msg(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    return pt;
}

void append_frustum_edges(std::vector<geometry_msgs::msg::Point>& points,
                          const Eigen::Affine3d& map_to_cam,
                          const double scale,
                          const double aspect) {
    const double depth = scale;
    const double half_w = scale * 0.5 * aspect;
    const double half_h = scale * 0.5;
    const Eigen::Vector3d apex = map_to_cam * Eigen::Vector3d::Zero();
    const Eigen::Vector3d c0 = map_to_cam * Eigen::Vector3d(-half_w, -half_h, depth);
    const Eigen::Vector3d c1 = map_to_cam * Eigen::Vector3d(half_w, -half_h, depth);
    const Eigen::Vector3d c2 = map_to_cam * Eigen::Vector3d(half_w, half_h, depth);
    const Eigen::Vector3d c3 = map_to_cam * Eigen::Vector3d(-half_w, half_h, depth);

    const auto add_line = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        points.push_back(to_point_msg(a));
        points.push_back(to_point_msg(b));
    };
    add_line(apex, c0);
    add_line(apex, c1);
    add_line(apex, c2);
    add_line(apex, c3);
    add_line(c0, c1);
    add_line(c1, c2);
    add_line(c2, c3);
    add_line(c3, c0);
}

void append_graph_edge(std::vector<geometry_msgs::msg::Point>& points,
                       const Eigen::Vector3d& start,
                       const Eigen::Vector3d& end) {
    points.push_back(to_point_msg(start));
    points.push_back(to_point_msg(end));
}
} // namespace

namespace {

void rgb8_to_bgr8(const uint8_t* src, uint8_t* dst, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; ++i) {
        const size_t o = i * 3;
        dst[o + 0] = src[o + 2];
        dst[o + 1] = src[o + 1];
        dst[o + 2] = src[o + 0];
    }
}

cv::Mat owned_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->data.empty()) {
        return {};
    }
    const int height = static_cast<int>(msg->height);
    const int width = static_cast<int>(msg->width);
    auto data_ptr = const_cast<uint8_t*>(msg->data.data());
    cv::Mat owned;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_32FC1, data_ptr, msg->step)
            .copyTo(owned);
        cv::patchNaNs(owned);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1
             || msg->encoding == sensor_msgs::image_encodings::MONO16) {
        cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_16UC1, data_ptr, msg->step)
            .copyTo(owned);
    }
    else {
        return {};
    }
    return owned;
}

cv::Mat owned_cv_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                       const std::string& encoding) {
    const std::string enc = encoding.empty() ? msg->encoding : encoding;
    if (msg->data.empty()) {
        return {};
    }

    auto make_view = [&](int type) {
        return cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), type,
                       const_cast<uint8_t*>(msg->data.data()), msg->step);
    };

    cv::Mat owned;
    if (enc == sensor_msgs::image_encodings::BGR8) {
        make_view(CV_8UC3).copyTo(owned);
    }
    else if (enc == sensor_msgs::image_encodings::RGB8) {
        owned.create(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3);
        rgb8_to_bgr8(msg->data.data(), owned.ptr<uint8_t>(),
                     static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height));
    }
    else if (enc == sensor_msgs::image_encodings::MONO8) {
        make_view(CV_8UC1).copyTo(owned);
    }
    return owned;
}
}  // namespace

namespace autonomy_slam {
namespace {
constexpr int kVizQosDepth = 10;
}  // namespace

system::system(const std::shared_ptr<atlas::system>& slam,
               rclcpp::Node* node,
               const std::string& mask_img_path)
    : slam_(slam), node_(node), custom_qos_(rmw_qos_profile_sensor_data),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("~/camera_pose", kVizQosDepth)),
      trajectory_pub_(node_->create_publisher<nav_msgs::msg::Path>("~/trajectory", kVizQosDepth)),
      keyframes_pub_(node_->create_publisher<geometry_msgs::msg::PoseArray>("~/keyframes", kVizQosDepth)),
      keyframes_2d_pub_(node_->create_publisher<geometry_msgs::msg::PoseArray>("~/keyframes_2d", kVizQosDepth)),
      tracking_image_pub_(node_->create_publisher<sensor_msgs::msg::Image>("~/tracking_image", kVizQosDepth)),
      map_points_pub_(node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", kVizQosDepth)),
      local_map_points_pub_(node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/local_map_points", kVizQosDepth)),
      feature_match_marker_pub_(node_->create_publisher<visualization_msgs::msg::Marker>("~/feature_matches", kVizQosDepth)),
      keyframe_frustums_pub_(node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/keyframe_frustums", kVizQosDepth)),
      pose_graph_pub_(node_->create_publisher<visualization_msgs::msg::Marker>("~/pose_graph", kVizQosDepth)),
      current_camera_frustum_pub_(node_->create_publisher<visualization_msgs::msg::Marker>("~/current_camera_frustum", kVizQosDepth)),
      frame_processed_pub_(node_->create_publisher<std_msgs::msg::Empty>(
          "~/frame_processed", rclcpp::QoS(10).reliable())),
      map_to_odom_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      tf_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_)),
      pending_viz_(false),
      last_published_keyframe_count_(0) {
    custom_qos_.depth = 1;
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&system::init_pose_callback,
                  this, std::placeholders::_1));
    setParams();
    update_map_frame_rotation();
    const double fps = slam_->get_camera()->fps_;
    slam_frame_period_ = (fps > 0.0)
        ? (static_cast<double>(frame_skip_) / fps)
        : (static_cast<double>(frame_skip_) / 30.0);

    if (visualization_hz_ > 0.0) {
        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / visualization_hz_));
        viz_timer_ = node_->create_wall_timer(
            period_ns, std::bind(&system::on_visualization_timer, this));
    }
}

void system::update_map_frame_rotation() {
    if (apply_ros_map_conversion_) {
        rot_ros_to_cv_map_frame_ = (Eigen::Matrix3d() << 0, 0, 1,
                                    -1, 0, 0,
                                    0, -1, 0)
                                       .finished();
    }
    else {
        rot_ros_to_cv_map_frame_ = Eigen::AngleAxisd(Eigen::Matrix3d::Identity());
    }
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp,
                          const bool append_trajectory) {
    const Eigen::Affine3d cam_to_world_ros = cam_pose_wc_to_ros(cam_pose_wc, rot_ros_to_cv_map_frame_);

    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(cam_to_world_ros);
    pose_pub_->publish(pose_msg);

    if (publish_trajectory_ && append_trajectory && !trajectory_keyframes_only_) {
        trajectory_frame_counter_++;
        if (trajectory_frame_counter_ % trajectory_stride_ == 0) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = pose_msg.header;
            pose_stamped.pose = pose_msg.pose.pose;
            trajectory_path_.header = pose_msg.header;
            trajectory_path_.poses.push_back(pose_stamped);
            trajectory_pub_->publish(trajectory_path_);
        }
    }

    last_cam_pose_wc_ = cam_pose_wc;
    has_last_cam_pose_ = true;

    publish_current_camera_frustum(stamp, cam_pose_wc);

    const Eigen::Affine3d cam_to_world_ros_for_tf = cam_to_world_ros;

    // Send map->camera transform for rviz / navigation.
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(
                camera_optical_frame_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
                tf2::durationFromSec(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            geometry_msgs::msg::TransformStamped map_to_odom_msg;
            if (odom2d_) {
                Eigen::Affine3d map_to_camera_affine_2d =
                    project_to_xy_plane(cam_to_world_ros_for_tf) * rot_ros_to_cv_map_frame_;
                Eigen::Affine3d camera_to_odom_affine_2d = (project_to_xy_plane(camera_to_odom_affine.inverse() * rot_ros_to_cv_map_frame_.inverse()) * rot_ros_to_cv_map_frame_).inverse();
                Eigen::Affine3d map_to_odom_affine_2d = map_to_camera_affine_2d * camera_to_odom_affine_2d;
                map_to_odom_msg = tf2::eigenToTransform(map_to_odom_affine_2d);
            }
            else {
                map_to_odom_msg = tf2::eigenToTransform(cam_to_world_ros_for_tf * camera_to_odom_affine);
            }
            tf2::TimePoint transform_timestamp = tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
            map_to_odom_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_broadcaster_->sendTransform(map_to_odom_msg);
        }
        catch (tf2::TransformException&) {
            geometry_msgs::msg::TransformStamped map_to_camera_msg =
                tf2::eigenToTransform(cam_to_world_ros_for_tf);
            tf2::TimePoint transform_timestamp =
                tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
            map_to_camera_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
            map_to_camera_msg.header.frame_id = map_frame_;
            map_to_camera_msg.child_frame_id = camera_frame_;
            map_to_odom_broadcaster_->sendTransform(map_to_camera_msg);
        }
    }
}

void system::publish_keyframes(const rclcpp::Time& stamp) {
    publish_keyframe_frustums(stamp);
    publish_pose_graph(stamp);
}

void system::publish_keyframe_frustums(const rclcpp::Time& stamp) {
    if (!publish_keyframes_) {
        return;
    }
    std::vector<std::shared_ptr<atlas::data::keyframe>> all_keyfrms;
    if (slam_->get_map_publisher()->get_keyframes(all_keyfrms) == 0) {
        visualization_msgs::msg::MarkerArray clear_msg;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_msg.markers.push_back(clear);
        keyframe_frustums_pub_->publish(clear_msg);
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    for (const auto& keyfrm : all_keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        const Eigen::Affine3d pose_ros = cam_pose_wc_to_ros(keyfrm->get_pose_wc(), rot_ros_to_cv_map_frame_);

        if (trajectory_keyframes_only_ && publish_trajectory_
            && trajectory_keyframe_ids_.insert(keyfrm->id_).second) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = stamp;
            pose_stamped.header.frame_id = map_frame_;
            pose_stamped.pose = tf2::toMsg(pose_ros);
            trajectory_path_.header = pose_stamped.header;
            trajectory_path_.poses.push_back(pose_stamped);
            trajectory_pub_->publish(trajectory_path_);
        }

        if (keyframe_frustum_skip_ > 1 && (keyfrm->id_ % static_cast<unsigned int>(keyframe_frustum_skip_) != 0)) {
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = map_frame_;
        marker.ns = "keyframe_frustums";
        marker.id = static_cast<int32_t>(keyfrm->id_);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.006;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        append_frustum_edges(marker.points, pose_ros, keyframe_frustum_scale_, keyframe_frustum_aspect_);
        if (!marker.points.empty()) {
            marker_array.markers.push_back(marker);
        }
    }
    keyframe_frustums_pub_->publish(marker_array);
}

void system::publish_pose_graph(const rclcpp::Time& stamp) {
    if (!publish_keyframes_ || !publish_pose_graph_) {
        return;
    }
    std::vector<std::shared_ptr<atlas::data::keyframe>> all_keyfrms;
    if (slam_->get_map_publisher()->get_keyframes(all_keyfrms) == 0) {
        return;
    }

    std::map<std::pair<unsigned int, unsigned int>, bool> drawn_edges;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "pose_graph";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.008;
    marker.color.r = 0.62f;
    marker.color.g = 0.2f;
    marker.color.b = 0.9f;
    marker.color.a = 0.65f;

    const auto add_edge = [&](const std::shared_ptr<atlas::data::keyframe>& a,
                              const std::shared_ptr<atlas::data::keyframe>& b) {
        if (!a || !b || a->id_ == b->id_) {
            return;
        }
        const auto edge = std::minmax(a->id_, b->id_);
        if (drawn_edges.count(edge) > 0) {
            return;
        }
        drawn_edges[edge] = true;
        const Eigen::Vector3d pa = cam_pose_wc_to_ros(a->get_pose_wc(), rot_ros_to_cv_map_frame_).translation();
        const Eigen::Vector3d pb = cam_pose_wc_to_ros(b->get_pose_wc(), rot_ros_to_cv_map_frame_).translation();
        append_graph_edge(marker.points, pa, pb);
    };

    for (const auto& keyfrm : all_keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased() || !keyfrm->graph_node_) {
            continue;
        }
        if (const auto parent = keyfrm->graph_node_->get_spanning_parent()) {
            add_edge(keyfrm, parent);
        }
        for (const auto& child : keyfrm->graph_node_->get_spanning_children()) {
            add_edge(keyfrm, child);
        }
        for (const auto& loop_kf : keyfrm->graph_node_->get_loop_edges()) {
            add_edge(keyfrm, loop_kf);
        }
    }

    if (!marker.points.empty()) {
        pose_graph_pub_->publish(marker);
    }
}

void system::publish_current_camera_frustum(const rclcpp::Time& stamp, const Eigen::Matrix4d& cam_pose_wc) {
    if (!publish_keyframes_) {
        return;
    }
    const Eigen::Affine3d pose_ros = cam_pose_wc_to_ros(cam_pose_wc, rot_ros_to_cv_map_frame_);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "current_camera";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.008;
    marker.color.r = 0.35f;
    marker.color.g = 0.95f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    append_frustum_edges(marker.points, pose_ros, current_camera_frustum_scale_, current_camera_frustum_aspect_);
    if (!marker.points.empty()) {
        current_camera_frustum_pub_->publish(marker);
    }
}

void system::publish_tracking_image(const rclcpp::Time& stamp) {
    if (!publish_tracking_image_) {
        return;
    }
    const auto frame_publisher = slam_->get_frame_publisher();
    if (!frame_publisher) {
        return;
    }
    cv::Mat img = frame_publisher->draw_frame();
    if (img.empty()) {
        return;
    }
    if (!img.isContinuous()) {
        img = img.clone();
    }
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = camera_frame_;
    msg.height = static_cast<uint32_t>(img.rows);
    msg.width = static_cast<uint32_t>(img.cols);
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(img.cols * img.elemSize());
    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), img.data, msg.data.size());
    tracking_image_pub_->publish(msg);
}

void system::fill_map_point_cloud(const std::vector<std::shared_ptr<atlas::data::landmark>>& landmarks,
                                  const std::set<std::shared_ptr<atlas::data::landmark>>* highlight,
                                  sensor_msgs::msg::PointCloud2& cloud) const {
    std::vector<float> points;
    points.reserve(landmarks.size() * 3);
    const int skip = std::max(1, map_points_skip_);
    for (std::size_t i = 0; i < landmarks.size(); i += static_cast<std::size_t>(skip)) {
        const auto& lm = landmarks.at(i);
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        if (highlight && highlight->count(lm) == 0) {
            continue;
        }
        const auto pos_ros = cv_map_point_to_ros(rot_ros_to_cv_map_frame_, lm->get_pos_in_world());
        points.push_back(static_cast<float>(pos_ros.x()));
        points.push_back(static_cast<float>(pos_ros.y()));
        points.push_back(static_cast<float>(pos_ros.z()));
    }

    cloud.header.frame_id = map_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size() / 3);
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud.width);
    if (cloud.width == 0) {
        return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (std::size_t i = 0; i < points.size(); i += 3) {
        *iter_x = points[i];
        *iter_y = points[i + 1];
        *iter_z = points[i + 2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
}

void system::publish_map_points(const rclcpp::Time& stamp) {
    if (!publish_map_points_) {
        return;
    }
    std::vector<std::shared_ptr<atlas::data::landmark>> all_landmarks;
    std::set<std::shared_ptr<atlas::data::landmark>> local_landmarks;
    if (slam_->get_map_publisher()->get_landmarks(all_landmarks, local_landmarks) == 0) {
        return;
    }

    sensor_msgs::msg::PointCloud2 map_cloud;
    map_cloud.header.stamp = stamp;
    fill_map_point_cloud(all_landmarks, nullptr, map_cloud);
    if (map_cloud.width > 0) {
        map_points_pub_->publish(map_cloud);
    }

    if (publish_local_map_points_) {
        sensor_msgs::msg::PointCloud2 local_cloud;
        local_cloud.header.stamp = stamp;
        fill_map_point_cloud(all_landmarks, &local_landmarks, local_cloud);
        if (local_cloud.width > 0) {
            local_map_points_pub_->publish(local_cloud);
        }
    }
}

void system::publish_feature_matches(const rclcpp::Time& stamp, const Eigen::Matrix4d& cam_pose_wc) {
    if (!publish_feature_matches_) {
        return;
    }
    const auto frame_publisher = slam_->get_frame_publisher();
    if (!frame_publisher) {
        return;
    }
    const auto keypoints_landmarks = frame_publisher->get_keypoints_and_landmarks();
    const auto& landmarks = keypoints_landmarks.second;

    const Eigen::Affine3d cam_to_world_ros = cam_pose_wc_to_ros(cam_pose_wc, rot_ros_to_cv_map_frame_);
    const Eigen::Vector3d cam_pos_ros = cam_to_world_ros.translation();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "feature_matches";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.r = 0.1f;
    marker.color.g = 1.0f;
    marker.color.b = 0.2f;
    marker.color.a = 0.6f;
    marker.points.reserve(landmarks.size() * 2);

    for (const auto& lm : landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const auto pos_ros = cv_map_point_to_ros(rot_ros_to_cv_map_frame_, lm->get_pos_in_world());
        geometry_msgs::msg::Point p_cam;
        p_cam.x = cam_pos_ros.x();
        p_cam.y = cam_pos_ros.y();
        p_cam.z = cam_pos_ros.z();
        geometry_msgs::msg::Point p_lm;
        p_lm.x = pos_ros.x();
        p_lm.y = pos_ros.y();
        p_lm.z = pos_ros.z();
        marker.points.push_back(p_cam);
        marker.points.push_back(p_lm);
    }
    if (!marker.points.empty()) {
        feature_match_marker_pub_->publish(marker);
    }
}

void system::publish_slam_visualizations(const rclcpp::Time& stamp,
                                         const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc) {
    publish_tracking_image(stamp);
    publish_map_points(stamp);
    if (cam_pose_wc) {
        publish_feature_matches(stamp, *cam_pose_wc);
    }
}

bool system::publish_keyframes_if_changed(const rclcpp::Time& stamp) {
    if (!publish_keyframes_) {
        return false;
    }
    std::vector<std::shared_ptr<atlas::data::keyframe>> all_keyfrms;
    const auto keyfrm_count = slam_->get_map_publisher()->get_keyframes(all_keyfrms);
    if (keyfrm_count == 0) {
        last_published_keyframe_count_ = 0;
        return false;
    }
    if (all_keyfrms.size() == last_published_keyframe_count_) {
        return false;
    }
    last_published_keyframe_count_ = all_keyfrms.size();
    publish_keyframes(stamp);
    return true;
}

void system::finish_frame_visualization(const rclcpp::Time& stamp,
                                        const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc) {
    if (visualization_hz_ > 0.0) {
        std::lock_guard<std::mutex> lock(viz_mutex_);
        pending_viz_stamp_ = stamp;
        pending_cam_pose_wc_ = cam_pose_wc;
        pending_viz_ = true;
    }
    else {
        publish_keyframes_if_changed(stamp);
        publish_slam_visualizations(stamp, cam_pose_wc);
    }
    notify_frame_processed();
}

void system::on_visualization_timer() {
    rclcpp::Time stamp;
    std::shared_ptr<Eigen::Matrix4d> cam_pose_wc;
    {
        std::lock_guard<std::mutex> lock(viz_mutex_);
        if (!pending_viz_) {
            return;
        }
        stamp = pending_viz_stamp_;
        cam_pose_wc = pending_cam_pose_wc_;
        pending_viz_ = false;
    }
    publish_keyframes_if_changed(stamp);
    publish_slam_visualizations(stamp, cam_pose_wc);
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);

    map_frame_ = std::string("map");
    map_frame_ = node_->declare_parameter("map_frame", map_frame_);

    robot_base_frame_ = std::string("base_link");
    robot_base_frame_ = node_->declare_parameter("robot_base_frame", robot_base_frame_);

    camera_frame_ = std::string("camera_frame");
    camera_frame_ = node_->declare_parameter("camera_frame", camera_frame_);

    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

    odom2d_ = false;
    odom2d_ = node_->declare_parameter("odom2d", odom2d_);

    publish_keyframes_ = true;
    publish_keyframes_ = node_->declare_parameter("publish_keyframes", publish_keyframes_);

    publish_tracking_image_ = true;
    publish_tracking_image_ = node_->declare_parameter("publish_tracking_image", publish_tracking_image_);

    publish_map_points_ = true;
    publish_map_points_ = node_->declare_parameter("publish_map_points", publish_map_points_);

    publish_local_map_points_ = false;
    publish_local_map_points_ = node_->declare_parameter("publish_local_map_points", publish_local_map_points_);

    publish_feature_matches_ = true;
    publish_feature_matches_ = node_->declare_parameter("publish_feature_matches", publish_feature_matches_);

    publish_pose_graph_ = true;
    publish_pose_graph_ = node_->declare_parameter("publish_pose_graph", publish_pose_graph_);

    map_points_skip_ = 1;
    map_points_skip_ = node_->declare_parameter("map_points_skip", map_points_skip_);

    camera_frustum_scale_ = 0.53;
    camera_frustum_scale_ = node_->declare_parameter("camera_frustum_scale", camera_frustum_scale_);

    camera_frustum_aspect_ = 2.0;
    camera_frustum_aspect_ = node_->declare_parameter("camera_frustum_aspect", camera_frustum_aspect_);

    keyframe_frustum_scale_ =
        node_->declare_parameter("keyframe_frustum_scale", camera_frustum_scale_);
    keyframe_frustum_aspect_ =
        node_->declare_parameter("keyframe_frustum_aspect", camera_frustum_aspect_);
    current_camera_frustum_scale_ =
        node_->declare_parameter("current_camera_frustum_scale", camera_frustum_scale_);
    current_camera_frustum_aspect_ =
        node_->declare_parameter("current_camera_frustum_aspect", camera_frustum_aspect_);

    landmark_point_size_ = 0.02;
    landmark_point_size_ = node_->declare_parameter("landmark_point_size", landmark_point_size_);

    transform_tolerance_ = 0.1;
    transform_tolerance_ = node_->declare_parameter("transform_tolerance", transform_tolerance_);

    encoding_ = "";
    encoding_ = node_->declare_parameter("encoding", encoding_);

    use_message_timestamp_ = false;
    use_message_timestamp_ = node_->declare_parameter("use_message_timestamp", use_message_timestamp_);

    frame_skip_ = 1;
    frame_skip_ = node_->declare_parameter("frame_skip", frame_skip_);
    frame_skip_ = std::max(1, frame_skip_);

    slam_frame_timestamp_ = 0.0;

    publish_trajectory_ = true;
    publish_trajectory_ = node_->declare_parameter("publish_trajectory", publish_trajectory_);

    trajectory_keyframes_only_ = false;
    trajectory_keyframes_only_ =
        node_->declare_parameter("trajectory_keyframes_only", trajectory_keyframes_only_);

    trajectory_stride_ = 1;
    trajectory_stride_ = node_->declare_parameter("trajectory_stride", trajectory_stride_);
    trajectory_stride_ = std::max(1, trajectory_stride_);

    keyframe_frustum_skip_ = 1;
    keyframe_frustum_skip_ = node_->declare_parameter("keyframe_frustum_skip", keyframe_frustum_skip_);
    keyframe_frustum_skip_ = std::max(1, keyframe_frustum_skip_);

    visualization_hz_ = 20.0;
    visualization_hz_ = node_->declare_parameter("visualization_hz", visualization_hz_);

    publish_last_pose_when_lost_ = true;
    publish_last_pose_when_lost_ =
        node_->declare_parameter("publish_last_pose_when_lost", publish_last_pose_when_lost_);

    apply_ros_map_conversion_ = false;
    apply_ros_map_conversion_ =
        node_->declare_parameter("apply_ros_map_conversion", apply_ros_map_conversion_);

    image_qos_reliable_ = false;
    image_qos_reliable_ = node_->declare_parameter("image_qos_reliable", image_qos_reliable_);

    map_had_keyframes_ = false;
    trajectory_keyframe_ids_.clear();
    trajectory_frame_counter_ = 0;
    last_published_keyframe_count_ = 0;

    has_last_cam_pose_ = false;
    last_cam_pose_wc_ = Eigen::Matrix4d::Identity();
}

double system::next_slam_timestamp(const rclcpp::Time& msg_stamp) {
    if (use_message_timestamp_) {
        return msg_stamp.seconds();
    }
    const double timestamp = slam_frame_timestamp_;
    slam_frame_timestamp_ += slam_frame_period_;
    return timestamp;
}

rclcpp::Time system::to_publish_stamp(const double /*slam_timestamp*/, const rclcpp::Time& msg_stamp) {
    if (use_message_timestamp_) {
        return msg_stamp;
    }
    // SLAM uses synthetic timestamps; rviz/TF need wall-clock stamps or displays expire.
    return node_->now();
}

void system::notify_frame_processed() {
    std_msgs::msg::Empty ack;
    frame_processed_pub_->publish(ack);
}

void system::handle_tracking_result(const std::shared_ptr<Eigen::Matrix4d>& cam_pose_wc,
                                    const rclcpp::Time& publish_stamp) {
    std::vector<std::shared_ptr<atlas::data::keyframe>> keyfrms;
    slam_->get_map_publisher()->get_keyframes(keyfrms);

    if (keyfrms.empty()) {
        if (map_had_keyframes_) {
            has_last_cam_pose_ = false;
            trajectory_path_.poses.clear();
            trajectory_keyframe_ids_.clear();
            trajectory_frame_counter_ = 0;
            last_published_keyframe_count_ = 0;
            slam_frame_timestamp_ = 0.0;
            visualization_msgs::msg::MarkerArray clear_msg;
            visualization_msgs::msg::Marker clear;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            clear_msg.markers.push_back(clear);
            keyframe_frustums_pub_->publish(clear_msg);
        }
    }
    else {
        map_had_keyframes_ = true;
    }

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, publish_stamp);
        return;
    }
    if (publish_last_pose_when_lost_ && has_last_cam_pose_) {
        publish_pose(last_cam_pose_wc_, publish_stamp, false);
    }
}

void system::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (camera_optical_frame_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Camera link is not set: no images were received yet");
        return;
    }

    Eigen::Translation3d trans(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond rot_q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Affine3d initialpose_affine(trans * rot_q);

    Eigen::Matrix3d rot_cv_to_ros_map_frame;
    rot_cv_to_ros_map_frame << 0, -1, 0,
        0, 0, -1,
        1, 0, 0;

    Eigen::Affine3d map_to_initialpose_frame_affine;
    try {
        auto map_to_initialpose_frame = tf_->lookupTransform(
            map_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        map_to_initialpose_frame_affine = tf2::transformToEigen(
            map_to_initialpose_frame.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    Eigen::Affine3d robot_base_frame_to_camera_affine;
    try {
        auto robot_base_frame_to_camera = tf_->lookupTransform(
            robot_base_frame_, camera_optical_frame_, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        robot_base_frame_to_camera_affine = tf2::transformToEigen(robot_base_frame_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    // Target transform is map_cv -> camera_link and known parameters are following:
    //   rot_cv_to_ros_map_frame: T(map_cv -> map)
    //   map_to_initialpose_frame_affine: T(map -> `msg->header.frame_id`)
    //   initialpose_affine: T(`msg->header.frame_id` -> base_link)
    //   robot_base_frame_to_camera_affine: T(base_link -> camera_link)
    // The flow of the transformation is as follows:
    //   map_cv -> map -> `msg->header.frame_id` -> base_link -> camera_link
    Eigen::Matrix4d cam_pose_cv = (rot_cv_to_ros_map_frame * map_to_initialpose_frame_affine
                                   * initialpose_affine * robot_base_frame_to_camera_affine)
                                      .matrix();

    const Eigen::Vector3d normal_vector = (Eigen::Vector3d() << 0., 1., 0.).finished();
    if (!slam_->relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
        RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<atlas::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path)
    : system(slam, node, mask_img_path) {
    custom_qos_.depth = 10;
    if (image_qos_reliable_) {
        custom_qos_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    }
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_), custom_qos_);
    raw_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", qos, [this](sensor_msgs::msg::Image::UniquePtr msg_unique_ptr) { callback(std::move(msg_unique_ptr)); });
}
void mono::callback(sensor_msgs::msg::Image::UniquePtr msg_unique_ptr) {
    sensor_msgs::msg::Image::ConstSharedPtr msg = std::move(msg_unique_ptr);
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
        RCLCPP_INFO(node_->get_logger(), "Receiving images on camera/image_raw (%ux%u)",
                    msg->width, msg->height);
    }
    const rclcpp::Time tp_1 = node_->now();
    const rclcpp::Time msg_stamp(msg->header.stamp);
    const double timestamp = next_slam_timestamp(msg_stamp);
    const rclcpp::Time publish_stamp = to_publish_stamp(timestamp, msg_stamp);

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(owned_cv_image(msg, encoding_), timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    handle_tracking_result(cam_pose_wc, publish_stamp);
    finish_frame_visualization(publish_stamp, cam_pose_wc);
}

void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const rclcpp::Time msg_stamp(msg->header.stamp);
    const double timestamp = next_slam_timestamp(msg_stamp);
    const rclcpp::Time publish_stamp = to_publish_stamp(timestamp, msg_stamp);

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(owned_cv_image(msg, encoding_), timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    handle_tracking_result(cam_pose_wc, publish_stamp);
    finish_frame_visualization(publish_stamp, cam_pose_wc);
}

stereo::stereo(const std::shared_ptr<atlas::system>& slam,
               rclcpp::Node* node,
               const std::string& mask_img_path,
               const std::shared_ptr<StereoRectifier>& rectifier)
    : system(slam, node, mask_img_path),
      rectifier_(rectifier),
      left_sf_(node_, "camera/left/image_raw"),
      right_sf_(node_, "camera/right/image_raw") {
    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, left_sf_, right_sf_);
        exact_time_sync_->registerCallback(&stereo::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, left_sf_, right_sf_);
        approx_time_sync_->registerCallback(&stereo::callback, this);
    }
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = left->header.frame_id;
    }
    auto leftcv = owned_cv_image(left, encoding_);
    auto rightcv = owned_cv_image(right, encoding_);
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const rclcpp::Time msg_stamp(left->header.stamp);
    const double timestamp = next_slam_timestamp(msg_stamp);
    const rclcpp::Time publish_stamp = to_publish_stamp(timestamp, msg_stamp);

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    handle_tracking_result(cam_pose_wc, publish_stamp);
    finish_frame_visualization(publish_stamp, cam_pose_wc);
}

rgbd::rgbd(const std::shared_ptr<atlas::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path)
    : system(slam, node, mask_img_path),
      color_sf_(node_, "camera/color/image_raw"),
      depth_sf_(node_, "camera/depth/image_raw") {
    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, color_sf_, depth_sf_);
        exact_time_sync_->registerCallback(&rgbd::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, color_sf_, depth_sf_);
        approx_time_sync_->registerCallback(&rgbd::callback, this);
    }
}

void rgbd::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = color->header.frame_id;
    }
    auto colorcv = owned_cv_image(color, encoding_);
    auto depthcv = owned_depth_image(depth);
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }

    const rclcpp::Time tp_1 = node_->now();
    const rclcpp::Time msg_stamp(color->header.stamp);
    const double timestamp = next_slam_timestamp(msg_stamp);
    const rclcpp::Time publish_stamp = to_publish_stamp(timestamp, msg_stamp);

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track time in seconds
    track_times_.push_back(track_time);

    handle_tracking_result(cam_pose_wc, publish_stamp);
    finish_frame_visualization(publish_stamp, cam_pose_wc);
}

} // namespace autonomy_slam
