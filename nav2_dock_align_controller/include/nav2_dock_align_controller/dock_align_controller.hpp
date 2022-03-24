#ifndef NAV2_DOCK_ALIGN_CONTROLLER__DOCK_ALIGN_CONTROLLER_HPP_
#define NAV2_DOCK_ALIGN_CONTROLLER__DOCK_ALIGN_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_dock_align_controller/common_functions.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_dock_align_controller
{

class DockAlignController : public nav2_core::Controller
{
public:
  DockAlignController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  ~DockAlignController() = default;


  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
  ) const;

  TagDetector tag_detector_;
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::msg::CameraInfo::SharedPtr info_ptr;
  nav2_dock_align_controller::msg::AprilTagDetectionArray detected_tags;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("DockAlignController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav2_dock_align_controller::msg::AprilTagDetectionArray>> tag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr global_img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr global_info_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace nav2_dock_align_controller

#endif  // NAV2_DOCK_ALIGN_CONTROLLER__DOCK_ALIGN_CONTROLLER_HPP_