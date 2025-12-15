/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#ifndef VECTORNAV__VN_SENSOR_MSGS_HPP_
#define VECTORNAV__VN_SENSOR_MSGS_HPP_
#include <chrono>
#include <functional>
#include <optional>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vectornav_msgs/msg/attitude_group.hpp>
#include <vectornav_msgs/msg/common_group.hpp>
#include <vectornav_msgs/msg/gps_group.hpp>
#include <vectornav_msgs/msg/imu_group.hpp>
#include <vectornav_msgs/msg/ins_group.hpp>
#include <vectornav_msgs/msg/time_group.hpp>

namespace vectornav
{
class VnSensorMsgs : public rclcpp::Node
{
public:
  VnSensorMsgs(const rclcpp::NodeOptions & options);

private:
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in);
  void sub_vn_time(const vectornav_msgs::msg::TimeGroup::SharedPtr msg_in) const;
  void sub_vn_imu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg_in) const;
  void sub_vn_gps(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in);
  void sub_vn_attitude(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg_in) const;
  void sub_vn_ins(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in);
  void sub_vn_gps2(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in) const;
  void fill_covariance_from_param(std::string param_name, std::array<double, 9> & array) const;
  /// Convert from DEG to RAD
inline static double deg2rad(double in) { return in * M_PI / 180.0; }

  // Publishes odometry computed from the ENU pose
  void publishOdomFromEnu(
    const geometry_msgs::msg::PoseWithCovarianceStamped & enu_pose);

  // Updates initial ENU pose for transform computation (averaging)
  void updateEnuInitialization(
    const geometry_msgs::msg::PoseWithCovarianceStamped & enu_pose);

  // Creates a tf2::Transform from an ENU pose (planar only)
  tf2::Transform makeEnuPoseTransform(
    const geometry_msgs::msg::PoseWithCovarianceStamped & enu_pose);

  // Callback for ROS service to reset odometry
  void handleResetEnuOdometry(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  //
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_startup_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_gps_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_syncin_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_time_pps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_uncompensated_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_magnetic_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_pressure_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;

  // egocentric (base_link -> odom frame) odometry
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
  rclcpp::Subscription<vectornav_msgs::msg::TimeGroup>::SharedPtr sub_vn_time_;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr sub_vn_imu_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps_;
  rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr sub_vn_attitude_;
  rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr sub_vn_ins_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps2_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_enu_srv_;


  bool use_enu = true;

  /// Default orientation Covariance
  const std::vector<double> orientation_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                       0.0000, 0.0000, 0.0000, 0.0000};

  /// Default angular_velocity Covariance
  const std::vector<double> angular_velocity_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                            0.0000, 0.0000, 0.0000, 0.0000};

  /// Default linear_acceleration Covariance
  const std::vector<double> linear_acceleration_covariance_ = {
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};

  /// Default magnetic field Covariance
  const std::vector<double> magnetic_field_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                          0.0000, 0.0000, 0.0000, 0.0000};

  /// TODO(Dereck): Find default covariance values

  // State Vars
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 gps_posu_;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;

  // used to produce egocentric odometry from the ENU pose
  // if not populated, then we haven't collected enough ENU poses to initialize
  std::optional<tf2::Transform> enu_to_odom_;

  // Averaging window for generating enu_to_odom_
  static constexpr std::size_t ENU_INIT_WINDOW = 50;

  // accumulators
  std::size_t enu_init_count_{0};
  double enu_x_sum_{0.0};
  double enu_y_sum_{0.0};
  double enu_yaw_sin_sum_{0.0};
  double enu_yaw_cos_sum_{0.0};
};
}  // namespace vectornav
#endif  // VECTORNAV__VN_SENSOR_MSGS_HPP_
