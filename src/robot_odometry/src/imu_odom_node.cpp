/**
 * @file imu_odom_node.cpp
 * @brief Odometry from IMU heading + laser odometry (rf2o) forward speed.
 *
 * Publishes odom -> base_footprint (TF) and nav_msgs/Odometry on /odom:
 * - Heading: integrated IMU yaw rate. The gyro vector is projected onto the measured gravity
 *   direction, so this works for any IMU mounting (on this robot the stereo board stands
 *   upright and tilted, so gravity is mostly on the IMU's y axis, not z).
 * - Position: forward speed from rf2o_laser_odometry (/odom_rf2o twist.linear.x), integrated
 *   along the IMU heading. Without laser odometry the position stays at the origin.
 *
 * base_footprint, not base_link: robot_state_publisher already gives base_link its parent.
 * TF chain: map -> odom (slam_toolbox) -> base_footprint (this node) -> base_link -> base_laser.
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr double kG = 9.80665;

double stamp_s(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

class ImuOdomNode : public rclcpp::Node
{
public:
  ImuOdomNode() : Node("imu_odom_node")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const double rate = declare_parameter<double>("publish_rate", 20.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto laser_topic = declare_parameter<std::string>("laser_odom_topic", "odom_rf2o");
    gravity_samples_ = static_cast<int>(declare_parameter<int>("gravity_samples", 100));  // ~1 s
    // rf2o occasionally mis-matches a scan (seen on the robot: 1.96 m/s while standing still).
    // Readings beyond what the robot can physically do are dropped.
    max_speed_ = declare_parameter<double>("max_laser_speed", 1.0);    // m/s
    max_accel_ = declare_parameter<double>("max_laser_accel", 3.0);    // m/s^2
    // rf2o reads ~0.003 m/s (p99 ~0.026) while standing still; treat that as zero
    deadband_ = declare_parameter<double>("laser_speed_deadband", 0.02);  // m/s

    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw", rclcpp::QoS(50),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});
    if (!laser_topic.empty()) {
      laser_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        laser_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_laser(*m);});
    }
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this] {publish();});
    status_timer_ = create_wall_timer(std::chrono::seconds(30), [this] {status();});
    RCLCPP_INFO(get_logger(),
      "imu_odom_node: %s -> %s, heading from IMU (gravity-projected), position from %s",
      odom_frame_.c_str(), base_frame_.c_str(),
      laser_topic.empty() ? "nothing (fixed at origin)" : laser_topic.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    const auto & a = msg.linear_acceleration;
    const double t = stamp_s(msg.header.stamp);
    if (!have_up_) {
      // Gravity direction from the first samples (robot still at startup, as for the gyro bias)
      acc_sum_[0] += a.x;
      acc_sum_[1] += a.y;
      acc_sum_[2] += a.z;
      if (++acc_n_ >= gravity_samples_) {
        const double n = std::sqrt(acc_sum_[0] * acc_sum_[0] + acc_sum_[1] * acc_sum_[1] +
          acc_sum_[2] * acc_sum_[2]);
        if (n > 0) {
          for (int i = 0; i < 3; ++i) {
            up_[i] = acc_sum_[i] / n;
          }
          have_up_ = true;
          RCLCPP_INFO(get_logger(), "gravity in IMU frame: (%.2f, %.2f, %.2f); yaw rate = gyro . up",
            up_[0], up_[1], up_[2]);
        }
      }
      last_imu_t_ = t;
      return;
    }
    // Slowly track the gravity direction while the robot is not accelerating
    const double norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (std::abs(norm - kG) < 0.3) {
      constexpr double k = 0.002;
      const std::array<double, 3> acc{a.x / norm, a.y / norm, a.z / norm};
      double n = 0;
      for (int i = 0; i < 3; ++i) {
        up_[i] = (1 - k) * up_[i] + k * acc[i];
        n += up_[i] * up_[i];
      }
      n = std::sqrt(n);
      for (auto & u : up_) {
        u /= n;
      }
    }
    const auto & w = msg.angular_velocity;
    const double rate = w.x * up_[0] + w.y * up_[1] + w.z * up_[2];
    if (last_imu_t_ > 0) {
      const double dt = t - last_imu_t_;
      if (dt > 0.0 && dt < 0.5) {
        yaw_ = wrap(yaw_ + rate * dt);
      }
    }
    last_imu_t_ = t;
    yaw_rate_ = rate;
  }

  void on_laser(const nav_msgs::msg::Odometry & msg)
  {
    const double t = stamp_s(msg.header.stamp);
    double v = msg.twist.twist.linear.x;
    ++laser_msgs_;
    const double dt = last_laser_t_ > 0 ? t - last_laser_t_ : 0.0;
    last_laser_t_ = t;
    if (!(dt > 0.0 && dt < 1.0)) {
      return;
    }
    // Gate against the last accepted speed: a real ramp passes, a scan-match spike does not.
    // A jump that persists (3 consistent readings, 0.3 s) is real motion and is accepted.
    if (!std::isfinite(v) || std::abs(v) > max_speed_) {
      ++laser_rejected_;
      pending_.clear();
      v = v_ * 0.5;
    } else if (std::abs(v - v_) > max_accel_ * dt + 0.05) {
      pending_.push_back(v);
      bool confirmed = false;
      if (pending_.size() >= 3) {
        const auto first = pending_.end() - 3;
        const auto mm = std::minmax_element(first, pending_.end());
        confirmed = *mm.second - *mm.first < 0.1;
      }
      if (confirmed) {
        pending_.clear();
      } else {
        ++laser_rejected_;
        v = v_ * 0.5;  // hold a decaying estimate through the glitch
      }
    } else {
      pending_.clear();
    }
    v_ = v;
    if (std::abs(v) < deadband_) {
      v = 0.0;
    }
    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
  }

  void publish()
  {
    const auto now = this->now();
    const double qz = std::sin(yaw_ * 0.5);
    const double qw = std::cos(yaw_ * 0.5);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.rotation.z = qz;
      t.transform.rotation.w = qw;
      tf_->sendTransform(t);
    }
    auto o = std::make_unique<nav_msgs::msg::Odometry>();
    o->header.stamp = now;
    o->header.frame_id = odom_frame_;
    o->child_frame_id = base_frame_;
    o->pose.pose.position.x = x_;
    o->pose.pose.position.y = y_;
    o->pose.pose.orientation.z = qz;
    o->pose.pose.orientation.w = qw;
    o->twist.twist.linear.x = v_;
    o->twist.twist.angular.z = yaw_rate_;
    // Rough, diagonal: x, y, z, roll, pitch, yaw
    o->pose.covariance[0] = o->pose.covariance[7] = 0.01;
    o->pose.covariance[14] = o->pose.covariance[21] = o->pose.covariance[28] = 1e6;
    o->pose.covariance[35] = 0.02;
    o->twist.covariance[0] = 0.01;
    o->twist.covariance[7] = o->twist.covariance[14] = 1e6;
    o->twist.covariance[21] = o->twist.covariance[28] = 1e6;
    o->twist.covariance[35] = 0.005;
    odom_pub_->publish(std::move(o));
  }

  void status()
  {
    RCLCPP_INFO(get_logger(), "odom x=%.2f y=%.2f yaw=%.1f deg; laser odom msgs %d (%d rejected) in 30 s",
      x_, y_, yaw_ * 180.0 / M_PI, laser_msgs_, laser_rejected_);
    laser_msgs_ = 0;
    laser_rejected_ = 0;
  }

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  int gravity_samples_;
  double max_speed_;
  double max_accel_;
  double deadband_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double v_ = 0.0;
  double yaw_rate_ = 0.0;
  bool have_up_ = false;
  std::array<double, 3> up_{0.0, 0.0, 1.0};
  std::array<double, 3> acc_sum_{0.0, 0.0, 0.0};
  int acc_n_ = 0;
  double last_imu_t_ = 0.0;
  double last_laser_t_ = 0.0;
  int laser_msgs_ = 0;
  int laser_rejected_ = 0;
  std::vector<double> pending_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

// launch sends SIGINT twice on Ctrl+C. rclcpp chains to the handler that was installed before
// rclcpp::init and restores it when it uninstalls its own during shutdown; with the default
// handler the second SIGINT would kill the process (exit -2) instead of a clean exit.
void ignore_signal(int) {}

int main(int argc, char ** argv)
{
  struct sigaction sa {};
  sa.sa_handler = ignore_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdomNode>());
  rclcpp::shutdown();
  return 0;
}
