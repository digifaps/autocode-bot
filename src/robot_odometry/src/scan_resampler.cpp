/**
 * @file scan_resampler.cpp
 * @brief Resample /scan onto a fixed angular grid centred on straight ahead, for rf2o.
 *
 * The D500 driver publishes 0..360 deg with a point count that changes from scan to scan
 * (501-505), so the angle of index i drifts between scans. rf2o_laser_odometry assumes a fixed
 * number of columns at angles -fov/2 + i*fov/(n-1), centred on the laser's x axis. Every output
 * column takes the nearest input beam by angle; invalid ranges become 0 ("no measurement" in rf2o).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <csignal>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

class ScanResampler : public rclcpp::Node
{
public:
  ScanResampler() : Node("scan_resampler")
  {
    const auto input = declare_parameter<std::string>("input_topic", "scan");
    const auto output = declare_parameter<std::string>("output_topic", "scan_fixed");
    // 0.72 deg; the D500 delivers ~503 beams per turn
    n_ = static_cast<int>(declare_parameter<int>("num_beams", 500));
    if (n_ < 2) {
      n_ = 500;
    }
    inc_ = 2.0 * M_PI / n_;
    fov_ = inc_ * (n_ - 1);  // full circle without duplicating +-180 deg, symmetric around 0
    angles_.resize(n_);
    for (int i = 0; i < n_; ++i) {
      angles_[i] = -0.5 * fov_ + i * inc_;
    }
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output, 5);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {on_scan(std::move(msg));});
    RCLCPP_INFO(get_logger(), "scan_resampler: %s -> %s, %d beams over %.2f deg centred on 0",
      input.c_str(), output.c_str(), n_, fov_ * 180.0 / M_PI);
  }

private:
  void on_scan(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    const size_t m = msg->ranges.size();
    if (m < 2 || msg->angle_increment == 0.0f) {
      return;
    }
    const double ainc = msg->angle_increment;
    const float rmin = std::max(msg->range_min, 1e-3f);
    const float rmax = msg->range_max;
    const bool full_turn = ainc * m >= 2.0 * M_PI - 1.5 * std::abs(ainc);

    auto out = std::make_unique<sensor_msgs::msg::LaserScan>();
    out->header = msg->header;
    out->angle_min = static_cast<float>(-0.5 * fov_);
    out->angle_max = static_cast<float>(0.5 * fov_);
    out->angle_increment = static_cast<float>(inc_);
    out->time_increment = msg->time_increment * static_cast<float>(m) / static_cast<float>(n_);
    out->scan_time = msg->scan_time;
    out->range_min = msg->range_min;
    out->range_max = msg->range_max;
    out->ranges.assign(n_, 0.0f);
    for (int i = 0; i < n_; ++i) {
      double rel = std::fmod(angles_[i] - msg->angle_min, 2.0 * M_PI);
      if (rel < 0) {
        rel += 2.0 * M_PI;
      }
      long idx = std::lround(rel / ainc);
      if (full_turn) {
        idx %= static_cast<long>(m);
      } else if (idx < 0 || idx >= static_cast<long>(m)) {
        continue;  // partial scan: no measurement outside it
      }
      const float r = msg->ranges[idx];
      if (std::isfinite(r) && r >= rmin && r <= rmax) {
        out->ranges[i] = r;
      }
    }
    pub_->publish(std::move(out));
  }

  int n_;
  double inc_;
  double fov_;
  std::vector<double> angles_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
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
  rclcpp::spin(std::make_shared<ScanResampler>());
  rclcpp::shutdown();
  return 0;
}
