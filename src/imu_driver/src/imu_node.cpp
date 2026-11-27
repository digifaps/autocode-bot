/**
 * @file imu_node.cpp
 * @brief ICM20948 + AK09916 IMU driver for the Waveshare IMX219-83 stereo board.
 *
 * Publishes imu/data_raw (sensor_msgs/Imu) at publish_rate and imu/mag (MagneticField) whenever
 * the magnetometer has a new sample, plus a status on /diagnostics once per second.
 *
 * - One I2C burst read per sample: accel, gyro, temperature and the AK09916 block, which the
 *   ICM20948's internal I2C master copies into EXT_SLV_SENS_DATA (magnetometer in continuous
 *   100 Hz mode).
 * - Gyro noise is filtered by the chip's DLPF (gyro_dlpf_mode); the accelerometer gets a light EMA.
 * - Gyro bias is averaged over the first gyro_calibration_time seconds (robot must be still), then
 *   tracked while the robot stands still: the bias drifts as the board warms up (seen: 0.3 deg/s
 *   after 45 min, i.e. the heading turned 19 deg/min). Every gyro_bias_window seconds the window is
 *   checked for stillness (low gyro and accelerometer spread, mean close to the current bias, so a
 *   steady turn is never taken for bias) and the bias moves a fraction towards the window mean.
 * - A missing or failing chip never stops the node: it retries every 2 s and reports ERROR on
 *   /diagnostics, so Foxglove shows the problem. A chip reset behind our back (another process)
 *   is detected once per second and the chip is reconfigured.
 *
 * The init sequence follows what the Python node did through the Pimoroni icm20948 library.
 */

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
// ICM20948 registers (bank in comment)
constexpr uint8_t REG_BANK_SEL = 0x7F;
constexpr uint8_t REG_WHO_AM_I = 0x00;        // 0
constexpr uint8_t CHIP_ID = 0xEA;
constexpr uint8_t REG_USER_CTRL = 0x03;       // 0
constexpr uint8_t USER_CTRL_I2C_MST_EN = 0x20;
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;      // 0
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;      // 0
constexpr uint8_t REG_INT_PIN_CFG = 0x0F;     // 0
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;    // 0: accel(6) gyro(6) temp(2) EXT_SLV_SENS_DATA(9)
constexpr uint8_t REG_EXT_SLV_SENS_DATA_00 = 0x3B;  // 0
constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00;  // 2
constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;    // 2
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1 = 0x10;  // 2
constexpr uint8_t REG_ACCEL_SMPLRT_DIV_2 = 0x11;  // 2
constexpr uint8_t REG_ACCEL_CONFIG = 0x14;     // 2
constexpr uint8_t REG_I2C_MST_CTRL = 0x01;     // 3
constexpr uint8_t REG_I2C_MST_DELAY_CTRL = 0x02;  // 3
constexpr uint8_t REG_I2C_SLV0_ADDR = 0x03;    // 3
constexpr uint8_t REG_I2C_SLV0_REG = 0x04;     // 3
constexpr uint8_t REG_I2C_SLV0_CTRL = 0x05;    // 3
constexpr uint8_t REG_I2C_SLV0_DO = 0x06;      // 3
// AK09916 magnetometer behind the ICM20948 I2C master
constexpr uint8_t AK_ADDR = 0x0C;
constexpr uint8_t AK_WIA = 0x01;
constexpr uint8_t AK_CHIP_ID = 0x09;
constexpr uint8_t AK_ST1 = 0x10;   // ST1, HXL..HZH, TMPS, ST2 = 9 bytes; reading ST2 releases the lock
constexpr uint8_t AK_CNTL2 = 0x31;
constexpr uint8_t AK_CNTL3 = 0x32;
constexpr uint8_t AK_MODE_CONT_100HZ = 0x08;
constexpr uint8_t AK_ST1_DRDY = 0x01;
constexpr uint8_t AK_ST2_HOFL = 0x08;
constexpr double AK_UT_PER_LSB = 0.15;

constexpr int BURST_LEN = 12 + 2 + 9;
constexpr double ICM_INTERNAL_RATE_HZ = 1125.0;
constexpr std::array<double, 4> ACCEL_LSB_PER_G{16384.0, 8192.0, 4096.0, 2048.0};
constexpr std::array<double, 4> GYRO_LSB_PER_DPS{131.0, 65.5, 32.8, 16.4};
constexpr double G = 9.80665;
constexpr double DEG2RAD = M_PI / 180.0;

class I2cError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Minimal /dev/i2c-N access for one 7-bit device.
class I2cDevice
{
public:
  ~I2cDevice() {close();}

  void open(int bus, uint8_t addr)
  {
    close();
    const std::string path = "/dev/i2c-" + std::to_string(bus);
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw I2cError(path + ": " + std::strerror(errno));
    }
    // FORCE first (as the Python node did with smbus2 force=True), plain SLAVE as fallback
    if (ioctl(fd_, I2C_SLAVE_FORCE, addr) < 0 && ioctl(fd_, I2C_SLAVE, addr) < 0) {
      const std::string err = std::strerror(errno);
      close();
      throw I2cError(path + " select address: " + err);
    }
    addr_ = addr;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const {return fd_ >= 0;}
  uint8_t addr() const {return addr_;}

  void write(uint8_t reg, uint8_t value)
  {
    const uint8_t buf[2] = {reg, value};
    if (::write(fd_, buf, 2) != 2) {
      throw I2cError(std::string("write 0x") + hex(reg) + ": " + std::strerror(errno));
    }
    std::this_thread::sleep_for(100us);  // as the Pimoroni library
  }

  uint8_t read(uint8_t reg)
  {
    uint8_t v = 0;
    read_block(reg, &v, 1);
    return v;
  }

  void read_block(uint8_t reg, uint8_t * out, uint16_t len)
  {
    uint8_t r = reg;
    i2c_msg msgs[2];
    msgs[0] = {addr_, 0, 1, &r};
    msgs[1] = {addr_, I2C_M_RD, len, out};
    i2c_rdwr_ioctl_data data{msgs, 2};
    if (ioctl(fd_, I2C_RDWR, &data) < 0) {
      throw I2cError(std::string("read 0x") + hex(reg) + ": " + std::strerror(errno));
    }
  }

private:
  static std::string hex(uint8_t v)
  {
    char b[3];
    std::snprintf(b, sizeof(b), "%02x", v);
    return b;
  }
  int fd_ = -1;
  uint8_t addr_ = 0;
};

int16_t be16(const uint8_t * p) {return static_cast<int16_t>((p[0] << 8) | p[1]);}
int16_t le16(const uint8_t * p) {return static_cast<int16_t>((p[1] << 8) | p[0]);}
}  // namespace

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("imu_node")
  {
    bus_ = static_cast<int>(declare_parameter<int>("i2c_bus", 8));
    addr_param_ = static_cast<int>(declare_parameter<int>("i2c_addr", 0x68));
    rate_ = declare_parameter<double>("publish_rate", 100.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    // Software EMA on the accelerometer only (0 < alpha <= 1; 1 = off). Gyro uses the hardware DLPF.
    alpha_ = declare_parameter<double>("lowpass_alpha", 0.08);
    // GYRO_DLPFCFG: 0=197 Hz, 1=152, 2=120, 3=51, 4=24, 5=12, 6=6 Hz (3 dB bandwidth)
    dlpf_ = static_cast<int>(declare_parameter<int>("gyro_dlpf_mode", 4));
    // Stationary seconds averaged for the gyro bias (0 = off). 5 s -> ~1 deg/min residual drift.
    calib_time_ = declare_parameter<double>("gyro_calibration_time", 5.0);
    calib_max_std_ = declare_parameter<double>("gyro_calibration_max_std", 2.0);    // deg/s
    calib_max_bias_ = declare_parameter<double>("gyro_calibration_max_bias", 5.0);  // deg/s
    // Bias tracking while still. Measured standing still (DLPF 4, 2 s windows): gyro std
    // 0.06-0.10 deg/s per axis, window means within +-0.02 deg/s of each other.
    track_bias_ = declare_parameter<bool>("gyro_bias_tracking", true);
    track_window_ = declare_parameter<double>("gyro_bias_window", 2.0);            // s
    still_gyro_std_ = declare_parameter<double>("gyro_still_max_std", 0.2);        // deg/s
    still_accel_std_ = declare_parameter<double>("accel_still_max_std", 0.01);     // g
    // A window whose mean is further than this from the current bias is motion, not drift
    track_max_step_ = declare_parameter<double>("gyro_bias_max_step", 0.5);        // deg/s
    track_gain_ = declare_parameter<double>("gyro_bias_gain", 0.2);  // per accepted window
    log_interval_ = static_cast<int>(declare_parameter<int>("log_interval", 0));
    if (!(alpha_ > 0.0 && alpha_ <= 1.0)) {
      RCLCPP_WARN(get_logger(), "lowpass_alpha=%.3f out of range (0, 1]; using 1.0 (off)", alpha_);
      alpha_ = 1.0;
    }
    if (dlpf_ < 0 || dlpf_ > 7) {
      RCLCPP_WARN(get_logger(), "gyro_dlpf_mode=%d out of range 0..7; using 4", dlpf_);
      dlpf_ = 4;
    }

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_)),
      [this] {tick();});
    diag_timer_ = create_wall_timer(1s, [this] {publish_diagnostics();});
    try_init();
  }

private:
  enum class State {Init, Calibrating, Running};

  // ------------------------------------------------------------------ chip access
  void bank(uint8_t b)
  {
    if (bank_ != b) {
      dev_.write(REG_BANK_SEL, static_cast<uint8_t>(b << 4));
      bank_ = b;
    }
  }

  void trigger_mag_io()
  {
    const uint8_t user = dev_.read(REG_USER_CTRL);
    dev_.write(REG_USER_CTRL, user | USER_CTRL_I2C_MST_EN);
    std::this_thread::sleep_for(5ms);
    dev_.write(REG_USER_CTRL, user);
  }

  uint8_t mag_read(uint8_t reg)
  {
    bank(3);
    dev_.write(REG_I2C_SLV0_ADDR, AK_ADDR | 0x80);
    dev_.write(REG_I2C_SLV0_REG, reg);
    dev_.write(REG_I2C_SLV0_DO, 0xFF);
    dev_.write(REG_I2C_SLV0_CTRL, 0x80 | 1);
    bank(0);
    trigger_mag_io();
    return dev_.read(REG_EXT_SLV_SENS_DATA_00);
  }

  void mag_write(uint8_t reg, uint8_t value)
  {
    bank(3);
    dev_.write(REG_I2C_SLV0_ADDR, AK_ADDR);
    dev_.write(REG_I2C_SLV0_REG, reg);
    dev_.write(REG_I2C_SLV0_DO, value);
    dev_.write(REG_I2C_SLV0_CTRL, 0x80 | 1);
    bank(0);
    trigger_mag_io();
  }

  /// Find the chip (0x68 / 0x69), reset and configure it. Throws I2cError / runtime_error.
  void open_and_configure()
  {
    std::string who_report;
    bool found = false;
    for (int addr : {addr_param_, addr_param_ == 0x68 ? 0x69 : 0x68}) {
      dev_.open(bus_, static_cast<uint8_t>(addr));
      bank_ = 0xFF;
      try {
        bank(0);
        const uint8_t who = dev_.read(REG_WHO_AM_I);
        char b[32];
        std::snprintf(b, sizeof(b), "0x%02x=0x%02x ", addr, who);
        who_report += b;
        if (who == CHIP_ID) {
          found = true;
          break;
        }
      } catch (const I2cError & e) {
        char b[32];
        std::snprintf(b, sizeof(b), "0x%02x=no answer ", addr);
        who_report += b;
      }
    }
    if (!found) {
      dev_.close();
      throw std::runtime_error("ICM20948 not found on /dev/i2c-" + std::to_string(bus_) +
              " (WHO_AM_I " + who_report + ", expected 0xea)");
    }
    // Reset and wake (Pimoroni ICM20948.__init__)
    dev_.write(REG_PWR_MGMT_1, 0x80);
    std::this_thread::sleep_for(10ms);
    bank_ = 0xFF;  // reset puts the chip back in bank 0
    bank(0);
    dev_.write(REG_PWR_MGMT_1, 0x01);
    dev_.write(REG_PWR_MGMT_2, 0x00);
    bank(2);
    // Sample rate above publish_rate so every tick sees a fresh sample
    const double odr = std::min(ICM_INTERNAL_RATE_HZ, std::max(2.0 * rate_, 100.0));
    const int div = std::max(0, static_cast<int>(std::lround(ICM_INTERNAL_RATE_HZ / odr)) - 1);
    dev_.write(REG_GYRO_SMPLRT_DIV, static_cast<uint8_t>(div & 0xFF));
    // Gyro: DLPF on with dlpf_, full scale +-250 dps
    dev_.write(REG_GYRO_CONFIG_1, static_cast<uint8_t>(0x01 | ((dlpf_ & 0x07) << 3) | (0 << 1)));
    dev_.write(REG_ACCEL_SMPLRT_DIV_1, static_cast<uint8_t>((div >> 8) & 0x0F));
    dev_.write(REG_ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(div & 0xFF));
    // Accel: DLPF on (mode 5), full scale +-16 g (as the Pimoroni defaults)
    dev_.write(REG_ACCEL_CONFIG, static_cast<uint8_t>(0x01 | (5 << 3) | (3 << 1)));
    const uint8_t accel_cfg = dev_.read(REG_ACCEL_CONFIG);
    const uint8_t gyro_cfg = dev_.read(REG_GYRO_CONFIG_1);
    accel_scale_ = 1.0 / ACCEL_LSB_PER_G[(accel_cfg >> 1) & 0x03];
    gyro_scale_ = 1.0 / GYRO_LSB_PER_DPS[(gyro_cfg >> 1) & 0x03];
    bank(0);
    dev_.write(REG_INT_PIN_CFG, 0x30);
    bank(3);
    dev_.write(REG_I2C_MST_CTRL, 0x4D);
    dev_.write(REG_I2C_MST_DELAY_CTRL, 0x01);
    const uint8_t wia = mag_read(AK_WIA);
    if (wia != AK_CHIP_ID) {
      char b[96];
      std::snprintf(b, sizeof(b), "AK09916 magnetometer not found (WIA 0x%02x, expected 0x09)", wia);
      throw std::runtime_error(b);
    }
    mag_write(AK_CNTL3, 0x01);  // soft reset
    for (int i = 0; i < 100 && mag_read(AK_CNTL3) == 0x01; ++i) {
      std::this_thread::sleep_for(1ms);
    }
    mag_write(AK_CNTL2, AK_MODE_CONT_100HZ);
    const uint8_t mode = mag_read(AK_CNTL2);
    if (mode != AK_MODE_CONT_100HZ) {
      char b[96];
      std::snprintf(b, sizeof(b), "AK09916 CNTL2 readback 0x%02x, expected 0x08", mode);
      throw std::runtime_error(b);
    }
    // Slave 0: read ST1..ST2 (9 bytes) every sample into EXT_SLV_SENS_DATA_00
    bank(3);
    dev_.write(REG_I2C_SLV0_ADDR, AK_ADDR | 0x80);
    dev_.write(REG_I2C_SLV0_REG, AK_ST1);
    dev_.write(REG_I2C_SLV0_CTRL, 0x80 | 9);
    bank(0);
    const uint8_t user = dev_.read(REG_USER_CTRL);
    dev_.write(REG_USER_CTRL, user | USER_CTRL_I2C_MST_EN);  // keep the I2C master running
    std::this_thread::sleep_for(50ms);  // first magnetometer sample
    odr_ = ICM_INTERNAL_RATE_HZ / (div + 1);
  }

  struct Sample
  {
    std::array<double, 3> accel;  // g
    std::array<double, 3> gyro;   // deg/s
    double temp;                  // deg C
    uint8_t st1;
    std::array<int16_t, 3> mag;
    uint8_t st2;
  };

  Sample read_burst()
  {
    uint8_t d[BURST_LEN];
    dev_.read_block(REG_ACCEL_XOUT_H, d, BURST_LEN);
    Sample s;
    for (int i = 0; i < 3; ++i) {
      s.accel[i] = be16(d + 2 * i) * accel_scale_;
      s.gyro[i] = be16(d + 6 + 2 * i) * gyro_scale_;
      s.mag[i] = le16(d + 15 + 2 * i);
    }
    s.temp = be16(d + 12) / 333.87 + 21.0;  // datasheet: room temp offset 21 C, 333.87 LSB/C
    s.st1 = d[14];
    s.st2 = d[22];
    return s;
  }

  // ------------------------------------------------------------------ state machine
  void try_init()
  {
    const auto now = std::chrono::steady_clock::now();
    next_init_ = now + 2s;
    try {
      open_and_configure();
      last_mag_time_ = now;
      accel_filtered_valid_ = false;
      have_last_mag_ = false;
      consecutive_errors_ = 0;
      init_error_.clear();
      char b[200];
      std::snprintf(b, sizeof(b), "ICM20948 initialized on /dev/i2c-%d addr 0x%02x, ODR %.1f Hz, "
        "accel %.3f mg/LSB, gyro %.2f mdps/LSB", bus_, dev_.addr(), odr_, accel_scale_ * 1000,
        gyro_scale_ * 1000);
      RCLCPP_INFO(get_logger(), "%s", b);
      if (!bias_done_ && calib_time_ > 0) {
        state_ = State::Calibrating;
        calib_samples_.clear();
        calib_end_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(calib_time_));
        RCLCPP_INFO(get_logger(), "Calibrating gyro bias for %.0f s; keep the robot still", calib_time_);
      } else {
        bias_done_ = true;
        start_running();
      }
    } catch (const std::exception & e) {
      dev_.close();
      if (init_error_ != e.what()) {
        RCLCPP_ERROR(get_logger(), "%s; retrying every 2 s", e.what());
      }
      init_error_ = e.what();
      state_ = State::Init;
    }
  }

  void start_running()
  {
    if (state_ != State::Running) {
      RCLCPP_INFO(get_logger(), "IMU running at %.0f Hz (burst read, mag continuous 100 Hz, "
        "gyro DLPF mode %d)", rate_, dlpf_);
    }
    state_ = State::Running;
  }

  void reinit(const std::string & reason)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_reinit_) {
      return;
    }
    next_reinit_ = now + 5s;
    RCLCPP_WARN(get_logger(), "Re-initializing ICM20948: %s", reason.c_str());
    ++reinits_;
    dev_.close();
    try_init();
    if (state_ == State::Init) {
      next_reinit_ = std::chrono::steady_clock::now() + 1s;
    }
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (state_ == State::Init) {
      if (now >= next_init_) {
        try_init();
      }
      return;
    }
    Sample s;
    try {
      s = read_burst();
    } catch (const I2cError & e) {
      handle_error(e.what());
      return;
    }
    if (consecutive_errors_) {
      RCLCPP_INFO(get_logger(), "IMU reads recovered after %d errors", consecutive_errors_);
      consecutive_errors_ = 0;
    }
    if (state_ == State::Calibrating) {
      calib_samples_.push_back(s.gyro);
      if (now >= calib_end_) {
        finish_calibration();
        start_running();
      }
      return;
    }
    temp_ = s.temp;
    publish_sample(s, now);
    if (track_bias_) {
      track_bias(s);
    }
  }

  /// Running per-axis sums over one window; on a still window, nudge the bias towards its mean.
  void track_bias(const Sample & s)
  {
    for (int i = 0; i < 3; ++i) {
      win_g_[i] += s.gyro[i];
      win_g2_[i] += s.gyro[i] * s.gyro[i];
      win_a_[i] += s.accel[i];
      win_a2_[i] += s.accel[i] * s.accel[i];
    }
    if (++win_n_ < std::max(10, static_cast<int>(track_window_ * rate_))) {
      return;
    }
    const double n = win_n_;
    std::array<double, 3> mean{};
    double g_sd = 0, a_sd = 0, step = 0;
    for (int i = 0; i < 3; ++i) {
      mean[i] = win_g_[i] / n;
      g_sd = std::max(g_sd, std::sqrt(std::max(0.0, win_g2_[i] / n - mean[i] * mean[i])));
      const double am = win_a_[i] / n;
      a_sd = std::max(a_sd, std::sqrt(std::max(0.0, win_a2_[i] / n - am * am)));
      step = std::max(step, std::abs(mean[i] - bias_[i] / DEG2RAD));
    }
    win_n_ = 0;
    win_g_ = win_g2_ = win_a_ = win_a2_ = {0, 0, 0};
    ++track_windows_;
    // Without a startup calibration the bias is still 0: allow up to the calibration limit once.
    const double max_step = track_accepted_ || calib_ok_ ? track_max_step_ : calib_max_bias_;
    if (g_sd > still_gyro_std_ || a_sd > still_accel_std_ || step > max_step) {
      return;  // moving (or a steady turn)
    }
    for (int i = 0; i < 3; ++i) {
      bias_[i] += track_gain_ * (mean[i] * DEG2RAD - bias_[i]);
    }
    ++track_accepted_;
    char b[160];
    std::snprintf(b, sizeof(b), "x=%.3f y=%.3f z=%.3f deg/s (tracked, %.1f C)",
      bias_[0] / DEG2RAD, bias_[1] / DEG2RAD, bias_[2] / DEG2RAD, temp_);
    calib_status_ = b;
    if (track_accepted_ == 1 || track_accepted_ % 150 == 0) {  // first, then every ~5 min still
      RCLCPP_INFO(get_logger(), "Gyro bias %s", b);
    }
  }

  void finish_calibration()
  {
    bias_done_ = true;
    const size_t n = calib_samples_.size();
    if (n < 10) {
      calib_status_ = "too few samples, bias not applied";
      RCLCPP_WARN(get_logger(), "Gyro calibration: too few samples (%zu); bias not applied", n);
      return;
    }
    std::array<double, 3> mean{0, 0, 0}, sd{0, 0, 0};
    for (const auto & g : calib_samples_) {
      for (int i = 0; i < 3; ++i) {
        mean[i] += g[i];
      }
    }
    for (auto & m : mean) {
      m /= n;
    }
    for (const auto & g : calib_samples_) {
      for (int i = 0; i < 3; ++i) {
        sd[i] += (g[i] - mean[i]) * (g[i] - mean[i]);
      }
    }
    for (auto & v : sd) {
      v = std::sqrt(v / n);
    }
    const double max_sd = std::max({sd[0], sd[1], sd[2]});
    const double max_mean = std::max({std::abs(mean[0]), std::abs(mean[1]), std::abs(mean[2])});
    char b[300];
    if (max_sd > calib_max_std_ || max_mean > calib_max_bias_) {
      std::snprintf(b, sizeof(b), "Gyro calibration: robot moving (mean %.2f/%.2f/%.2f, std "
        "%.2f/%.2f/%.2f deg/s; limits |mean| %.1f, std %.1f); bias NOT applied. Restart the node "
        "while stationary.", mean[0], mean[1], mean[2], sd[0], sd[1], sd[2], calib_max_bias_,
        calib_max_std_);
      RCLCPP_WARN(get_logger(), "%s", b);
      calib_status_ = "robot moved during calibration, bias not applied";
      calib_ok_ = false;
      return;
    }
    for (int i = 0; i < 3; ++i) {
      bias_[i] = mean[i] * DEG2RAD;
    }
    calib_ok_ = true;
    std::snprintf(b, sizeof(b), "x=%.3f y=%.3f z=%.3f deg/s", mean[0], mean[1], mean[2]);
    calib_status_ = b;
    RCLCPP_INFO(get_logger(), "Gyro bias (deg/s): x=%.3f y=%.3f z=%.3f (std %.3f/%.3f/%.3f, %zu "
      "samples)", mean[0], mean[1], mean[2], sd[0], sd[1], sd[2], n);
  }

  void publish_sample(const Sample & s, std::chrono::steady_clock::time_point now)
  {
    const auto stamp = this->now();
    if (!accel_filtered_valid_ || alpha_ >= 1.0) {
      accel_filtered_ = s.accel;
      accel_filtered_valid_ = true;
    } else {
      for (int i = 0; i < 3; ++i) {
        accel_filtered_[i] = alpha_ * s.accel[i] + (1 - alpha_) * accel_filtered_[i];
      }
    }
    auto imu = std::make_unique<sensor_msgs::msg::Imu>();
    imu->header.stamp = stamp;
    imu->header.frame_id = frame_id_;
    imu->linear_acceleration.x = accel_filtered_[0] * G;
    imu->linear_acceleration.y = accel_filtered_[1] * G;
    imu->linear_acceleration.z = accel_filtered_[2] * G;
    imu->angular_velocity.x = s.gyro[0] * DEG2RAD - bias_[0];
    imu->angular_velocity.y = s.gyro[1] * DEG2RAD - bias_[1];
    imu->angular_velocity.z = s.gyro[2] * DEG2RAD - bias_[2];
    imu->orientation_covariance[0] = -1.0;  // no orientation estimate
    imu_pub_->publish(std::move(imu));
    ++count_;
    ++count_window_;

    // Magnetometer: only new, non-overflowed samples (~100 Hz from the AK09916)
    const bool zero = s.mag[0] == 0 && s.mag[1] == 0 && s.mag[2] == 0;
    const bool changed = !have_last_mag_ || s.mag != last_mag_;
    if (!zero && !(s.st2 & AK_ST2_HOFL) && ((s.st1 & AK_ST1_DRDY) || changed)) {
      last_mag_ = s.mag;
      have_last_mag_ = true;
      last_mag_time_ = now;
      auto mag = std::make_unique<sensor_msgs::msg::MagneticField>();
      mag->header.stamp = stamp;
      mag->header.frame_id = frame_id_;
      mag->magnetic_field.x = s.mag[0] * AK_UT_PER_LSB * 1e-6;
      mag->magnetic_field.y = s.mag[1] * AK_UT_PER_LSB * 1e-6;
      mag->magnetic_field.z = s.mag[2] * AK_UT_PER_LSB * 1e-6;
      mag_pub_->publish(std::move(mag));
      ++mag_window_;
    }

    if (count_ % std::max(1, static_cast<int>(rate_)) == 0) {  // ~1 Hz
      check_health(now);
    }
    if (log_interval_ > 0 && count_ % log_interval_ == 0) {
      RCLCPP_INFO(get_logger(), "[IMU] accel=(%.2f, %.2f, %.2f) m/s^2 | gyro=(%.2f, %.2f, %.2f) "
        "deg/s | count=%lu", accel_filtered_[0] * G, accel_filtered_[1] * G, accel_filtered_[2] * G,
        s.gyro[0] - bias_[0] / DEG2RAD, s.gyro[1] - bias_[1] / DEG2RAD,
        s.gyro[2] - bias_[2] / DEG2RAD, static_cast<unsigned long>(count_));
    }
  }

  /// Detect a chip that was reset or reconfigured behind our back (reads still succeed then).
  void check_health(std::chrono::steady_clock::time_point now)
  {
    uint8_t bank_sel = 0, user = 0;
    try {
      bank_sel = dev_.read(REG_BANK_SEL);
      if (bank_sel == 0) {
        user = dev_.read(REG_USER_CTRL);
      }
    } catch (const I2cError &) {
      return;  // handled by the error counter
    }
    if (bank_sel != 0) {
      reinit("register bank changed to " + std::to_string(bank_sel >> 4));
    } else if (!(user & USER_CTRL_I2C_MST_EN)) {
      reinit("I2C master disabled (chip was reset?)");
    } else if (now - last_mag_time_ > 1s) {
      reinit("no new magnetometer data for 1 s");
    }
  }

  void handle_error(const std::string & what)
  {
    ++consecutive_errors_;
    ++errors_total_;
    if (consecutive_errors_ == 1 || consecutive_errors_ == 10 || consecutive_errors_ % 100 == 0) {
      RCLCPP_WARN(get_logger(), "IMU read error (%d in a row): %s", consecutive_errors_, what.c_str());
    }
    if (consecutive_errors_ >= 20) {
      reinit(std::to_string(consecutive_errors_) + " consecutive read errors");
    }
  }

  void publish_diagnostics()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_diag_).count();
    last_diag_ = now;
    const double hz = dt > 0 ? count_window_ / dt : 0.0;
    const double mag_hz = dt > 0 ? mag_window_ / dt : 0.0;
    count_window_ = 0;
    mag_window_ = 0;

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "IMU (ICM20948)";
    st.hardware_id = "/dev/i2c-" + std::to_string(bus_);
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    char b[200];
    if (state_ == State::Init) {
      st.level = DS::ERROR;
      st.message = init_error_.empty() ? "initializing" : init_error_;
    } else if (state_ == State::Calibrating) {
      st.level = DS::WARN;
      st.message = "calibrating gyro bias - keep the robot still";
    } else if (hz < 0.8 * rate_) {
      st.level = hz > 0 ? DS::WARN : DS::ERROR;
      std::snprintf(b, sizeof(b), "low rate: %.0f Hz (expected %.0f)", hz, rate_);
      st.message = b;
    } else if (mag_hz < 50.0) {
      st.level = DS::WARN;
      std::snprintf(b, sizeof(b), "magnetometer at %.0f Hz (expected ~80-100)", mag_hz);
      st.message = b;
    } else if (!calib_ok_ && calib_time_ > 0 && !track_accepted_) {
      st.level = DS::WARN;
      st.message = "gyro bias not calibrated (robot moved at startup); heading will drift";
    } else {
      st.level = DS::OK;
      std::snprintf(b, sizeof(b), "%.0f Hz, magnetometer %.0f Hz", hz, mag_hz);
      st.message = b;
    }
    auto kv = [](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue p;
        p.key = k;
        p.value = v;
        return p;
      };
    std::snprintf(b, sizeof(b), "%.1f", hz);
    st.values.push_back(kv("rate_hz", b));
    std::snprintf(b, sizeof(b), "%.1f", mag_hz);
    st.values.push_back(kv("mag_rate_hz", b));
    st.values.push_back(kv("gyro_bias", calib_status_.empty() ? "-" : calib_status_));
    if (track_bias_) {
      std::snprintf(b, sizeof(b), "%lu of %lu windows still",
        static_cast<unsigned long>(track_accepted_), static_cast<unsigned long>(track_windows_));
      st.values.push_back(kv("bias_tracking", b));
    }
    if (state_ == State::Running) {
      std::snprintf(b, sizeof(b), "%.1f", temp_);
      st.values.push_back(kv("temperature_c", b));
    }
    st.values.push_back(kv("read_errors", std::to_string(errors_total_)));
    st.values.push_back(kv("reinits", std::to_string(reinits_)));
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  // parameters
  int bus_;
  int addr_param_;
  double rate_;
  std::string frame_id_;
  double alpha_;
  int dlpf_;
  double calib_time_;
  double calib_max_std_;
  double calib_max_bias_;
  int log_interval_;
  bool track_bias_;
  double track_window_;
  double still_gyro_std_;
  double still_accel_std_;
  double track_max_step_;
  double track_gain_;

  // device
  I2cDevice dev_;
  uint8_t bank_ = 0xFF;
  double accel_scale_ = 1.0;
  double gyro_scale_ = 1.0;
  double odr_ = 0.0;

  // state
  State state_ = State::Init;
  std::string init_error_;
  std::chrono::steady_clock::time_point next_init_{};
  std::chrono::steady_clock::time_point next_reinit_{};
  std::chrono::steady_clock::time_point calib_end_{};
  std::chrono::steady_clock::time_point last_mag_time_{};
  std::chrono::steady_clock::time_point last_diag_ = std::chrono::steady_clock::now();
  std::vector<std::array<double, 3>> calib_samples_;
  bool bias_done_ = false;
  bool calib_ok_ = false;
  std::string calib_status_;
  std::array<double, 3> bias_{0, 0, 0};  // rad/s
  // bias tracking window sums (deg/s, g)
  std::array<double, 3> win_g_{0, 0, 0}, win_g2_{0, 0, 0}, win_a_{0, 0, 0}, win_a2_{0, 0, 0};
  int win_n_ = 0;
  uint64_t track_windows_ = 0;
  uint64_t track_accepted_ = 0;
  double temp_ = 0.0;
  std::array<double, 3> accel_filtered_{0, 0, 0};
  bool accel_filtered_valid_ = false;
  std::array<int16_t, 3> last_mag_{0, 0, 0};
  bool have_last_mag_ = false;
  uint64_t count_ = 0;
  int count_window_ = 0;
  int mag_window_ = 0;
  int consecutive_errors_ = 0;
  uint64_t errors_total_ = 0;
  int reinits_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

// launch sends SIGINT twice on Ctrl+C; rclcpp chains to this pre-installed handler and restores
// it on shutdown, so the second SIGINT is ignored instead of killing the process (exit -2).
void ignore_signal(int) {}

int main(int argc, char ** argv)
{
  struct sigaction sa {};
  sa.sa_handler = ignore_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
