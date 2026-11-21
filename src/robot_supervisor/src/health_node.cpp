/**
 * @file health_node.cpp
 * @brief Robot health on /diagnostics, for Foxglove's Diagnostics panels.
 *
 * Once per second publishes OK / WARN / ERROR for:
 * - LiDAR: /scan rate; "not connected" when the USB device (/dev/lidar) is missing
 * - Odometry: /odom and laser odometry (/odom_rf2o) rates
 * - Jetson: temperatures, memory, disk, load
 * - Network: Ethernet link + address, Wi-Fi access point mode + clients
 * - Status watchdog: nodes that report their own status (IMU, cameras, mapping) are flagged as
 *   ERROR under their own name when their status stops arriving (node not running).
 */

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using DS = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) {
    s.pop_back();
  }
  return s;
}

std::string run(const std::string & cmd)
{
  std::string out;
  if (FILE * p = popen(cmd.c_str(), "r")) {
    char buf[256];
    while (fgets(buf, sizeof(buf), p)) {
      out += buf;
    }
    pclose(p);
  }
  return out;
}

std::string ipv4_of(const std::string & ifname)
{
  std::string ip;
  ifaddrs * list = nullptr;
  if (getifaddrs(&list) == 0) {
    for (ifaddrs * a = list; a; a = a->ifa_next) {
      if (a->ifa_addr && a->ifa_addr->sa_family == AF_INET && ifname == a->ifa_name) {
        char buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in *>(a->ifa_addr)->sin_addr, buf, sizeof(buf));
        ip = buf;
        break;
      }
    }
    freeifaddrs(list);
  }
  return ip;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue p;
  p.key = k;
  p.value = v;
  return p;
}

std::string fmt(const char * f, double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

/// Messages per second over a sliding window.
class RateMeter
{
public:
  void tick() {++count_;}
  double rate(double dt)
  {
    const double r = dt > 0 ? count_ / dt : 0.0;
    count_ = 0;
    return r;
  }

private:
  uint64_t count_ = 0;
};
}  // namespace

class HealthNode : public rclcpp::Node
{
public:
  HealthNode() : Node("robot_health")
  {
    lidar_device_ = declare_parameter<std::string>("lidar_device", "/dev/lidar");
    lidar_min_hz_ = declare_parameter<double>("lidar_min_hz", 7.0);
    eth_ = declare_parameter<std::string>("ethernet_interface", "eth0");
    wifi_ = declare_parameter<std::string>("wifi_interface", "wlan0");
    // Names of statuses published by other nodes; flagged when they stop arriving
    const auto sources = declare_parameter<std::vector<std::string>>(
      "watched_statuses", {"IMU (ICM20948)", "Camera left", "Camera right", "Mapping"});
    stale_after_ = declare_parameter<double>("stale_after", 5.0);
    temp_warn_ = declare_parameter<double>("temperature_warn", 80.0);
    temp_error_ = declare_parameter<double>("temperature_error", 90.0);
    for (const auto & s : sources) {
      watched_[s] = Watched{};
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {scan_.tick();});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr) {odom_.tick();});
    rf2o_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_rf2o", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr) {rf2o_.tick();});
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 50,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {on_diag(*m);});
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    started_ = last_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(1s, [this] {publish();});
    RCLCPP_INFO(get_logger(), "robot_health: publishing /diagnostics at 1 Hz");
  }

private:
  struct Watched
  {
    bool seen = false;
    std::chrono::steady_clock::time_point last{};
  };

  void on_diag(const diagnostic_msgs::msg::DiagnosticArray & m)
  {
    const auto now = std::chrono::steady_clock::now();
    for (const auto & s : m.status) {
      if (s.hardware_id == kOwnId) {
        continue;  // our own (stale) reports
      }
      auto it = watched_.find(s.name);
      if (it != watched_.end()) {
        it->second.seen = true;
        it->second.last = now;
      }
    }
  }

  void publish()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    const double uptime = std::chrono::duration<double>(now - started_).count();
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    arr.status.push_back(lidar(scan_.rate(dt), uptime));
    arr.status.push_back(odometry(odom_.rate(dt), rf2o_.rate(dt), uptime));
    arr.status.push_back(jetson());
    if (++net_tick_ % 5 == 1) {  // network checks fork a process: every 5 s
      network_ = network();
    }
    arr.status.push_back(network_);
    for (auto & [name, w] : watched_) {
      const double age = std::chrono::duration<double>(now - (w.seen ? w.last : started_)).count();
      if (age > stale_after_) {
        DS st;
        st.name = name;
        st.hardware_id = kOwnId;
        st.level = DS::ERROR;
        st.message = w.seen ? "no status for " + fmt("%.0f", age) + " s (node stopped?)" :
          "no status since start (node not running?)";
        arr.status.push_back(st);
      }
    }
    diag_pub_->publish(arr);
  }

  DS lidar(double hz, double uptime)
  {
    DS st;
    st.name = "LiDAR (D500)";
    st.hardware_id = kOwnId;
    struct stat sb {};
    const bool device = lidar_device_.empty() || stat(lidar_device_.c_str(), &sb) == 0;
    if (hz >= lidar_min_hz_) {
      st.level = DS::OK;
      st.message = fmt("%.1f Hz", hz);
    } else if (hz > 0) {
      st.level = DS::WARN;
      st.message = fmt("low scan rate: %.1f Hz", hz);
    } else if (!device) {
      st.level = DS::ERROR;
      st.message = "not connected (" + lidar_device_ + " missing): check the USB cable";
    } else {
      st.level = uptime < 15 ? DS::WARN : DS::ERROR;
      st.message = uptime < 15 ? "starting" : "no scans (driver restarting, or LiDAR not powered?)";
    }
    st.values.push_back(kv("scan_hz", fmt("%.1f", hz)));
    st.values.push_back(kv("device", lidar_device_.empty() ? "-" : lidar_device_ +
      (device ? " present" : " missing")));
    return st;
  }

  DS odometry(double odom_hz, double rf2o_hz, double uptime)
  {
    DS st;
    st.name = "Odometry";
    st.hardware_id = kOwnId;
    if (odom_hz <= 0) {
      st.level = uptime < 15 ? DS::WARN : DS::ERROR;
      st.message = uptime < 15 ? "starting" : "no /odom";
    } else if (rf2o_hz <= 0) {
      st.level = DS::WARN;
      st.message = "laser odometry missing: heading only, position does not update";
    } else {
      st.level = DS::OK;
      st.message = fmt("/odom %.0f Hz", odom_hz) + fmt(", laser odometry %.1f Hz", rf2o_hz);
    }
    st.values.push_back(kv("odom_hz", fmt("%.1f", odom_hz)));
    st.values.push_back(kv("laser_odom_hz", fmt("%.1f", rf2o_hz)));
    return st;
  }

  DS jetson()
  {
    DS st;
    st.name = "Jetson";
    st.hardware_id = kOwnId;
    st.level = DS::OK;
    std::vector<std::string> problems;
    // Temperatures (PMIC-Die and the fan estimate are not real sensors)
    double tmax = 0;
    for (int i = 0; i < 16; ++i) {
      const std::string base = "/sys/class/thermal/thermal_zone" + std::to_string(i);
      const std::string type = read_file(base + "/type");
      if (type.empty()) {
        break;
      }
      if (type == "PMIC-Die" || type.find("fan") != std::string::npos) {
        continue;
      }
      const double t = std::atof(read_file(base + "/temp").c_str()) / 1000.0;
      st.values.push_back(kv(type, fmt("%.1f C", t)));
      tmax = std::max(tmax, t);
    }
    if (tmax >= temp_error_) {
      st.level = DS::ERROR;
      problems.push_back(fmt("hot: %.0f C", tmax));
    } else if (tmax >= temp_warn_) {
      st.level = std::max<uint8_t>(st.level, DS::WARN);
      problems.push_back(fmt("warm: %.0f C", tmax));
    }
    // Memory
    std::ifstream mi("/proc/meminfo");
    std::string key;
    long value = 0, avail_kb = -1;
    std::string unit;
    while (mi >> key >> value >> unit) {
      if (key == "MemAvailable:") {
        avail_kb = value;
        break;
      }
    }
    if (avail_kb >= 0) {
      st.values.push_back(kv("memory_available", fmt("%.0f MB", avail_kb / 1024.0)));
      if (avail_kb < 400 * 1024) {
        st.level = std::max<uint8_t>(st.level, DS::WARN);
        problems.push_back(fmt("low memory: %.0f MB free", avail_kb / 1024.0));
      }
    }
    // Disk
    struct statvfs vfs {};
    if (statvfs("/", &vfs) == 0) {
      const double free_gb = static_cast<double>(vfs.f_bavail) * vfs.f_frsize / 1e9;
      st.values.push_back(kv("disk_free", fmt("%.1f GB", free_gb)));
      if (free_gb < 2.0) {
        st.level = std::max<uint8_t>(st.level, DS::WARN);
        problems.push_back(fmt("disk almost full: %.1f GB free", free_gb));
      }
    }
    st.values.push_back(kv("load_1min", read_file("/proc/loadavg").substr(0, 4)));
    if (problems.empty()) {
      st.message = fmt("%.0f C", tmax) + ", " + (avail_kb >= 0 ? fmt("%.1f GB RAM free", avail_kb / 1048576.0) : "");
    } else {
      st.message = problems[0];
      for (size_t i = 1; i < problems.size(); ++i) {
        st.message += "; " + problems[i];
      }
    }
    return st;
  }

  DS network()
  {
    DS st;
    st.name = "Network";
    st.hardware_id = kOwnId;
    st.level = DS::OK;
    // Ethernet: normal networking; unplugged is fine (robot in the field), just reported
    const bool carrier = read_file("/sys/class/net/" + eth_ + "/carrier") == "1";
    const std::string eth_ip = ipv4_of(eth_);
    const std::string eth = carrier ? (eth_ip.empty() ? "link up, no address" : eth_ip) : "unplugged";
    st.values.push_back(kv(eth_, eth));
    // Wi-Fi access point: expected at all times
    const std::string info = run("iw dev " + wifi_ + " info 2>/dev/null");
    const bool ap = info.find("type AP") != std::string::npos;
    std::string ssid;
    const auto p = info.find("ssid ");
    if (p != std::string::npos) {
      ssid = info.substr(p + 5, info.find('\n', p) - p - 5);
    }
    int clients = 0;
    if (ap) {
      const std::string dump = run("iw dev " + wifi_ + " station dump 2>/dev/null");
      for (size_t i = dump.find("Station"); i != std::string::npos; i = dump.find("Station", i + 1)) {
        ++clients;
      }
    }
    const std::string wifi_ip = ipv4_of(wifi_);
    st.values.push_back(kv(wifi_, ap ? "access point '" + ssid + "' " + wifi_ip +
      ", " + std::to_string(clients) + " client(s)" : (info.empty() ? "missing" : "not in AP mode")));
    if (!ap) {
      st.level = DS::WARN;
      st.message = "Wi-Fi access point is not running";
    } else {
      st.message = "AP '" + ssid + "' (" + std::to_string(clients) + " client" +
        (clients == 1 ? "" : "s") + "), " + eth_ + " " + eth;
    }
    return st;
  }

  static constexpr const char * kOwnId = "robot_health";
  std::string lidar_device_;
  double lidar_min_hz_;
  std::string eth_;
  std::string wifi_;
  double stale_after_;
  double temp_warn_;
  double temp_error_;
  std::map<std::string, Watched> watched_;
  RateMeter scan_, odom_, rf2o_;
  DS network_;
  int net_tick_ = 0;
  std::chrono::steady_clock::time_point started_, last_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// launch sends SIGINT twice on Ctrl+C; see robot_odometry for why this handler exists
void ignore_signal(int) {}

int main(int argc, char ** argv)
{
  struct sigaction sa {};
  sa.sa_handler = ignore_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthNode>());
  rclcpp::shutdown();
  return 0;
}
