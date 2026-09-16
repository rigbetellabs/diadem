#include <arpa/inet.h>
#include <array>
#include <ifaddrs.h>
#include <memory>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>

struct PipeDeleter {
  void operator()(FILE *fp) const {
    if (fp) {
      pclose(fp);
    }
  }
};

std::string exec(const char *cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, PipeDeleter> pipe(popen(cmd, "r"));
  if (!pipe) {
    return "";
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  if (!result.empty() && result.back() == '\n')
    result.pop_back();
  if (!result.empty() && result.back() == '\r')
    result.pop_back();
  return result;
}

void clean_string(std::string &s) {
  s.erase(std::remove_if(s.begin(), s.end(),
                         [](unsigned char c) {
                           return c == '\r' || c == '\n' || c == '"' ||
                                  c < 32 || c > 126;
                         }),
          s.end());
}

class NetworkStatusPublisher : public rclcpp::Node {
public:
  NetworkStatusPublisher() : Node("network_status_publisher") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("network_status", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&NetworkStatusPublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(),
                "Optimized C++ Network Status Publisher started.");
  }

private:
  std::string get_ip() {
    struct ifaddrs *ifAddrStruct = NULL;
    struct ifaddrs *ifa = NULL;
    void *tmpAddrPtr = NULL;
    std::string ip = "No IP";

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
      if (!ifa->ifa_addr)
        continue;
      if (ifa->ifa_addr->sa_family == AF_INET) {
        tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
        char addressBuffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
        std::string ifName(ifa->ifa_name);

        if (ifName != "lo" && ifName.find("docker") == std::string::npos &&
            ifName.find("br-") == std::string::npos) {

          if (ifName.find("wl") != std::string::npos ||
              ifName.find("wlan") != std::string::npos ||
              ifName.find("ap") != std::string::npos) {
            ip = addressBuffer;
            break;
          }

          else if (ifName.find("en") != std::string::npos ||
                   ifName.find("eth") != std::string::npos) {
            ip = addressBuffer;
          }

          else if (ip == "No IP") {
            ip = addressBuffer;
          }
        }
      }
    }
    if (ifAddrStruct != NULL)
      freeifaddrs(ifAddrStruct);
    return ip;
  }

  std::string get_network_info(std::string &mode, std::string &status) {
    mode = "none";
    status = "disconnected";
    std::string info = "No Network";

    std::string output =
        exec("nmcli -t -f TYPE,NAME connection show --active 2>/dev/null");
    if (!output.empty()) {
      std::istringstream stream(output);
      std::string line;
      while (std::getline(stream, line)) {
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
          std::string type = line.substr(0, colon_pos);
          std::string current_info = line.substr(colon_pos + 1);

          if (type.find("loopback") != std::string::npos ||
              type.find("bridge") != std::string::npos) {
            continue;
          }

          if (type.find("wireless") != std::string::npos) {
            mode = "wifi";
            std::string lower_info = current_info;
            for (auto &c : lower_info)
              c = tolower(c);
            if (lower_info.find("hotspot") != std::string::npos) {
              mode = "hotspot";
            }
            info = current_info;
            status = "connected";
            return info;
          } else if (type.find("ethernet") != std::string::npos) {
            mode = "ethernet";
            info = current_info;
            status = "connected";
            return info;
          }
        }
      }
    }

    std::string iw_output = exec("iw dev 2>/dev/null");
    if (iw_output.find("type AP") != std::string::npos) {
      mode = "hotspot";
      status = "connected";

      std::string ap_ssid = exec("iw dev | grep ssid | awk '{print $2}'");
      if (!ap_ssid.empty()) {
        return ap_ssid;
      }
      return "Hotspot Active";
    }

    std::string fallback = exec("iwgetid -r 2>/dev/null");
    if (!fallback.empty()) {
      mode = "wifi";
      status = "connected";
      return fallback;
    }

    std::string ip_links = exec("ip -o link show up 2>/dev/null");
    if (ip_links.find("eth0") != std::string::npos ||
        ip_links.find("en") != std::string::npos) {
      std::string eth_ip = exec("ip -4 a show | grep -E 'eth|en' | grep inet");
      if (!eth_ip.empty()) {
        mode = "ethernet";
        status = "connected";
        return "Wired Connection";
      }
    }

    return info;
  }

  void timer_callback() {
    auto msg = std_msgs::msg::String();
    std::string ip = get_ip();

    std::string mode, status, info;
    info = get_network_info(mode, status);

    clean_string(mode);
    clean_string(status);
    clean_string(info);
    clean_string(ip);

    std::ostringstream ss;
    ss << "{\"mode\": \"" << mode << "\", \"status\": \"" << status
       << "\", \"info\": \"" << info << "\", \"ip\": \"" << ip << "\"}";

    msg.data = ss.str();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
