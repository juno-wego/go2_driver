#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "go2_interface/msg/sport_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/low_cmd.hpp"

namespace
{
constexpr int32_t kApiDamp = 1001;
constexpr int32_t kApiBalanceStand = 1002;
constexpr int32_t kApiStopMove = 1003;
constexpr int32_t kApiStandUp = 1004;
constexpr int32_t kApiStandDown = 1005;
constexpr int32_t kApiRecoveryStand = 1006;
constexpr int32_t kApiEuler = 1007;
constexpr int32_t kApiMove = 1008;
constexpr int32_t kApiSit = 1009;
constexpr int32_t kApiRiseSit = 1010;
constexpr int32_t kApiSpeedLevel = 1015;
constexpr int32_t kApiHello = 1016;
constexpr int32_t kApiStretch = 1017;
constexpr int32_t kApiContent = 1020;
constexpr int32_t kApiDance1 = 1022;
constexpr int32_t kApiDance2 = 1023;
constexpr int32_t kApiSwitchJoystick = 1027;
constexpr int32_t kApiPose = 1028;
constexpr int32_t kApiScrape = 1029;
constexpr int32_t kApiFrontFlip = 1030;
constexpr int32_t kApiFrontJump = 1031;
constexpr int32_t kApiFrontPounce = 1032;
constexpr int32_t kApiHeart = 1036;
constexpr int32_t kApiStaticWalk = 1061;
constexpr int32_t kApiTrotRun = 1062;
constexpr int32_t kApiEconomicGait = 1063;
constexpr int32_t kApiLeftFlip = 2041;
constexpr int32_t kApiBackFlip = 2043;
constexpr int32_t kApiHandStand = 2044;
constexpr int32_t kApiFreeWalk = 2045;
constexpr int32_t kApiFreeBound = 2046;
constexpr int32_t kApiFreeJump = 2047;
constexpr int32_t kApiFreeAvoid = 2048;
constexpr int32_t kApiClassicWalk = 2049;
constexpr int32_t kApiWalkUpright = 2050;
constexpr int32_t kApiCrossStep = 2051;
constexpr int32_t kApiAutoRecoverySet = 2054;
constexpr int32_t kApiSwitchAvoidMode = 2058;

double apply_deadband(double value, double deadband)
{
  return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}

std::string normalize_command(std::string command)
{
  std::transform(
    command.begin(), command.end(), command.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return command;
}

std::string make_xyz_payload(double x, double y, double z)
{
  std::ostringstream stream;
  stream << "{\"x\":" << x << ",\"y\":" << y << ",\"z\":" << z << "}";
  return stream.str();
}

std::string make_data_payload(bool data)
{
  return std::string("{\"data\":") + (data ? "true" : "false") + "}";
}

std::string make_data_payload(int32_t data)
{
  return "{\"data\":" + std::to_string(data) + "}";
}
}  // namespace

class Go2CmdVelBridge : public rclcpp::Node
{
public:
  Go2CmdVelBridge()
  : Node("go2_cmd_vel_bridge")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    sport_command_topic_ =
      declare_parameter<std::string>("sport_command_topic", "/go2/sport_cmd");
    low_cmd_input_topic_ =
      declare_parameter<std::string>("low_cmd_input_topic", "/go2/low_cmd");
    low_cmd_output_topic_ =
      declare_parameter<std::string>("low_cmd_output_topic", "/lowcmd");
    request_topic_ = declare_parameter<std::string>("request_topic", "/api/sport/request");
    deadband_ = declare_parameter<double>("deadband", 0.01);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    stop_on_timeout_ = declare_parameter<bool>("stop_on_timeout", true);
    auto_balance_stand_ = declare_parameter<bool>("auto_balance_stand_on_start", false);

    request_pub_ = create_publisher<unitree_api::msg::Request>(request_topic_, rclcpp::QoS(10));
    low_cmd_pub_ =
      create_publisher<unitree_go::msg::LowCmd>(low_cmd_output_topic_, rclcpp::QoS(10));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
        sent_timeout_stop_ = false;
      });
    sport_cmd_sub_ = create_subscription<go2_interface::msg::SportCmd>(
      sport_command_topic_, rclcpp::QoS(10),
      std::bind(&Go2CmdVelBridge::on_sport_command, this, std::placeholders::_1));
    low_cmd_sub_ = create_subscription<unitree_go::msg::LowCmd>(
      low_cmd_input_topic_, rclcpp::QoS(10),
      [this](const unitree_go::msg::LowCmd::SharedPtr msg) {
        low_cmd_pub_->publish(*msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Go2CmdVelBridge::on_timer, this));

    if (auto_balance_stand_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        publish_motion_request(kApiBalanceStand);
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Sent BalanceStand request to GO2.");
      });
    }

    RCLCPP_INFO(
      get_logger(),
      "Bridging cmd_vel=%s sport_cmd=%s low_cmd=%s -> request=%s lowcmd=%s.",
      cmd_vel_topic_.c_str(), sport_command_topic_.c_str(), low_cmd_input_topic_.c_str(),
      request_topic_.c_str(), low_cmd_output_topic_.c_str());
  }

private:
  void publish_motion_request(int32_t api_id, const std::string & payload = "")
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = api_id;
    if (!payload.empty()) {
      request.parameter = payload;
    }
    request_pub_->publish(request);
  }

  void publish_move(double vx, double vy, double vyaw)
  {
    publish_motion_request(kApiMove, make_xyz_payload(vx, vy, vyaw));
  }

  void publish_bool_request(int32_t api_id, bool flag)
  {
    publish_motion_request(api_id, make_data_payload(flag));
  }

  bool dispatch_named_command(const go2_interface::msg::SportCmd & msg)
  {
    const std::string command = normalize_command(msg.command);
    if (command.empty()) {
      return false;
    }

    if (command == "damp") {
      publish_motion_request(kApiDamp);
      return true;
    }
    if (command == "balance_stand") {
      publish_motion_request(kApiBalanceStand);
      return true;
    }
    if (command == "stop_move") {
      have_cmd_ = false;
      sent_timeout_stop_ = true;
      publish_motion_request(kApiStopMove);
      return true;
    }
    if (command == "stand_up") {
      publish_motion_request(kApiStandUp);
      return true;
    }
    if (command == "stand_down") {
      publish_motion_request(kApiStandDown);
      return true;
    }
    if (command == "recovery_stand") {
      publish_motion_request(kApiRecoveryStand);
      return true;
    }
    if (command == "sit") {
      publish_motion_request(kApiSit);
      return true;
    }
    if (command == "rise_sit") {
      publish_motion_request(kApiRiseSit);
      return true;
    }
    if (command == "hello") {
      publish_motion_request(kApiHello);
      return true;
    }
    if (command == "stretch") {
      publish_motion_request(kApiStretch);
      return true;
    }
    if (command == "content") {
      publish_motion_request(kApiContent);
      return true;
    }
    if (command == "dance1") {
      publish_motion_request(kApiDance1);
      return true;
    }
    if (command == "dance2") {
      publish_motion_request(kApiDance2);
      return true;
    }
    if (command == "scrape") {
      publish_motion_request(kApiScrape);
      return true;
    }
    if (command == "heart") {
      publish_motion_request(kApiHeart);
      return true;
    }
    if (command == "front_flip") {
      publish_motion_request(kApiFrontFlip);
      return true;
    }
    if (command == "front_jump") {
      publish_motion_request(kApiFrontJump);
      return true;
    }
    if (command == "front_pounce") {
      publish_motion_request(kApiFrontPounce);
      return true;
    }
    if (command == "left_flip") {
      publish_motion_request(kApiLeftFlip);
      return true;
    }
    if (command == "back_flip") {
      publish_motion_request(kApiBackFlip);
      return true;
    }
    if (command == "free_walk") {
      publish_motion_request(kApiFreeWalk);
      return true;
    }
    if (command == "static_walk") {
      publish_motion_request(kApiStaticWalk);
      return true;
    }
    if (command == "trot_run") {
      publish_motion_request(kApiTrotRun);
      return true;
    }
    if (command == "economic_gait") {
      publish_motion_request(kApiEconomicGait);
      return true;
    }
    if (command == "hand_stand") {
      publish_bool_request(kApiHandStand, msg.flag);
      return true;
    }
    if (command == "free_bound") {
      publish_bool_request(kApiFreeBound, msg.flag);
      return true;
    }
    if (command == "free_jump") {
      publish_bool_request(kApiFreeJump, msg.flag);
      return true;
    }
    if (command == "free_avoid") {
      publish_bool_request(kApiFreeAvoid, msg.flag);
      return true;
    }
    if (command == "classic_walk") {
      publish_bool_request(kApiClassicWalk, msg.flag);
      return true;
    }
    if (command == "walk_upright") {
      publish_bool_request(kApiWalkUpright, msg.flag);
      return true;
    }
    if (command == "cross_step") {
      publish_bool_request(kApiCrossStep, msg.flag);
      return true;
    }
    if (command == "auto_recovery_set") {
      publish_bool_request(kApiAutoRecoverySet, msg.flag);
      return true;
    }
    if (command == "switch_joystick") {
      publish_bool_request(kApiSwitchJoystick, msg.flag);
      return true;
    }
    if (command == "pose") {
      publish_bool_request(kApiPose, msg.flag);
      return true;
    }
    if (command == "switch_avoid_mode") {
      publish_motion_request(kApiSwitchAvoidMode);
      return true;
    }
    if (command == "euler") {
      publish_motion_request(kApiEuler, make_xyz_payload(msg.roll, msg.pitch, msg.yaw));
      return true;
    }
    if (command == "speed_level") {
      publish_motion_request(kApiSpeedLevel, make_data_payload(msg.speed_level));
      return true;
    }

    return false;
  }

  void on_sport_command(const go2_interface::msg::SportCmd::SharedPtr msg)
  {
    if (!dispatch_named_command(*msg)) {
      RCLCPP_WARN(
        get_logger(), "Unsupported GO2 sport command: '%s'.", msg->command.c_str());
    }
  }

  void on_timer()
  {
    if (!have_cmd_) {
      return;
    }

    const auto age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_) {
      if (stop_on_timeout_ && !sent_timeout_stop_) {
        publish_motion_request(kApiStopMove);
        sent_timeout_stop_ = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "cmd_vel timed out; sent StopMove.");
      }
      return;
    }

    sent_timeout_stop_ = false;
    publish_move(
      apply_deadband(last_cmd_.linear.x, deadband_),
      apply_deadband(last_cmd_.linear.y, deadband_),
      apply_deadband(last_cmd_.angular.z, deadband_));
  }

  std::string cmd_vel_topic_;
  std::string sport_command_topic_;
  std::string low_cmd_input_topic_;
  std::string low_cmd_output_topic_;
  std::string request_topic_;
  double deadband_{0.01};
  double command_rate_hz_{20.0};
  double cmd_timeout_{0.5};
  bool stop_on_timeout_{true};
  bool auto_balance_stand_{false};
  bool have_cmd_{false};
  bool sent_timeout_stop_{false};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<go2_interface::msg::SportCmd>::SharedPtr sport_cmd_sub_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
