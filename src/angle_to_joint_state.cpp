#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <cmath>
#include <chrono>

class AngleToJointState : public rclcpp::Node {
public:
  AngleToJointState()
  : Node("angle_to_joint_state")
  {
    rclcpp::QoS qos(10);
    qos.transient_local();
    // Publisher: /joint_states
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscriber: /angle_cmd
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "angle_cmd", 10,
      std::bind(&AngleToJointState::angleCallback, this, std::placeholders::_1));

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&AngleToJointState::publishLoop, this));


    // モジュールID＋軸 のキー → URDF上の joint 名
    joint_names_ = {
      {"1r","module1_gimbal_roll"},   {"1p","module1_gimbal_pitch"},
      {"2r","module2_gimbal_roll"},   {"2p","module2_gimbal_pitch"},
      {"3r","module3_gimbal_roll"},   {"3p","module3_gimbal_pitch"},
      {"4r","module4_gimbal_roll"},   {"4p","module4_gimbal_pitch"},
      {"5r","module5_gimbal_roll"},   {"5p","module5_gimbal_pitch"},
      {"6r","module6_gimbal_roll"},   {"6p","module6_gimbal_pitch"},
    };

    // 全関節をゼロ初期化
    for (auto &kv : joint_names_) {
      joint_positions_[kv.second] = 0.0;
    }
  }

private:
  void angleCallback(const std_msgs::msg::String::SharedPtr msg) {
    auto s = msg->data;  // 例: "C1p+020"
    if (s.size() != 7) return;

    int  id    = s[1] - '0';          // モジュール番号
    char axis  = s[2];                // 'p' or 'r'
    char sign  = s[3];                // '+' or '-'
    int  deg   = std::stoi(s.substr(4,3)); // 角度 (0～180)
    double rad = deg * M_PI/180.0 * (sign=='+'?1:-1);

    // キーを作成して joint 名を取り出す
    std::string key = s.substr(1,2);  // "1p" など
    auto it = joint_names_.find(key);
    if (it == joint_names_.end()) return;
    std::string joint = it->second;

    // 状態を更新
    joint_positions_[joint] = rad;

    // JointState メッセージを組み立て
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    for (auto &kv : joint_positions_) {
      js.name.push_back(kv.first);
      js.position.push_back(kv.second);
    }
    joint_pub_->publish(js);

  }

  void publishLoop() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    for (auto &kv : joint_positions_) {
      js.name.push_back(kv.first);
      js.position.push_back(kv.second);
    }
    joint_pub_->publish(js);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::map<std::string,double> joint_positions_;
  std::map<std::string,std::string> joint_names_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleToJointState>());
  rclcpp::shutdown();
  return 0;
}
