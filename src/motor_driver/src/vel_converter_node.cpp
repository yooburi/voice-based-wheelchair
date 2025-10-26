#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <memory> 
#include <chrono>   
#include <exception> 
#include "rclcpp/exceptions.hpp"


class VelConverterNode : public rclcpp::Node
{
public:
  VelConverterNode(const rclcpp::NodeOptions & options)
  : Node("vel_converter_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "CmdVel -> WheelVel (RPM) 변환 노드 시작...");

    //파라미터 선언 (기본값 설정)
    this->declare_parameter<double>("wheel_base", 0.4);   // m
    this->declare_parameter<double>("wheel_radius", 0.1); // m

    //파라미터 값 읽기
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_radius", wheel_radius_);

    RCLCPP_INFO(this->get_logger(), "로봇 사양: WheelBase=%.2fm, WheelRadius=%.2fm", wheel_base_, wheel_radius_);

    //cmd_vel 구독
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel",
      10,
      std::bind(&VelConverterNode::cmdVelCallback, this, std::placeholders::_1)
    );

    wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/target_wheel_velocities",
      rclcpp::QoS(1).transient_local()
    );
  }

  void publishStop()
  {
    RCLCPP_INFO(this->get_logger(), "정지 명령 (RPM 0) 발행.");
    auto stop_msg = std_msgs::msg::Float32MultiArray();
    stop_msg.data.push_back(0.0f);
    stop_msg.data.push_back(0.0f);
    wheel_vel_pub_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "정지 메시지 발행 완료.");
  }

private:
  
  //cmd_vel 토픽 콜백 함수
  
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    double V = msg->twist.linear.x;    //로봇 목표 선속도 (m/s)
    double W = msg->twist.angular.z; //로봇 목표 각속도 (rad/s)

    //역기구학
    double v_right_mps = V + (W * wheel_base_ / 2.0);
    double v_left_mps  = V - (W * wheel_base_ / 2.0);

    //m/s를 바퀴의 각속도(rad/s)로 변환
    double target_rad_s_right = v_right_mps / wheel_radius_;
    double target_rad_s_left  = v_left_mps / wheel_radius_;

    //rad/s를 RPM으로 변환 ---
    double target_rpm_right = target_rad_s_right * RAD_S_TO_RPM;
    double target_rpm_left  = target_rad_s_left * RAD_S_TO_RPM;

    //micro-ROS로 보낼 메시지 생성 ---
    auto wheel_vel_msg = std_msgs::msg::Float32MultiArray();
    
    //왼쪽 바퀴 목표 RPM, 오른쪽 바퀴 목표 RPM]
    wheel_vel_msg.data.push_back(static_cast<float>(target_rpm_left));
    wheel_vel_msg.data.push_back(static_cast<float>(target_rpm_right));

    // 토픽 발행
    wheel_vel_pub_->publish(wheel_vel_msg);
  }

  // 멤버 변수
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;

  double wheel_base_;
  double wheel_radius_;
  
  //RPM 변환 상수 (1 rad/s = 9.5493 RPM)
  const double RAD_S_TO_RPM = 9.5493; 
};



#include <csignal>

//전역 노드 포인터
std::shared_ptr<VelConverterNode> node_ptr;

//시그널 핸들러 함수
void signalHandler(int signum) {
  RCLCPP_INFO(node_ptr->get_logger(), "Signal %d 수신. 노드 종료 중...", signum);
  node_ptr->publishStop();
  //잠시 대기하여 메시지 발행 보장
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  node_ptr = std::make_shared<VelConverterNode>(options);

  //시그널 핸들러 등록
  signal(SIGINT, signalHandler);

  rclcpp::spin(node_ptr);

  return 0;
}