#include "md_controller/com.hpp"
#include "md_controller/kinematics.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"      // 👈 [추가] Odometry 메시지 헤더
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
Communication Com;  
MotorVar Motor;

geometry_msgs::msg::TransformStamped odom_tf;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdRpm = OFF;
int left_rpm_ = 0;
int right_rpm_ = 0;

// 👇 [추가] 오도메트리 변수
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;
long last_left_tick = 0;
long last_right_tick = 0;

void CmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;
    cmdVelToRpm(linear_x, angular_z, left_rpm_, right_rpm_);
    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    rclcpp::Time stamp_now;

    // 👇 [추가] Odometry Publisher
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, CmdVelCallBack);
    auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Motor driver setup
    node->declare_parameter("MDUI", 184);
    node->declare_parameter("MDT", 183);
    node->declare_parameter("Port", "/dev/ttyMotor");
    node->declare_parameter("Baudrate", 57600);
    node->declare_parameter("ID", 1);
    node->declare_parameter("GearRatio", 15);
    node->declare_parameter("poles", 10);
    node->declare_parameter("wheel_radius", 0.033);
    node->declare_parameter("wheel_base", 0.16);

    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID", Motor.ID);
    node->get_parameter("GearRatio", Motor.GearRatio);
    node->get_parameter("poles", Motor.poles);

    float wheel_radius_param, wheel_base_param;
    node->get_parameter("wheel_radius", wheel_radius_param);
    node->get_parameter("wheel_base", wheel_base_param);
    setRobotParams(wheel_radius_param, wheel_base_param);

    Motor.PPR = 5542;
    Motor.Tick2RAD = (360.0 / Motor.PPR) * PI / 180;

    IByte iData, left_iData, right_iData;

    int nArray[20];
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep = 1;
    Motor.InitMotor = ON;
    fgInitsetting = OFF;
    Motor.InitError = 0;
    Motor.last_rad = 0;
    Motor.last_tick = 0;
    
    joint_states.name.push_back("left_wheel_joint");  
    joint_states.name.push_back("right_wheel_joint"); 
    joint_states.position.resize(2);
    

    InitSerial();
    
    auto last_time = node->now();

    while (rclcpp::ok()) {
        ReceiveDataFromController(Motor.InitMotor);
        
        if(++byCnt2500us == 50) {
            byCnt2500us = 0;
            
            if(fgInitsetting == ON) {
                switch(++byCntComStep) {
                case 1: {
                    // 👇 [수정] 오도메트리 계산
                    auto current_time = node->now();
                    double dt = (current_time - last_time).seconds();
                    last_time = current_time;

                    // Com.position에서 좌우 엔코더 값 추출 (상위/하위 16비트로 가정)
                    // 실제 프로토콜에 따라 수정 필요
                    long current_left_tick = (Com.position >> 16) & 0xFFFF;  // 상위 16비트
                    long current_right_tick = Com.position & 0xFFFF;         // 하위 16비트
                    
                    // 부호 확장 (16비트 -> 32비트)
                    if (current_left_tick & 0x8000) current_left_tick |= 0xFFFF0000;
                    if (current_right_tick & 0x8000) current_right_tick |= 0xFFFF0000;

                    // 틱 차이 계산
                    long delta_left = current_left_tick - last_left_tick;
                    long delta_right = current_right_tick - last_right_tick;
                    last_left_tick = current_left_tick;
                    last_right_tick = current_right_tick;

                    // 거리 계산 (tick -> meter)
                    double left_distance = delta_left * Motor.Tick2RAD * wheel_radius_param;
                    double right_distance = delta_right * Motor.Tick2RAD * wheel_radius_param;

                    // 로봇 중심 이동 거리 및 회전
                    double delta_s = (left_distance + right_distance) / 2.0;
                    double delta_theta = (right_distance - left_distance) / wheel_base_param;

                    // 오도메트리 업데이트
                    odom_x += delta_s * cos(odom_theta + delta_theta / 2.0);
                    odom_y += delta_s * sin(odom_theta + delta_theta / 2.0);
                    odom_theta += delta_theta;

                    // Odometry 메시지 발행
                    nav_msgs::msg::Odometry odom_msg;
                    odom_msg.header.stamp = current_time;
                    odom_msg.header.frame_id = "odom";
                    odom_msg.child_frame_id = "base_link";

                    odom_msg.pose.pose.position.x = odom_x;
                    odom_msg.pose.pose.position.y = odom_y;
                    odom_msg.pose.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, odom_theta);
                    odom_msg.pose.pose.orientation.x = q.x();
                    odom_msg.pose.pose.orientation.y = q.y();
                    odom_msg.pose.pose.orientation.z = q.z();
                    odom_msg.pose.pose.orientation.w = q.w();

                    // 속도 정보
                    odom_msg.twist.twist.linear.x = delta_s / dt;
                    odom_msg.twist.twist.angular.z = delta_theta / dt;
                    joint_states.header.stamp = current_time;
                    joint_states.position[0] = current_left_tick * Motor.Tick2RAD;  
		    joint_states.position[1] = current_right_tick * Motor.Tick2RAD;

                    odom_pub->publish(odom_msg);
                    
                    joint_state_pub->publish(joint_states);
                    break;
                }
                case 2:
                    if(++byCntCase[byCntComStep] == TIME_100MS) {
                        byCntCase[byCntComStep] = 0;

                        if(SendCmdRpm) {
                            left_iData = Short2Byte(left_rpm_ * 4.33);
                            right_iData = Short2Byte(right_rpm_ *4.33);

                            nArray[0] = 1;
                            nArray[1] = left_iData.byLow;
                            nArray[2] = left_iData.byHigh;
                            nArray[3] = 1;
                            nArray[4] = right_iData.byLow;
                            nArray[5] = right_iData.byHigh;
                            nArray[6] = 0;

                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);

                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                            SendCmdRpm = OFF;
                        } else {
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);
                        }
                    }
                    byCntComStep = 0;
                    break;  
                }
            } else {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else {
                    switch (byCntInitStep) {
                    case 1:
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                        if(Motor.InitMotor == ON)
                            Motor.InitError++;
                        else
                            byCntInitStep++;

                        if(Motor.InitError > 10) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ID %d MOTOR INIT ERROR!!", Motor.ID); 
                            return 0;
                        }
                        break;
                    case 2:
                        byCntInitStep++;
                        break;
                    case 3:
                        nArray[0] = 0;
                        nArray[1] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);
                        byCntInitStep++;
                        break;
                    case 4:
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                        byCntInitStep++;
                        break;
                    case 5:
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOTOR INIT END\n");
                        fgInitsetting = ON;
                        last_time = node->now();  // 초기화 완료 후 시간 기록
                        break;
                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
