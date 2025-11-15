#pragma once

// 로봇 HW 파라미터
extern float wheel_radius;
extern float wheel_base;

// ROS param loading from launch
void setRobotParams(float wheel_radius, float wheel_base);

// 차동구동 로봇의 cmd_vel을 좌우 RPM으로 변환
void cmdVelToRpm(float linear_x, float angular_z, int& left_rpm, int& right_rpm);



