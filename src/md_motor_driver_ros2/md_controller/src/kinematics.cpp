#include "md_controller/kinematics.hpp"
#include <cmath>

#ifndef PI
#define PI 3.14159265359
#endif

float wheel_radius = 0.103;  // default 바퀴 반지름 (m)  
float wheel_base = 0.4;     // default 좌우 바퀴 간격 (m)

void setRobotParams(float radius, float base)
{
    wheel_radius = radius;
    wheel_base = base;
}


void cmdVelToRpm(float linear_x, float angular_z, int& left_rpm, int& right_rpm)
{
    // 좌우 바퀴의 선속도 계산 (m/s)
    // linear_x: 로봇 중심의 직진 속도
    // angular_z: 로봇의 회전 속도 (rad/s)
    // wheel_base/2: 로봇 중심에서 바퀴까지의 거리
    
    float left_vel = linear_x - (angular_z * wheel_base / 2.0);   // 좌측 바퀴 선속도
    float right_vel = linear_x + (angular_z * wheel_base / 2.0);  // 우측 바퀴 선속도
    
    // 선속도(m/s)를 RPM으로 변환
    // 공식: RPM = (선속도 * 60) / (바퀴 둘레)
    // 바퀴 둘레 = 2 * π * wheel_radius
    
    left_rpm = static_cast<int>(std::round((left_vel * 60.0) / (2.0 * PI * wheel_radius)));
    right_rpm = static_cast<int>(std::round((right_vel * 60.0) / (2.0 * PI * wheel_radius)));
    
    // 디버깅용 출력 (필요시 주석 해제)
    // printf("linear_x: %.3f, angular_z: %.3f -> left_rpm: %d, right_rpm: %d\n", 
    //        linear_x, angular_z, left_rpm, right_rpm);
}