#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
import json
import time
import math 

from rcl_interfaces.msg import ParameterDescriptor

class llm_motor(Node):
    def __init__(self):
        super().__init__('llm_motor')

        self.declare_parameter('linear_speed', 0.2, ParameterDescriptor(description='Default linear speed in m/s'))
        default_angular_deg = 30.0
        self.declare_parameter('angular_speed', default_angular_deg, ParameterDescriptor(description='Default angular speed in deg/s'))
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_speed_deg = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.angular_speed_rad = math.radians(angular_speed_deg)
        self.get_logger().info(f"Parameters loaded: linear_speed={self.linear_speed} m/s, angular_speed={angular_speed_deg} deg/s ({self.angular_speed_rad:.2f} rad/s)")
        self.sub = self.create_subscription(String, '/voice_cmd', self.on_voice_command, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info("llm_motor started. Listening on /voice_cmd.")
        self.periodic_action_timer = None
        self.one_shot_stop_timer = None
        self.current_command_id = 0

    def on_voice_command(self, msg: String):
        try:
            if not msg.data or not msg.data.strip():
                self.get_logger().warn("Received empty command, ignoring.")
                return
                
            command_data = json.loads(msg.data)
            action = command_data.get('action')
            
            self.current_command_id += 1
            command_id = self.current_command_id
            
            params = command_data.get('params', {})
            self.get_logger().info(f"Received command (ID: {command_id}): action={action}, params={params}")

            self.cancel_previous_action()

            if action == 'move':
                self.execute_move(params, command_id)
            elif action == 'rotate':
                self.execute_rotate(params, command_id)
            else:
                self.get_logger().warn(f"Unknown action: {action}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"An error occurred in on_voice_command: {e}")

    def cancel_previous_action(self):
        if self.periodic_action_timer is not None:
            self.periodic_action_timer.cancel()
            self.periodic_action_timer = None
        if self.one_shot_stop_timer is not None:
            self.one_shot_stop_timer.cancel()
            self.one_shot_stop_timer = None

    def execute_move(self, params, command_id):
        distance = params.get('distance', 0.0)
        is_forward = params.get('is_forward', True)

        if distance <= 0 or self.linear_speed <= 0: return

        speed = self.linear_speed if is_forward else -self.linear_speed
        
        #duration 계산 수정 
        duration = distance / self.linear_speed

        #TwistStamped 메시지 생성
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link" 
        twist_msg.twist.linear.x = speed
        twist_msg.twist.angular.z = 0.0 

        #주기적으로 TwistStamped 메시지를 발행
        #람다 함수 내에서 매번 현재 시간을 스탬프에 찍어줍니다.
        def publish_move_cmd():
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(twist_msg)

        self.periodic_action_timer = self.create_timer(0.1, publish_move_cmd)
        
        #정지 함수를 stop_all_movement로 통일
        self.one_shot_stop_timer = self.create_timer(duration, lambda: self.stop_all_movement(command_id))

        self.get_logger().info(f"Moving (ID: {command_id}) for {duration:.2f}s.")

    #execute_rotate 함수 (새로 작성)
    def execute_rotate(self, params, command_id):
        angle_deg = params.get('angular_velocity', 0.0) 
        is_clockwise = params.get('is_clockwise', True)

        if angle_deg <= 0 or self.angular_speed_rad <= 0: return
        
        #파라미터로 받은 '각도(deg)'를 '시간(sec)'으로 변환
        duration = angle_deg / math.degrees(self.angular_speed_rad) # (deg / (deg/s))
        
        #회전 방향 설정 (시계방향이 z축 마이너스)
        speed_rad = -self.angular_speed_rad if is_clockwise else self.angular_speed_rad

        #TwistStamped 메시지 생성
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = 0.0 # rotate 중에는 직진 0
        twist_msg.twist.angular.z = speed_rad

        #주기적으로 TwistStamped 메시지를 발행
        def publish_rotate_cmd():
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(twist_msg)
        self.periodic_action_timer = self.create_timer(0.1, publish_rotate_cmd)
        
        # 정지 함수를 stop_all_movement로 통일
        self.one_shot_stop_timer = self.create_timer(duration, lambda: self.stop_all_movement(command_id))
        self.get_logger().info(f"Rotating (ID: {command_id}) for {duration:.2f}s.")


    #stop_movement와 stop_rotation을 하나로 통합
    def stop_all_movement(self, command_id):
        #현재 유효한 명령의 정지 요청인지 확인
        if command_id != self.current_command_id:
            self.get_logger().info(f"Ignoring outdated stop call from ID: {command_id} (Current ID: {self.current_command_id})")
            return

        self.get_logger().info(f"Stopping all movement for ID: {command_id}")
        self.cancel_previous_action()
        
        #TwistStamped 메시지로 정지 명령 (linear=0, angular=0)
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        
        self.pub_cmd.publish(stop_msg)
        self.get_logger().info("Movement stopped.")
        
    

def main(args=None):
    rclpy.init(args=args)
    node = llm_motor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 ID를 0으로 설정하여 모든 진행중인 동작을 멈추게 함
        node.current_command_id = 0
        node.stop_all_movement(0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()