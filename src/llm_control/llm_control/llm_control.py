#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist
import json
import time

from rcl_interfaces.msg import ParameterDescriptor

class llm_control(Node):
    def __init__(self):
        super().__init__('llm_control')

        self.declare_parameter('linear_speed', 0.2, ParameterDescriptor(description='Default linear speed in m/s'))
        self.declare_parameter('angular_speed', 30.0, ParameterDescriptor(description='Default angular speed in deg/s'))

        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.get_logger().info(f"Parameters loaded: linear_speed={self.linear_speed} m/s, angular_speed={self.angular_speed} deg/s")

        self.sub = self.create_subscription(String, '/voice_cmd', self.on_voice_command, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_turn = self.create_publisher(Int8, '/turn_dir', 10)

        self.get_logger().info("llm_control started. Listening on /voice_cmd.")

        self.periodic_action_timer = None
        self.one_shot_stop_timer = None
        
        # <<< 변경점 1: 명령의 고유성을 추적하기 위한 ID 변수 추가
        self.current_command_id = 0

    def on_voice_command(self, msg: String):
        try:
            # 빈 메시지나 더미 데이터 무시
            if not msg.data or not msg.data.strip():
                self.get_logger().warn("Received empty command, ignoring.")
                return
                
            command_data = json.loads(msg.data)
            action = command_data.get('action')
            
            # <<< 변경점 2: 새로운 명령이 들어오면 ID를 증가시켜 이전 명령과 구별
            self.current_command_id += 1
            # 람다 함수에서 현재 ID 값을 캡처하기 위해 변수에 할당
            command_id = self.current_command_id
            
            params = command_data.get('params', {})
            self.get_logger().info(f"Received command (ID: {command_id}): action={action}, params={params}")

            self.cancel_previous_action()

            if action == 'move':
                # <<< 변경점 3: 실행 함수에 현재 command_id 전달
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
        # self.get_logger().info("Previous timers have been cancelled.") # 로그가 너무 많아질 수 있어 주석 처리

    # <<< 변경점 4: 모든 실행/정지 함수가 command_id를 인자로 받도록 수정
    def execute_move(self, params, command_id):
        distance = params.get('distance', 0.0)
        is_forward = params.get('is_forward', True)

        if distance <= 0 or self.linear_speed <= 0: return

        speed = self.linear_speed if is_forward else -self.linear_speed
        duration = distance / self.linear_speed

        turn_msg = Int8(); turn_msg.data = 0
        self.pub_turn.publish(turn_msg)
        

        twist_msg = Twist(); twist_msg.linear.x = speed

        self.periodic_action_timer = self.create_timer(0.1, lambda: self.pub_cmd.publish(twist_msg))
        
        # stop_movement 함수 호출 시 현재 command_id를 함께 넘겨주도록 람다 함수 수정
        self.one_shot_stop_timer = self.create_timer(duration, lambda: self.stop_movement(command_id))

        self.get_logger().info(f"Moving (ID: {command_id}) for {duration:.2f}s.")

    def stop_movement(self, command_id):
        # <<< 변경점 5: 이 함수를 호출한 타이머의 ID가 현재 유효한 최신 ID인지 확인
        if command_id != self.current_command_id:
            self.get_logger().info(f"Ignoring outdated stop_movement call from ID: {command_id} (Current ID: {self.current_command_id})")
            return

        self.get_logger().info(f"Stopping movement for ID: {command_id}")
        self.cancel_previous_action()
        stop_msg = Twist(); stop_msg.linear.x = 0.0
        self.pub_cmd.publish(stop_msg)
        turn_msg = Int8(); turn_msg.data = 0
        self.pub_turn.publish(turn_msg)
        self.get_logger().info("Movement stopped.")

    def execute_rotate(self, params, command_id):
        angle = params.get('angular_velocity', 0.0)
        is_clockwise = params.get('is_clockwise', True)

        if angle <= 0 or self.angular_speed <= 0: return
        
        duration = angle / self.angular_speed
        
        turn_msg = Int8(); turn_msg.data = 1 if is_clockwise else -1

        self.periodic_action_timer = self.create_timer(0.1, lambda: self.pub_turn.publish(turn_msg))
        self.one_shot_stop_timer = self.create_timer(duration, lambda: self.stop_rotation(command_id))
        
        self.get_logger().info(f"Rotating (ID: {command_id}) for {duration:.2f}s.")

    def stop_rotation(self, command_id):
        if command_id != self.current_command_id:
            self.get_logger().info(f"Ignoring outdated stop_rotation call from ID: {command_id} (Current ID: {self.current_command_id})")
            return
            
        self.get_logger().info(f"Stopping rotation for ID: {command_id}")
        self.cancel_previous_action()
        stop_msg = Int8(); stop_msg.data = 0
        self.pub_turn.publish(stop_msg)
        self.get_logger().info("Rotation stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = llm_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 ID를 0으로 설정하여 모든 진행중인 동작을 멈추게 함
        node.current_command_id = 0
        node.stop_movement(0)
        node.stop_rotation(0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()