#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import serial


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # === 시리얼 포트 설정 ===
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # === 구독 토픽 ===
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.turn_sub = self.create_subscription(Int8, '/turn_dir', self.on_turn, 10)

        # 내부 상태
        self.last_turn = 0       # -1=좌회전, 0=전진, 1=우회전
        self.last_cmd = Twist()  # cmd_vel 저장

        self.get_logger().info("MotorBridge started, listening on /cmd_vel and /turn_dir")

    def on_turn(self, msg: Int8):
        self.last_turn = msg.data
        self.send_command()

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        if self.last_turn == 0:  # 전진일 때만 반영
            self.send_command()

    def send_command(self):
        # === 회전 모드 우선 ===
        if self.last_turn == -1:   # 좌회전
            m1, m2 = 100, -100
        elif self.last_turn == 1:  # 우회전
            m1, m2 = -100, 100
        else:
            # cmd_vel의 linear.x를 속도로 사용 (단위: rps)
            # 원하는 스케일에 맞게 조정 필요
            v = self.last_cmd.linear.x
            m1, m2 = int(v), int(v)

        # === 시리얼 전송 ===
        cmd_str = f"CMD M1:{m1} M2:{m2}\n"
        self.ser.write(cmd_str.encode('utf-8'))
        self.get_logger().info(f"Sent -> {cmd_str.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
