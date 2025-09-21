#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocationCommandNode(Node):
    def __init__(self):
        super().__init__('location_command_node')

        # 파라미터
        self.in_topic = self.declare_parameter('in_topic', '/text_to_location') \
            .get_parameter_value().string_value
        self.location_topic = self.declare_parameter('location_topic', '/target_location') \
            .get_parameter_value().string_value

        # 퍼블리셔/구독자
        self.pub = self.create_publisher(String, self.location_topic, 10)
        self.sub = self.create_subscription(String, self.in_topic, self._on_text, 10)

        # 모든 정규식 패턴과 추출 로직은 IntentRouter로 이전되었습니다.
        
        self.get_logger().info(
            f"LocationCommandNode started (Refactored): in={self.in_topic}, out={self.location_topic}"
        )

    def _on_text(self, msg: String):
        # 이제 입력 토픽의 데이터는 IntentRouter가 추출한 순수 목적지 이름입니다.
        target = (msg.data or '').strip()
        
        if target:
            # 전달받은 목적지 이름을 그대로 발행합니다.
            out = String()
            out.data = target
            self.pub.publish(out)
            self.get_logger().info(f"Received target location '{target}'. Publishing to {self.location_topic}")

def main(args=None):
    rclpy.init(args=args)
    node = LocationCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
