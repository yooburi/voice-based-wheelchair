#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocationCommandNode(Node):
    def __init__(self):
        super().__init__('location_command_node')

        # 파라미터
        self.in_topic = self.declare_parameter('in_topic', '/text_command') \
            .get_parameter_value().string_value
        self.location_topic = self.declare_parameter('location_topic', '/target_location') \
            .get_parameter_value().string_value

        # 퍼블리셔/구독자
        self.pub = self.create_publisher(String, self.location_topic, 10)
        self.sub = self.create_subscription(String, self.in_topic, self._on_text, 10)

        # 명령 패턴들 (우선순위: 숫자 → 명칭)
        # - 숫자 패턴: "1번으로 가", "1 번 가", "1번 위치로 이동" 등
        # - 명칭 패턴: "화장실로 가", "문 앞으로 가", "문쪽으로 가", "문까지 가" 등
        # 문장 중 어디에 있어도 허용. 첫 번째 매칭만 사용.
        self._num_patterns = [
            re.compile(r"(\d+)\s*번\s*(?:위치)?\s*(?:으로|로)?\s*(?:가|이동|가자|가줘|가라)")
        ]

        # 한글/영문으로 된 단어(간단 버전). 필요 시 공백 포함 확장 가능.
        word = r"([가-힣A-Za-z]{1,20})"
        verbs = r"(?:가|이동|가자|가줘|가라)"
        # '앞으로/쪽으로/방향으로/까지' 같은 표현 포함
        self._name_patterns = [
            re.compile(fr"{word}\s*앞(?:으)?로\s*{verbs}"),
            re.compile(fr"{word}(?:으)?로\s*{verbs}"),
            re.compile(fr"{word}\s*(?:쪽|방향|근처|까지)\s*(?:으로|로)?\s*{verbs}"),
        ]

        self.get_logger().info(
            f"target_location시작: in={self.in_topic}, out={self.location_topic}"
        )

    def _on_text(self, msg: String):
        text = (msg.data or '').strip()
        target = self._extract_location_target(text)
        if target is not None:
            out = String()
            out.data = target
            self.pub.publish(out)
            self.get_logger().info(f"위치 명령 인식: target='{target}' → {self.location_topic}")
        else:
            # 필요 시 디버그: self.get_logger().debug(f"no location in: {text}")
            pass

    def _extract_location_target(self, text: str) -> Optional[str]:
        # 1) 숫자 기반 (기존 동작 유지)
        for pat in self._num_patterns:
            m = pat.search(text)
            if m:
                return str(int(m.group(1)))

        # 2) 명칭 기반 (한글/영문 단어)
        for pat in self._name_patterns:
            m = pat.search(text)
            if m:
                name = m.group(1)
                # 안전 차원 간단 정리
                name = name.strip()
                if name:
                    return name

        return None


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
