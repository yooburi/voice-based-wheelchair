#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import unicodedata

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FilterInputNode(Node):
    def __init__(self):
        super().__init__('filter_input_node')

        # 파라미터
        self.in_topic = self.declare_parameter('in_topic', '/voice2text') \
            .get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/text_command') \
            .get_parameter_value().string_value
        self.echo_topic = self.declare_parameter('echo_topic', True) \
            .get_parameter_value().bool_value  # /voice2text 들어온 걸 콘솔에도 보여줄지

        # 🔎 필터링 관련 파라미터
        # 필터링 키워드 (단일)
        self.trigger_keyword = self.declare_parameter('trigger_keyword', '도리') \
            .get_parameter_value().string_value
        # 필터링 키워드 배열 (여러 개를 허용). 미지정 시 단일 키워드만 사용
        try:
            self.trigger_keywords = list(
                self.declare_parameter('trigger_keywords', ['도리', '돌이']) \
                .get_parameter_value().string_array_value
            )
        except Exception:
            self.trigger_keywords = []
        # 대소문자 구분 여부 (한글은 영향 없음)
        self.case_sensitive = self.declare_parameter('case_sensitive', False) \
            .get_parameter_value().bool_value
        
        # 키워드가 문장 앞(prefix)에 있을 때만 통과시킬지 여부
        self.prefix_only = self.declare_parameter('prefix_only', True) \
            .get_parameter_value().bool_value

        # Pub/Sub
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.sub = self.create_subscription(String, self.in_topic, self._on_voice2text, 10)

        # 안내 출력
        shown_keywords = self.trigger_keywords[:] if self.trigger_keywords else [self.trigger_keyword]
        self.get_logger().info(
            f"필터링 input 노드 시작. voice2text (filters: {shown_keywords}, "
            f"case_sensitive={self.case_sensitive}, prefix_only={self.prefix_only})"
        )
        print(">>> ", end="", flush=True)

        # 표준입력 스레드 시작
        self._stdin_thread = threading.Thread(target=self._read_stdin_loop, daemon=True)
        self._stdin_thread.start()

    # ----- filter -----
    # trigger_keyword가 조건에 맞으면 True 반환하는 함수
    def _should_forward(self, text: str) -> bool:
        """
        'trigger_keyword'가 조건에 맞으면 True.
        - prefix_only=True: 문장 시작이 키워드로 시작해야 통과
        - prefix_only=False: 문장 내에 키워드가 포함되면 통과
        한글 정규화를 위해 NFC로 맞추고, case_sensitive=False면 casefold() 비교.
        """
        # 유니코드 정규화 (한글 조합/분해 차이 방지)
        t = unicodedata.normalize('NFC', text or '').lstrip()

        # 키워드 목록 구성 (배열 설정이 있으면 우선, 없으면 단일)
        raw_keywords = self.trigger_keywords if self.trigger_keywords else [self.trigger_keyword]
        keywords = [unicodedata.normalize('NFC', k or '') for k in raw_keywords if (k or '').strip()]
        if not keywords:
            return False

        if self.case_sensitive:
            t_cmp = t
            ks_cmp = keywords
        else:
            t_cmp = t.casefold()
            ks_cmp = [k.casefold() for k in keywords]

        if self.prefix_only:
            return any(t_cmp.startswith(k) for k in ks_cmp)
        else:
            return any(k in t_cmp for k in ks_cmp)

    def _forward(self, text: str, source: str):
        """필터 통과 시 퍼블리시, 아니면 로그만 남김."""
        if self._should_forward(text):
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"[PASS] {source} → {self.out_topic}: {text}")
        else:
            shown_keywords = self.trigger_keywords[:] if self.trigger_keywords else [self.trigger_keyword]
            self.get_logger().info(f"[DROP] {source} (no keywords {shown_keywords})")

    # ----- /voice2text → /text_command 브리지 -----
    def _on_voice2text(self, msg: String):
        # 콘솔에도 표시
        if self.echo_topic:
            print(f"\n[TOPIC {self.in_topic}] {msg.data}")
            print(">>> ", end="", flush=True)

        # 필터 후 전달
        self._forward(msg.data, f"topic:{self.in_topic}")

    # ----- 표준입력 → /text_command -----
    def _read_stdin_loop(self):
        try:
            for line in sys.stdin:
                text = (line or '').strip()
                if not text:
                    print(">>> ", end="", flush=True)
                    continue
                self._forward(text, "stdin")
                print(">>> ", end="", flush=True)
        except Exception as e:
            self.get_logger().error(f"stdin loop error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FilterInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
