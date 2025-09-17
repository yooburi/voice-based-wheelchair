#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# /text_command 구독 → LLM에 질의 → /voice_cmd 퍼블리시

import os
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        # 파라미터: 구독/퍼블리시 토픽명 (필요 시 변경 가능)
        self.in_topic = self.declare_parameter('in_topic', '/text_command') \
            .get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/voice_cmd') \
            .get_parameter_value().string_value

        # OpenAI API 키 (환경변수에서 읽음)
        self.api_key = os.getenv('OPENAI_API_KEY', '')

        # 구독/퍼블리셔 설정
        self.sub = self.create_subscription(String, self.in_topic, self._on_command, 10)
        self.pub = self.create_publisher(String, self.out_topic, 10)

        # 게으른 임포트: openai
        self._openai = None
        try:
            import openai  # type: ignore
            self._openai = openai
            if self.api_key:
                self._openai.api_key = self.api_key
        except Exception as e:
            self.get_logger().error(f"openai 모듈 임포트 실패: {e}")

        if not self.api_key:
            self.get_logger().warn("환경변수 OPENAI_API_KEY가 설정되지 않았습니다. 요청이 실패할 수 있습니다.")

        self.get_logger().info(
            f"LLMNode 시작: in={self.in_topic}, out={self.out_topic}"
        )

    def _on_command(self, msg: String):
        text = msg.data
        self.get_logger().info(f"수신: {text}")
        json_result = self._ask_gpt(text)
        if json_result:
            out = String()
            out.data = json_result
            self.pub.publish(out)
            self.get_logger().info(f"퍼블리시: {self.out_topic}")

    def _ask_gpt(self, text_command: str):
        """
        주어진 자연어 명령을 JSON 스키마에 맞춰 변환하도록 LLM에 질의.
        응답에서 JSON 부분만 추출하여 문자열로 반환. 실패 시 None.
        """
        if not self._openai:
            self.get_logger().error("openai 라이브러리가 로드되지 않았습니다.")
            return None
        if not (self.api_key and self.api_key.strip()):
            self.get_logger().error("OPENAI_API_KEY가 비어있습니다.")
            return None

        prompt = f'''
You are a robot that provides only JSON as a response.
Consider the following ontology:
{{
    "action": "move",
    "params": {{
        "linear_speed": "a float number",
        "distance": "a float number",
        "is_forward": "a boolean"
    }}
}},
{{
    "action": "rotate",
    "params": {{
        "angular_velocity": "a float number, degrees per second",
        "is_clockwise": "a boolean"
    }}
}}
The user will provide a prompt and you will return a JSON response with the parsed action and parameters.

prompt: "앞으로 1미터 가줘"
returns: {{"action": "move", "params": {{"linear_speed": 0.2, "distance": 1.0, "is_forward": true}}}}

prompt: "시계 방향으로 90도 회전해줘"
returns: {{"action": "rotate", "params": {{"angular_velocity": 90.0, "is_clockwise": true}}}}

prompt: "{text_command}"
returns:
'''

        try:
            # 구 버전 ChatCompletion 사용 (원본 스크립트 호환)
            response = self._openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that only returns JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.0
            )
            reply = response.choices[0].message['content']

            # 응답 문자열에서 JSON만 추출
            try:
                start = reply.find('{')
                end = reply.rfind('}') + 1
                json_response = reply[start:end]
                json.loads(json_response)  # 유효성 검사
                return json_response
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"JSON 파싱 실패: {reply}. Error: {e}")
                return None

        except Exception as e:
            self.get_logger().error(f"[GPT Error] {str(e)}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

