#!/usr/bin/env python3

import os
import json
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.subscription = self.create_subscription(String, 'text_command', self.callback, 10)
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)
        self.api_key = os.getenv('OPENAI_API_KEY')
        openai.api_key = self.api_key
        self.get_logger().info('LLMNode initialized.')

    def callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received command: {text}')
        json_result = self.ask_gpt(text)
        if json_result:
            out_msg = String()
            out_msg.data = json_result
            self.publisher.publish(out_msg)
            self.get_logger().info(f'Published GPT response to /voice_cmd')

    def ask_gpt(self, text_command):
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
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that only returns JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.0
            )
            reply = response.choices[0].message['content']
            
            # Extract JSON from the response
            try:
                start = reply.find('{')
                end = reply.rfind('}') + 1
                json_response = reply[start:end]
                # Validate that the extracted string is valid JSON
                json.loads(json_response)
                return json_response
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Failed to parse JSON from GPT response: {reply}. Error: {e}')
                return None

        except Exception as e:
            self.get_logger().error(f'[GPT Error] {str(e)}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
