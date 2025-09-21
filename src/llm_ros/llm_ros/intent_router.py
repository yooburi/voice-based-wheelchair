# intent_router.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from typing import Optional

# 호출 단어 제거 로직
WAKEWORD_PATTERN = re.compile(r'^\s*(도리|돌이)\s*(?:야|아)?[\s,:\-]*', re.IGNORECASE)
def _strip_wakeword(text: str) -> str:
    return WAKEWORD_PATTERN.sub('', text, count=1).strip()

# 모든 장소 관련 정규식 패턴을 이곳으로 통합하고, 목적어를 추출하도록 수정합니다.

# 1. 장소 저장/관리 관련 패턴
SAVE_PATTERNS = [
    re.compile(r"여기를\s*(.+?)\s*(?:으로|로)\s*저장"),
    re.compile(r"(장소|위치)\s*(저장|삭제|목록)"),
    re.compile(r"(저장해줘|저장|기억해줘)"),
]

# 2. 장소 이동 관련 패턴
MOVE_PATTERNS = [
    re.compile(r"(\d+)\s*번\s*(?:위치)?\s*(?:으로|로)?\s*(?:가|이동|가자|가줘|가라)"),
]
# f-string을 사용하여 더 복잡한 패턴 구성
# Greedy 방지를 위해 non-greedy match(+?) 사용
word = r"([가-힣A-Za-z0-9]+?)" 
verbs = r"(?:가|이동|가자|가줘|가라)"
MOVE_PATTERNS.extend([
    re.compile(fr"{word}\s*앞(?:으)?로\s*{verbs}"),
    re.compile(fr"{word}(?:으)?로\s*{verbs}"),
    re.compile(fr"{word}\s*(?:쪽|방향|근처|까지)\s*(?:으로|로)?\s*{verbs}"),
    re.compile(fr"{word}\s*{verbs}"),
])


def extract_location_command(text: str) -> Optional[str]:
    """
    주어진 텍스트에서 장소 관련 명령을 분석합니다.
    명령이 감지되면, 추출된 목적지나 행동을 반환합니다.
    아니면 None을 반환합니다.
    """
    for pat in SAVE_PATTERNS:
        m = pat.search(text)
        if m:
            return m.group(m.lastindex or 1)

    for pat in MOVE_PATTERNS:
        m = pat.search(text)
        if m:
            return m.group(1)

    return None


class IntentRouter(Node):
    def __init__(self):
        super().__init__('intent_router')
        self.sub = self.create_subscription(String, '/text_command', self.cb, 10)
        self.pub_loc = self.create_publisher(String, '/text_to_location', 10)
        self.pub_llm = self.create_publisher(String, '/text_to_llm', 10)
        self.get_logger().info("IntentRouter started (Refactored).")

    def cb(self, msg: String):
        # 1. 호출 단어 제거
        cleaned_text = _strip_wakeword(msg.data)
        if not cleaned_text:
            return # 호출 단어만 들어온 경우 무시

        # 2. 장소 관련 명령어인지 확인 및 목적지 추출
        location_target = extract_location_command(cleaned_text)
        
        if location_target:
            # 장소 명령이면, 추출된 목적어를 발행
            out_msg = String()
            out_msg.data = location_target.strip()
            self.get_logger().info(f"Intent: Location. Target: '{out_msg.data}'. Routing to /text_to_location")
            self.pub_loc.publish(out_msg)
        else:
            # 장소 명령이 아니면, 호출 단어가 제거된 텍스트를 LLM으로 라우팅
            llm_msg = String()
            llm_msg.data = cleaned_text
            self.get_logger().info(f"Intent: General. Routing to /text_to_llm: '{cleaned_text}'")
            self.pub_llm.publish(llm_msg)


def main():
    rclpy.init()
    node = IntentRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
