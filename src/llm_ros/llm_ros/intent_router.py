# intent_router.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from typing import Optional

# 장소 의도 금지어: 방향/위치 부사들은 "장소"가 아님
DIRECTION_STOPWORDS = {
    "앞", "앞으로", "뒤", "뒤로", "옆", "오른쪽", "왼쪽", "좌측", "우측",
    "위", "아래", "정면", "후방", "근처", "밖", "안", "내부", "외부"
}

MOVE_VERBS_PATTERN = r"(?:가|가자|가줘|이동|이동해|이동하|이동시켜|이동해줘)$"

# 호출 단어 제거 로직
WAKEWORD_PATTERN = re.compile(r'^\s*(도리|돌이|둘리)[야아]?[,\s]*', re.IGNORECASE)
def _strip_wakeword(text: str) -> str:
    return WAKEWORD_PATTERN.sub('', text, count=1).strip()


def extract_location_command(text: str) -> Optional[str]:
    """
    주어진 텍스트에서 장소 관련 명령을 분석하여 목적지나 행동을 추출합니다.
    - 저장/삭제/목록 조회 명령을 처리합니다.
    - 목적지 이동 명령을 처리하며, 방향/위치 부사는 목적지에서 제외합니다.
    """
    # 1. "여기를 X로 저장"
    match = re.search(r"여기를\s*(.+?)(?:으로|로)\s*저장", text)
    if match:
        return match.group(1).strip()

    # 2. "장소/위치 + (저장|삭제|목록)"
    match = re.search(r"(장소|위치)\s*(저장|삭제|목록)", text)
    if match:
        return match.group(2).strip() # "저장", "삭제", "목록" 반환

    # 3. "X(으)로 + 이동동사" 패턴 (사용자 로직 기반)
    match = re.search(r"(.+?)(?:으로|로)\s*" + MOVE_VERBS_PATTERN, text)
    if match:
        base = match.group(1).strip()
        if base:
            last_token = base.split()[-1]
            if last_token in DIRECTION_STOPWORDS:
                return None  # 방향/위치 부사어이므로 장소 명령이 아님
            if len(last_token) < 2 and len(base.split()) == 1:
                return None  # 한 글자 목적지는 노이즈로 간주
            return base  # 목적지 반환

    # 4. 숫자 + "번" 이동
    match = re.search(r"(\d+)\s*번", text)
    if match and re.search(MOVE_VERBS_PATTERN, text):
        return f"{match.group(1)}번" # "1번" 과 같이 숫자와 번을 함께 반환

    # 5. "저장해줘" 등 간단한 저장 명령
    if re.search(r"(저장해줘|기억해줘)", text):
        return "저장" # "저장" 액션 반환

    return None


class IntentRouter(Node):
    def __init__(self):
        super().__init__('intent_router')
        self.sub = self.create_subscription(String, '/text_command', self.cb, 10)
        self.pub_loc = self.create_publisher(String, '/text_to_location', 10)
        self.pub_llm = self.create_publisher(String, '/text_to_llm', 10)
        self.get_logger().info("IntentRouter started (User-logic applied).")

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