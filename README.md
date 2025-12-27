# VOICE_BASED_WHEELCHAIR

전신마비 장애인의 자율적인 이동을 지원하는 ROS 2 기반 음성 제어 전동휠체어 시스템.

## Video/GIF/rqt_graph
- ![Image](https://github.com/user-attachments/assets/0e381624-1816-4ee6-92d2-233467a90550)
- https://www.youtube.com/watch?v=IIPmA_FyYE8
- RViz 설정 파일: `dolchair.rviz` (TF/Path/Marker 시각화에 활용)

## 핵심 기능 요약
- 웨이크 워드 감지: Picovoice Porcupine으로 “도리야” 감지 → STT 활성화 토글(`/activate_stt`)
- 음성 인식(STT): 마이크 → 하이패스/노이즈감소/실레로 VAD → Whisper 전사 → `/voice2text`
- 의도 라우팅: 장소 관련 명령 추출(`save/go/delete/list`) → `/text_to_location`, 일반 대화/명령 → `/text_to_llm`
- LLM 명령 변환: OpenAI API로 자연어 → 제어 JSON 스키마(`move/rotate`) → `/voice_cmd`
- 저수준 제어: JSON 수신 후 `/cmd_vel`, `/turn_dir`를 주기적으로 발행하여 이동/회전 실행, 타이머/ID로 안전 중단
- 내비게이션: 라벨 좌표(`/target_location`) → Nav2 NavigateToPose 액션으로 자율 주행
- 모터 브리지: `/cmd_vel`+`/auto_steer`+`/turn_dir` → 좌우 바퀴 rps 변환 → 시리얼 명령 전송

## 전체 알고리즘 흐름
1) 웨이크: `llm_ros/wake_word_detector(_filter)`가 “도리야” 감지 → `/activate_stt`=True 발행(타임아웃 시 False)
2) STT: `voice2text`가 활성 상태에서 오디오를 버퍼링 → 하이패스 → DSP 노이즈 감소(noisereduce) → Silero VAD(선택) → Whisper 전사 → `/voice2text`
3) 라우팅: `llm_ros/intent_router`가 텍스트에서 장소 명령을 추출하면 `/text_to_location`으로, 아니면 `/text_to_llm`으로 전달
4-a) 장소 흐름: `llm_ros/location_command`
   - save: 현재 TF(map←base_link) 포즈를 라벨로 저장(config/location/location.yaml)
   - go: 라벨을 `/target_location`으로 발행 → `path_planner/nav2_pathnplan`이 YAML에서 좌표를 읽고 Nav2 `navigate_to_pose` 액션 목표 전송
4-b) 일반 명령 흐름: `llm_ros/llm_node`가 OpenAI ChatCompletion으로 JSON 스키마를 생성 → `/voice_cmd` 발행
5) 저수준 실행: `llm_control/llm_control` 또는 `llm_control/llm_motor`
   - move: 거리와 속도로 지속시간 계산 → 10Hz로 `/cmd_vel` 발행, 종료 시 정지
   - rotate: 방향에 따라 `/turn_dir`=±1 주기 발행, 종료 시 0
   - 안전장치: 명령 고유 ID로 이전 타이머의 오동작(늦은 stop)을 차단
6) 구동: `motor_bridge`가 `/cmd_vel`·`/auto_steer`·`/turn_dir`을 받아 양 바퀴 rps로 변환 후 시리얼로 전송(또는 `md_controller` 사용 시 /cmd_vel→RPM 변환/엔코더→/odom)

## 주요 노드와 토픽
- Wake Word: `/activate_stt`(Bool)
- STT: `/voice2text`(String)
- Intent Router: `/text_to_location`, `/text_to_llm`(String)
- Location Command: `/target_location`(String), YAML: `config/location/location.yaml`
- Nav2 Path Planner: 액션 `navigate_to_pose`(Nav2), 프레임 `map`/`base_link`
- LLM Node: 입력 `/text_to_llm` → 출력 `/voice_cmd`(String, JSON)
- LLM Control: 입력 `/voice_cmd` → 출력 `/cmd_vel`(Twist), `/turn_dir`(Int8)
- Motor Bridge: 입력 `/cmd_vel`/`/auto_steer`/`/turn_dir` → 시리얼 `CMD M1:.. M2:..`

## 빠른 실행(필수 흐름)
- 센서/내비게이션
  - SLAM으로 맵 생성(선택): `ros2 launch nav2_wheel wheelchair.launch.py slam:=True`
  - Localization: `ros2 launch nav2_wheel wheelchair.launch.py`
- 음성 파이프라인
  - `ros2 run llm_ros wake_word_detector`
  - `ros2 run voice2text voice2text`
  - `ros2 run llm_ros intent_router`
  - `ros2 run llm_ros location_command`
  - `ros2 run llm_ros llm_node`
- 구동(택1)
  - 시리얼 브리지: `ros2 run motor_bridge motor_bridge`
  - 모터 드라이버: `ros2 launch md_controller md_controller.launch.py`

## 설정 포인트
- LLM(OpenAI): 환경변수 `OPENAI_API_KEY` 필요 (`llm_ros/llm_node.py`)
- 웨이크워드: `picovoice_access_key`, `ppn_file_path`, `pv_model_path` 파라미터 (`src/llm_ros/config/wake_word/*` 참고)
- STT: Whisper 모델/장치 자동 선택, VAD/NR/하이패스 등 파라미터(`src/voice2text/voice2text/voice2text.py`)
- 장소 라벨: `config/location/location.yaml`에 좌표(x,y,yaw 라디안) 저장/관리
- 저수준 제어 파라미터
  - `llm_control/config/wheelchair_params.yaml`(선속/각속 기본값)
  - `motor_bridge`의 기하/제한/시리얼 설정(port/baud, wheel_radius, track_width, L_virtual, spin_rps_fixed, tx_scale 등)

## 테스트 예시
```bash
# 목적지 내비게이션
ros2 topic pub -1 /target_location std_msgs/msg/String "{data: '화장실'}"

# 일반 명령(LLM 경유)
ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '도리야 앞으로 1m 가줘.'}"
ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '도리야 시계방향으로 90도 돌아줘.'}"
```

## 참고
- Nav2 설정은 `src/nav2_wheel/params/nav2_params.yaml` 참고
- LiDAR/IMU/Depth 센서 런치: `src/2D_LiDAR/wheelchair_slam_bringup/launch/*`, `src/IMU/*`, `src/realsense-ros/*`
- TF 트리: `map → odom → base_link → {laser_frame, imu_link}`
