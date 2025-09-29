# VOICE_BASED_WHEELCHAIR

전신마비 장애인의 자율적인 이동을 지원하는 ROS 2 기반 음성 제어 전동휠체어 시스템.

## 영상/데모/스크린샷
- ![Image](https://github.com/user-attachments/assets/0e381624-1816-4ee6-92d2-233467a90550)
- https://www.youtube.com/watch?v=IIPmA_FyYE8
- rqt_graph 토폴로지
  - ![rqt_graph](image-1.png)
- RViz 설정 파일: `dolchair.rviz` (TF/Path/Marker 시각화에 활용)

## Features(주요 기능)
- 음성 인식(STT)
  - `voice2text`: 마이크 입력 → Whisper 기반 전사(ko/en 등), 간단 VAD, 하이패스 필터, CPU/GPU 자동 선택
  - 출력 토픽: `/voice2text` (String)
- 명령 이해/라우팅
  - `llm_ros/filter_input_text`: 호출어 제거/전처리 → `/text_command`
  - `llm_ros/intent_router`: 장소 이동/저장 등 의도 분리 → 장소(`/text_to_location`) vs 일반(`/text_to_llm`)
  - `llm_ros/location_command`: 추출된 목적지 → `/target_location`
  - `llm_ros/llm_node`: OpenAI API로 자연어 → 제어 JSON(`/voice_cmd`)
- 경로 계획/추종
  - `path_planner/make_pathnplan`: 현재 위치 → 라벨 좌표(Path 생성, `/plan`)
  - `path_planner/path_follower`: 정렬(스핀) → 순수추종(Pure Pursuit) 주행, `/turn_dir`+`/auto_steer`+`/cmd_vel` 발행, 마커 시각화
- 저수준 구동 브리지
  - `motor_bridge`: 시리얼로 아두이노와 통신, 좌우 바퀴 rps 명령 전송
  - 스핀 모드(`/turn_dir`)와 조향각(`/auto_steer`)·속도(`/cmd_vel`) 처리, 타임아웃/스타트업 지연 등 안전장치
- 로컬라이제이션/SLAM
  - LiDAR 오도메트리(rf2o) + EKF(robot_localization) + AMCL
  - 맵 생성/저장(slam_toolbox) 및 맵 서버 연동
  - TF: `map → odom → base_link → {laser_frame, imu_link}`
- 설정/리소스
  - 장소 라벨: `config/location/location.yaml`
  - 맵 파일: `config/maps/*.yaml|*.pgm`
  - URDF: `src/2D_LiDAR/wheelchair_slam_bringup/urdf/rplidar_myahrs.urdf`

## Usage
- 빌드/셋업
  - 워크스페이스 루트에서:
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```
  - OpenAI API 키(LLM 사용 시):
    ```
    export OPENAI_API_KEY=your_api_key
    ```
- 권장 실행 순서
  1) 정적 TF/URDF
     ```
     ros2 run robot_state_publisher robot_state_publisher \
       --ros-args -p robot_description:="$(cat src/2D_LiDAR/wheelchair_slam_bringup/urdf/rplidar_myahrs.urdf)"
     ```
  2) 센서 드라이버
     ```
     ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
     ros2 launch myahrs_ros2_driver myahrs_ros2_driver.launch.py
     ros2 launch imu_preprocess imu_preprocess.launch.py
     ```
  3) LiDAR 오도메트리(rf2o)
     - EKF를 함께 쓰면 `publish_tf:=false` 권장
     ```
     ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
     ```
  4) EKF(robot_localization)
     ```
     ros2 run robot_localization ekf_node \
       --ros-args -r __node:=ekf_filter_node \
       --params-file src/2D_LiDAR/wheelchair_slam_bringup/config/ekf_odom.yaml
     ```
  5) 맵 로드 & 로컬라이제이션(AMCL)
     ```
     ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/yoo/workspace/dolchair_ws/config/maps/gongA_map.yaml
     ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
     ros2 run nav2_amcl amcl --ros-args --params-file src/2D_LiDAR/wheelchair_slam_bringup/config/amcl.yaml
     ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate
     ```
  6) 음성 파이프라인(명령 처리)
     ```
     ros2 run voice2text voice2text
     ros2 run llm_ros filter_input_text
     ros2 run llm_ros intent_router
     ros2 run llm_ros location_command
     ros2 run llm_ros llm_node
     ```
  7) 경로 생성 & 주행
     ```
     ros2 run path_planner make_pathnplan
     ros2 run path_planner path_follower
     ```
- SLAM(맵 생성/저장)
  ```
  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$PWD/src/2D_LiDAR/wheelchair_slam_bringup/config/slam.yaml

  ros2 run nav2_map_server map_saver_cli -f ./config/maps/dolbang_map
  ```
  - EKF 없이 맵 생성만: rf2o `publish_tf:=true`
  - EKF와 함께: rf2o `publish_tf:=false`, EKF가 `odom→base_link` TF 단일 발행
- 토픽 테스트 예시
  ```
  ros2 topic pub /target_location std_msgs/msg/String "{data: '화장실'}"
  ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '도리야 앞으로 1m 가줘.'}"
  ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '도리야 시계방향으로 90도 돌아줘.'}"
  ```
- 의존성(주요)
  - ROS 2 + nav2_map_server, nav2_amcl, slam_toolbox, robot_localization
  - Python: `openai`, `openai-whisper`, `torch`, `sounddevice`, `scipy`, `numpy`, `PyYAML`, `tf2_ros`
  - 하드웨어: 2D LiDAR(A2M8 등), myAHRS+, 마이크, 아두이노(시리얼 `/dev/ttyACM0` 기본)

## 구조
```
.
├─ config/
│  ├─ location/location.yaml           # 장소 라벨 좌표
│  └─ maps/*.yaml, *.pgm               # 맵 리소스
├─ src/
│  ├─ voice2text/                      # STT 노드
│  │  └─ voice2text/voice2text.py
│  ├─ llm_ros/                         # LLM/라우팅
│  │  └─ llm_ros/
│  │     ├─ filter_input_text.py
│  │     ├─ intent_router.py
│  │     ├─ location_command.py
│  │     └─ llm_node.py
│  ├─ path_planner/                    # 경로 계획/추종
│  │  └─ path_planner/
│  │     ├─ make_pathnplan.py          # 라벨 → Path
│  │     └─ path_follower.py           # 정렬+순수추종
│  ├─ motor_bridge/                    # 시리얼 브리지
│  │  └─ motor_bridge/motor_bridge_node.py
│  ├─ 2D_LiDAR/                        # LiDAR/SLAM/URDF
│  │  ├─ sllidar_ros2/launch/view_sllidar_a2m8_launch.py
│  │  ├─ rf2o_laser_odometry/launch/rf2o_laser_odometry.launch.py
│  │  └─ wheelchair_slam_bringup/
│  │     ├─ urdf/rplidar_myahrs.urdf
│  │     └─ config/{ekf_odom.yaml, amcl.yaml, slam.yaml}
│  └─ IMU/
│     ├─ myahrs_ros2_driver/...
│     └─ imu_preprocess/launch/imu_preprocess.launch.py
├─ Arduino/
│  └─ WHEEL_CHAIR_SERIAL/WHEEL_CHAIR_SERIAL.ino
└─ dolchair.rviz
```

## 로드맵
- 장애물 회피 연동(로컬/전역 플래너 보강, 동적 재계획)
- 장소 저장/관리(서비스/GUI) 및 사용자 교정 워크플로우
- LLM 의존도 축소(온디바이스 명령 파서/룰 기반 보완)
- 음성 파이프라인 안정화(VAD/노이즈 처리/디바이스 관리)
- 브리지/아두이노 파라미터 자동 보정(스케일/지연/속도한계)
- 통합 launch/bringup 패키지화, 로깅/리플레이 도구
- 테스트/시뮬레이션(RViz/Navi2) 및 CI 파이프라인 정비

## 참고/인용
- rf2o_laser_odometry, slam_toolbox, robot_localization, nav2(map_server, amcl)
- sllidar_ros2 (Slamtec)
- OpenAI Whisper(STT), OpenAI API(LLM)
- myAHRS+ ROS2 드라이버(WITHROBOT)
