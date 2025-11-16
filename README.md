# VOICE_BASED_WHEELCHAIR

전신마비 장애인의 자율적인 이동을 지원하는 ROS 2 기반 음성 제어 전동휠체어 시스템.

## Video/GIF/rqt_graph
- ![Image](https://github.com/user-attachments/assets/0e381624-1816-4ee6-92d2-233467a90550)
- https://www.youtube.com/watch?v=IIPmA_FyYE8
- rqt_graph 토폴로지
![rqt_graph](rqt_graph.png)
- RViz 설정 파일: `dolchair.rviz` (TF/Path/Marker 시각화에 활용)

## Features(주요 기능)
- 음성 파이프라인
  - `llm_ros/wake_word_detector`: "도리야" 음성 호출어를 감지하고 `/activate_stt` (Bool) 토픽을 발행하여 STT 노드를 제어합니다.
  - `voice2text`: `/activate_stt`가 `True`일 때만 활성화되어, 마이크 입력을 Whisper STT를 통해 텍스트로 변환하고 `/voice2text` 토픽에 발행합니다.
  - `llm_ros/intent_router`: `/voice2text` 토픽을 구독하여, 사용자 발화의 의도를 '장소 이동'과 '일반 명령'으로 분리합니다.
  - `llm_ros/location_command`: '장소 이동' 의도인 경우, 목적지를 추출하여 `/target_location` 토픽에 발행합니다.
  - `llm_ros/llm_node`: '일반 명령' 의도인 경우, OpenAI API를 사용하여 자연어를 휠체어 제어 명령(JSON 형식)으로 변환하고 `/voice_cmd` 토픽에 발행합니다.
- 경로 계획/추종
  - `path_planner/make_pathnplan`: 현재 위치 → 라벨 좌표(Path 생성, `/plan`)
  - `path_planner/path_follower`: 정렬(스핀) → 순수추종(Pure Pursuit) 주행, `/turn_dir`+`/auto_steer`+`/cmd_vel` 발행, 마커 시각화
- 저수준 구동 브리지
  - `MD_motor_drive : /cmd_vel > RPM으로 변환 후 모터 제어, 엔코더 > /odom 토픽으로 변환
- 로컬라이제이션/SLAM
  - LiDAR 오도메트리(rf2o) + EKF(robot_localization) + AMCL
  - 맵 생성/저장(slam_toolbox) 및 맵 서버 연동
  - TF: `map → odom → base_link → {laser_frame, imu_link}`
- 설정/리소스
  - 장소 라벨: `nav2_wheel/location/location.yaml`
  - 맵 파일: `nav2_wheel/maps/*.yaml|*.pgm`
  - URDF: `src/nav2_wheel/urdf/model.xacro`

- 권장 실행 순서
  1) 센서 드라이버
     ```
     # 개별 실행
     ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
     ros2 launch wheelchair_slam_bringup rs_camera.launch.py
     ros2 launch myahrs_ros2_driver myahrs_ros2_driver.launch.py
     ros2 launch imu_preprocess imu_preprocess.launch.py
     ```

     ```
     # 센서 일괄 실행 

     ros2 launch wheelchair_slam_bringup sensors_bringup.launch.py imu_use_rviz:=false
     ```

  2) tf발행, rviz, slam, localization(AMCL), EKF(엔코더+imu)가 TF발행 실행+ **2d pose estimate 해줘야 함**
   
     1.맵파일 생성시 slam=True(localizaition 작동 X)
     ```
     ros2 launch nav2_wheel wheelchair.launch.py slam:=True
     ```
     2.Localization
     ```
<<<<<<< HEAD
     ros2 launch nav2_wheel wheelchair.launch.py 
=======
     ros2 launch wheelchair_slam_bringup nav2_planner_only.launch.py  map:=/home/yoo/workspace/dolchair_ws/config/maps/gongA_map.yaml  params_file:=/home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/nav2_params_planner.yaml
     ```
  6) 음성 파이프라인(명령 처리)
     ```
     ros2 run llm_ros wake_word_detector
     ros2 run voice2text voice2text
     ros2 run llm_ros intent_router
     ros2 run llm_ros location_command
     ros2 run llm_ros llm_node
     ```
  7) 경로 생성 & 주행
     ```
     ros2 run path_planner make_pathnplan
     ros2 run path_planner path_follower
>>>>>>> origin/main
     ```
     
  3) 음성 파이프라인(명령 처리)
      ```
      ros2 run llm_ros wake_word_detector
      ros2 run voice2text voice2text
      ros2 run llm_ros intent_router
      ros2 run llm_ros location_command
      ros2 run llm_ros llm_node
      ```

- SLAM(맵 생성/저장)
  ```
  ros2 run nav2_map_server map_saver_cli -f ~/src/nav2_wheel/maps
  ```

- 토픽 테스트 예시
  ```
  ros2 topic pub /target_location std_msgs/msg/String "{data: '화장실'}"
  ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '앞으로 1m 가줘.'}"
  ros2 topic pub -1 /voice2text std_msgs/msg/String "{data: '시계방향으로 90도 돌아줘.'}"
  ```
- 의존성(주요)
  - ROS 2 + nav2_map_server, nav2_amcl, slam_toolbox, robot_localization
  - Python: `openai`, `openai-whisper`, `torch`, `sounddevice`, `scipy`, `numpy`, `PyYAML`, `tf2_ros`
  - 하드웨어: 2D LiDAR(A2M8 등), myAHRS+, 마이크, 아두이노(시리얼 `/dev/ttyACM0` 기본)

<<<<<<< HEAD
=======
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
│  │     ├─ wake_word_detector.py
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

## 향후 로드맵
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
>>>>>>> origin/main


test: ros2 topic pub -1 /target_location std_msgs/String
  "{data: '거실'}"
