# VOICE_BASED_WHEELCHAIR

전신마비 장애인의 자율적인 이동을 지원하기 위한 음성 기반 전동휠체어 제어 시스템.

---

## 개요

- 센서: 2D LiDAR, IMU(myAHRS)
- 로컬라이제이션: LiDAR 오도메트리(rf2o) + EKF(robot_localization) + AMCL
- TF 트리: `map → odom → base_link → {lidar_frame, imu_link}`
- 음성 처리: `voice2text → llm_ros(filter_input_text → intent_router → llm_node/location_command)`
- 경로 계획/추종: `path_planner(make_pathnplan, path_follower)`

---

## 빠른 시작(권장 실행 순서)

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

- EKF를 함께 사용할 때는 rf2o의 TF 발행을 끄세요(`publish_tf:=false`).

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

6) 음성 파이프라인(명령어 처리)

```
ros2 run voice2text voice2text
ros2 run llm_ros filter_input_text
ros2 run llm_ros intent_router
ros2 run llm_ros location_command   # 주행 명령 처리
ros2 run llm_ros llm_node
```

7) 경로 생성 & 주행

```
ros2 run path_planner make_pathnplan
ros2 run path_planner path_follower
```

---

## SLAM(맵 생성/저장)

```
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$PWD/src/2D_LiDAR/wheelchair_slam_bringup/config/slam.yaml
ros2 run nav2_map_server map_saver_cli -f ./config/maps/dolbang_map
```

- 맵 생성만 할 때(EKF 없이): rf2o의 `publish_tf:=true`로 단독 오도메트리 TF 사용 가능
- EKF와 함께 운용할 때: rf2o `publish_tf:=false`, EKF가 `odom→base_link` TF를 단일 발행

---

## 참고(로컬라이제이션 구성)

- EKF 파라미터: `src/2D_LiDAR/wheelchair_slam_bringup/config/ekf_odom.yaml`
  - `two_d_mode: true`, `publish_tf: true`, `world_frame: odom`
  - 입력: `odom0:=/odom`, `imu0:=/imu/preprocessed`(또는 `/imu/data` 리매핑)
  - 사용 축: LiDAR의 `x, y, yaw, vx, vyaw`, IMU의 `yaw, yaw_rate`
- TF 중복 금지: `odom→base_link`는 EKF만 발행, `map→odom`은 AMCL 발행

검증 팁
- `ros2 run tf2_ros tf2_echo odom base_link`: EKF 출력 확인
- `ros2 run tf2_ros tf2_echo map odom`: AMCL 보정 확인
- `ros2 topic echo /imu/data` 또는 `/imu/preprocessed`: orientation/gyro 공분산 확인

---

## rqt graph

![rqt_graph](image-1.png)

---

## 변경 이력

### 2025-09-21: 명령어 처리 아키텍처 개선

- 문제: `/text_command`를 `LLMNode`와 `LocationCommandNode`가 동시에 구독해 중복 반응 가능성
- 해결: `IntentRouter` 추가로 의도 분리
  - `/text_command` 단독 구독 → 정규식으로 장소 명령 식별
  - 장소 명령 → `/text_to_location`, 일반 명령 → `/text_to_llm`로 라우팅
- 관련 변경
  - `LLMNode`/`LocationCommandNode`: 각자 전용 토픽 구독
  - `LLMNode`: 호출어(예: "도리야") 자동 제거 전처리 추가
- 요약: `filter_input_text → /text_command → intent_router → (/text_to_location → LocationCommandNode) | (/text_to_llm → LLMNode)`


ros2 topic pub /target_location std_msgs/msg/String "{data: '화장실'}"
