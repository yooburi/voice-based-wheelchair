# VOICE_BASED_WHEELCHAIR
---
### 목적
전신마비 장애인의 자율적인 이동을 지원하기 위한 음성 기반 전동휠체어 제어 시스템을 개발 

---
### 실행 명령어 정리 

**Voice to text**
1. ros2 run voice2text voice2text
2. ros2 run llm_ros filter_input_text
3. ros2 run llm_ros intent_router
4. ros2 run llm_ros location_command #주행 명령
5. ros2 run llm_ros llm_node

**정적 TF 발행 & SLAM 맵 생성,저장**
1. ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/2D_LiDAR/wheelchair_slam_bringup/urdf/rplidar_myahrs.urdf)"
2. ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
3. ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
4. ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$PWD/src/2D_LiDAR/wheelchair_slam_bringup/config/slam.yaml
5. ros2 run nav2_map_server map_saver_cli -f ./config/maps/dolbang_map

**Map load & Localization**
1. ros2 run nav2_map_server map_server   --ros-args -p yaml_filename:=/home/yoo/workspace/dolchair_ws/config/maps/dolbang_map.yaml
2. ros2 lifecycle set /map_server configure & activate
3. ros2 run nav2_amcl amcl   --ros-args --params-file /home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/amcl.yaml
4. ros2 lifecycle set /amcl configure & activate

**/target_location 라벨 로드 & 경로 생성 & 주행 명령**
1. ros2 run path_planner make_pathnplan
2. ros2 run path_planner path_follower
---
### rqt graph
![alt text](image-1.png)

---

### 2025-09-21: 명령어 처리 아키텍처 개선

- **문제점**: 하나의 토픽(`/text_command`)을 `LLMNode`와 `LocationCommandNode`가 동시에 구독하여, 특정 명령어에 대해 중복으로 반응하고 충돌할 가능성이 있었습니다.
- **해결책**: 명령어의 의도를 파악하여 각 노드에 적절히 분배해주는 `IntentRouter` 노드를 새로 도입했습니다.
- **주요 변경 내용**:
  - `IntentRouter` 노드 추가:
    - `/text_command`를 단독으로 구독합니다.
    - 정규식을 통해 '장소 이동/저장' 관련 명령어인지 판단합니다.
    - 장소 명령은 `/text_to_location` 토픽으로, 그 외 일반 명령은 `/text_to_llm` 토픽으로 분리하여 발행합니다.
  - `LLMNode` 및 `LocationCommandNode` 수정:
    - 각각 `/text_to_llm`, `/text_to_location` 토픽을 구독하도록 변경하여 자신의 역할에 맞는 명령어만 수신합니다.
  - `LLMNode` 개선:
    - "도리야" 같은 호출 단어를 명령어에서 자동으로 제거하는 전처리 로직을 추가하여 LLM의 인식률을 높였습니다.
- **요약**: Filternode -> '/text_command' -> IntentRouter -> '/text_to_location` ->LocationCommandNode
                                                          -> `/text_to_llm`      ->LLMNode