- 권장 실행 순서
  1) 정적 TF/URDF (base_link+imu+2dLiDAR+D435i)
     ```
     ros2 run robot_state_publisher robot_state_publisher   --ros-args --params-file src/2D_LiDAR/wheelchair_slam_bringup/urdf/params.yaml
     ```
  2) 센서 드라이버
     ```
     # 개별 실행
     ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
     ros2 launch wheelchair_slam_bringup rs_camera.launch.py
     ros2 launch myahrs_ros2_driver myahrs_ros2_driver.launch.py
     ros2 launch imu_preprocess imu_preprocess.launch.py
     ```

     # 또는 일괄 실행(추천):
     ```
     ros2 launch wheelchair_slam_bringup sensors_bringup.launch.py imu_use_rviz:=false
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
     rviz2 dolchair.rviz
     ```

     - Planner Only 모드(기존 Planneronly + Pure pursuit):
       ```
       ros2 launch wheelchair_slam_bringup nav2_planner_only.launch.py  map:=/home/yoo/workspace/dolchair_ws/config/maps/GongHall_map.yaml  params_file:=/home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/nav2_params_planner.yaml
       ```
     - RViz2에서 2D Pose Estimate 수행하기
  
  6) 음성 파이프라인(명령 처리)
     ```
     ros2 run voice2text voice2text
     ros2 run llm_ros filter_input_text
     ros2 run llm_ros intent_router
     ros2 run llm_ros location_command
     ros2 run llm_ros llm_node
     ```
  7) 경로 생성 & 주행
     - Planner Only 파이프라인(기존 직선경로+커스텀 추종):
       ```
       ros2 run path_planner nav2_pathnplan
       ros2 run path_planner path_follower
       ```
     - 로컬 회피 플래너 사용(코리도 점유 기반, Pure Pursuit는 로컬 경로 추종):
       ```
       # 전역 경로는 Nav2가 /plan으로 발행
       ros2 run path_planner local_avoid_planner \
         --ros-args -p plan_topic:=/plan -p out_topic:=/local_plan -p scan_topic:=/scan -p points_topic:=/depth_camera/depth/color/points

       # 추종기는 /local_plan을 구독하도록 설정
       ros2 run path_planner path_follower --ros-args -p plan_topic:=/local_plan
       ```
       - RViz 표시: `Path(/plan)`, `Path(/local_plan)`, `MarkerArray(/local_plan_marker)` 활성화
       - 주요 파라미터(초기값): `horizon_length=8.0`, `publish_length=5.0`, `corridor_half_width=0.5`, `min_clear_margin=0.45`, `occupancy_threshold=12`, `curvature_max=0.6`, `fusion_window_sec=0.5`

     - 세 노드를 한 번에 실행(추천 런치):
       ```
       ros2 launch path_planner local_planning_pipeline.launch.py \
         plan_topic:=/plan out_topic:=/local_plan scan_topic:=/scan \
         points_topic:=/depth_camera/depth/color/points global_frame:=map base_frame:=base_link
       ```
  
- 토픽 테스트 예시
  ```
  ros2 topic pub -1 /target_location std_msgs/msg/String "{data: '화장실'}"
  ```
