- 권장 실행 순서
  1) 정적 TF/URDF (base_link+imu+2dLiDAR+D435i)
     ```
     ros2 run robot_state_publisher robot_state_publisher   --ros-args --params-file src/2D_LiDAR/wheelchair_slam_bringup/urdf/params.yaml
     ```
  2) 센서 드라이버
     ```
     ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
     ros2 launch wheelchair_slam_bringup rs_camera.launch.py
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
     - Planner Only 모드(기존 Planneronly + Pure pursuit):
       ```
       ros2 launch wheelchair_slam_bringup nav2_planner_only.launch.py  map:=/home/yoo/workspace/dolchair_ws/config/maps/gongA_map.yaml  params_file:=/home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/nav2_params_planner.yaml
       ```
     - Full Nav2 모드(Planner+Controller+BT, RPP/복구 포함):
       ```
       ros2 launch wheelchair_slam_bringup nav2_full.launch.py \
         map:=/home/yoo/workspace/dolchair_ws/config/maps/gongA_map.yaml \
         params_file:=/home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/nav2_params_full.yaml
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
     - Full Nav2 파이프라인(BT로 경로계산+RPP 추종+복구):
       ```
       ros2 run path_planner nav2_nav_to_pose
       ```
       - `/target_location` 수신 시 BT Navigator에 NavigateToPose 목표를 전송하여 자동 복구/재계획 포함 주행 수행
  
- 토픽 테스트 예시
  ```
  ros2 topic pub -1 /target_location std_msgs/msg/String "{data: '화장실'}"
  ```
