- 실행 명령어 정리

1. ros2 run voice2text voice2text
2. ros2 run llm_ros filter_input_text
3. ros2 run llm_ros location_command


- 정적 TF 발행 & 맵 생성 및 저장
  
1. ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/2D_LiDAR/wheelchair_slam_bringup/urdf/rplidar_myahrs.urdf)"
2. ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
3. ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
4. ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$(cat src/2D_LiDAR/wheelchair_slam_bringup/config/slam.yaml)"
