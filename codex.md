- 현재 진행 상황
1. 음성으로 특정 공간 라벨을 string으로 받을 수 있는 상황. /target_location 토픽
2. 2d LiDAR로 맵을 저장 가능함.
---------------------------------------------------------------
- 맵 저장 & 로드 & 로컬리제이션.... 맵 상에 현재 위치 파악 가능.
ros2 run nav2_map_server map_saver_cli -f ./config/maps/dolbang_map

ros2 run nav2_map_server map_server   --ros-args -p yaml_filename:=/home/yoo/workspace/dolchair_ws/config/maps/dolbang_map.yaml

ros2 lifecycle set /map_server configure & activate

ros2 run nav2_amcl amcl   --ros-args --params-file /home/yoo/workspace/dolchair_ws/src/2D_LiDAR/wheelchair_slam_bringup/config/amcl.yaml

ros2 lifecycle set /amcl configure & activate
--------------------------------------------------------------

현재 TF
: map -> odom -> base_link -> lidar_frame & imu

위 명령어들을 통해 맵 상에서 로봇이 움직임에 따라 이동하는 것을 확인하였음. 하지만 imu를 사용하지 않은 odometry로 정밀한 추적이 불가능한 상태에 있음. 추후에 imu를 사용하여 정확한 localization을 할 것임.

### 나의 목표
: 특정 라벨을 음성으로 받아 해당 위치 토픽이 들어오면 yaml파일을 읽고 해당 위치로 이동하는 것. 

상세 설명
: 자신의 위치에서 라벨의 위치로 Path를 생성. 생성된 Path와 자신의 heading 방향간의 각도를 구함. 구한 각도를 이용해 왼쪽 및 오른쪽 회전 토픽을 publish.(-1 , +1)형태. 오차가 0정도의 오차범위내에 들어왔다면 주행 토픽 /cmd_vel publish. 도착시 정지.


