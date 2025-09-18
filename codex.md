# 목표
src/llm_ros/llm_ros/location_command.py 코드에서 발행되는 /location_target을 받아 slam 저장된 맵 라벨링과 비교하여 해당 라벨링된 위치로 이동.

# 해야할 일:
1. 사전에 SLAM으로 map을 따놓기. 저장된 맵에 특정 부분 좌표에 라벨링을 해놓기. 
실행 명령어는 README.md에서 확인할 수 있음.

2. path_planning 노드를 작성하여 /location_target을 subscribe 받아서 일치되는 라벨 + 맵 상에서 현재 자기의 위치를 localization하여 일직선으로 연결.

3. 주행 및 조향 명령. 속도는 일정하게.