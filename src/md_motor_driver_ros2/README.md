# MD_controller (For Dual Channel motor driver)
This is a package that makes MDROBOT's motor driver available in ROS2(humble). [ https://www.mdrobot.co.kr ]

## 🔧 소개

본 레포지토리는 **MD 모터 드라이버**의 듀얼채널 모터를 제어하기 위한 **ROS 2 Humble** 기반 패키지입니다.
---

## ✅ 지원되는 모터 드라이버
- 본코드는 MDrobot의 듀얼채널 모터드라이버를 지원합니다.
- `md200t`
- `md400t`

---

## 🧪 테스트 환경 (실제 하드웨어 검증 완료 2025-07/30)

- `md200t` + `mdh80` with RS485(waveshare)
- `md400t` + `mdh250` with RS485(waveshare)

---

## 🧭 기능 설명

본 패키지는 ROS 2의 `/cmd_vel` 메시지를 받아 **좌우 바퀴 RPM 제어**로 변환하여 듀얼채널 MD 모터 드라이버를 제어합니다.

- `/cmd_vel`의 **linear.x (전진/후진 속도)** 와 **angular.z (회전 속도)** 를 수신
- 내부적으로 이를 좌/우 바퀴 속도(RPM)로 변환
- 변환된 RPM은 **MD 모터 드라이버**로 전송되어 각 바퀴를 제어

### 💻 동작 흐름 요약

/cmd_vel (linear.x, angular.z) <br>
↓ <br>
cmdVelToRpm() 변환 <br>
↓ <br>
left, right 모터 RPM 계산 <br>
↓ <br>
MD 모터 드라이버로 전송 <br>
↓ <br>
좌측/우측 바퀴 개별 회전 <br>

---

## 🔧 런치 파라미터 변경(port, buadrate ...)
in launch/md_controller.launch.py

Change parameters suitable for motor driver and motor.

<img width="411" height="355" alt="Screenshot from 2025-07-30 15-39-21" src="https://github.com/user-attachments/assets/16a82fda-5027-42b9-b966-627484fb38d7" />

---

## 📦 Dependencies
There is no official release of the serial package available for ROS2, so you need to install an unofficial version that is compatible with ROS2.

```
To install it, follow the steps below.

~$ git clone https://github.com/RoverRobotics-forks/serial-ros2.git
~$ cd serial-ros2
~$ mkdir build
~$ cd build
~$ cmake ..
~$ make
~$ cd ..
~$ colcon build --packages-select serial
~$ cd build
~$ sudo make install
```

## 🚀 실행방법

1. 모터 드라이버 런치 실행
 
```bash
#run motor controller
        
~$ ros2 launch md_controller md_controller.launch.py
```

2.(option) 모터드라이버 테스트
> 단순히 RPM을 인가하여 양쪽 모터가 도는지 확인하고 싶다면 아래의 노드 이용.

```bash
#control motor

~$ ros2 run md_teleop md_teleop_key_node
```

3. /cmd_vel 발행 및 모터 전/후진, 회전 테스트
> WASD를 이용하여 cmd_vel을 발행, 인가된 linear, angular에 따라 양쪽 모터가 정방향, 역방향 회전을 잘 하는지 확인

```bash
#turtlebot3 package needs to be pre-installed

ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 🤝 Contributors

| 이름            | GitHub ID | 소속                                         | 기여 내용                                 |
|-----------------|-----------|----------------------------------------------|-------------------------------------------|
| JungHo Cheon    | [c-jho](https://github.com/c-jho) | Korea Institute of Science and Technology | 원본 코드 작성 및 ROS2 포팅 기반 제공      |
| Seokgwon Lee    | [Lee-seokgwon](https://github.com/Lee-seokgwon)         | Kyungpook National Univ. (SEE)              | 듀얼채널 모터 드라이버 로직 추가 및 기능 확장 |
