# 1단계
<Br>

## 작업

지능형로보틱스 강의 자료 `07_moveit2` 패키지를 복사하여
패키지명을 `sandwich_robot`으로 변경한 뒤,
샌드위치 조립 로봇 시스템의 기반으로 활용함.

Franka Panda 로봇팔 + MoveIt2 + Gazebo Fortress 환경에서
사용자 주문을 받아 재료를 Pick-and-Place하는 시스템을 구현 예정

<Br>

---
## 개발 명령어

### 1) workspace sourcing<Br>
`source ~/sandwich_ws/install/setup.bash`

### 2) python script 수정 시<Br>
`colcon build --symlink-install`

### 3) 실행: Gazebo와 RViz2가 실행되며 Franka Panda 로봇이 시뮬레이션 환경에 로드됨<Br>
`ros2 launch sandwich_robot sim.launch.py`

<Br>

---
## 패키지 구조

```
sandwich_robot/
├── config/
│   ├── initial_positions.yaml         # 로봇 초기 관절 위치
│   ├── panda_moveit_config_demo.rviz  # RViz 설정
│   ├── ros2_controllers_demo.yaml     # 데모용 컨트롤러 설정
│   └── ros2_controllers_sim.yaml      # 시뮬레이션용 컨트롤러
설정
├── launch/
│   ├── sim.launch.py                  # Gazebo + MoveIt2 + RViz2 통합 실행
│   ├── pick_and_place.launch.py       # Pick-and-Place 실행
│   └── demo.launch.py                 # 데모 실행
├── scripts/
│   ├── pick_and_place.py              # Pick-and-Place 핵심 로직
│   ├── plan_and_execute.py            # 경로 계획 및 실행
│   ├── print_ee_pose.py               # 엔드이펙터 포즈 출력
│   └── scene_publisher.py             # MoveIt2 플래닝 씬 관리
├── urdf/
│   ├── panda.urdf                     # Franka Panda 로봇 모델
│   ├── panda_fake.urdf.xacro          # 데모용 모델
│   └── panda_gazebo.urdf.xacro        # Gazebo 시뮬레이션용 모델
└── worlds/
    ├── pick_and_place.sdf             # Pick-and-Place 실습용 world
    └── table.sdf                      # 테이블 world
```

<Br>

---
## 오류 발생 및 해결

### 1) package 'ros_gz_bridge' not found<Br>
=> `sudo apt-get update && sudo apt install ros-humble-ros-gz`

### 2) RViz에서 moveit_rviz_plugin X 표시<Br>
=> MoveIt RViz 플러그인 누락. 아래 설치 후 재실행: `sudo apt install ros-humble-moveit-ros-visualization`

### 3) gz_ros2_control-system 플러그인 로드 실패<Br>
=> `sudo apt install ros-humble-ign-ros2-control`

<Br>

---
## 개발 계획

> **1단계** 환경 세팅, Gazebo + Franka Panda 동작 확인: 완료✅
>
> **2단계** 샌드위치 재료 트레이 World SDF 작성
>
> **3단계** Pick-and-Place 동작 구현
>
> **4단계** PyQt5 노드 구현
>
> **5단계** LLM Parser 노드 구현
>
> **6단계** YOLO QC 노드 구현
>
> **7단계** 전체 노드 통합 및 점검
>
> **8단계** 최종 테스트 및 프로젝트 보완
