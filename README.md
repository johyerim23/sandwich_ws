# sandwich_ws
<Br>

26-1 지능형로보틱스 프로젝트

<Br>

## 주제
대화형 HRI를 활용한 맞춤형 샌드위치 조립 로봇 시스템

<Br>

## 목표
본 프로젝트의 목표는 서브웨이 프랜차이즈 아르바이트생의 주문 및 제조 프로세스를 모사하여, 사용자와 채팅 인터페이스로 소통하며 취향에 맞는 샌드위치를 자율 제작하는 로봇 시스템을 구현하는 것이다. 로봇이 빵 종류, 채소, 추가 재료, 소스 선택까지 이어지는 샌드위치 커스텀 주문 과정을 단계별로 질문하고, 사용자의 답변을 정확히 인식해 제조 공정을 동적으로 결정하여 샌드위치를 완성한다. 이 과정에서 사용자-로봇의 소통은 대화형 창에서 텍스트로 입력받고 출력하며, 사용자의 입력이 명확하지 않거나 재료가 소진된 경우 로봇은 재확인 질문을 통해 주문을 확정한다. 샌드위치를 완성한 후에는 YOLO 기반 품질 검증으로 주문 내역과 실제 제작 결과물을 대조하고, 누락 재료가 있다면 감지해 해당 재료의 pick-and-place 동작을 재실행하는 자율 복구 기능까지 포함한다.

<Br>

## 개발 환경
- OS: Ubuntu 22.04 LTS (WSL2)
- ROS2: Humble
- 시뮬레이터: Gazebo Fortress
- Python: 3.10
- 워크스페이스: ~/sandwich_ws
- 패키지: ~/sandwich_ws/src/sandwich_robot

<Br>

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

<Br>

## 참고자료
- 지능형로보틱스 강의자료: https://github.com/Ewha-AIRLab/intelligent-robotics-ros2
- Ultralytics YOLO Detection:https://docs.ultralytics.com/ko/
- YOLO-ROS: https://docs.ultralytics.com/guides/ros-quickstart/#key-features-of-ros
- MoveIt2 로봇 팔 제어:https://moveit.picknik.ai/main/index.html
- Pick-and-Place:https://automaticaddison.com/pick-and-place-with-the-moveit-task-constructor-for-ros-2/
- Franka Panda: https://github.com/frankarobotics/franka_ros2
- PyQt5:https://www.riverbankcomputing.com/static/Docs/PyQt5/
- 서브웨이 샌드위치:https://www.subway.co.kr/menuList/sandwich
- PyQt5-ROS2: https://github.com/tasada038/pyqt_ros2_app
- LLM-ROS2: https://github.com/aniskoubaa/rosgpt

<Br>

## Commit 메시지 규칙
- feat: 새로운 기능 추가
- fix: 버그 수정
- docs: 문서 수정 (README, 주석 등)
- style: 코드 포맷팅, 세미콜론 누락 등 (코드 변경 없음)
- refactor: 코드 리팩토링
- test: 테스트 코드 추가/수정
- chore: 빌드 업무 수정, 패키지 매니저 설정 등
