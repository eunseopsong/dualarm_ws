# Dual-Arm & Hand Force Control System (v3)

Isaac Sim 환경과 연동하여 6축 양팔 및 20축 핸드의 상태를 500Hz 주기로 모니터링하고 제어하기 위한 ROS 2 패키지입니다.

## 🛠 주요 업데이트 (v3)

### 1. 정밀 모니터링 인터페이스
* **데이터 시각화:** Current(Cyan) 값과 Target(Yellow) 값을 동일 선상에 배치하여 제어 성능 실시간 모니터링 가능.
* **Arm (6-DOF):** 6축 관절 및 3축 Force(Fx, Fy, Fz) 출력 (모멘트 제외).
* **Hand (20-DOF):** 손가락당 4개 관절(Total 20) 및 5개 손가락 끝단 3축 Force 출력.

### 2. 구조적 개선
* **Layout Alignment:** 괄호를 제거하고 수치 데이터를 수직 열(Column)로 정렬하여 가독성 극대화.
* **Eigen 기반 설계:** 모든 내부 데이터 연산은 Eigen 라이브러리를 통해 최적화된 연산 수행.
* **데이터 일치:** `ros2 topic echo`의 실제 Dimension과 내부 데이터 구조를 1:1 매핑.

## 📊 시스템 구성


| 구분 | Arm (L/R) | Hand (L/R) |
| :--- | :--- | :--- |
| **관절수** | 6-DOF | 20-DOF (4 x 5 Fingers) |
| **힘 센서** | 3-axis Force | 15-axis (3 x 5 Fingers) |
| **제어 주기** | 500Hz | 500Hz |
| **출력 색상** | Cyan (현재값) | Yellow (목표값) |

## 🚀 실행 및 빌드

```bash
# 워크스페이스 빌드
cd ~/dualarm_ws
colcon build --symlink-install

# 노드 실행
ros2 run dualarm_ctrl dualarm_ctrl_node
