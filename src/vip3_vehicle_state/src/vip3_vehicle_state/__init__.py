"""vip3_vehicle_state — MORAI 26.R1 차량 상태 정규화.

이 패키지의 `src/vip3_vehicle_state/*.py` 는 **rospy 를 import 하지 않는다**.
순수 계산만 두고 ROS 배선은 `scripts/*.py` 노드가 담당한다. 그래야 ROS 없이
`python3 -m unittest` 로 단위 테스트를 돌릴 수 있다. 이 규칙을 깨지 말 것.
"""
