"""morai_msgs/EgoVehicleStatus (26.R1) -> 팀 내부 상태로 정규화. 순수 함수.

ASMC `vehicle_state_adapter.cpp` 의 `ConvertEgoVehicleStatus` 를 옮기고
26.R1 변경분을 반영했다.

단위 주의 (26.R1):
  heading            [deg]    -> yaw [rad]
  angular_velocity   [deg/s]  -> yaw_rate [rad/s]   (beta_drive 에는 없던 필드)
  velocity           [m/s]
  acceleration       [m/s^2]
  position           [m] ENU, MORAI 로컬 map 좌표
CtrlCmd 쪽 steer 는 [rad] 이다. 절대 섞지 말 것.
"""

from __future__ import annotations

import math

DEG_TO_RAD = math.pi / 180.0


class EgoState(object):
    """/vehicle/state, /vehicle/odom, map->base_link TF 가 공유하는 중간 표현."""

    __slots__ = (
        "stamp", "stamp_valid", "position_zero", "finite",
        "x", "y", "z", "yaw",
        "vx_body", "vy_body", "speed",
        "acceleration", "yaw_rate",
    )

    def __init__(self, **kwargs):
        for name in self.__slots__:
            setattr(self, name, kwargs[name])

    @property
    def valid(self):
        """source stamp 까지 요구하는 엄격한 유효성."""
        return bool(self.finite and not self.position_zero and self.stamp_valid)

    def valid_ignoring_stamp(self):
        """stamp_mode='receive' 용. MORAI 가 stamp 를 0 으로 보내도 값은 쓴다."""
        return bool(self.finite and not self.position_zero)

    def reason(self):
        """무효 사유 한 줄. 로그용."""
        if not self.finite:
            return "non-finite field"
        if self.position_zero:
            return "position == (0,0,0)"
        if not self.stamp_valid:
            return "header.stamp == 0"
        return ""


def _finite(*values):
    for value in values:
        if not math.isfinite(value):
            return False
    return True


def yaw_to_quaternion(yaw):
    """yaw[rad] (roll=pitch=0) -> (x, y, z, w). tf_conversions 없이 쓰려고 둔다."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def convert_ego(stamp_s, position, velocity, acceleration_x,
                angular_velocity_z_dps, heading_deg,
                velocity_is_body_frame=True, signed_speed=True):
    """EgoVehicleStatus 필드 -> EgoState.

    velocity_is_body_frame:
        True  = MORAI 가 차량 기준(전방 +x) 속도를 준다고 본다. **기본값.**
                ASMC UDP 경로가 `ego_velocity_in_ms=true` 로 그렇게 다뤘다.
        False = map(ENU) 기준 속도로 보고 -yaw 만큼 회전시켜 body 로 바꾼다.
        rosbridge 에서 어느 쪽인지는 실기 확인 필요. 정지 상태에서 차를 비스듬히
        세워두고 전진시켰을 때 vy_body 가 0 근처면 True 가 맞다.
    signed_speed:
        True  = speed 에 전후 부호가 있는 종방향 속도(vx_body)를 넣는다. **기본값.**
                후진 주차에는 부호가 필요하다. ASMC 는 hypot 이라 항상 >= 0 이었다.
        False = ASMC 와 동일한 hypot(vx, vy).
    """
    px, py, pz = position
    vx, vy = velocity[0], velocity[1]

    yaw = heading_deg * DEG_TO_RAD
    if velocity_is_body_frame:
        vx_body, vy_body = vx, vy
    else:
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx_body = vx * cos_yaw + vy * sin_yaw
        vy_body = -vx * sin_yaw + vy * cos_yaw

    speed = vx_body if signed_speed else math.hypot(vx, vy)
    # 26.R1 에서 새로 생긴 필드. beta_drive 에는 없어서 ASMC 는 0 을 넣었다.
    yaw_rate = angular_velocity_z_dps * DEG_TO_RAD

    return EgoState(
        stamp=stamp_s,
        stamp_valid=bool(stamp_s and math.isfinite(stamp_s) and stamp_s > 0.0),
        # MORAI 가 아직 ego 를 스폰하기 전이거나 시나리오 리로드 중이면
        # position 이 정확히 (0,0,0) 인 패킷이 온다. 그걸로 TF 를 쏘면
        # 차가 원점으로 순간이동한다.
        position_zero=(px == 0.0 and py == 0.0 and pz == 0.0),
        finite=_finite(px, py, pz, yaw, vx_body, vy_body, speed,
                       acceleration_x, yaw_rate),
        x=px, y=py, z=pz, yaw=yaw,
        vx_body=vx_body, vy_body=vy_body, speed=speed,
        acceleration=acceleration_x, yaw_rate=yaw_rate,
    )


def convert_ego_message(ego, stamp_s, velocity_is_body_frame=True,
                        signed_speed=True):
    """morai_msgs/EgoVehicleStatus 객체에서 필드를 뽑아 convert_ego 호출.

    rospy 를 import 하지 않으려고 stamp 는 노드가 `msg.header.stamp.to_sec()`
    으로 바꿔서 넘긴다.
    """
    return convert_ego(
        stamp_s=stamp_s,
        position=(ego.position.x, ego.position.y, ego.position.z),
        velocity=(ego.velocity.x, ego.velocity.y),
        acceleration_x=ego.acceleration.x,
        angular_velocity_z_dps=ego.angular_velocity.z,
        heading_deg=ego.heading,
        velocity_is_body_frame=velocity_is_body_frame,
        signed_speed=signed_speed,
    )
