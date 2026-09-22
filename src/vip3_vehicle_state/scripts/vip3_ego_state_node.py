#!/usr/bin/env python3
"""/Ego_topic (26.R1 EgoVehicleStatus) -> /vehicle/state, /vehicle/odom, 그리고 선택적 TF.

계산은 전부 `vip3_vehicle_state` 라이브러리(rospy 비의존)에 있고 여기는 배선만 한다.

TF 주의: MORAI 가 `/tf` 로 map->base_link 를 50 Hz 로 이미 쏜다
(VIP3_network_v1.json 의 TF2Publisher). 그래서 `~publish_map_tf` 기본값은 false 다.
켜면 같은 변환을 두 곳에서 쏘게 되어 RViz 에서 떨림이 생긴다 — 노드가 경고한다.
"""

from __future__ import annotations

import time

import rospy
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry

from vip3_msgs.msg import VehicleState
from vip3_vehicle_state.ego_state import convert_ego_message, yaw_to_quaternion
from vip3_vehicle_state.freshness import age_seconds

try:
    import tf2_ros
except ImportError:  # TF 를 안 쓰면 없어도 된다
    tf2_ros = None


class EgoStateNode:
    def __init__(self) -> None:
        self.frame_id = str(rospy.get_param("~frame_id", "map"))
        self.child_frame_id = str(rospy.get_param("~child_frame_id", "base_link"))
        self.ego_topic = str(rospy.get_param("~ego_topic", "/Ego_topic"))
        self.velocity_is_body_frame = bool(
            rospy.get_param("~velocity_is_body_frame", True)
        )
        self.signed_speed = bool(rospy.get_param("~signed_speed", True))
        # 'source'  = header.stamp 를 쓰고 0 이면 버린다 (엄격).
        # 'receive' = stamp 가 0 이어도 수신 시각으로 대체한다. MORAI rosbridge 가
        #             stamp 를 안 채우는 것이 확인되면 이쪽으로 바꾼다.
        self.stamp_mode = str(rospy.get_param("~stamp_mode", "source"))
        if self.stamp_mode not in ("source", "receive"):
            raise ValueError("stamp_mode must be 'source' or 'receive'")
        self.publish_map_tf = bool(rospy.get_param("~publish_map_tf", False))
        self.stale_timeout = float(rospy.get_param("~stale_timeout_sec", 0.5))

        self.state_publisher = rospy.Publisher(
            "/vehicle/state", VehicleState, queue_size=1
        )
        self.odom_publisher = rospy.Publisher(
            "/vehicle/odom", Odometry, queue_size=1
        )
        self.broadcaster = None
        if self.publish_map_tf:
            if tf2_ros is None:
                raise RuntimeError("publish_map_tf=true 인데 tf2_ros 가 없다")
            self.broadcaster = tf2_ros.TransformBroadcaster()
            rospy.logwarn(
                "publish_map_tf=true — MORAI 도 /tf 로 %s->%s 를 쏜다. "
                "둘 다 켜면 RViz 에서 떨린다. MORAI 쪽 TF2Publisher 를 끄거나 "
                "이 파라미터를 false 로 둘 것.",
                self.frame_id,
                self.child_frame_id,
            )

        self.received_at = 0.0
        self.accepted = 0
        self.rejected = 0
        self.last_reason = ""
        self.velocity_branch_logged = False

        self.subscriber = rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self._callback, queue_size=1
        )
        rospy.Timer(rospy.Duration(2.0), self._log_status)
        rospy.loginfo(
            "ego state ready topic=%s frame=%s->%s stamp_mode=%s tf=%s",
            self.ego_topic,
            self.frame_id,
            self.child_frame_id,
            self.stamp_mode,
            self.publish_map_tf,
        )

    def _callback(self, message: EgoVehicleStatus) -> None:
        self.received_at = time.monotonic()
        now = rospy.Time.now()
        source_stamp = message.header.stamp
        stamp = source_stamp if source_stamp.to_sec() > 0.0 else now
        if self.stamp_mode == "receive":
            stamp = now

        state = convert_ego_message(
            message,
            stamp_s=source_stamp.to_sec(),
            velocity_is_body_frame=self.velocity_is_body_frame,
            signed_speed=self.signed_speed,
        )
        usable = (
            state.valid if self.stamp_mode == "source" else state.valid_ignoring_stamp()
        )
        if not usable:
            self.rejected += 1
            self.last_reason = state.reason()
            rospy.logwarn_throttle(2.0, "ego state rejected: %s", self.last_reason)
            return
        self.accepted += 1
        if not self.velocity_branch_logged:
            rospy.loginfo(
                "velocity_is_body_frame=%s -> vx_body=%.3f vy_body=%.3f "
                "(직진 중 vy_body 가 0 에서 멀면 이 설정이 틀린 것이다)",
                self.velocity_is_body_frame,
                state.vx_body,
                state.vy_body,
            )
            self.velocity_branch_logged = True

        vehicle_state = VehicleState()
        vehicle_state.header.stamp = stamp
        vehicle_state.header.frame_id = self.frame_id
        vehicle_state.source = VehicleState.SOURCE_EGO_STATUS
        vehicle_state.valid = True
        vehicle_state.x = state.x
        vehicle_state.y = state.y
        vehicle_state.yaw = state.yaw
        vehicle_state.speed = state.speed
        vehicle_state.acceleration = state.acceleration
        vehicle_state.yaw_rate = state.yaw_rate
        self.state_publisher.publish(vehicle_state)

        quaternion = yaw_to_quaternion(state.yaw)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position.x = state.x
        odometry.pose.pose.position.y = state.y
        odometry.pose.pose.position.z = state.z
        (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        ) = quaternion
        # nav_msgs/Odometry 의 twist 는 child_frame_id 기준, 즉 차량 기준이다.
        odometry.twist.twist.linear.x = state.vx_body
        odometry.twist.twist.linear.y = state.vy_body
        odometry.twist.twist.angular.z = state.yaw_rate
        # MORAI 는 GT 를 주므로 공분산은 의미가 없다. 0 으로 두고 소비자가 읽지
        # 않도록 문서에 명시한다.
        self.odom_publisher.publish(odometry)

        if self.broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = state.x
            transform.transform.translation.y = state.y
            transform.transform.translation.z = state.z
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = quaternion
            self.broadcaster.sendTransform(transform)

    def _log_status(self, _event) -> None:
        age = age_seconds(self.received_at, time.monotonic())
        if age > self.stale_timeout:
            rospy.logwarn_throttle(
                5.0,
                "%s 가 %.1fs 동안 조용하다 (accepted=%d rejected=%d last=%s)",
                self.ego_topic,
                age,
                self.accepted,
                self.rejected,
                self.last_reason or "-",
            )


def main() -> None:
    rospy.init_node("vip3_ego_state")
    EgoStateNode()
    rospy.spin()


if __name__ == "__main__":
    main()
