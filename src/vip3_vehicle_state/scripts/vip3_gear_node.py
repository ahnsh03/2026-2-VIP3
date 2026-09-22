#!/usr/bin/env python3
"""기어와 제어 모드의 **유일한 소유자**.

자율주차는 전진/후진을 반복하므로 기어가 핵심이다. 그런데 MORAI 에서 기어는 토픽이
아니라 서비스(`/Service_MoraiEventCmd`)로 바꾼다. 두 노드가 각자 서비스를 호출하면
후진 중에 D 로 튀는 사고가 난다. 그래서 기어 writer 를 이 노드 하나로 못박는다.

이 노드가 하는 일
  1. 기동 시 ctrl_mode = 3 (External) 로 바꾼다.
     **이걸 안 하면 MORAI 가 키보드 제어를 유지하고 /ctrl_cmd 를 조용히 무시한다.**
  2. `~set_gear` 서비스(vip3_msgs/SetGear)로 기어 변경을 받는다.
  3. 현재 기어를 `/vehicle/gear` (std_msgs/Int32, latched) 로 알린다.
  4. 주기적으로 option=0 호출로 실제 기어를 폴링해 시뮬과 어긋나면 고친다.

수동 확인 (노드 없이):
    rosservice call /Service_MoraiEventCmd "{request: {option: 1, ctrl_mode: 3}}"
    rosservice call /Service_MoraiEventCmd "{request: {option: 16, gear: 2}}"   # 후진
"""

from __future__ import annotations

import rospy
from morai_msgs.msg import EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from std_msgs.msg import Int32

from vip3_msgs.srv import SetGear, SetGearResponse

# EventInfo.option 비트마스크
OPTION_NONE = 0
OPTION_CTRL_MODE = 0x0001
OPTION_GEAR = 0x0010

CTRL_MODE_EXTERNAL = 3

GEAR_NAMES = {1: "P", 2: "R", 3: "N", 4: "D"}
VALID_GEARS = frozenset(GEAR_NAMES)


class GearNode:
    def __init__(self) -> None:
        self.service_name = str(
            rospy.get_param("~event_service", "/Service_MoraiEventCmd")
        )
        self.poll_period = float(rospy.get_param("~poll_period_sec", 1.0))
        self.set_external_on_start = bool(
            rospy.get_param("~set_external_on_start", True)
        )
        self.initial_gear = int(rospy.get_param("~initial_gear", 0))
        wait_timeout = float(rospy.get_param("~service_wait_sec", 30.0))

        rospy.loginfo("waiting for %s (<= %.0fs)", self.service_name, wait_timeout)
        try:
            rospy.wait_for_service(self.service_name, timeout=wait_timeout)
        except rospy.ROSException:
            rospy.logerr(
                "%s 가 없다. MORAI Network 설정에서 MoraiEventCmd 서비스가 켜져 "
                "있는지 확인한다 (docs/simulator.md §3). 기어를 바꿀 수 없으므로 "
                "자율주차도 불가능하다.",
                self.service_name,
            )
            raise
        self.proxy = rospy.ServiceProxy(self.service_name, MoraiEventCmdSrv)

        self.gear = 0
        self.ctrl_mode = 0
        self.failures = 0
        self.gear_publisher = rospy.Publisher(
            "/vehicle/gear", Int32, queue_size=1, latch=True
        )
        self.set_gear_service = rospy.Service("~set_gear", SetGear, self._handle_set_gear)

        if self.set_external_on_start:
            self._call(OPTION_CTRL_MODE, ctrl_mode=CTRL_MODE_EXTERNAL)
        if self.initial_gear in VALID_GEARS:
            self._call(OPTION_GEAR, gear=self.initial_gear)
        else:
            self._call(OPTION_NONE)

        if self.poll_period > 0.0:
            self.timer = rospy.Timer(
                rospy.Duration(self.poll_period), self._poll
            )
        rospy.loginfo(
            "gear node ready service=%s ctrl_mode=%s gear=%s",
            self.service_name,
            self.ctrl_mode,
            GEAR_NAMES.get(self.gear, self.gear),
        )

    def _call(self, option, gear=-1, ctrl_mode=0):
        """EventInfo 를 보내고 응답으로 현재 상태를 갱신한다.

        option=0 이면 아무것도 바꾸지 않고 현재 상태만 읽는 폴링이 된다.
        """
        request = EventInfo()
        request.option = int(option)
        request.ctrl_mode = int(ctrl_mode)
        request.gear = int(gear)
        request.set_pause = False
        try:
            response = self.proxy(request)
        except rospy.ServiceException as exc:
            self.failures += 1
            rospy.logwarn_throttle(2.0, "%s 호출 실패: %s", self.service_name, exc)
            return None
        result = getattr(response, "response", response)
        new_gear = int(getattr(result, "gear", self.gear))
        new_mode = int(getattr(result, "ctrl_mode", self.ctrl_mode))
        if new_gear != self.gear:
            rospy.loginfo(
                "gear %s -> %s",
                GEAR_NAMES.get(self.gear, self.gear),
                GEAR_NAMES.get(new_gear, new_gear),
            )
            self.gear = new_gear
            self.gear_publisher.publish(Int32(data=self.gear))
        elif self.gear_publisher.get_num_connections() >= 0 and not hasattr(
            self, "_published_once"
        ):
            self.gear_publisher.publish(Int32(data=self.gear))
            self._published_once = True
        if new_mode != self.ctrl_mode:
            rospy.loginfo("ctrl_mode %s -> %s", self.ctrl_mode, new_mode)
            self.ctrl_mode = new_mode
        return result

    def _handle_set_gear(self, request):
        gear = int(request.gear)
        if gear not in VALID_GEARS:
            return SetGearResponse(
                success=False,
                gear=self.gear,
                message="gear must be one of {} (P/R/N/D)".format(sorted(VALID_GEARS)),
            )
        result = self._call(OPTION_GEAR, gear=gear)
        if result is None:
            return SetGearResponse(
                success=False, gear=self.gear, message="MoraiEventCmd 호출 실패"
            )
        if self.gear != gear:
            # MORAI 는 주행 중 P 로의 전환 등을 거부할 수 있다.
            return SetGearResponse(
                success=False,
                gear=self.gear,
                message="시뮬레이터가 기어 변경을 받아들이지 않았다 (속도가 0 인지 확인)",
            )
        return SetGearResponse(success=True, gear=self.gear, message="")

    def _poll(self, _event):
        # 아무것도 바꾸지 않는 조회. 시뮬레이터가 외부 요인으로 기어를 바꿨을 때
        # (시나리오 리로드 등) 우리 상태가 뒤처지는 것을 막는다.
        self._call(OPTION_NONE)
        if self.ctrl_mode != CTRL_MODE_EXTERNAL and self.set_external_on_start:
            rospy.logwarn_throttle(
                10.0,
                "ctrl_mode 가 %s 다 (External=3 이 아니다). /ctrl_cmd 가 무시된다.",
                self.ctrl_mode,
            )
            self._call(OPTION_CTRL_MODE, ctrl_mode=CTRL_MODE_EXTERNAL)


def main() -> None:
    rospy.init_node("vip3_gear")
    GearNode()
    rospy.spin()


if __name__ == "__main__":
    main()
