"""단조시계(monotonic) 기준 신선도 판정.

ASMC `include/vehicle_state/steady_freshness.hpp` 포팅.
ros::SteadyTime 대신 `time.monotonic()` 초 단위 float 를 쓴다. rosbridge 는
WebSocket 이 끊겨도 토픽 광고는 살아 있어서 "조용히 멈춘" 상태가 생긴다.
그걸 잡으려면 메시지 stamp 가 아니라 **수신 시각**으로 판단해야 한다.
"""

from __future__ import annotations

import math

NEVER_RECEIVED = 0.0


def is_steady_fresh(received_at, now, timeout_seconds):
    # type: (float, float, float) -> bool
    """`received_at` 이후 `timeout_seconds` 안이면 True. 한 번도 못 받았으면 False."""
    if received_at == NEVER_RECEIVED:
        return False
    age = now - received_at
    if not math.isfinite(age) or age < 0.0:
        return False
    if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
        return False
    return age <= timeout_seconds


def age_seconds(received_at, now):
    # type: (float, float) -> float
    """수신 후 경과 시간. 한 번도 못 받았으면 inf."""
    if received_at == NEVER_RECEIVED:
        return float("inf")
    return now - received_at
