"""GPS(WGS84)/IMU 원시값 -> 로컬 평면 좌표(x=East, y=North)/yaw 변환. 순수 함수.

ASMC `src/vehicle_state/src/gps_transform.cpp` 를 그대로 옮겼다(값 동일).
호출부는 `/gps` (morai_msgs/GPSMessage) 콜백이다. UDP 와는 무관하다.

MORAI global path 좌표 = UTM52N 좌표 - GPS Info 의 East/North Offset.
rosbridge 경로에서는 MORAI 가 `GPSMessage.eastOffset/northOffset` 을 직접
채워 보내므로 launch 에서 offset 을 주입하지 않는다. (ASMC 는 UDP 라서
K-City 원점 302595/4124145 를 손으로 넣었다. KATRI 에서는 틀린 값이다.)
"""

from __future__ import annotations

import math

WGS84_A = 6378137.0          # WGS-84 타원체 장축 반경 [m]
WGS84_E2 = 0.006694379991    # WGS-84 이심률의 제곱
UTM52N_CENTRAL_MERIDIAN_DEG = 129.0   # KATRI(~126.77E)·K-City 모두 zone 52N
UTM_SCALE = 0.9996
UTM_FALSE_EASTING = 500000.0


def _lla_to_ecef(lat_rad, lon_rad, alt_m):
    sin_lat = math.sin(lat_rad)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_m) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def gps_to_enu(lat_deg, lon_deg, alt_m, ref_lat_deg, ref_lon_deg, ref_alt_m):
    """WGS84 -> ECEF -> 기준점 기준 ENU. (east, north, up) 반환.

    세 축 모두 기준점(ref_lat/ref_lon)으로 회전행렬을 만든다. 원본(Study_ITS
    Global.cpp)은 Up 축만 현재 위치 기준으로 만드는 버그가 있었다.
    """
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    ref_lat_rad = math.radians(ref_lat_deg)
    ref_lon_rad = math.radians(ref_lon_deg)

    x, y, z = _lla_to_ecef(lat_rad, lon_rad, alt_m)
    ref_x, ref_y, ref_z = _lla_to_ecef(ref_lat_rad, ref_lon_rad, ref_alt_m)
    dx, dy, dz = x - ref_x, y - ref_y, z - ref_z

    sin_lat, cos_lat = math.sin(ref_lat_rad), math.cos(ref_lat_rad)
    sin_lon, cos_lon = math.sin(ref_lon_rad), math.cos(ref_lon_rad)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def gps_to_utm52n(lat_deg, lon_deg):
    """WGS84 위경도 -> UTM Zone 52N(EPSG:32652) (easting, northing) [m]."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lon0 = math.radians(UTM52N_CENTRAL_MERIDIAN_DEG)

    ep2 = WGS84_E2 / (1.0 - WGS84_E2)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a = cos_lat * (lon - lon0)

    e4 = WGS84_E2 * WGS84_E2
    e6 = e4 * WGS84_E2
    m = WGS84_A * (
        (1.0 - WGS84_E2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat
        - (3.0 * WGS84_E2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
        * math.sin(2.0 * lat)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * math.sin(4.0 * lat)
        - (35.0 * e6 / 3072.0) * math.sin(6.0 * lat)
    )

    easting = UTM_FALSE_EASTING + UTM_SCALE * n * (
        a
        + (1.0 - t + c) * a ** 3 / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * a ** 5 / 120.0
    )
    northing = UTM_SCALE * (
        m
        + n * tan_lat * (
            a * a / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c * c) * a ** 4 / 24.0
            + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a ** 6 / 720.0
        )
    )
    if lat_deg < 0.0:
        northing += 10000000.0
    return easting, northing


def quaternion_to_yaw(w, x, y, z):
    """IMU 쿼터니언 -> yaw [rad], (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
