"""KATRI MGeo HD-map loading and RViz visualisation helpers for VIP3.

MGeo JSON은 읽기 전용 입력이다. 이 패키지는 MGeo를 절대 다시 쓰지 않는다.
발행물은 전부 stock ``visualization_msgs/MarkerArray`` 이며 custom msgs가 없다.

KATRI 맵 좌표계
    모든 레이어의 ``points`` 는 이미 로컬 미터 좌표다 (변환 불필요).
    ``map_xy = utm52n_xy - (302459.942, 4122635.537)``, EPSG:32652.
    ``global_info.workspace_origin`` 은 MORAI 에디터 뷰포트 값이므로 쓰지 않는다.
"""

from .frames import LocalMapFrame, origin_translation
from .mgeo_layers import KatriMGeo

__all__ = [
    "KatriMGeo",
    "LocalMapFrame",
    "origin_translation",
]
