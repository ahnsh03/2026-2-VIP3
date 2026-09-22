"""KATRI MGeo 레이어 로더 — ROS 비의존.

MORAI 가 준 ``KATRI 맵 데이터 자료/`` 의 JSON 들을 한 번에 읽고, 판본마다 흔들리는
필드 타입을 정규화해서 레이어별 폴리라인과 NGII 코드 의미를 돌려준다.
``scripts/inspect_katri_mgeo.py`` (ROS 없음) 와 ``scripts/katri_map_viz_node.py``
(ROS) 가 같은 코드를 공유한다.

좌표계
    모든 ``points`` 는 이미 **로컬 미터** 좌표다. RViz ``map`` frame 에 그대로 찍으면 된다.
    ``map_xy = utm52n_xy - global_info.local_origin_in_global[:2]``  (KATRI: 302459.942, 4122635.537)
    ``global_info.workspace_origin`` 은 MORAI 에디터 뷰포트 값이다. 절대 쓰지 않는다.

필드 타입 함정 (KATRI 실측)
    ``lane_boundary.lane_type``      리스트 ``[505]``
    ``lane_boundary.lane_shape``     리스트 ``['Solid']`` / ``['Solid Solid']``
    ``surface_marking.type/sub_type`` **문자열** ``'1'`` / ``'5371'``
    ``link.link_type``               문자열 ``'6'`` 에 ``None`` 7개와 ``'Driving'`` 1개가 섞임
    ``global_info.mgeo_file_hash``   JSON object 가 아니라 **파이썬 dict 를 repr 한 문자열**
"""

from __future__ import annotations

import ast
import json
import os
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np

from .frames import LocalMapFrame
from .io_mgeo import load_json, sha256_file


# ---------------------------------------------------------------------------
# NGII 코드 라벨. 측정 근거가 확실한 것만 확정 라벨을 주고, 나머지는 UNVERIFIED_ 를
# 붙여 정직하게 둔다. 근거는 docs/map 의 KATRI MGeo 문서와 inspect_katri_mgeo.py 출력.
# ---------------------------------------------------------------------------
LANE_TYPE_LABEL = {
    501: ("center_line", "중앙선"),                    # yellow solid, 최대 1146 m
    503: ("lane_line", "차선"),                        # white broken, dash 3/5
    504: ("bus_lane", "버스전용차선"),                  # blue, 3개
    505: ("road_edge", "길가장자리구역선"),             # white solid, 최다 572개
    506: ("no_lane_change", "진로변경제한선"),
    515: ("UNVERIFIED_515", "정차금지지대(추정)"),
    525: ("guide_line", "유도선"),                     # dash 0.75/0.75
    530: ("stop_line", "정지선"),                      # CONFIRMED: 폭 0.6 m
    531: ("UNVERIFIED_531", "안전지대/도류화(추정)"),
    535: ("UNVERIFIED_535", "미확정 (주차금지/회전교차로 후보)"),
    599: ("other", "기타"),
}

SURFACE_MARKING_LABEL = {   # 전부 type='1', 4점 AABB, 0.5~2.3 m x ~5 m = 화살표/문자
    5371: ("arrow_straight", "직진(추정)"),
    5372: ("arrow_left", "좌회전(추정)"),
    5373: ("arrow_right", "우회전(추정)"),
    5374: ("arrow_straight_left", "직진+좌회전(추정)"),
    5379: ("UNVERIFIED_5379", "미확정"),
    5381: ("UNVERIFIED_5381", "미확정"),
    5382: ("UNVERIFIED_5382", "미확정"),
    5431: ("UNVERIFIED_5431", "미확정"),
    5432: ("UNVERIFIED_5432", "미확정"),
}

# KATRI 배포본 lane_boundary 에는 주차구획선에 해당하는 코드가 하나도 없다.
# 535 의 의미는 확정하지 못했다. 27개가 (82.2, 1190.2) 주변 60x90 m 에 흩어져 있는데,
# centroid 기준 반경 표준편차/평균 = 0.536 이고 방위 12분할 중 2칸이 비어 있어 **링이 아니다**
# (회전교차로 도색이면 0 에 가까워야 한다). 주차 규격(2.3~2.5 m x 5.0 m 평행 반복)도 아니다.
# NGII B2_SURFACELINEMARK 코드표 원문으로 확인하기 전까지 UNVERIFIED 로 둔다.
# 어느 쪽이든 주차면 기하가 아니라는 결론은 바뀌지 않으므로 주차 기능에는 영향이 없다.
PARKING_LANE_TYPES = frozenset()

# sha256("[]") — MORAI export 당시 '비어 있던' 레이어가 갖는 해시.
EMPTY_LIST_SHA256 = "4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"
# sha256("") — 0바이트 파일.
EMPTY_FILE_SHA256 = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"

# 우리 사본에 실제로 있는 레이어. 없으면 로드 실패가 아니라 경고다.
DEFAULT_LAYERS = {
    "global_info": "global_info.json",
    "link_set": "link_set.json",
    "node_set": "node_set.json",
    "lane_node_set": "lane_node_set.json",
    "lane_boundary_set": "lane_boundary_set.json",
    "surface_marking_set": "surface_marking_set.json",
    "singlecrosswalk_set": "singlecrosswalk_set.json",
    "crosswalk_set": "crosswalk_set.json",
    "traffic_light_set": "traffic_light_set.json",
    "traffic_sign_set": "traffic_sign_set.json",
}

# MORAI 원본 export 에는 있었으나 우리 사본에서 빠진 레이어. 파일만 놓으면 살아난다.
DEFAULT_OPTIONAL_LAYERS = {
    "parking_space_set": "parking_space_set.json",
    "object_set": "object_set.json",
    "lane_marking_set": "lane_marking_set.json",
}

# 원본 export 에 실재했다는 증거 (global_info.mgeo_file_hash). 비어 있는 레이어의
# 해시(EMPTY_LIST_SHA256)와 다르므로 '내용이 있었는데 전달만 안 됐다'가 확정이다.
EXPECTED_MISSING_SHA256 = {
    "parking_space_set.json": "0d410f76591ad9bd028c8950fe79fb07453511446caebecf6a9156280783f25b",
    "object_set.json": "ee3a8f6aed261977a884aa2fb6e930f6e1bd8470f0ea26bd8d376360a82ae125",
}

# MORAI ParkingSpace (MGeoModule class_defs/parking_space.py) 기본값.
PARKING_SPACE_DEFAULTS = {
    "idx": "",
    "points": [],
    "center_point": None,
    "parking_type": None,
    "parking_target_type": None,
    "parking_direction": None,
    "distance": 0.0,
    "width": 2.5,
    "length": 5.0,
    "angle": 90.0,
    "linked_left_list_idx": [],
    "linked_right_list_idx": [],
}


def expand_path(raw: str) -> Path:
    """``${VIP3_DATA}`` 같은 환경변수와 ``~`` 를 풀어 준다."""

    return Path(os.path.expandvars(os.path.expanduser(str(raw))))


def as_int(value, default: Optional[int] = None) -> Optional[int]:
    """리스트/문자열/None 이 섞여 들어오는 MGeo 코드 필드를 정수로 정규화한다."""

    if isinstance(value, (list, tuple)):
        for item in value:
            result = as_int(item, None)
            if result is not None:
                return result
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def points_xyz(feature: dict, key: str = "points") -> np.ndarray:
    """``points`` (또는 ``point``) 를 Nx3 float 배열로. 비면 (0,3)."""

    raw = feature.get(key)
    if raw is None and key == "points":
        single = feature.get("point")
        raw = [single] if single is not None else None
    if not raw:
        return np.zeros((0, 3), dtype=np.float64)
    array = np.asarray(raw, dtype=np.float64)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] < 2:
        return np.zeros((0, 3), dtype=np.float64)
    if array.shape[1] == 2:
        array = np.column_stack((array, np.zeros(len(array))))
    return array[:, :3]


def bounds_of(features: Iterable[dict]) -> Optional[tuple]:
    """레이어의 (min_x, min_y, min_z, max_x, max_y, max_z). 비면 None."""

    lows: list[np.ndarray] = []
    highs: list[np.ndarray] = []
    for feature in features:
        array = points_xyz(feature)
        if not len(array):
            continue
        finite = array[np.isfinite(array).all(axis=1)]
        if not len(finite):
            continue
        lows.append(finite.min(axis=0))
        highs.append(finite.max(axis=0))
    if not lows:
        return None
    low = np.min(np.asarray(lows), axis=0)
    high = np.max(np.asarray(highs), axis=0)
    return (
        float(low[0]), float(low[1]), float(low[2]),
        float(high[0]), float(high[1]), float(high[2]),
    )


def parse_file_hash_manifest(global_info: dict) -> dict:
    """``global_info.mgeo_file_hash`` 를 dict 로.

    MORAI 는 이걸 JSON object 가 아니라 **파이썬 dict 를 repr 한 문자열** 로 저장한다.
    ``json.loads`` 로는 못 읽는다 (작은따옴표). ``ast.literal_eval`` 을 쓴다.
    """

    raw = global_info.get("mgeo_file_hash")
    if isinstance(raw, dict):
        return dict(raw)
    if not isinstance(raw, str) or not raw.strip():
        return {}
    try:
        parsed = ast.literal_eval(raw)
    except (SyntaxError, ValueError):
        try:
            parsed = json.loads(raw)
        except ValueError:
            return {}
    return dict(parsed) if isinstance(parsed, dict) else {}


def normalise_parking_space(raw: dict) -> dict:
    """MORAI ``ParkingSpace.from_dict`` 와 같은 필드 집합으로 맞춘다.

    ``parking_space_set.json`` 과 손으로 적은 ``config/vip3_parking_spaces.json``
    이 **같은 코드 경로**를 타게 하는 것이 핵심이다. 원본 파일이 도착하면
    파일만 갈아 끼우면 된다.
    """

    space = dict(PARKING_SPACE_DEFAULTS)
    space["linked_left_list_idx"] = []
    space["linked_right_list_idx"] = []
    space.update({key: raw[key] for key in PARKING_SPACE_DEFAULTS if key in raw})
    points = points_xyz({"points": space.get("points")})
    space["points"] = [[float(v) for v in point] for point in points]
    center = space.get("center_point")
    if center is None or len(np.ravel(np.asarray(center, dtype=object))) < 2:
        space["center_point"] = (
            [float(v) for v in points.mean(axis=0)] if len(points) else None
        )
    else:
        center_array = np.asarray(center, dtype=np.float64).ravel()
        if len(center_array) == 2:
            center_array = np.append(center_array, 0.0)
        space["center_point"] = [float(v) for v in center_array[:3]]
    for key, default in (("distance", 0.0), ("width", 2.5), ("length", 5.0), ("angle", 90.0)):
        try:
            space[key] = float(space[key])
        except (TypeError, ValueError):
            space[key] = default
    space["idx"] = str(space.get("idx") or "")
    return space


class KatriMGeo:
    """KATRI MGeo 한 벌. 레이어를 통째로 메모리에 올린다 (약 29 MB)."""

    def __init__(
        self,
        map_dir,
        layers: Optional[dict] = None,
        optional_layers: Optional[dict] = None,
        manual_parking_path=None,
    ) -> None:
        self.map_dir = Path(map_dir)
        self._layer_files = dict(layers or DEFAULT_LAYERS)
        self._optional_files = dict(optional_layers or DEFAULT_OPTIONAL_LAYERS)
        self.manual_parking_path = (
            None if manual_parking_path is None else Path(manual_parking_path)
        )
        self.layers: dict[str, list] = {}
        self.missing_layers: list[str] = []
        self.missing_optional_layers: list[str] = []
        self.global_info: dict = {}
        self.parking_space_source = "none"
        self._load()

    # -- construction -------------------------------------------------------

    @classmethod
    def from_dir(cls, map_dir, manual_parking_path=None) -> "KatriMGeo":
        return cls(expand_path(map_dir), manual_parking_path=manual_parking_path)

    @classmethod
    def from_config(cls, config_path, map_dir_override: str = "") -> "KatriMGeo":
        """``config/katri_map_sources.yaml`` 에서 만든다.

        ``map_dir_override`` (ROS 의 ``~map_dir`` 파라미터) 가 비어 있지 않으면
        config 의 ``map_dir`` 을 덮어쓴다. 데이터 디렉터리 이름이 한글+공백이라
        런치 파일에 리터럴로 박지 않고 파라미터로 넘기는 편이 안전하다.
        """

        import yaml  # config 를 쓸 때만 필요하다.

        config_path = Path(config_path)
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        map_dir = expand_path(str(map_dir_override).strip() or config.get("map_dir", ""))
        manual = config.get("manual_parking_spaces")
        manual_path = expand_path(manual) if manual else None
        if manual_path is not None and not manual_path.is_absolute():
            manual_path = (config_path.parent / manual_path).resolve()
        if manual_path is not None and not manual_path.is_file():
            # 기본 위치(config 옆)로 한 번 더 시도한다. ${VIP3_WS_ROOT} 미설정 대비.
            fallback = config_path.parent / "vip3_parking_spaces.json"
            manual_path = fallback if fallback.is_file() else manual_path
        return cls(
            map_dir,
            layers=config.get("layers"),
            optional_layers=config.get("optional_layers"),
            manual_parking_path=manual_path,
        )

    # -- loading ------------------------------------------------------------

    def _load(self) -> None:
        for name, filename in self._layer_files.items():
            path = self.map_dir / filename
            if not path.is_file():
                self.missing_layers.append(name)
                if name != "global_info":
                    self.layers[name] = []
                continue
            data = load_json(path)
            if name == "global_info":
                self.global_info = data if isinstance(data, dict) else {}
            else:
                self.layers[name] = data if isinstance(data, list) else list(data or [])
        for name, filename in self._optional_files.items():
            path = self.map_dir / filename
            if path.is_file():
                data = load_json(path)
                self.layers[name] = data if isinstance(data, list) else list(data or [])
            else:
                self.missing_optional_layers.append(name)

    # -- accessors ----------------------------------------------------------

    def layer(self, name: str) -> list:
        return self.layers.get(name, [])

    def links(self) -> list:
        return self.layer("link_set")

    def lane_boundaries(self) -> list:
        return self.layer("lane_boundary_set")

    def lane_boundaries_of_type(self, lane_type: int) -> list:
        target = int(lane_type)
        return [
            feature for feature in self.lane_boundaries()
            if as_int(feature.get("lane_type")) == target
        ]

    def surface_markings(self) -> list:
        return self.layer("surface_marking_set")

    def crosswalks(self) -> list:
        """singlecrosswalk_set — 실제 폴리곤이 있는 쪽. crosswalk_set 은 묶음 인덱스다."""

        return self.layer("singlecrosswalk_set")

    def frame(self) -> LocalMapFrame:
        return LocalMapFrame.from_global_info(self.global_info)

    def map_origin_xy(self) -> tuple:
        origin = self.frame().origin_xyz
        return (float(origin[0]), float(origin[1]))

    def bounds(self) -> Optional[tuple]:
        """모든 폴리라인 레이어를 합친 전체 XYZ 범위."""

        merged = [
            feature
            for name, features in self.layers.items()
            if name != "crosswalk_set"
            for feature in features
        ]
        return bounds_of(merged)

    # -- parking ------------------------------------------------------------

    def parking_spaces(self) -> list:
        """주차면. ``parking_space_set.json`` → 손으로 적은 fallback → 빈 리스트.

        KATRI 배포본에는 ``parking_space_set.json`` 이 없다 (원본에는 있었다).
        받기 전까지는 ``config/vip3_parking_spaces.json`` 이 유일한 주차 GT 다.
        """

        raw = self.layer("parking_space_set")
        if raw:
            self.parking_space_source = "parking_space_set"
            return [normalise_parking_space(item) for item in raw if isinstance(item, dict)]
        if self.manual_parking_path is not None and self.manual_parking_path.is_file():
            data = load_json(self.manual_parking_path)
            if isinstance(data, list) and data:
                self.parking_space_source = "manual"
                return [normalise_parking_space(item) for item in data if isinstance(item, dict)]
        self.parking_space_source = "none"
        return []

    # -- statistics / audit -------------------------------------------------

    def layer_stats(self) -> dict:
        stats = {}
        for name, features in sorted(self.layers.items()):
            stats[name] = {"count": len(features), "bounds": bounds_of(features)}
        return stats

    def lane_type_counts(self) -> dict:
        counts: dict = {}
        for feature in self.lane_boundaries():
            key = as_int(feature.get("lane_type"), -1)
            counts[key] = counts.get(key, 0) + 1
        return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))

    def value_counts(self, layer_name: str, field: str) -> dict:
        counts: dict = {}
        for feature in self.layer(layer_name):
            raw = feature.get(field)
            if isinstance(raw, (list, tuple)):
                raw = " ".join(str(value) for value in raw)
            key = str(raw)
            counts[key] = counts.get(key, 0) + 1
        return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))

    def surface_marking_sub_type_counts(self) -> dict:
        counts: dict = {}
        for feature in self.surface_markings():
            key = as_int(feature.get("sub_type"), -1)
            counts[key] = counts.get(key, 0) + 1
        return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))

    def file_hash_manifest(self) -> dict:
        """``.meta`` 를 뺀 실제 데이터 파일의 기대 sha256 표."""

        manifest = parse_file_hash_manifest(self.global_info)
        return {
            name: digest
            for name, digest in sorted(manifest.items())
            if not name.endswith(".meta")
        }

    def manifest_report(self, verify: bool = True) -> list:
        """매니페스트 대비 실제 파일 상태.

        이 표가 이 패키지에서 가장 쓸모 있는 출력이다. MORAI 가
        ``parking_space_set.json`` 을 보내 줬는지 1초에 확인할 수 있다.

        status
            ``ok``                파일이 있고 sha256 도 일치
            ``hash_mismatch``     파일은 있는데 내용이 다르다 (누가 편집했다)
            ``missing_empty``     빠졌지만 원본도 비어 있었다 (받을 게 없다)
            ``missing_nonempty``  **빠졌는데 원본에는 내용이 있었다 — 요청해야 한다**
        """

        report = []
        for name, expected in self.file_hash_manifest().items():
            path = self.map_dir / name
            present = path.is_file()
            upstream_empty = expected in (EMPTY_LIST_SHA256, EMPTY_FILE_SHA256)
            actual = sha256_file(path) if (present and verify) else None
            if present:
                status = "ok" if (actual is None or actual == expected) else "hash_mismatch"
            else:
                status = "missing_empty" if upstream_empty else "missing_nonempty"
            report.append({
                "file": name,
                "expected_sha256": expected,
                "actual_sha256": actual,
                "present": present,
                "upstream_empty": upstream_empty,
                "status": status,
            })
        return report

    def missing_nonempty_files(self) -> list:
        return [row for row in self.manifest_report(verify=False)
                if row["status"] == "missing_nonempty"]


def lane_type_label(lane_type) -> tuple:
    """(slug, 한국어 이름). 모르는 코드는 ``UNVERIFIED_<code>`` 로."""

    code = as_int(lane_type, -1)
    if code in LANE_TYPE_LABEL:
        return LANE_TYPE_LABEL[code]
    return ("UNVERIFIED_%d" % code, "미확정(%s)" % code)


def surface_marking_label(sub_type) -> tuple:
    code = as_int(sub_type, -1)
    if code in SURFACE_MARKING_LABEL:
        return SURFACE_MARKING_LABEL[code]
    return ("UNVERIFIED_%d" % code, "미확정(%s)" % code)


def parking_space_polygon(space: dict) -> np.ndarray:
    """주차면 코너를 닫힌 폴리곤(N+1 x 3)으로. 마커 LINE_STRIP 에 바로 쓴다."""

    points = points_xyz(space)
    if len(points) < 2:
        return np.zeros((0, 3), dtype=np.float64)
    return np.vstack((points, points[:1]))


def closed_polygon(feature: dict) -> np.ndarray:
    """4점 AABB 쿼드(노면표시/횡단보도)를 닫힌 폴리곤으로."""

    return parking_space_polygon(feature)


def polyline_segments(features: Iterable[dict]) -> np.ndarray:
    """폴리라인 묶음을 Nx2x2 선분 배열로 (map frame XY)."""

    segments = []
    for feature in features:
        points = points_xyz(feature)
        if len(points) < 2:
            continue
        xy = points[:, :2]
        segments.append(np.stack((xy[:-1], xy[1:]), axis=1))
    if not segments:
        return np.zeros((0, 2, 2), dtype=np.float64)
    return np.concatenate(segments, axis=0)
