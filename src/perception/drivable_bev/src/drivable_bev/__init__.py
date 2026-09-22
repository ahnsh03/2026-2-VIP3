"""카메라 semantic 출력을 base_link 기준 미터 BEV 로 투영한다.

ASMC `drivable_bev` 에서 기하 코어와 재사용 가능한 선 피팅만 가져왔다. K-City 음영구간
연구용이던 shadow_*, boundary_*, invalid_*, planning_usability, bag_batch, offline_evaluation
계열은 가져오지 않았다 — 자율주차와 무관하고 `asmc_msgs` 에 묶여 있었다.
"""

from .calibration import (
    CameraCalibration,
    GroundPlane,
    calibration_from_dict,
    calibrations_from_dict,
    ground_plane_from_dict,
    load_calibration_snapshot,
    validate_sensor_set_snapshot,
)
from .grid import BevGridSpec
from .homography import HomographyModel, build_homography
from .projector import CameraBevProjector, ProjectedSemantic
from .fusion import (
    CameraBevFusion,
    FusedSemantic,
    build_quality_maps,
    timestamp_span_ns,
    timestamps_within_slop,
)
from .model_grid_projection import MetricEvidence, ModelGridGroundProjector
from .lane_evidence import LaneNode, reduce_evidence_to_nodes
from .lane_tracking import LaneTrack, associate_lane_nodes
from .spline_fitting import FittedLane, fit_lane_track, fit_lane_tracks
from .performance_monitor import PerformanceMonitor, source_time_gate_with_reset

__all__ = [
    "BevGridSpec",
    "CameraCalibration",
    "GroundPlane",
    "HomographyModel",
    "CameraBevProjector",
    "ProjectedSemantic",
    "CameraBevFusion",
    "FusedSemantic",
    "MetricEvidence",
    "ModelGridGroundProjector",
    "LaneNode",
    "LaneTrack",
    "FittedLane",
    "PerformanceMonitor",
    "build_homography",
    "build_quality_maps",
    "calibration_from_dict",
    "calibrations_from_dict",
    "ground_plane_from_dict",
    "load_calibration_snapshot",
    "validate_sensor_set_snapshot",
    "timestamp_span_ns",
    "timestamps_within_slop",
    "reduce_evidence_to_nodes",
    "associate_lane_nodes",
    "fit_lane_track",
    "fit_lane_tracks",
    "source_time_gate_with_reset",
]
