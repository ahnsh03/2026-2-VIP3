#!/usr/bin/env python3
"""Compare hypothetical pinhole FOVs; this does not simulate vehicle occlusion.

Run from any directory. Writes a JSON analysis alongside the sensor design doc.
Uses the project's projection equations without importing ROS or model packages.
"""

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT / "src/perception/drivable_bev/src/drivable_bev/calibration.py"
    spec = importlib.util.spec_from_file_location("vip3_sensor_analysis_calibration", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    ground = module.GroundPlane("base_link", -0.35, 0.0, 0.0, "hypothetical_ioniq5")

    def camera(name, idx, width, height, fov, translation, rotation):
        return module.CameraCalibration(
            name, idx, width, height, fov, tuple(translation), tuple(rotation),
            "", "", "", "", ground,
        )

    sensor_path = ROOT / "config/VIP3_sensor_set_v1_ros.json"
    sensor_set = json.loads(sensor_path.read_text())
    cases = {"v1": [], "pinhole_candidate_140": []}
    for row in sensor_set["cameraList"]:
        cc = row["cc"]
        cases["v1"].append(camera(
            cc["sensorName"], row["m_SensorUniqueID"],
            cc["cameraResWidth"], cc["cameraResHeight"], cc["cameraFOV"],
            [float(row["pos"][k]) for k in ("x", "y", "z")],
            [float(row["rot"][k]) for k in ("roll", "pitch", "yaw")],
        ))
    candidates = [
        ("front", (3.90, 0.0, 0.65), (0.0, 45.0, 0.0)),
        ("left", (2.40, 1.05, 0.95), (0.0, 55.0, 90.0)),
        ("right", (2.40, -1.05, 0.95), (0.0, 55.0, -90.0)),
        ("rear", (-0.85, 0.0, 0.65), (0.0, 45.0, 180.0)),
    ]
    for idx, (name, translation, rotation) in enumerate(candidates, 1):
        cases["pinhole_candidate_140"].append(
            camera(name, idx, 1280, 960, 140.0, translation, rotation)
        )

    result = {
        "status": "hypothetical_projection_only_not_simulator_measurement",
        "assumptions": {
            "body_x_m": [-0.790, 3.845], "body_y_m": [-0.945, 0.945],
            "ground_z_m": -0.35,
            "vehicle_mesh_occlusion_modelled": False,
            "lens_verified": False, "line_pixel_width_verified": False,
            "note": "FOV fractions are upper bounds on these samples, not observed ground coverage. Corner regions are not sampled.",
        },
        "sensor_file": str(sensor_path.relative_to(ROOT)),
        "candidate": [
            {"view": n, "translation_m": t, "rotation_deg": r,
             "width": 1280, "height": 960, "horizontal_fov_deg": 140.0}
            for n, t, r in candidates
        ],
        "offsets_m": {},
    }
    for distance in (0.1, 0.2, 0.3, 0.5, 1.0, 2.0):
        x = np.linspace(-0.790, 3.845, 500)
        y = np.linspace(-0.945, 0.945, 200)
        sides = {
            "left": np.column_stack((x, np.full(len(x), 0.945 + distance))),
            "right": np.column_stack((x, np.full(len(x), -0.945 - distance))),
            "front": np.column_stack((np.full(len(y), 3.845 + distance), y)),
            "rear": np.column_stack((np.full(len(y), -0.790 - distance), y)),
        }
        result["offsets_m"][str(distance)] = {}
        for name, cameras in cases.items():
            stats = {}
            for side, points in sides.items():
                seen = []
                for cam in cameras:
                    uv, depth = cam.project_ground(points)
                    seen.append(
                        (depth > 0.1) & (uv[:, 0] >= 0) & (uv[:, 0] < cam.width)
                        & (uv[:, 1] >= 0) & (uv[:, 1] < cam.height)
                    )
                count = np.sum(seen, axis=0)
                stats[side] = {
                    "samples": len(points),
                    "in_fov_fraction": float(np.mean(count > 0)),
                    "two_camera_fraction": float(np.mean(count >= 2)),
                }
            result["offsets_m"][str(distance)][name] = stats
    target = ROOT / "docs/analysis/ss-psd-pinhole-candidates.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(target)
    for name in cases:
        stats = result["offsets_m"]["0.2"][name]
        print(name, {side: round(v["in_fov_fraction"], 3) for side, v in stats.items()})


if __name__ == "__main__":
    main()
