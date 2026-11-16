"""Capture Mode helpers: schedule, manifest, and the 12-file saved-output check."""
import json
import os
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

from vip3_data import capture as cap


def write_png(path, hw=cap.IMAGE_HW):
    """최소 유효 PNG (IHDR + 빈 IDAT + IEND). 크기 검사에는 충분하다."""
    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))
    ihdr = struct.pack(">IIBBBBB", hw[1], hw[0], 8, 0, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(b"\x00")) + chunk(b"IEND", b""))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as stream:
        stream.write(png)


def write_frame(root, name, skip=(), hw=cap.IMAGE_HW):
    for view, modality, path in cap.frame_files(root, name):
        if (view, modality) not in skip:
            write_png(path, hw)


class NamingTest(unittest.TestCase):
    def test_sanitize_and_run_id(self):
        self.assertEqual(cap.sanitize_name("a b/../c"), "a_b_c")
        self.assertEqual(cap.sanitize_name("", fallback="x"), "x")
        self.assertRegex(cap.make_run_id("my run"), r"^my_run_\d{8}T\d{6}_\d{3}$")

    def test_capture_name(self):
        self.assertEqual(cap.capture_name("r", 7), "r_000007")


class ScheduleTest(unittest.TestCase):
    def test_rejects_bad_hz(self):
        for hz in (0, -1, 11, float("nan")):
            with self.assertRaises(ValueError):
                cap.CaptureSchedule(hz)

    def test_no_catchup_burst(self):
        clock = [0.0]
        s = cap.CaptureSchedule(2.0, start_delay_sec=1.0, monotonic_fn=lambda: clock[0])
        self.assertEqual(s.remaining(), 1.0)
        clock[0] = 10.0  # 오래 멈췄다 돌아옴
        self.assertEqual(s.remaining(), 0.0)
        s.advance()
        self.assertAlmostEqual(s.remaining(), 0.5)  # 밀린 슬롯을 연속 발사하지 않는다


class ManifestTest(unittest.TestCase):
    def test_writer_and_reader(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = cap.CaptureRunWriter(tmp, "run1", {"capture_hz": 1})
            w.write_capture({"sequence": 0, "custom_name": "run1_000000", "success": True})
            w.write_capture({"sequence": 1, "custom_name": "run1_000001", "success": False})
            w.write_capture({"sequence": 2, "custom_name": "run1_000002", "success": True})
            w.close()
            self.assertEqual(cap.load_published_names(w.manifest_path),
                             [(0, "run1_000000"), (2, "run1_000002")])
            with open(os.path.join(w.root, "summary.json")) as stream:
                summary = json.load(stream)
            self.assertEqual((summary["attempted"], summary["published"], summary["failed"]),
                             (3, 2, 1))
            with self.assertRaises(FileExistsError):
                cap.CaptureRunWriter(tmp, "run1")

    def test_duplicate_name_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "m.jsonl")
            with open(path, "w") as f:
                for seq in (0, 1):
                    f.write(json.dumps({"sequence": seq, "custom_name": "x", "success": True}) + "\n")
            with self.assertRaises(ValueError):
                cap.load_published_names(path)


class FrameCheckTest(unittest.TestCase):
    def test_twelve_files_layout(self):
        files = cap.frame_files("/r", "n")
        self.assertEqual(len(files), 12)
        self.assertEqual(files[0][2], os.path.join("/r", "CAMERA_1", "n_Intensity.png"))
        self.assertEqual({m for _, m, _ in files}, {"Intensity", "Semantic", "Instance"})
        self.assertNotIn("Depth", {m for _, m, _ in files})

    def test_complete_frame_ok(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_frame(tmp, "n")
            self.assertEqual(cap.check_frame(tmp, "n"), [])

    def test_missing_semantic_instance_detected(self):
        # Sensor Capture Mode 버튼이 꺼진 카메라의 전형적 증상: Intensity 만 나온다
        with tempfile.TemporaryDirectory() as tmp:
            for view, modality, path in cap.frame_files(tmp, "n"):
                if not (view == "rear" and modality != "Intensity"):
                    write_png(path)
            problems = cap.check_frame(tmp, "n")
            self.assertEqual({(p["view"], p["modality"], p["problem"]) for p in problems},
                             {("rear", "Semantic", "missing"), ("rear", "Instance", "missing")})

    def test_empty_wrong_size_and_bad_png(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_frame(tmp, "n")
            files = {(v, m): p for v, m, p in cap.frame_files(tmp, "n")}
            open(files[("front", "Intensity")], "wb").close()
            write_png(files[("left", "Semantic")], hw=(480, 640))
            with open(files[("right", "Instance")], "wb") as f:
                f.write(b"not a png at all, just text padding")
            kinds = {(p["view"], p["modality"]): p["problem"] for p in cap.check_frame(tmp, "n")}
            self.assertEqual(kinds[("front", "Intensity")], "empty")
            self.assertTrue(kinds[("left", "Semantic")].startswith("shape"))
            self.assertTrue(kinds[("right", "Instance")].startswith("bad_png"))
            self.assertEqual(len(kinds), 3)

    def test_check_run_skip_last_and_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_frame(tmp, "a")
            report = cap.check_run(tmp, ["a", "b"], skip_last=1)  # b 는 쓰는 중
            self.assertTrue(report["ok"])
            self.assertEqual((report["frames_checked"], report["frames_skipped"]), (1, 1))
            self.assertFalse(cap.check_run(tmp, ["a", "b"])["ok"])
            self.assertFalse(cap.check_run(tmp, [])["ok"])  # 검사할 게 없으면 통과 아님


class CheckScriptTest(unittest.TestCase):
    script = Path(__file__).resolve().parents[1] / "scripts" / "check_capture.py"

    def run_script(self, *args):
        env = dict(os.environ)
        src = str(Path(__file__).resolve().parents[1] / "src")
        env["PYTHONPATH"] = src + os.pathsep + env.get("PYTHONPATH", "")
        return subprocess.run([sys.executable, str(self.script)] + list(args),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, env=env)

    def test_run_id_flow(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, data = os.path.join(tmp, "SensorData"), os.path.join(tmp, "data")
            w = cap.CaptureRunWriter(data, "r")
            for seq in (0, 1):
                name = cap.capture_name("r", seq)
                w.write_capture({"sequence": seq, "custom_name": name, "success": True})
                write_frame(sensor, name)
            w.close()
            ok = self.run_script("--run-id", "r", "--data-root", data,
                                 "--sensor-root", sensor, "--dry-run")
            self.assertEqual(ok.returncode, 0, ok.stdout + ok.stderr)
            os.remove(cap.frame_files(sensor, "r_000001")[5][2])
            bad = self.run_script("--run-id", "r", "--data-root", data,
                                  "--sensor-root", sensor, "--dry-run")
            self.assertEqual(bad.returncode, 1)
            self.assertIn("missing", bad.stdout)

    def test_resolution_option_and_meta(self):
        with tempfile.TemporaryDirectory() as tmp:
            sensor, data = os.path.join(tmp, "S"), os.path.join(tmp, "d")
            w = cap.CaptureRunWriter(data, "r", {"image_width": 640, "image_height": 480})
            w.write_capture({"sequence": 0, "custom_name": "r_000000", "success": True})
            w.close()
            write_frame(sensor, "r_000000", hw=(480, 640))
            args = ("--sensor-root", sensor, "--dry-run")
            # meta.json 의 해상도를 따른다
            self.assertEqual(self.run_script("--run-id", "r", "--data-root", data, *args).returncode, 0)
            # 명시 옵션이 우선한다
            self.assertEqual(self.run_script("--run-id", "r", "--data-root", data,
                                             "--width", "1280", "--height", "960", *args).returncode, 1)
            self.assertEqual(self.run_script("--names", "r_000000", "--width", "640",
                                             "--height", "480", *args).returncode, 0)

    def test_input_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.run_script("--sensor-root", tmp).returncode, 2)
            self.assertEqual(self.run_script("--run-id", "nope", "--data-root", tmp,
                                             "--sensor-root", tmp).returncode, 2)
            self.assertEqual(self.run_script("--names", "x", "--sensor-root",
                                             os.path.join(tmp, "absent")).returncode, 2)


if __name__ == "__main__":
    unittest.main()
