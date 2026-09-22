#!/usr/bin/env python3
"""Move a closed raw rosbag run to rejected/ without deleting its lineage."""

from __future__ import print_function

import argparse
import os
from datetime import datetime, timezone

import yaml

from data_collection.bag_run import default_rosbag_root, slug


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--reason", required=True)
    parser.add_argument("--rosbag-root", default=default_rosbag_root())
    parser.add_argument(
        "--allow-stale-recording", action="store_true",
        help="allow a stale status=recording only when closed bags exist and no .active file remains",
    )
    args = parser.parse_args()

    root = os.path.abspath(os.path.expanduser(args.rosbag_root))
    run_id = slug(args.run_id, fallback="", max_length=120)
    if not run_id or run_id != args.run_id:
        raise SystemExit("run-id must already be a lowercase safe slug")
    source = os.path.join(root, "raw", run_id)
    target = os.path.join(root, "rejected", run_id)
    if not os.path.isdir(source):
        raise SystemExit("raw run not found: {}".format(source))
    if os.path.exists(target):
        raise SystemExit("rejected run already exists: {}".format(target))
    metadata_path = os.path.join(source, "metadata.yaml")
    metadata = {}
    if os.path.isfile(metadata_path):
        with open(metadata_path, "r") as stream:
            metadata = yaml.safe_load(stream) or {}
    if metadata.get("status") == "recording":
        bags_dir = os.path.join(source, "bags")
        has_closed = any(name.endswith(".bag") for name in os.listdir(bags_dir))
        has_active = any(name.endswith(".active") for name in os.listdir(bags_dir))
        if not args.allow_stale_recording or not has_closed or has_active:
            raise SystemExit(
                "refusing status=recording; stop the recorder first. For a proven stale run "
                "with closed bags and no .active file, pass --allow-stale-recording"
            )

    os.makedirs(os.path.dirname(target), exist_ok=True)
    os.rename(source, target)
    results_source = os.path.join(root, "results", run_id)
    results_moved = False
    if os.path.isdir(results_source):
        os.rename(results_source, os.path.join(target, "results"))
        results_moved = True
    rejection = {
        "schema_version": "vip3-rosbag-rejection-1.0.0",
        "run_id": run_id,
        "reason": args.reason.strip(),
        "rejected_at_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "previous_status": metadata.get("status", "unknown"),
        "recoverable": True,
        "original_location": os.path.join(root, "raw", run_id),
        "associated_results_moved": results_moved,
    }
    with open(os.path.join(target, "rejection.yaml"), "w") as stream:
        yaml.safe_dump(rejection, stream, allow_unicode=True, sort_keys=True)
    print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
