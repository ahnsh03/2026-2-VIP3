#!/usr/bin/env python3
"""Periodic, receive-time MORAI diagnostics; no image decoding or control output."""

import argparse
import math
import sys
import threading
import time
from pathlib import Path


class TopicWindow:
    """Constant-memory arrival counter, independent of ROS/simulator time.

    Access is protected by the caller's lock. Hz includes silence in the reporting
    window, so a brief burst followed by a stall cannot look like a healthy stream.
    """

    def __init__(self, started):
        self.since = started
        self.count = 0
        self.total = 0
        self.last = None

    def receive(self, now):
        self.count += 1
        self.total += 1
        self.last = now

    def snapshot(self, now, nominal_hz, stale_after, slow_ratio):
        elapsed = max(now - self.since, 0.0)
        hz = self.count / elapsed if elapsed else 0.0
        age = None if self.last is None else max(now - self.last, 0.0)
        # Low-rate sensors should not be called stale between expected samples.
        stale_limit = max(stale_after, 3.0 / nominal_hz) if nominal_hz else stale_after
        if age is None:
            status = "MISSING"
        elif age >= stale_limit:
            status = "STALE"
        elif nominal_hz and hz < nominal_hz * slow_ratio:
            status = "SLOW"
        else:
            status = "OK"
        row = dict(hz=hz, age=age, count=self.count, total=self.total, status=status)
        self.since, self.count = now, 0
        return row


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topics-config", type=Path, required=True)
    parser.add_argument("--interval", type=positive_float, default=5.0)
    parser.add_argument("--stale-after", type=positive_float, default=2.0)
    parser.add_argument("--profile", choices=("runtime", "full", "cameras"), default="runtime")
    return parser.parse_args(argv)


def format_report(entries, rows, clients, elapsed):
    healthy = sum(row["status"] == "OK" for row in rows)
    client_text = "unknown" if clients is None else str(clients)
    lines = ["[sensors] clients={} | OK={}/{} | window={:.1f}s (receive-time)".format(
        client_text, healthy, len(rows), elapsed)]
    for entry, row in zip(entries, rows):
        nominal = "-" if not entry["nominal_hz"] else "{:.1f}".format(entry["nominal_hz"])
        age = "never" if row["age"] is None else "{:.2f}s".format(row["age"])
        lines.append("  {:<10} {:<7} {:>6.1f}/{} Hz | last={:<8} | n={}".format(
            entry["name"], row["status"], row["hz"], nominal, age, row["count"]))
    if clients == 0:
        lines.append("  hint: MORAI Network/Sensor ROS Connect, then Play")
    elif any(row["status"] in ("MISSING", "STALE") for row in rows):
        lines.append("  hint: check Play/pause, sensor enable, topics and Network/Sensor Connect")
    return "\n".join(lines)


def verifier_directory():
    """Locate shared code independently of where a caller stores its topic YAML."""
    for parent in Path(__file__).resolve().parents:
        scripts = parent / 'scripts'
        if (scripts / 'verify_morai_topics.py').is_file():
            return scripts
    raise RuntimeError('Cannot locate workspace scripts/verify_morai_topics.py')


def main():
    import rospy
    from rosbridge_msgs.msg import ConnectedClients

    args = parse_args(rospy.myargv()[1:])
    # The verifier and monitor both use the workspace's single topic contract.
    sys.path.insert(0, str(verifier_directory()))
    from verify_morai_topics import SLOW_RATIO, load_topic_table, select_topics

    entries = select_topics(load_topic_table(args.topics_config), args.profile)
    if not entries:
        raise SystemExit("No topics selected: {}".format(args.topics_config))

    rospy.init_node("vip3_morai_topic_monitor", anonymous=True)
    started = time.monotonic()
    windows = {entry["topic"]: TopicWindow(started) for entry in entries}
    lock = threading.Lock()
    clients = {"count": None}
    stopped = threading.Event()
    rospy.on_shutdown(stopped.set)

    def receive(_message, topic):
        with lock:
            windows[topic].receive(time.monotonic())

    def receive_clients(message):
        with lock:
            clients["count"] = len(message.clients)

    subscribers = [rospy.Subscriber(
        entry["topic"], rospy.AnyMsg, receive, callback_args=entry["topic"],
        queue_size=100, buff_size=16 * 1024 * 1024,
    ) for entry in entries]
    subscribers.append(rospy.Subscriber(
        "/connected_clients", ConnectedClients, receive_clients, queue_size=1,
    ))
    rospy.loginfo("[sensors] interval=%.1fs, stale>=%.1fs, SLOW<%.0f%% nominal; "
                  "Hz=local receipt rate, last=time since last receipt (not source latency)",
                  args.interval, args.stale_after, SLOW_RATIO * 100)
    rospy.loginfo("[sensors] topics: %s", ", ".join(
        "{}={}".format(entry["name"], entry["topic"]) for entry in entries))
    previous = started
    try:
        # Event.wait uses real time and wakes immediately on ROS shutdown. rospy
        # timers/rates would stop reporting when simulated /clock is paused.
        while not stopped.wait(args.interval) and not rospy.is_shutdown():
            with lock:
                now = time.monotonic()
                rows = [windows[entry["topic"]].snapshot(
                    now, entry["nominal_hz"], args.stale_after, SLOW_RATIO,
                ) for entry in entries]
                count = clients["count"]
            report = format_report(entries, rows, count, now - previous)
            if all(row["status"] == "OK" for row in rows):
                rospy.loginfo(report)
            else:
                rospy.logwarn(report)
            previous = now
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == "__main__":
    main()
