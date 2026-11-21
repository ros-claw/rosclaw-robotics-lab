"""Record unique live overview frames through a bounded nonblocking ROS subscriber."""

import argparse
import hashlib
import json
from pathlib import Path
from queue import Full, Queue
import threading
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage


class Recorder(Node):
    def __init__(self, output):
        super().__init__("luna_display_recorder")
        self.output = output
        output.mkdir()
        self.inbox = Queue(maxsize=8)
        self.writer_error = None
        self.enqueued = self.dropped = self.written = self.duplicates = 0
        self.last_stamp = -1
        self.thread = threading.Thread(target=self.write_images, daemon=True)
        self.subscription = self.create_subscription(
            RosImage, "/luna/display/image_raw", self.receive, qos_profile_sensor_data)
        self.thread.start()

    def receive(self, msg):
        time_ns = time.monotonic_ns()
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp:
            return
        self.last_stamp = stamp_ns
        if (self.enqueued >= 3600 or self.writer_error
                or msg.header.frame_id != "luna_overview_optical"):
            self.dropped += 1
            return
        try:
            self.inbox.put_nowait((time_ns, stamp_ns, msg))
            self.enqueued += 1
        except Full:
            self.dropped += 1

    def write_images(self):
        try:
            self.encode_images()
        except Exception as error:
            self.writer_error = str(error)

    def encode_images(self):
        previous = None
        with (self.output / "frames.jsonl").open("x") as stream:
            while True:
                item = self.inbox.get()
                if item is None:
                    break
                time_ns, stamp_ns, msg = item
                if msg.encoding != "rgb8" or msg.step < 3 * msg.width:
                    raise ValueError(f"unsupported overview frame {msg.encoding} / {msg.step}")
                pixels = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                                    buffer=msg.data, strides=(msg.step, 3, 1))
                digest = hashlib.sha256(pixels.tobytes()).hexdigest()
                if digest == previous:
                    self.duplicates += 1
                    continue
                name = f"{self.written:06d}.jpg"
                Image.fromarray(pixels).save(self.output / name, quality=93)
                stream.write(json.dumps({
                    "source": "isaac-live-overview", "name": name, "host_mono_ns": str(time_ns),
                    "sim_stamp_ns": str(stamp_ns), "rgb_sha256": digest,
                }) + "\n")
                stream.flush()
                self.written += 1
                previous = digest

    def finish(self):
        if self.thread.is_alive():
            try:
                self.inbox.put(None, timeout=2)
                self.thread.join(timeout=3)
            except Full:
                pass
        status = {"enqueued": self.enqueued, "written": self.written, "dropped": self.dropped,
                  "duplicates": self.duplicates, "error": self.writer_error,
                  "drained": not self.thread.is_alive()}
        (self.output / "recorder.json").write_text(json.dumps(status, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
