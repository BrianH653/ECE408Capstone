#!/usr/bin/env python3
import os
import csv
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class ThermalCsvLogger(Node):
    """
    Periodically logs HT301 temperature summary to CSV:
      timestamp_seconds, center_c, min_c, max_c

    - Polls at 'sample_period' seconds (float)
    - Logs only if a temperature exceeds 'temp_threshold_c' (float)
      (checks center/min/max; logs if ANY exceeds threshold)
    - Output path controlled by 'log_path' parameter
    """

    def __init__(self):
        super().__init__("thermal_csv_logger")

        # Parameters (set from launch or CLI)
        self.declare_parameter("log_path", "thermal_log.csv")
        self.declare_parameter("sample_period", 1.0)
        self.declare_parameter("temp_threshold_c", 30.0)

        self.log_path = self.get_parameter("log_path").get_parameter_value().string_value
        self.log_path = os.path.expanduser(self.log_path)
        self.sample_period = float(self.get_parameter("sample_period").value)
        self.temp_threshold_c = float(self.get_parameter("temp_threshold_c").value)

        if self.sample_period <= 0.0:
            self.get_logger().warn("sample_period <= 0.0 is invalid; defaulting to 1.0s")
            self.sample_period = 1.0

        # Latest temperatures
        self.center_c: Optional[float] = None
        self.min_c: Optional[float] = None
        self.max_c: Optional[float] = None

        # Subscribers
        self.create_subscription(Float32, "/usbcam/temp/center_c", self._on_center, 10)
        self.create_subscription(Float32, "/usbcam/temp/min_c", self._on_min, 10)
        self.create_subscription(Float32, "/usbcam/temp/max_c", self._on_max, 10)

        # Prepare output file
        self._ensure_parent_dir(self.log_path)
        self._file = open(self.log_path, "a", newline="")
        self._writer = csv.writer(self._file)

        # Write header if empty
        if os.stat(self.log_path).st_size == 0:
            self._writer.writerow(["timestamp_s", "center_c", "min_c", "max_c"])
            self._file.flush()

        # Timer (poll)
        self._timer = self.create_timer(self.sample_period, self._on_timer)

        self.get_logger().info(
            f"ThermalCsvLogger running. log_path='{self.log_path}', "
            f"sample_period={self.sample_period}s, threshold={self.temp_threshold_c}C"
        )

    def _ensure_parent_dir(self, path: str) -> None:
        parent = os.path.dirname(os.path.abspath(path))
        if parent and not os.path.exists(parent):
            os.makedirs(parent, exist_ok=True)

    # Callbacks
    def _on_center(self, msg: Float32) -> None:
        self.center_c = float(msg.data)

    def _on_min(self, msg: Float32) -> None:
        self.min_c = float(msg.data)

    def _on_max(self, msg: Float32) -> None:
        self.max_c = float(msg.data)

    def _on_timer(self) -> None:
        # Only log once we have all three values
        if self.center_c is None or self.min_c is None or self.max_c is None:
            return

        # Threshold check: log if ANY exceeds threshold
        if max(self.center_c, self.min_c, self.max_c) < self.temp_threshold_c:
            return

        # Timestamp: ROS clock now, as float seconds
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        self._writer.writerow([f"{t:.6f}", f"{self.center_c:.3f}", f"{self.min_c:.3f}", f"{self.max_c:.3f}"])
        self._file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "_file") and self._file:
                self._file.flush()
                self._file.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ThermalCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()