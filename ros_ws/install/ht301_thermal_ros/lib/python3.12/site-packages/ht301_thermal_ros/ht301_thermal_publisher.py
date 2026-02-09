#!/usr/bin/env python3
"""
ROS 2 publisher for HT301 thermal camera (ht301_hacklib) -> sensor_msgs/Image

Topics (within your namespace, e.g. /usbcam):
- image_raw
- camera_info
"""

from __future__ import annotations

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo

from . import ht301_hacklib
import time

COLORMAPS = [
    cv2.COLORMAP_INFERNO,
    cv2.COLORMAP_MAGMA,
    cv2.COLORMAP_PLASMA,
    cv2.COLORMAP_TURBO,
    cv2.COLORMAP_JET,
]


def rotate_frame(frame: np.ndarray, orientation_deg: int) -> np.ndarray:
    if orientation_deg == 90:
        return np.rot90(frame).copy()
    if orientation_deg == 180:
        return np.rot90(frame, 2).copy()
    if orientation_deg == 270:
        return np.rot90(frame, 3).copy()
    return frame


def auto_scale(frame_u16_or_f32: np.ndarray, p_low: float, p_high: float) -> np.ndarray:
    """Percentile-based scaling -> stable exposure for thermal frames."""
    f = frame_u16_or_f32.astype(np.float32)
    lo, hi = np.percentile(f, (p_low, p_high))
    return np.clip((f - lo) * 255.0 / (hi - lo + 1e-6), 0, 255).astype(np.uint8)


def fit_to_landscape(frame: np.ndarray, out_w: int, out_h: int) -> np.ndarray:
    """Letterbox into a fixed (out_w,out_h) canvas without stretching."""
    h, w = frame.shape[:2]
    scale = min(out_w / w, out_h / h)
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

    canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    x0 = (out_w - new_w) // 2
    y0 = (out_h - new_h) // 2
    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas


def draw_temperature_bar(img: np.ndarray, tmin: float, tmax: float, colormap: int) -> None:
    """Vertical colorbar on the right side (kept fully within the image)."""
    h, w = img.shape[:2]
    bar_w = max(14, w // 32)
    pad = 30 #max(6, w // 80)

    margin = max(8, h // 30)
    usable_h = max(1, h - 2 * margin)

    grad = np.linspace(255, 0, usable_h, dtype=np.uint8)[:, None]
    grad = np.repeat(grad, bar_w, axis=1)
    bar = cv2.applyColorMap(grad, colormap)

    x0 = max(0, w - bar_w - pad)
    x1 = max(x0 + 1, w - pad)
    img[margin:margin + usable_h, x0:x1] = bar[:, : (x1 - x0)]

    # Label text
    font = cv2.FONT_HERSHEY_SIMPLEX
    fscale = max(0.45, min(1.0, h / 600.0))
    thick = 1 if h < 720 else 2

    top_y = max(14, margin - 4)
    bot_y = min(h - 6, h - margin + 14)

    cv2.putText(img, f"{tmax:.1f} C", (max(0, x0 - 20), top_y), font, fscale,
                (255, 255, 255), thick, cv2.LINE_AA)
    cv2.putText(img, f"{tmin:.1f} C", (max(0, x0 - 20), bot_y), font, fscale,
                (255, 255, 255), thick, cv2.LINE_AA)


def draw_marker_and_label(img: np.ndarray, x: int, y: int, text: str, color: tuple[int, int, int], show_text: bool) -> None:
    h, w = img.shape[:2]
    x = int(np.clip(x, 0, w - 1))
    y = int(np.clip(y, 0, h - 1))

    size = max(10, min(h, w) // 30)
    thickness = 1 if min(h, w) < 720 else 2
    cv2.drawMarker(img, (x, y), color, cv2.MARKER_CROSS, size, thickness, cv2.LINE_AA)

    if not show_text:
        return

    font = cv2.FONT_HERSHEY_SIMPLEX
    fscale = max(0.45, min(1.0, h / 650.0))
    thick = 1 if h < 720 else 2
    pad = 6
    tx = min(x + pad, w - 1)
    ty = max(y - pad, 18)
    cv2.putText(img, text, (tx, ty), font, fscale, color, thick, cv2.LINE_AA)


def cv_to_imgmsg_bgr8(frame_bgr: np.ndarray, stamp, frame_id: str) -> Image:
    """Create sensor_msgs/Image from a uint8 BGR image."""
    if frame_bgr.dtype != np.uint8:
        frame_bgr = frame_bgr.astype(np.uint8)
    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 BGR image, got shape {frame_bgr.shape}")

    h, w = frame_bgr.shape[:2]
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = h
    msg.width = w
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = w * 3
    msg.data = frame_bgr.tobytes()
    return msg

class FPSCounter:
    def __init__(self, alpha=0.9):
        self.alpha = alpha
        self.last_time = None
        self.fps_ema = None

    def tick(self):
        now = time.time()
        if self.last_time is None:
            self.last_time = now
            return None

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0:
            return self.fps_ema

        fps = 1.0 / dt
        if self.fps_ema is None:
            self.fps_ema = fps
        else:
            self.fps_ema = self.alpha * self.fps_ema + (1.0 - self.alpha) * fps

        return self.fps_ema

class HT301ThermalPublisher(Node):
    def __init__(self):
        super().__init__("ht301_thermal_publisher")

        # parameters (ints to avoid Jazzy launch typing surprises)
        self.declare_parameter("fps", 25)
        self.declare_parameter("out_width", 640)
        self.declare_parameter("out_height", 360)

        self.declare_parameter("colormap_index", 3)
        self.declare_parameter("orientation", 0) # 0/90/180/270
        self.declare_parameter("frame_id", "thermal_optical_frame")
        self.declare_parameter("landscape", True)

        # overlay toggles
        self.declare_parameter("overlay_fps", True)
        self.declare_parameter("overlay_min", True)
        self.declare_parameter("overlay_max", True)
        self.declare_parameter("overlay_center", False)
        self.declare_parameter("overlay_bar", True)
        self.declare_parameter("overlay_text", True) # show text labels next to markers
        self.declare_parameter("p_low", 2.0) # auto_scale percentiles
        self.declare_parameter("p_high", 98.0)

        self.fps = float(self.get_parameter("fps").value)
        self.out_w = int(self.get_parameter("out_width").value)
        self.out_h = int(self.get_parameter("out_height").value)

        self.colormap_index = int(self.get_parameter("colormap_index").value) % len(COLORMAPS)
        self.orientation = int(self.get_parameter("orientation").value) % 360
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.landscape = bool(self.get_parameter("landscape").value)

        self.overlay_fps = bool(self.get_parameter("overlay_fps").value)
        self.overlay_min = bool(self.get_parameter("overlay_min").value)
        self.overlay_max = bool(self.get_parameter("overlay_max").value)
        self.overlay_center = bool(self.get_parameter("overlay_center").value)
        self.overlay_bar = bool(self.get_parameter("overlay_bar").value)
        self.overlay_text = bool(self.get_parameter("overlay_text").value)
        self.p_low = float(self.get_parameter("p_low").value)
        self.p_high = float(self.get_parameter("p_high").value)

        # fps counter
        self.fps_counter = FPSCounter(alpha=0.85)
        
        # camera
        self.camera = ht301_hacklib.Camera()

        # one-time calibration at startup
        try:
            self.camera.calibrate()
        except Exception as e:
            self.get_logger().warn(f"Initial calibrate() failed: {e}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_image = self.create_publisher(Image, "image_raw", qos)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", qos)

        period = 1.0 / max(0.1, self.fps)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"HT301 publishing on {self.get_namespace()}/image_raw @ {self.fps:.1f} FPS, "
            f"out={self.out_w}x{self.out_h}, cmap={self.colormap_index}, rot={self.orientation}, landscape={self.landscape}"
        )

    def _make_camera_info(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = int(self.out_w)
        msg.height = int(self.out_h)

        # Unknown calibration -> simple centered intrinsics (fine for streaming)
        msg.k = [1.0, 0.0, msg.width / 2.0,
                 0.0, 1.0, msg.height / 2.0,
                 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, msg.width / 2.0, 0.0,
                 0.0, 1.0, msg.height / 2.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg

    def _tick(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from HT301")
            return

        current_fps = self.fps_counter.tick()
        
        info, _ = self.camera.info()

        # 1) exposure normalization -> uint8
        frame_u8 = auto_scale(frame, self.p_low, self.p_high)

        # 2) colormap
        cmap = COLORMAPS[self.colormap_index]
        frame_color = cv2.applyColorMap(frame_u8, cmap)

        # 3) rotate
        frame_color = rotate_frame(frame_color, self.orientation)

        # 4) resize to requested output
        if self.landscape:
            frame_color = fit_to_landscape(frame_color, self.out_w, self.out_h)
        else:
            frame_color = cv2.resize(frame_color, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

        # 5) overlays
        if self.overlay_bar:
            draw_temperature_bar(frame_color, info["Tmin_C"], info["Tmax_C"], cmap)

        # Approximate point mapping into the output image
        src_w = self.camera.width
        src_h = self.camera.height
        out_h, out_w = frame_color.shape[:2]

        def map_point(p):
            x, y = p
            xn = float(x) / max(1.0, (src_w - 1))
            yn = float(y) / max(1.0, (src_h - 1))
            return int(round(xn * (out_w - 1))), int(round(yn * (out_h - 1)))

        if self.overlay_min:
            min_xy = map_point(info["Tmin_point"])
            draw_marker_and_label(
                frame_color,
                min_xy[0],
                min_xy[1],
                f"MIN {info['Tmin_C']:.1f}C",
                (255, 170, 170),
                self.overlay_text,
            )

        if self.overlay_max:
            max_xy = map_point(info["Tmax_point"])
            draw_marker_and_label(
                frame_color,
                max_xy[0],
                max_xy[1],
                f"MAX {info['Tmax_C']:.1f}C",
                (170, 210, 255),
                self.overlay_text,
            )
        
        if self.overlay_center:
            c_xy = map_point(info["Tcenter_point"])
            draw_marker_and_label(frame_color, c_xy[0], c_xy[1], f"CTR {info['Tcenter_C']:.1f}C",
                                  (255, 255, 255), self.overlay_text)

        # if self.overlay_fps:
        #     font = cv2.FONT_HERSHEY_SIMPLEX
        #     fscale = max(0.45, min(1.0, frame_color.shape[0] / 650.0))
        #     thick = 1 if frame_color.shape[0] < 720 else 2
        #     cv2.putText(frame_color, f"FPS target: {self.fps:.0f}",
        #                 (10, 24), font, fscale, (255, 255, 255), thick, cv2.LINE_AA)
        if self.overlay_fps and current_fps is not None:
            font = cv2.FONT_HERSHEY_SIMPLEX
            fscale = max(0.45, min(1.0, frame_color.shape[0] / 650.0))
            thick = 1 if frame_color.shape[0] < 720 else 2

            cv2.putText(
                frame_color,
                f"FPS: {current_fps:.1f}",
                (10, 24),
                font,
                fscale,
                (255, 255, 255),
                thick,
                cv2.LINE_AA,
            )
        stamp = self.get_clock().now().to_msg()
        self.pub_image.publish(cv_to_imgmsg_bgr8(frame_color, stamp, self.frame_id))
        self.pub_info.publish(self._make_camera_info(stamp))


def main():
    rclpy.init()
    node = HT301ThermalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.camera.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
