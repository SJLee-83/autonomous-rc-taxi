#!/usr/bin/env python3
"""Server-free IMX708 lens correction, bird's-eye transform, and lane extraction."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class LaneExtraction:
    yellow_mask: np.ndarray
    dash_mask: np.ndarray
    stop_mask: np.ndarray
    overlay: np.ndarray
    metrics: dict[str, int]


@dataclass(frozen=True)
class ExtractionResult:
    undistorted: np.ndarray
    birdseye: np.ndarray
    lanes: LaneExtraction | None


class BirdseyeExtractor:
    """Reusable processor. Construct once, then call process() for every frame."""

    def __init__(self, calibration_dir: str | Path | None = None) -> None:
        root = Path(__file__).resolve().parent
        calibration_dir = (
            root / "calibration"
            if calibration_dir is None
            else Path(calibration_dir)
        )
        lens_path = calibration_dir / "calibration.json"
        ground_path = calibration_dir / "birdseye.json"
        self.lens = json.loads(lens_path.read_text(encoding="utf-8"))
        self.ground = json.loads(ground_path.read_text(encoding="utf-8"))

        image_size = self.lens.get("image_size")
        if (
            not isinstance(image_size, list)
            or len(image_size) != 2
            or any(int(value) <= 0 for value in image_size)
        ):
            raise ValueError("invalid image_size in calibration.json")
        self.image_width = int(image_size[0])
        self.image_height = int(image_size[1])

        ground_lens = self.ground.get("lens_calibration", {}).get("image_size")
        if ground_lens != image_size:
            raise ValueError(
                "birdseye.json was made for a different calibration image size"
            )

        self.homography = np.asarray(
            self.ground["homography"],
            dtype=np.float64,
        )
        if self.homography.shape != (3, 3):
            raise ValueError("invalid homography in birdseye.json")
        output_size = self.ground.get("output_size")
        if not isinstance(output_size, list) or len(output_size) != 2:
            raise ValueError("invalid output_size in birdseye.json")
        self.output_width = int(output_size[0])
        self.output_height = int(output_size[1])
        if self.output_width <= 0 or self.output_height <= 0:
            raise ValueError("bird's-eye output size must be positive")

        camera_matrix = np.asarray(
            self.lens["camera_matrix"],
            dtype=np.float64,
        )
        dist_coeffs = np.asarray(
            self.lens["dist_coeffs"],
            dtype=np.float64,
        ).reshape(-1, 1)
        size = (self.image_width, self.image_height)
        model = str(self.lens["selected_model"])
        if model == "fisheye":
            new_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                camera_matrix,
                dist_coeffs,
                size,
                np.eye(3),
                balance=1.0,
                new_size=size,
                fov_scale=1.05,
            )
            self.map_x, self.map_y = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix,
                dist_coeffs,
                np.eye(3),
                new_matrix,
                size,
                cv2.CV_16SC2,
            )
        else:
            new_matrix, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix,
                dist_coeffs,
                size,
                1.0,
                size,
            )
            self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                camera_matrix,
                dist_coeffs,
                None,
                new_matrix,
                size,
                cv2.CV_16SC2,
            )

    @property
    def input_size(self) -> tuple[int, int]:
        return self.image_width, self.image_height

    @property
    def output_size(self) -> tuple[int, int]:
        return self.output_width, self.output_height

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        extract_lanes: bool = True,
    ) -> ExtractionResult:
        """Process one BGR frame captured at the calibrated 2304x1296 mode."""
        if not isinstance(frame_bgr, np.ndarray):
            raise TypeError("frame_bgr must be a numpy array")
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must have shape (height, width, 3)")
        actual_size = (int(frame_bgr.shape[1]), int(frame_bgr.shape[0]))
        if actual_size != self.input_size:
            raise ValueError(
                f"frame size {actual_size} does not match calibration "
                f"{self.input_size}; do not resize before processing"
            )

        undistorted = cv2.remap(
            frame_bgr,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        birdseye = cv2.warpPerspective(
            undistorted,
            self.homography,
            self.output_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        lanes = extract_lane_regions(birdseye) if extract_lanes else None
        return ExtractionResult(
            undistorted=undistorted,
            birdseye=birdseye,
            lanes=lanes,
        )


def _copy_component_pixels(
    source_mask: np.ndarray,
    labels: np.ndarray,
    component_id: int,
    destination: np.ndarray,
) -> None:
    component = labels == component_id
    destination[component] = source_mask[component]


def extract_lane_regions(bird: np.ndarray) -> LaneExtraction:
    """Extract yellow paint, white dashes, and horizontal stop lines."""
    height, width = bird.shape[:2]
    hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bird, cv2.COLOR_BGR2LAB)
    valid_ground = np.max(bird, axis=2) > 22

    yellow_hsv = cv2.inRange(
        hsv,
        np.array([11, 38, 42], dtype=np.uint8),
        np.array([43, 255, 255], dtype=np.uint8),
    )
    blue, green, red = cv2.split(bird)
    yellow_lab = (
        (lab[:, :, 2] >= 130)
        | (
            (blue.astype(np.int16) + 12 < green.astype(np.int16))
            & (blue.astype(np.int16) + 12 < red.astype(np.int16))
        )
    ).astype(np.uint8) * 255
    yellow = cv2.bitwise_and(yellow_hsv, yellow_lab)
    yellow[~valid_ground] = 0
    yellow = cv2.morphologyEx(
        yellow,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    yellow = cv2.morphologyEx(
        yellow,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7)),
    )

    gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)
    local_gray = cv2.GaussianBlur(gray, (0, 0), 7.0)
    local_contrast = gray.astype(np.int16) - local_gray.astype(np.int16)
    neutral = hsv[:, :, 1] <= 125
    safe_ground = cv2.erode(
        valid_ground.astype(np.uint8) * 255,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
    ) > 0
    white = (
        neutral
        & safe_ground
        & (
            ((gray >= 112) & (lab[:, :, 0] >= 105) & (local_contrast >= 4))
            | ((gray >= 68) & (lab[:, :, 0] >= 72) & (local_contrast >= 11))
        )
    ).astype(np.uint8) * 255
    white = cv2.morphologyEx(
        white,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
    )

    hough_source = cv2.morphologyEx(
        white,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )
    raw_lines = cv2.HoughLinesP(
        hough_source,
        1,
        np.pi / 180.0,
        threshold=16,
        minLineLength=max(16, int(height * 0.08)),
        maxLineGap=max(8, int(width * 0.012)),
    )
    horizontal_lines: list[tuple[float, tuple[int, int, int, int]]] = []
    vertical_lines: list[tuple[float, tuple[int, int, int, int]]] = []
    if raw_lines is not None:
        for raw_line in np.asarray(raw_lines).reshape(-1, 4):
            x1, y1, x2, y2 = (int(value) for value in raw_line)
            delta_x = x2 - x1
            delta_y = y2 - y1
            length = float(np.hypot(delta_x, delta_y))
            angle = abs(float(np.degrees(np.arctan2(delta_y, delta_x))))
            angle = min(angle, 180.0 - angle)
            line = (x1, y1, x2, y2)
            if angle <= 14.0 and length >= max(32.0, width * 0.035):
                horizontal_lines.append((length, line))
            if angle >= 50.0 and length >= max(20.0, height * 0.10):
                vertical_lines.append((length, line))

    stop_mask = np.zeros_like(white)
    stop_lines: list[tuple[int, int, int, int]] = []
    for _, line in sorted(horizontal_lines, reverse=True):
        center_y = (line[1] + line[3]) / 2.0
        if center_y > height * 0.72:
            continue
        duplicate = any(
            abs(center_y - (existing[1] + existing[3]) / 2.0) < 8.0
            for existing in stop_lines
        )
        if not duplicate:
            stop_lines.append(line)
    if stop_lines:
        stop_support = np.zeros_like(white)
        for line in stop_lines:
            cv2.line(stop_support, line[0:2], line[2:4], 7, cv2.LINE_AA)
        stop_mask = cv2.bitwise_and(
            white,
            cv2.dilate(
                stop_support,
                cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5)),
            ),
        )

    long_dash_lines: list[tuple[int, int, int, int]] = []
    for _, line in sorted(vertical_lines, reverse=True):
        center_x = (line[0] + line[2]) / 2.0
        duplicate = any(
            abs(center_x - (existing[0] + existing[2]) / 2.0) < 18.0
            for existing in long_dash_lines
        )
        if not duplicate:
            long_dash_lines.append(line)

    long_dash_support = np.zeros_like(white)
    for line in long_dash_lines:
        cv2.line(long_dash_support, line[0:2], line[2:4], 7, cv2.LINE_AA)
    long_dash_mask = cv2.bitwise_and(
        white,
        cv2.dilate(
            long_dash_support,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        ),
    )

    white_exclusion = cv2.bitwise_or(
        cv2.dilate(
            stop_mask,
            cv2.getStructuringElement(cv2.MORPH_RECT, (9, 5)),
        ),
        cv2.dilate(
            long_dash_mask,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 9)),
        ),
    )
    white_without_stop = cv2.bitwise_and(
        white,
        cv2.bitwise_not(white_exclusion),
    )
    vertical = cv2.morphologyEx(
        white_without_stop,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7)),
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        vertical,
        connectivity=8,
    )
    dash_mask = np.zeros_like(white)
    dash_components = 0
    dash_boxes: list[tuple[int, int, int, int]] = []
    for component_id in range(1, count):
        x, y, component_width, component_height, area = stats[component_id]
        aspect = component_height / max(1.0, float(component_width))
        fill = area / max(1.0, float(component_width * component_height))
        if (
            area >= 8
            and area <= max(240, int(width * height * 0.003))
            and component_width <= max(14, int(width * 0.035))
            and component_height <= max(30, int(height * 0.22))
            and aspect >= 1.55
            and fill >= 0.32
        ):
            _copy_component_pixels(
                white_without_stop,
                labels,
                component_id,
                dash_mask,
            )
            dash_components += 1
            dash_boxes.append(
                (
                    int(x),
                    int(y),
                    int(component_width),
                    int(component_height),
                )
            )

    dash_mask = cv2.bitwise_or(dash_mask, long_dash_mask)
    dash_components += len(long_dash_lines)

    overlay = np.clip(bird.astype(np.float32) * 0.32, 0, 255).astype(np.uint8)
    for mask, color in (
        (yellow, np.array([0, 215, 255], dtype=np.float32)),
        (dash_mask, np.array([255, 220, 0], dtype=np.float32)),
        (stop_mask, np.array([40, 60, 255], dtype=np.float32)),
    ):
        selected = mask > 0
        if np.any(selected):
            overlay[selected] = np.clip(
                overlay[selected].astype(np.float32) * 0.08 + color * 0.92,
                0,
                255,
            ).astype(np.uint8)

    yellow_contours, _ = cv2.findContours(
        yellow,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    yellow_contours = [
        contour
        for contour in yellow_contours
        if cv2.contourArea(contour) >= 5.0
    ]
    cv2.drawContours(
        overlay,
        yellow_contours,
        -1,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    for x, y, box_width, box_height in dash_boxes:
        cv2.rectangle(
            overlay,
            (x, y),
            (x + box_width, y + box_height),
            (255, 220, 0),
            1,
        )
    for line in long_dash_lines:
        cv2.line(overlay, line[0:2], line[2:4], (255, 220, 0), 2, cv2.LINE_AA)
    for line in stop_lines:
        cv2.line(overlay, line[0:2], line[2:4], (40, 60, 255), 3, cv2.LINE_AA)

    metrics = {
        "yellow_pixels": int(cv2.countNonZero(yellow)),
        "dash_pixels": int(cv2.countNonZero(dash_mask)),
        "stop_pixels": int(cv2.countNonZero(stop_mask)),
        "dash_components": dash_components,
        "stop_components": len(stop_lines),
    }
    return LaneExtraction(
        yellow_mask=yellow,
        dash_mask=dash_mask,
        stop_mask=stop_mask,
        overlay=overlay,
        metrics=metrics,
    )
