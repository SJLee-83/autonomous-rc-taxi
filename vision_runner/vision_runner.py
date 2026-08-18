#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vision_runner — 비전 실행체 (계약 v0.3: 카메라는 vision 소유, 임베디드는 pull).

역할 분담 (2026-08-05 사용자 확정 — 흰색 혼동 제거 개정):
  황색선 = 색 휴리스틱 (노란색만 — 색으로 유일하게 구분 가능한 대상)
  점선·정지선·횡단보도·화살표 = 세그 모델 단독 (흰색 페인트는 색으로 구분 불가)

파이프라인 (별도 프로세스, 주행 코드와 완전 분리):
  CSI 캡처 -> BirdseyeExtractor(워프만, extract_lanes=False) -> 노란선 전용 검출(경량)
  -> best.engine 세그 추론 -> seg 계산(노란선+모델 점선 쌍)
  -> /dev/shm/vision_latest.json 원자적 게시 (timestamp — 어댑터 신선도 판정)

운용 프로필 (2026-08-05 WiFi 동결 사고 후 분리):
  주행용:  --record-dir ~/vision_rec/<run>   (무선 0 — 온보드 녹화, 사후 재생)
  점검용:  --stream-port 8090                (정차 중에만 — 브라우저 실시간)

무중단 폴백: 이 프로세스가 죽으면 게시가 낡고 차량 어댑터 신선도 판정(0.5s)이
자동 invalid 처리 -> GPS 단독 주행 지속.
"""
from __future__ import annotations

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from birdseye_extractor import BirdseyeExtractor
from run_csi_direct import camera_frames, gstreamer_command

_latest_jpeg: bytes | None = None
_jpeg_lock = threading.Lock()

# ---------- 노란선 전용 검출 (birdseye_extractor 임계값 이식 — 흰색 경로 제거) ----------


def extract_yellow(bird: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bird, cv2.COLOR_BGR2LAB)
    valid_ground = np.max(bird, axis=2) > 22
    yellow_hsv = cv2.inRange(hsv, np.array([11, 38, 42], dtype=np.uint8),
                             np.array([43, 255, 255], dtype=np.uint8))
    blue, green, red = cv2.split(bird)
    yellow_lab = ((lab[:, :, 2] >= 130)
                  | ((blue.astype(np.int16) + 12 < green.astype(np.int16))
                     & (blue.astype(np.int16) + 12 < red.astype(np.int16)))
                  ).astype(np.uint8) * 255
    yellow = cv2.bitwise_and(yellow_hsv, yellow_lab)
    yellow[~valid_ground] = 0
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7)))
    return yellow


# ---------- seg 계산 (계약 §5 생산자 — 노란선(색) + 점선(모델) 쌍) ----------

PPM = 250.0                  # birdseye.json pixels_per_meter
VEHICLE_AXIS_PX = 477        # 차량 진행축 열 (V2 정차 실측으로 보정 대상)
LANE_W_PX = (0.17 * PPM, 0.33 * PPM)   # 차선 폭 0.25m ± 30%
SEG_ROWS = range(140, 236, 12)
MIN_ROWS = 5
MAX_OFFSET_M = 0.20
MAX_HEAD_DEG = 10.0   # 차선 보정은 정렬 근처에서만 의미 — 코너 시야(±26° 관측,
                      # 2026-08-05 저녁 오인)를 게이트에서 기각

# 편측 추정 (2026-08-06 — 쌍 매칭 병목 완화. 0806 주행 valid 25% -> 개선 목적)
# 한쪽 경계선만 보이는 행은 공칭 반폭으로 중심을 추정한다. 이웃 차선 경계 오인을
# 막기 위해 축에서 SINGLE_GATE_PX 안의 "유일한" 후보만 쓴다 (여럿이면 모호 -> 행 포기).
LANE_HALF_PX = 0.125 * PPM   # 공칭 차선 반폭 (0.25m)
SINGLE_GATE_PX = 0.25 * PPM  # 편측 후보 허용 반경 (중앙 주행 시 이웃 경계는 ~0.375m 밖)


def _runs_centers(row_pixels: np.ndarray) -> list[float]:
    cols = np.flatnonzero(row_pixels)
    if cols.size == 0:
        return []
    splits = np.flatnonzero(np.diff(cols) > 4)
    return [float(seg.mean()) for seg in np.split(cols, splits + 1)]


def compute_seg(yellow_mask: np.ndarray, dash_boxes: list[tuple]) -> dict:
    """행별 후보(노란선 단면 중심 + 그 행을 지나는 모델 점선 상자 중심) 중
    차량 축을 감싸는 차선 폭 간격 쌍의 중점 = 차선 중심. 쌍이 없는 행은
    편측 추정(경계선 + 공칭 반폭)으로 보충한다. 그래도 미달 = invalid."""
    pts = []
    pair_rows = 0
    for row in SEG_ROWS:
        centers = _runs_centers(yellow_mask[row])
        centers += [(x1 + x2) / 2.0 for x1, y1, x2, y2 in dash_boxes
                    if y1 <= row <= y2]
        centers.sort()
        best = None
        for i in range(len(centers) - 1):
            l, r = centers[i], centers[i + 1]
            if l < VEHICLE_AXIS_PX < r and LANE_W_PX[0] <= r - l <= LANE_W_PX[1]:
                if best is None or (r - l) < (best[1] - best[0]):
                    best = (l, r)
        if best:
            pts.append((row, (best[0] + best[1]) / 2.0))
            pair_rows += 1
            continue
        # 편측 폴백 — 축 근처의 유일한 후보를 우리 차선 경계로 보고 반폭 보정.
        # 축 왼쪽이면 좌측 경계(+반폭), 오른쪽이면 우측 경계(-반폭).
        near = [c for c in centers if abs(c - VEHICLE_AXIS_PX) <= SINGLE_GATE_PX]
        if len(near) == 1:
            c = near[0]
            mid = c + LANE_HALF_PX if c < VEHICLE_AXIS_PX else c - LANE_HALF_PX
            pts.append((row, mid))
    if len(pts) < MIN_ROWS:
        return {"valid": False, "rows": len(pts), "pair_rows": pair_rows}
    rows = np.array([p[0] for p in pts], dtype=float)
    xs = np.array([p[1] for p in pts], dtype=float)
    a, b = np.polyfit(rows, xs, 1)
    x_near = a * rows.max() + b
    offset_m = (VEHICLE_AXIS_PX - x_near) / PPM
    heading_deg = float(np.degrees(np.arctan2(a, 1.0)))
    if abs(offset_m) > MAX_OFFSET_M or abs(heading_deg) > MAX_HEAD_DEG:
        return {"valid": False, "rows": len(pts), "pair_rows": pair_rows,
                "reject": {"offset_m": round(offset_m, 3),
                           "heading_deg": round(heading_deg, 1)}}
    return {"valid": True,
            "lateral_offset_m": round(float(offset_m), 4),
            "heading_error_deg": round(heading_deg, 2),
            "rows": len(pts),
            "pair_rows": pair_rows}   # 관측용 — 쌍 몇 행 / 편측 몇 행인지 사후 분석


# ---------- 스트림 (점검용 — 주행 중 사용 금지: 2026-08-05 WiFi 동결 사고) ----------


class _StreamHandler(BaseHTTPRequestHandler):
    stream_fps = 2.0

    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path.startswith("/snap"):
            with _jpeg_lock:
                data = _latest_jpeg
            if data is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _jpeg_lock:
                    data = _latest_jpeg
                if data is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / self.stream_fps)
        except (BrokenPipeError, ConnectionResetError):
            pass


def publish_atomic(path: Path, payload: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


def build_overlay(yolo_result, yellow_mask, seg: dict | None = None) -> np.ndarray:
    canvas = yolo_result.plot()
    canvas[yellow_mask > 0] = (0, 210, 255)   # 노란선 = 주황 틴트
    # 차량 진행축 — offset 을 눈으로 읽는 기준선 (2026-08-08, 녹화 판독용)
    cv2.line(canvas, (VEHICLE_AXIS_PX, 0), (VEHICLE_AXIS_PX, canvas.shape[0] - 1),
             (200, 200, 200), 1)
    # seg 수치 오버레이 (2026-08-08, 사용자 요청 "seg 나온 값 보이게 녹화").
    # 주행 개입 없음 — 게시 payload 를 그리기만 한다. 검정 테두리 + 색 본문 이중 출력.
    if seg is not None:
        if seg.get("valid"):
            txt = "seg OK  off %+.3fm  hdg %+.1fdeg  rows %d(pair %d)" % (
                seg["lateral_offset_m"], seg["heading_error_deg"],
                seg["rows"], seg["pair_rows"])
            color = (80, 220, 80)                       # 초록 (BGR)
        else:
            txt = "seg INVALID  rows %d(pair %d)" % (
                seg.get("rows", 0), seg.get("pair_rows", 0))
            rej = seg.get("reject")
            if rej:
                txt += "  rej off %+.3f hdg %+.1f" % (
                    rej["offset_m"], rej["heading_deg"])
            color = (60, 60, 230)                       # 빨강 (BGR)
        cv2.putText(canvas, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 1, cv2.LINE_AA)
    return canvas


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=int, default=5)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--engine", default=str(Path.home() / "vision" / "best.engine"))
    ap.add_argument("--calibration-dir", type=Path,
                    default=Path(__file__).resolve().parent / "calibration")
    ap.add_argument("--publish", type=Path, default=Path("/dev/shm/vision_latest.json"))
    ap.add_argument("--record-dir", type=Path, help="주행 프로필: 오버레이 온보드 녹화")
    ap.add_argument("--record-every", type=int, default=2, help="N프레임마다 저장")
    ap.add_argument("--stream-port", type=int, default=0, help="점검 프로필 전용 (정차 중)")
    ap.add_argument("--stream-fps", type=float, default=2.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--sensor-id", type=int, default=0)
    ap.add_argument("--sensor-mode", type=int, default=1)
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.engine, task="segment")
    extractor = BirdseyeExtractor(args.calibration_dir)
    names = None

    if args.record_dir:
        args.record_dir.mkdir(parents=True, exist_ok=True)
        trace = open(args.record_dir / "trace.jsonl", "a", encoding="utf-8")
    else:
        trace = None
    if args.stream_port:
        _StreamHandler.stream_fps = args.stream_fps
        server = ThreadingHTTPServer(("0.0.0.0", args.stream_port), _StreamHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"stream: http://<board-ip>:{args.stream_port}/  (점검용 — 주행 중 금지)",
              flush=True)

    command = gstreamer_command(
        sensor_id=args.sensor_id, sensor_mode=args.sensor_mode,
        width=extractor.image_width, height=extractor.image_height, fps=args.fps)

    global _latest_jpeg
    last_log = 0.0
    for n, frame in enumerate(camera_frames(command), start=1):
        t0 = time.perf_counter()
        result = extractor.process(frame, extract_lanes=False)   # 워프만 (흰색 휴리스틱 제거)
        yellow = extract_yellow(result.birdseye)
        t1 = time.perf_counter()
        r = model.predict(source=result.birdseye, imgsz=960, conf=args.conf,
                          device=0, retina_masks=False, verbose=False)[0]
        t2 = time.perf_counter()
        if names is None:
            names = r.names

        dets, counts, dash_boxes = [], {}, []
        if r.boxes is not None:
            for b in r.boxes:
                cls = names[int(b.cls)]
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                dets.append({"cls": cls, "conf": round(float(b.conf), 3),
                             "xyxy_px": [round(x1, 1), round(y1, 1),
                                         round(x2, 1), round(y2, 1)],
                             "center_px": [round((x1 + x2) / 2, 1),
                                           round((y1 + y2) / 2, 1)]})
                counts[cls] = counts.get(cls, 0) + 1
                if cls == "dashed_line":
                    dash_boxes.append((x1, y1, x2, y2))

        payload = {
            "timestamp": time.time(),
            "frame": n,
            "proc_ms": {"extract": round((t1 - t0) * 1e3, 1),
                        "infer": round((t2 - t1) * 1e3, 1)},
            "birdseye_size": list(extractor.output_size),
            "pixels_per_meter": PPM,
            "vehicle_axis_px": VEHICLE_AXIS_PX,   # 차량 진행축 열 — 회전 트리거의 횡거리 게이트가
                                                  # 이 값을 쓴다 (2026-08-06 §0-45). 게시해 두면
                                                  # 축을 재보정해도 차량 쪽 코드가 따라온다

            "model": {"conf": args.conf, "counts": counts, "detections": dets},
            "heuristic": {"yellow_pixels": int(np.count_nonzero(yellow))},
            "seg": compute_seg(yellow, dash_boxes),
        }
        publish_atomic(args.publish, payload)

        need_jpeg = args.stream_port or (
            args.record_dir and n % args.record_every == 0)
        if need_jpeg:
            canvas = build_overlay(r, yellow, payload["seg"])
            if args.stream_port:
                ok, buf = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with _jpeg_lock:
                        _latest_jpeg = buf.tobytes()
            if args.record_dir and n % args.record_every == 0:
                cv2.imwrite(str(args.record_dir / f"f{n:06d}.jpg"), canvas,
                            [cv2.IMWRITE_JPEG_QUALITY, 70])
                trace.write(json.dumps(payload, ensure_ascii=False) + "\n")
                trace.flush()

        now = time.monotonic()
        if now - last_log >= 2.0:
            print(json.dumps({"frame": n, "counts": counts,
                              "seg": payload["seg"].get("valid"),
                              "ms": payload["proc_ms"]}, ensure_ascii=False),
                  flush=True)
            last_log = now
        if args.max_frames and n >= args.max_frames:
            break
    if trace:
        trace.close()
    print("vision_runner stopped", flush=True)


if __name__ == "__main__":
    main()
