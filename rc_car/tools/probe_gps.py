"""위치 서버 측정 도구 — A3·A4 실측용 (명세서 §28 단계 2).

`config/network.yaml`의 실제 설정과 `PoseValidator`를 그대로 써서 서버에 붙고,
수신 통계를 낸다. 차량 로직은 돌리지 않는다 — 값을 보기 위한 도구다.

    python tools/probe_gps.py                       # config의 서버로 10초
    python tools/probe_gps.py --seconds 30          # 노이즈 측정은 길게
    python tools/probe_gps.py --url wss://127.0.0.1:8000/ws/v1/localization
    python tools/probe_gps.py --marker-id 999       # close 4404 확인

읽는 법
  - **실효 Hz / dt**: A4. 요청 주기(interval_ms)보다 느리면 카메라 FPS가 상한이다.
    같은 captured_at 반복은 `out_of_order`로 잡히므로 그 비율이 곧 중복 프레임 비율.
  - **heading 평균**: A3. 차량을 맵 +x 방향으로 정렬해 두고 재면 그 값이 곧
    `marker_yaw_offset_deg`(부호 반대)다. +y 정렬 시 90°가 나와야 한다.
  - **표준편차·최대 튐**: 차량을 세워 둔 채 재면 `max_jump_m`·`max_heading_jump_deg`
    임계값이 실제 노이즈보다 충분히 큰지 판단할 수 있다.
"""
import argparse
import json
import math
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.config import load_config                       # noqa: E402
from localization.pose_validator import PoseValidator     # noqa: E402
from network.localization_client import LocalizationClient  # noqa: E402


class Probe:
    def __init__(self, cfg: dict, map_cfg: dict, show_raw: int):
        self._validator = PoseValidator(cfg, map_cfg["x_max"], map_cfg["y_max"])
        self._lock = threading.Lock()
        self._show_raw = show_raw
        self.received = 0
        self.reasons: dict[str, int] = {}
        self.poses: list = []
        self.first_monotonic: float | None = None
        self.last_monotonic: float | None = None

    def on_message(self, raw: str) -> None:
        now = time.monotonic()
        result = self._validator.validate(raw, now)
        with self._lock:
            self.received += 1
            if self.first_monotonic is None:
                self.first_monotonic = now
            self.last_monotonic = now
            if self.received <= self._show_raw:
                print(f"  RAW[{self.received}] {raw[:150]}")
            if result.accepted:
                self.poses.append(result.pose)
                self.reasons["accepted"] = self.reasons.get("accepted", 0) + 1
            else:
                self.reasons[result.reason] = self.reasons.get(result.reason, 0) + 1

    # ---------- 통계 ----------

    def report(self, elapsed: float) -> None:
        with self._lock:
            received, reasons, poses = self.received, dict(self.reasons), list(self.poses)
            span = ((self.last_monotonic - self.first_monotonic)
                    if self.first_monotonic is not None else 0.0)

        print("\n" + "=" * 62)
        if received == 0:
            print("수신 0건 - 접속 실패이거나 서버가 push하지 않았다.")
            print("위 로그에 '위치 서버 접속·구독'이 없으면 접속 자체가 안 된 것이다.")
            print("=" * 62)
            return

        rate = received / span if span > 0 else 0.0
        print(f"수신 {received}건 / {elapsed:.1f}초  →  실효 {rate:.1f} Hz")
        print("판정: " + ", ".join(f"{k}={v}" for k, v in sorted(reasons.items())))

        if len(poses) < 2:
            print("채택 pose가 2건 미만 - 마커가 안 보이거나 검증에서 걸린다.")
            print("=" * 62)
            return

        # A4 — 서버 timestamp(촬영 시각) 간격이 실제 위치 갱신 주기다
        dts = [b.source_timestamp_s - a.source_timestamp_s for a, b in zip(poses, poses[1:])]
        print(f"\n[A4] 촬영 timestamp 간격  평균 {statistics.mean(dts) * 1000:.0f}ms "
              f"/ 최소 {min(dts) * 1000:.0f} / 최대 {max(dts) * 1000:.0f}"
              f"  →  실질 갱신 {1 / statistics.mean(dts):.1f} Hz")

        # 수신~채택 지연은 잴 수 없다(서버·차량 시계가 다름). 대신 도착 간격을 본다
        arrivals = [b.received_monotonic_s - a.received_monotonic_s
                    for a, b in zip(poses, poses[1:])]
        print(f"     차량 도착 간격        평균 {statistics.mean(arrivals) * 1000:.0f}ms "
              f"/ 최대 {max(arrivals) * 1000:.0f}ms")

        xs = [p.x for p in poses]
        ys = [p.y for p in poses]
        hs = [p.heading_deg for p in poses]
        print(f"\n[위치] x {statistics.mean(xs):.4f} ± {statistics.pstdev(xs):.4f} m"
              f"  (범위 {min(xs):.3f}~{max(xs):.3f})")
        print(f"       y {statistics.mean(ys):.4f} ± {statistics.pstdev(ys):.4f} m"
              f"  (범위 {min(ys):.3f}~{max(ys):.3f})")

        # heading은 0/360 순환이라 원형 평균으로 낸다 — 359°와 1°의 평균은 0°다
        sin_sum = sum(math.sin(math.radians(h)) for h in hs)
        cos_sum = sum(math.cos(math.radians(h)) for h in hs)
        mean_h = math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0
        devs = [abs((h - mean_h + 180.0) % 360.0 - 180.0) for h in hs]
        steps = [abs((b - a + 180.0) % 360.0 - 180.0) for a, b in zip(hs, hs[1:])]
        print(f"\n[A3] heading 평균 {mean_h:.2f}°  (편차 최대 {max(devs):.2f}°)")

        jumps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:])]
        print(f"[임계] 프레임간 최대 이동 {max(jumps) * 100:.2f}cm  "
              f"(max_jump_m = {self._validator._max_jump_m * 100:.0f}cm)")
        print(f"       프레임간 최대 회전 {max(steps):.2f}°  "
              f"(max_heading_jump_deg = {self._validator._max_heading_jump_deg:.0f}°)")
        print("=" * 62)


def main() -> int:
    ap = argparse.ArgumentParser(description="위치 서버 수신 통계")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--url", help="config의 websocket_url 대신 사용할 주소")
    ap.add_argument("--marker-id", type=int, help="config의 marker_id 대신 사용할 ID")
    ap.add_argument("--raw", type=int, default=3, help="원문을 몇 건까지 출력할지")
    args = ap.parse_args()

    # 윈도우 콘솔(cp949)에서 매핑 불가 문자가 측정 결과 출력을 죽이지 않게 한다
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

    import logging
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)-7s [%(name)s] %(message)s")

    config = load_config()
    cfg = dict(config.network["localization"])
    if args.url:
        cfg["websocket_url"] = args.url
    if args.marker_id is not None:
        cfg["marker_id"] = args.marker_id

    print(f"대상 {cfg['websocket_url']}  marker_id={cfg['marker_id']} "
          f"interval_ms={cfg['interval_ms']}  ({args.seconds:.0f}초)\n")

    probe = Probe(cfg, config.network["map"], args.raw)
    client = LocalizationClient(cfg, on_message=probe.on_message,
                                on_disconnected=lambda: None)
    started = time.monotonic()
    client.start()
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    client.stop()
    probe.report(time.monotonic() - started)
    return 0 if probe.received else 1


if __name__ == "__main__":
    raise SystemExit(main())
