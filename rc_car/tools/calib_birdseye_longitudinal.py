# -*- coding: utf-8 -*-
"""버드아이 세로(전방) 캘리브레이션 — 검출 행(y_px) → 차량 기준 전방거리(m).

🔴 2026-08-06 현재 이 값은 **미측정**이다. 횡방향(VEHICLE_AXIS_PX=477)만 정차 실측으로
   맞춰져 있고(오차 1.1cm), 세로는 한 번도 잰 적이 없다.

필요한 이유: 비전 회전 트리거("횡단보도가 화면 하단에 오면 회전 개시")를 미터 단위로
   해석·시뮬하려면 화면 하단 끝이 차 앞 몇 m인지 알아야 한다.
   ※ 트리거 자체는 픽셀 행으로 정의하면 이 값 없이도 동작한다 — 시뮬·튜닝 효율용이다.

구하는 식:  d_ahead(m) = (Y_NEAR - y_px)/PPM + D0        (PPM=250, Y_NEAR=222.87)
   미지수는 D0 (차량 기준점 → 버드아이 하단 끝 거리) 하나뿐이다.

■ 방법 A — 실측 5분 (가장 확실. 차가 있으면 이걸 하라)
   ① 정지선 앞에 차를 세우고 줄자로 앞범퍼~정지선 = 정확히 0.30m
   ② vision_runner를 점검 프로필(--stream-port 8090)로 켜고 stop_line 박스의 y값을 읽는다
   ③ D0 = 0.30 - (Y_NEAR - y_px)/250      (0.50m 에서 한 번 더 = 검산)

■ 방법 B — 로그 자동 역산 (이 스크립트)
   맵 stop_line_zones(정지선 실좌표) 대비 그 시각 GPS pose의 전방거리를 정답으로 두고,
   같은 시각 프레임의 stop_line 검출 행과 맞춘다.

   ⚠️ 2026-08-05 데이터로는 **실패했다** (회귀 기울기 부호가 기하 예상과 반대, RMS 23.5cm /
      소멸이벤트 표본 4건에 -8cm~+80cm 산포). 원인은 **pose+heading이 '차선 매칭 실패'
      WARNING 줄에만 기록**되어 정상 주행 heading 시계열이 없었기 때문이다.
      → pose+heading 정상 로깅(수신 주기 10Hz)으로 바꾼 뒤 다시 돌리면 유효하다.

사용법:
    python tools/calib_birdseye_longitudinal.py <veh.log> <vision_trace.jsonl>
    (인자 없으면 자료/주행데이터/0805 의 조합으로 실행 — 실패 재현용)
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]  # rc_car 의 부모 (map 과 형제)
MAP_YAML = ROOT / "map/main_track_map.yaml"

PPM = 250.0            # birdseye.json pixels_per_meter
VEHICLE_AXIS_PX = 477  # 차량 진행축 열 (횡방향 캘리브 완료)
Y_NEAR = 222.87        # 지면 사다리꼴 하단 끝 행 (birdseye.json full_ground 기하)
KST = 9 * 3600


def load_stop_zones() -> dict[str, tuple[float, float]]:
    txt = MAP_YAML.read_text(encoding="utf-8")
    body = txt[txt.index("stop_line_zones:"):]
    zones, cur = {}, None
    for line in body.split("\n")[1:]:
        if re.match(r"^  [a-z_0-9]+:", line):
            cur = line.strip().rstrip(":")
            zones[cur] = {}
        elif cur and "expected_center" in line:
            v = re.findall(r"-?\d+\.?\d*", line.split(":", 1)[1])
            zones[cur]["c"] = (float(v[0]), float(v[1]))
        elif line and not line.startswith("  "):
            break
    return {k: v["c"] for k, v in zones.items() if "c" in v}


def load_poses(path: Path):
    """(초, x, y, heading). heading 있는 줄 우선, 없으면 직전 heading 유지."""
    poses, last_h = [], None
    p1 = re.compile(r"^(\d\d):(\d\d):(\d\d).*?\((\d+\.\d+), ?(\d+\.\d+)\).*?heading=(\d+\.?\d*)")
    p2 = re.compile(r"^(\d\d):(\d\d):(\d\d).*?pose=\((\d+\.\d+),(\d+\.\d+)\)")
    for line in path.read_text(encoding="utf-8", errors="replace").split("\n"):
        m = p1.search(line)
        if m:
            hh, mm, ss, x, y, h = m.groups()
            last_h = float(h)
            poses.append((int(hh)*3600+int(mm)*60+int(ss), float(x), float(y), float(h)))
            continue
        m = p2.search(line)
        if m and last_h is not None:
            hh, mm, ss, x, y = m.groups()
            poses.append((int(hh)*3600+int(mm)*60+int(ss), float(x), float(y), last_h))
    poses.sort()
    return poses


def pose_at(poses, t):
    lo, hi = 0, len(poses)
    while lo < hi:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return None
    p = poses[lo - 1]
    return p if abs(p[0] - t) <= 1 else None


def main():
    args = sys.argv[1:]
    if len(args) >= 2:
        pairs = [(Path(args[0]), Path(args[1]))]
    else:
        d = ROOT / "자료/주행데이터/0805"
        pairs = [(d / "veh_0805_board_final.log", d / "vision_drive4_trace.jsonl"),
                 (d / "veh_0805_evening.log", d / "vision_drive3_trace.jsonl")]

    zones = load_stop_zones()
    print(f"정지선 존 {len(zones)}개")
    samples = []
    for logp, tracep in pairs:
        if not logp.exists() or not tracep.exists():
            print(f"  {logp.name} / {tracep.name}: 파일 없음, 건너뜀")
            continue
        poses = load_poses(logp)
        frames = [json.loads(l) for l in tracep.read_text(encoding="utf-8").split("\n") if l.strip()]
        hit = 0
        for fr in frames:
            p = pose_at(poses, int((fr["timestamp"] + KST) % 86400))
            if not p:
                continue
            _, x, y, hdg = p
            ux, uy = math.cos(math.radians(hdg)), math.sin(math.radians(hdg))
            best = None
            for zx, zy in zones.values():
                dx, dy = zx - x, zy - y
                fwd, lat = dx*ux + dy*uy, -dx*uy + dy*ux
                if 0.05 < fwd < 1.20 and abs(lat) < 0.20 and (best is None or fwd < best):
                    best = fwd
            if best is None:
                continue
            cands = [d for d in fr["model"]["detections"]
                     if d["cls"] == "stop_line" and abs(d["center_px"][0] - VEHICLE_AXIS_PX)/PPM < 0.20]
            if not cands:
                continue
            samples.append((max(c["center_px"][1] for c in cands), best))
            hit += 1
        print(f"  {tracep.name}: 조인 {hit} / {len(frames)}프레임")

    n = len(samples)
    print(f"\n표본 {n}개")
    if n < 20:
        print("→ 표본 부족. 방법 A(실측) 또는 pose 로깅(10Hz) 후 재시도.")
        return
    ys = [s[0] for s in samples]
    ds = [s[1] for s in samples]
    my, md = sum(ys)/n, sum(ds)/n
    slope = sum((a-my)*(b-md) for a, b in zip(ys, ds)) / sum((a-my)**2 for a in ys)
    inter = md - slope*my
    rms = (sum((b - (slope*a+inter))**2 for a, b in zip(ys, ds))/n) ** .5
    print(f"회귀: d_ahead = {slope:.6f}*y_px + {inter:.4f}   (RMS {rms*100:.1f}cm)")
    print(f"  기하 예상 기울기 = -1/PPM = {-1/PPM:.6f}")
    if slope > 0 or rms > 0.10:
        print("  ⚠️ 신뢰 불가 — 기울기 부호가 기하와 반대이거나 잔차가 너무 크다.")
        print("     pose+heading 정상 로깅(10Hz) 후 재시도하거나, 방법 A(실측)로 갈 것.")
        return
    print(f"  → D0(차 → 버드아이 하단끝) = {slope*Y_NEAR + inter:.3f} m")


if __name__ == "__main__":
    main()
