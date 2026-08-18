# -*- coding: utf-8 -*-
"""seg_replay — seg 오프라인 재처리 하네스 (2026-08-07 신설)

보드·차량 없이, 녹화된 오버레이 프레임만으로 compute_seg 변형의 가동률을 측정한다.

원리: build_overlay() 가 canvas[yellow_mask > 0] = BGR(0,210,255) 로 칠해 저장하므로
      RGB(255,210,0) 근방을 고르면 **원 노란 마스크가 복원된다**.
      dash 상자는 trace.jsonl 의 detections(xyxy_px) 에서 그대로 가져온다.
검증: 0806_run1 5,140 프레임에서 원본 코드(변형 A) 재현 — rows 99.2% / valid 100% 일치.

사용:  python seg_replay.py [프레임수]
       (기본 = 전체. 녹화 경로는 REC 상수)

🔴 주의: 이 도구는 **가동률**만 측정한다. 값의 부호·스케일 검증은 못 한다 —
   0806_run1 의 GPS 는 구 frame_transform 이라 편향이 15~18cm(차선 반폭 초과)이고
   실제 횡변동 σ 가 1~3cm 로 잡음 수준이라 기준으로 쓸 수 없다. 부호는 정지 3점 실측으로.
"""

from __future__ import annotations
import io, json, re, sys, time
from pathlib import Path

import numpy as np
from PIL import Image

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

REC = Path(r"<주행데이터경로>\0806\0806_run1")
LOG = Path(r"<주행데이터경로>\0806\veh_0806_mirror.log")

PPM = 250.0
VEHICLE_AXIS_PX = 477
LANE_W_PX = (0.17 * PPM, 0.33 * PPM)
SEG_ROWS_BASE = list(range(140, 236, 12))
NEAR_ROWS = list(range(190, 236, 6))
MIN_ROWS = 5
MAX_OFFSET_M = 0.20
LANE_HALF_PX = 0.125 * PPM
SINGLE_GATE_PX = 0.25 * PPM
OVERLAY_RGB = np.array([255, 210, 0], dtype=np.int16)
TOL = 150


def yellow_from_overlay(img):
    return (np.abs(img.astype(np.int16) - OVERLAY_RGB).sum(axis=2) < TOL)


def runs_centers(row_bool):
    cols = np.flatnonzero(row_bool)
    if cols.size == 0:
        return []
    splits = np.flatnonzero(np.diff(cols) > 4)
    return [float(s.mean()) for s in np.split(cols, splits + 1)]


def _collect(mask, dash, seg_rows, slope, single_side):
    row_max = max(seg_rows)
    pts, pair = [], 0
    for row in seg_rows:
        axis = VEHICLE_AXIS_PX + slope * (row - row_max)
        centers = runs_centers(mask[row])
        centers += [(b[0] + b[2]) / 2.0 for b in dash if b[1] <= row <= b[3]]
        centers.sort()
        best = None
        for i in range(len(centers) - 1):
            l, r = centers[i], centers[i + 1]
            if l < axis < r and LANE_W_PX[0] <= r - l <= LANE_W_PX[1]:
                if best is None or (r - l) < (best[1] - best[0]):
                    best = (l, r)
        if best:
            pts.append((row, (best[0] + best[1]) / 2.0)); pair += 1; continue
        if single_side:
            near = [c for c in centers if abs(c - axis) <= SINGLE_GATE_PX]
            if len(near) == 1:
                c = near[0]
                pts.append((row, c + LANE_HALF_PX if c < axis else c - LANE_HALF_PX))
    return pts, pair


def compute_seg(mask, dash, *, seg_rows, max_head, slope_prev=0.0,
                axis_mode="fixed", single_side=False):
    """axis_mode: fixed | tilt | best  (best = 고정·기울임 둘 다 해보고 rows 많은 쪽)"""
    if axis_mode == "fixed" or abs(slope_prev) < 1e-6:
        pts, pair = _collect(mask, dash, seg_rows, 0.0, single_side)
    elif axis_mode == "tilt":
        pts, pair = _collect(mask, dash, seg_rows, slope_prev, single_side)
    else:  # best
        p0, q0 = _collect(mask, dash, seg_rows, 0.0, single_side)
        p1, q1 = _collect(mask, dash, seg_rows, slope_prev, single_side)
        pts, pair = (p1, q1) if len(p1) > len(p0) else (p0, q0)

    if len(pts) < MIN_ROWS:
        return {"valid": False, "rows": len(pts), "pair_rows": pair, "slope": None}
    rr = np.array([p[0] for p in pts], float)
    xx = np.array([p[1] for p in pts], float)
    a, b = np.polyfit(rr, xx, 1)
    off = (VEHICLE_AXIS_PX - (a * rr.max() + b)) / PPM
    hdg = float(np.degrees(np.arctan2(a, 1.0)))
    if abs(off) > MAX_OFFSET_M or abs(hdg) > max_head:
        return {"valid": False, "rows": len(pts), "pair_rows": pair,
                "reject": True, "slope": a}
    return {"valid": True, "lateral_offset_m": off, "heading_error_deg": hdg,
            "rows": len(pts), "pair_rows": pair, "slope": a}


VARIANTS = [
    ("A 원본",                      dict(seg_rows=SEG_ROWS_BASE, max_head=10.0, axis_mode="fixed", single_side=False)),
    ("B15 게이트 15°",              dict(seg_rows=SEG_ROWS_BASE, max_head=15.0, axis_mode="fixed", single_side=False)),
    ("B20 게이트 20°",              dict(seg_rows=SEG_ROWS_BASE, max_head=20.0, axis_mode="fixed", single_side=False)),
    ("B25 게이트 25°",              dict(seg_rows=SEG_ROWS_BASE, max_head=25.0, axis_mode="fixed", single_side=False)),
    ("B30 게이트 30°",              dict(seg_rows=SEG_ROWS_BASE, max_head=30.0, axis_mode="fixed", single_side=False)),
    ("C' 축 best-of-two",           dict(seg_rows=SEG_ROWS_BASE, max_head=10.0, axis_mode="best",  single_side=False)),
    ("E 편측폴백만",                dict(seg_rows=SEG_ROWS_BASE, max_head=10.0, axis_mode="fixed", single_side=True)),
    ("B25+C'",                      dict(seg_rows=SEG_ROWS_BASE, max_head=25.0, axis_mode="best",  single_side=False)),
    ("B25+E",                       dict(seg_rows=SEG_ROWS_BASE, max_head=25.0, axis_mode="fixed", single_side=True)),
    ("B25+C'+E",                    dict(seg_rows=SEG_ROWS_BASE, max_head=25.0, axis_mode="best",  single_side=True)),
    ("B25+C'+E+근거리행",           dict(seg_rows=NEAR_ROWS,     max_head=25.0, axis_mode="best",  single_side=True)),
]

# ---- GPS (1차) ----
TS = re.compile(r"^(\d\d):(\d\d):(\d\d)\.(\d\d\d)\s+\w+\s+\[([a-z._]+)\]\s+(.*)$")
RPOSE = re.compile(r"^pose \(([-\d.]+), ([-\d.]+)\) heading=([-\d.]+)")
BASE_CLK = 14 * 3600 + 39 * 60 + 38.115
# GPS 가 건전했던 구간 (course-heading 정합 |오차|<5°) — 차선, t0, t1, 중심 y, 진행방위
GOOD = [(7, "14:54:25", "14:54:43", 1.63, 180.0),
        (14, "14:55:07", "14:55:17", 0.39, 180.0),
        (3, "14:56:25", "14:56:50", 2.61, 0.0)]


def load_gps():
    P = []
    for ln in io.open(LOG, encoding="utf-8", errors="replace"):
        mo = TS.match(ln.rstrip("\n"))
        if not mo:
            continue
        h, m, s, ms, mod, msg = mo.groups()
        t = int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000.0
        if t > 14 * 3600 + 58 * 60:
            break
        p = RPOSE.match(msg)
        if p:
            P.append((t, float(p.group(1)), float(p.group(2))))
    return P


def main():
    trace = [json.loads(l) for l in io.open(REC / "trace.jsonl", encoding="utf-8") if l.strip()]
    by_frame = {r["frame"]: r for r in trace}
    files = sorted(REC.glob("f*.jpg"))
    if len(sys.argv) > 1:
        files = files[:int(sys.argv[1])]      # 빠른 점검용 부분 실행
    P = load_gps()
    print("프레임 %d / 변형 %d / GPS 표본 %d" % (len(files), len(VARIANTS), len(P)), flush=True)

    def gps_lat(t, cy, nom):
        c = [q for q in P if q[0] <= t]
        if not c:
            return None
        _, x, y = c[-1]
        return (y - cy) if nom == 0.0 else (cy - y)

    def hms(s):
        a, b, c = s.split(":")
        return int(a) * 3600 + int(b) * 60 + int(c)

    stats = {n: {"valid": 0, "reject": 0, "norows": 0} for n, _ in VARIANTS}
    slope = {n: 0.0 for n, _ in VARIANTS}
    series = {n: [] for n, _ in VARIANTS}      # (t, offset) valid 만
    pairs = {n: [] for n, _ in VARIANTS}       # (seg_off, gps_lat) 건전 구간
    newly = {n: [] for n, _ in VARIANTS}       # A 는 무효인데 이 변형은 유효한 프레임의 offset
    a_valid = {}
    t0 = time.time()

    for i, f in enumerate(files):
        n = int(f.stem[1:])
        tr = by_frame.get(n)
        if tr is None:
            continue
        t = BASE_CLK + (tr["timestamp"] - trace[0]["timestamp"])
        img = np.asarray(Image.open(f).convert("RGB"))
        mask = yellow_from_overlay(img)
        dash = [d["xyxy_px"] for d in (tr.get("model", {}).get("detections") or [])
                if d.get("cls") == "dashed_line"]
        good = next(((cy, nom) for lane, a, b, cy, nom in GOOD if hms(a) <= t <= hms(b)), None)

        for name, kw in VARIANTS:
            res = compute_seg(mask, dash, slope_prev=slope[name], **kw)
            if res.get("slope") is not None:
                slope[name] = res["slope"]
            s = stats[name]
            if res["valid"]:
                s["valid"] += 1
                series[name].append((t, res["lateral_offset_m"]))
                if good:
                    g = gps_lat(t, good[0], good[1])
                    if g is not None:
                        pairs[name].append((res["lateral_offset_m"], g))
                if name == "A 원본":
                    a_valid[n] = True
                elif not a_valid.get(n):
                    newly[name].append(res["lateral_offset_m"])
            elif res.get("reject"):
                s["reject"] += 1
            else:
                s["norows"] += 1
        if (i + 1) % 1000 == 0:
            print("  %d/%d (%.0fs)" % (i + 1, len(files), time.time() - t0), flush=True)

    tot = sum(stats["A 원본"].values())
    base = stats["A 원본"]["valid"]
    print()
    print("## 1. 가동률 (%d 프레임)" % tot)
    print()
    print("| 변형 | **valid** | 기각 | rows 부족 | baseline 대비 | 인접 프레임 offset 점프 중앙값 |")
    print("|---|---|---|---|---|---|")
    for name, _ in VARIANTS:
        s = stats[name]
        sr = series[name]
        jumps = [abs(sr[k + 1][1] - sr[k][1]) for k in range(len(sr) - 1)
                 if 0 < sr[k + 1][0] - sr[k][0] < 0.6]
        j = float(np.median(jumps)) if jumps else float("nan")
        d = s["valid"] - base
        print("| %s | **%.1f%%** | %.1f%% | %.1f%% | %s | %.4f m |" % (
            name, 100.0 * s["valid"] / tot, 100.0 * s["reject"] / tot,
            100.0 * s["norows"] / tot,
            ("%+.1f%%p" % (100.0 * d / tot)) if d else "—", j))

    print()
    print("## 2. 값 품질 — GPS 건전 구간(차선 7·14·3) 대조")
    print()
    print("| 변형 | 표본 | 상관계수 | 기울기 | RMS 차 | 판정 |")
    print("|---|---|---|---|---|---|")
    for name, _ in VARIANTS:
        pr = pairs[name]
        if len(pr) < 20:
            print("| %s | %d | – | – | – | 표본부족 |" % (name, len(pr)))
            continue
        a = np.array([p[0] for p in pr]); b = np.array([p[1] for p in pr])
        r = float(np.corrcoef(a, b)[0, 1])
        sl = float(np.polyfit(b, a, 1)[0])
        rms = float(np.sqrt(np.mean((a + b) ** 2)))   # seg ≈ -GPS 예상
        print("| %s | %d | **%+.3f** | %+.2f | %.4f m | %s |" % (
            name, len(pr), r, sl, rms, "seg≈−GPS 일치" if r < -0.5 else ""))

    print()
    print("## 3. 새로 살아난 프레임의 값 분포 (A 무효 → 해당 변형 유효)")
    print()
    print("| 변형 | 신규 유효 | offset |중앙값| | offset |최대| |")
    print("|---|---|---|---|")
    for name, _ in VARIANTS:
        v = newly[name]
        if not v:
            continue
        av = np.abs(np.array(v))
        print("| %s | %d | %.3f m | %.3f m |" % (name, len(v), float(np.median(av)), float(av.max())))


if __name__ == "__main__":
    main()
