# -*- coding: utf-8 -*-
"""차선 번호 맵 생성기 (2026-08-06 신설).

`자료/차선번호맵_0805.html` 은 일회성으로 만들어져 생성기가 남아 있지 않았다.
이 도구는 그 맵을 **재현**하면서 시연 장소 6곳(call_point / stop_point)을 함께 찍는다.

배경: 알고리즘을 "차선 번호 노선 + 비전 회전 트리거"로 전환하면서(§0-45),
      노선을 번호로 설계하려면 "어느 장소가 몇 번 차선에 붙어 있는지"를 눈으로 봐야 한다.

좌표 변환 (기존 0805 맵과 동일 — 픽셀 단위까지 일치 확인):
    px = MARGIN + SCALE * x_m
    py = H - MARGIN - SCALE * y_m          (맵 원점 bottom_left, 5.0m × 3.0m)

번호 체계는 tools/route_analysis.py 의 NUM2ID 를 그대로 import 한다 (단일 소스).

사용법:
    python tools/build_lane_number_map.py                 # 자료/차선번호맵_0806.html
    python tools/build_lane_number_map.py --out 경로.html
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_analysis import ID2NUM, NUM2ID  # noqa: E402  번호 체계 단일 소스

ROOT = Path(__file__).resolve().parents[2]          # rc_car 의 부모 (map 과 형제)
MAP_YAML = ROOT / "map/main_track_map.yaml"
PLACES_YAML = ROOT / "map/places.yaml"
DEFAULT_OUT = ROOT / "자료/차선번호맵_0806.html"

SCALE = 280.0        # px/m
MARGIN = 40          # px
W, H = 1480, 920     # 5.0m·3.0m + 여백

# 도로 그룹 (번호 구간 → 이름·색). 0805 맵의 색을 그대로 유지한다.
GROUPS = [
    ((1, 4), "상단 가로", "#1f77b4"),
    ((5, 12), "중앙 가로", "#d62728"),
    ((13, 16), "하단 가로", "#2ca02c"),
    ((17, 20), "좌측 세로", "#9467bd"),
    ((21, 28), "중앙 세로", "#ff7f0e"),
    ((29, 32), "우측 세로", "#8c564b"),
]

C_CALL = "#ffffff"   # 승객 호출 지점 (블록 위)
C_STOP = "#ffd400"   # 차량 정차 지점 (차선 위)


def group_of(num: int):
    for (lo, hi), name, color in GROUPS:
        if lo <= num <= hi:
            return name, color
    return "?", "#888"


# ---------- 좌표 변환 ----------

def px(x_m: float) -> float:
    return MARGIN + SCALE * x_m


def py(y_m: float) -> float:
    return H - MARGIN - SCALE * y_m


def pt(p) -> str:
    return f"{px(p[0]):.0f},{py(p[1]):.0f}"


# ---------- YAML 파싱 (pyyaml 비의존 — 이 저장소 도구 관례) ----------

def _nums(s: str):
    """주석을 떼고 숫자만 뽑는다 (주석에 날짜가 들어 있어 반드시 먼저 제거)."""
    return [float(z) for z in re.findall(r"-?\d+\.?\d*", s.split("#")[0])]


def parse_map():
    """(lanes, connectors, blocks) — 렌더에 필요한 필드만."""
    lanes, conns, blocks = {}, {}, {}
    section = cur = listkey = None
    for ln in MAP_YAML.read_text(encoding="utf-8").split("\n"):
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        if re.match(r"^[a-z_]+:", ln):
            section, cur, listkey = ln.split(":")[0], None, None
            continue
        m = re.match(r"^  ([a-z_0-9]+):\s*$", ln)
        if m:
            cur, listkey = m.group(1), None
            {"lanes": lanes, "connectors": conns, "blocks": blocks}.get(
                section, {})[cur] = {"cl": []}
            continue
        if cur is None:
            continue
        bucket = {"lanes": lanes, "connectors": conns, "blocks": blocks}.get(section)
        if bucket is None or cur not in bucket:
            continue
        m = re.match(r"^    ([a-z_0-9]+):\s*(.*)$", ln)
        if m:
            key, val = m.group(1), m.group(2).strip()
            listkey = key if val == "" else None
            if val:
                bucket[cur][key] = val.split("#")[0].strip()
            continue
        m = re.match(r"^      - (.+)$", ln)          # 정확히 6칸 — 더 깊은 목록은 무시
        if m and listkey in ("centerline", "polygon"):
            n = _nums(m.group(1))
            if len(n) >= 2:
                bucket[cur]["cl"].append((n[0], n[1]))
    return lanes, conns, blocks


def parse_places():
    places, cur = {}, None
    inside = False
    for ln in PLACES_YAML.read_text(encoding="utf-8").split("\n"):
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        if re.match(r"^[a-z_]+:", ln):
            inside = ln.split(":")[0] == "places"
            cur = None
            continue
        if not inside:
            continue
        m = re.match(r"^  ([a-z_0-9]+):\s*$", ln)
        if m:
            cur = m.group(1)
            places[cur] = {"key": cur}
            continue
        if cur is None:
            continue
        m = re.match(r"^    ([a-z_0-9]+):\s*(.+)$", ln)
        if m:
            key, val = m.group(1), m.group(2).split("#")[0].strip()
            if key in ("call_point", "stop_point"):
                places[cur][key] = tuple(_nums(val)[:2])
            else:
                places[cur][key] = val
    return places


# ---------- SVG 조각 ----------

def arrow_head(p_prev, p_end, color: str) -> str:
    """차선 끝에 진행 방향 삼각형."""
    x0, y0, x1, y1 = px(p_prev[0]), py(p_prev[1]), px(p_end[0]), py(p_end[1])
    ang = math.atan2(y1 - y0, x1 - x0)
    size, spread = 11.0, 0.42
    a = (x1 - size * math.cos(ang - spread), y1 - size * math.sin(ang - spread))
    b = (x1 - size * math.cos(ang + spread), y1 - size * math.sin(ang + spread))
    return (f'<polygon points="{x1:.0f},{y1:.0f} {a[0]:.0f},{a[1]:.0f} {b[0]:.0f},{b[1]:.0f}" '
            f'fill="{color}" stroke="{color}" stroke-width="1"/>')


def grid() -> str:
    """0.5m 격자 + 1.0m 축 눈금 (좌표 감각용)."""
    out = []
    x = 0.0
    while x <= 5.0001:
        major = abs(x - round(x)) < 1e-6
        out.append(f'<line x1="{px(x):.0f}" y1="{py(3.0):.0f}" x2="{px(x):.0f}" y2="{py(0.0):.0f}" '
                   f'stroke="#3a4048" stroke-width="{1 if major else 0.5}" '
                   f'opacity="{0.55 if major else 0.3}"/>')
        if major:
            out.append(f'<text x="{px(x):.0f}" y="{py(0.0)+22:.0f}" fill="#8b93a1" font-size="12" '
                       f'text-anchor="middle">{x:.0f}</text>')
        x += 0.5
    y = 0.0
    while y <= 3.0001:
        major = abs(y - round(y)) < 1e-6
        out.append(f'<line x1="{px(0.0):.0f}" y1="{py(y):.0f}" x2="{px(5.0):.0f}" y2="{py(y):.0f}" '
                   f'stroke="#3a4048" stroke-width="{1 if major else 0.5}" '
                   f'opacity="{0.55 if major else 0.3}"/>')
        if major:
            out.append(f'<text x="{px(0.0)-12:.0f}" y="{py(y)+4:.0f}" fill="#8b93a1" font-size="12" '
                       f'text-anchor="end">{y:.0f}</text>')
        y += 0.5
    return "".join(out)


def place_marker(p) -> str:
    """호출점(블록) → 정차점(차선) 한 쌍 + 이름표."""
    call, stop = p["call_point"], p["stop_point"]
    lane_num = ID2NUM.get(p.get("stop_lane"), "?")
    hd = float(p.get("heading_deg", 0))
    cx, cy, sx, sy = px(call[0]), py(call[1]), px(stop[0]), py(stop[1])
    out = [f'<line x1="{cx:.0f}" y1="{cy:.0f}" x2="{sx:.0f}" y2="{sy:.0f}" stroke="{C_STOP}" '
           f'stroke-width="1.5" stroke-dasharray="4 3" opacity="0.6"/>']
    # 호출점 — 승객이 서는 곳
    out.append(f'<circle cx="{cx:.0f}" cy="{cy:.0f}" r="6" fill="{C_CALL}" stroke="#111" stroke-width="1.5"/>')
    # 정차점 — 마름모 + heading 화살표
    d = 9
    out.append(f'<polygon points="{sx:.0f},{sy-d} {sx+d},{sy:.0f} {sx:.0f},{sy+d} {sx-d},{sy:.0f}" '
               f'fill="{C_STOP}" stroke="#111" stroke-width="1.5"/>')
    ang = math.radians(hd)
    hx, hy = sx + 30 * math.cos(ang), sy - 30 * math.sin(ang)
    out.append(f'<line x1="{sx:.0f}" y1="{sy:.0f}" x2="{hx:.0f}" y2="{hy:.0f}" stroke="{C_STOP}" stroke-width="2.5"/>')
    ax, ay = sx + 22 * math.cos(ang), sy - 22 * math.sin(ang)
    out.append(f'<polygon points="{hx:.0f},{hy:.0f} '
               f'{ax - 6*math.sin(ang):.0f},{ay - 6*math.cos(ang):.0f} '
               f'{ax + 6*math.sin(ang):.0f},{ay + 6*math.cos(ang):.0f}" fill="{C_STOP}"/>')
    # 이름표 — heading 화살표 **반대쪽**에 둔다 (화살표가 글씨를 관통하지 않게)
    right = math.cos(ang) < 0
    tx = sx + (18 if right else -18)
    anchor = "start" if right else "end"
    label = f'{p.get("label", p["key"])} · {lane_num}번'
    out.append(f'<text x="{tx:.0f}" y="{sy+5:.0f}" fill="{C_STOP}" font-size="16" font-weight="bold" '
               f'text-anchor="{anchor}" stroke="#20242a" stroke-width="4" paint-order="stroke">{label}</text>')
    out.append(f'<text x="{tx:.0f}" y="{sy+22:.0f}" fill="#c8cdd6" font-size="12" '
               f'text-anchor="{anchor}" stroke="#20242a" stroke-width="3" paint-order="stroke">'
               f'({stop[0]:.2f}, {stop[1]:.2f}) {hd:.0f}°</text>')
    return "".join(out)


# ---------- 본문 ----------

def build() -> str:
    lanes, conns, blocks = parse_map()
    places = parse_places()

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'style="background:#20242a;font-family:Consolas,monospace">']
    svg.append(grid())

    for b in blocks.values():
        if b["cl"]:
            svg.append(f'<polygon points="{" ".join(pt(p) for p in b["cl"])}" fill="#2f5d34" opacity="0.55"/>')

    for c in conns.values():                       # 이음매/회전 — 배경
        if len(c["cl"]) >= 2:
            svg.append(f'<polyline points="{" ".join(pt(p) for p in c["cl"])}" fill="none" '
                       f'stroke="#555" stroke-width="2" opacity="0.6"/>')

    for lid, l in lanes.items():                   # 차선 + 번호
        num = ID2NUM.get(lid)
        if num is None or len(l["cl"]) < 2:
            continue
        _, color = group_of(num)
        svg.append(f'<polyline points="{" ".join(pt(p) for p in l["cl"])}" fill="none" '
                   f'stroke="{color}" stroke-width="4" opacity="0.9"/>')
        svg.append(arrow_head(l["cl"][-2], l["cl"][-1], color))
        ex, ey = px(l["cl"][-1][0]), py(l["cl"][-1][1])
        svg.append(f'<circle cx="{ex:.0f}" cy="{ey:.0f}" r="13" fill="#111" stroke="{color}" stroke-width="2"/>')
        svg.append(f'<text x="{ex:.0f}" y="{ey+5:.0f}" fill="#fff" font-size="15" font-weight="bold" '
                   f'text-anchor="middle">{num}</text>')

    for p in places.values():                      # 장소 6곳 — 최상단
        svg.append(place_marker(p))

    svg.append("</svg>")

    # 장소 표
    rows = []
    for p in sorted(places.values(), key=lambda q: ID2NUM.get(q.get("stop_lane"), 99)):
        num = ID2NUM.get(p.get("stop_lane"), "?")
        _, color = group_of(num if isinstance(num, int) else 0)
        rows.append(
            f"<tr><td style='color:{C_STOP};font-weight:bold'>{p.get('label')}</td>"
            f"<td style='color:{color};font-weight:bold'>{num}</td>"
            f"<td>{p.get('stop_lane')}</td>"
            f"<td>{p['call_point'][0]:.2f}, {p['call_point'][1]:.2f}</td>"
            f"<td>{p['stop_point'][0]:.2f}, {p['stop_point'][1]:.2f}</td>"
            f"<td>{p.get('heading_deg')}°</td>"
            f"<td>{p.get('block')} / {p.get('curb')}</td></tr>")

    grp = " · ".join(f"<span style='color:{c}'>{lo}~{hi} {n}</span>" for (lo, hi), n, c in GROUPS)

    # 범례는 SVG 밖(HTML)에 둔다 — 지도 안에 넣으면 외곽 차선 번호와 겹친다
    legend = (
        "<p style='font-size:14px'>"
        f"<svg width='16' height='16' style='vertical-align:-3px'>"
        f"<circle cx='8' cy='8' r='6' fill='{C_CALL}' stroke='#111' stroke-width='1.5'/></svg> "
        "호출점 <span style='color:#8b93a1'>call_point — 승객이 서는 블록 위 지점</span>"
        "&nbsp;&nbsp;&nbsp;"
        f"<svg width='20' height='16' style='vertical-align:-3px'>"
        f"<polygon points='9,1 17,8 9,15 1,8' fill='{C_STOP}' stroke='#111' stroke-width='1.5'/></svg> "
        "정차점 <span style='color:#8b93a1'>stop_point — 차량이 실제 서는 차선 위 지점 "
        "(화살표 = 정차 heading)</span></p>")

    return (
        "<html><head><meta charset='utf-8'><title>차선 번호 맵 + 시연 장소 6곳</title></head>"
        "<body style='background:#181b20;color:#ddd;font-family:Consolas,monospace'>"
        "<h2>차선 번호 맵 (32차선) + 시연 장소 6곳</h2>"
        f"<p style='color:#8b93a1'>격자 0.5m · 축 눈금 1.0m · 맵 5.0m × 3.0m (원점 좌하단)<br>{grp}</p>"
        + legend + "".join(svg) +
        "<h3>시연 장소 6곳</h3>"
        "<table border='1' cellpadding='6' style='border-collapse:collapse;color:#ddd'>"
        "<tr style='background:#2a2f37'><th>장소</th><th>정차 차선</th><th>차선 ID</th>"
        "<th>call_point</th><th>stop_point</th><th>heading</th><th>블록/연석</th></tr>"
        + "".join(rows) + "</table>"
        "<p style='color:#8b93a1'>생성: rc_car/tools/build_lane_number_map.py "
        "(map/main_track_map.yaml + map/places.yaml)</p>"
        "</body></html>")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    a = ap.parse_args()
    out = Path(a.out)
    out.write_text(build(), encoding="utf-8")
    print(f"wrote {out}")
