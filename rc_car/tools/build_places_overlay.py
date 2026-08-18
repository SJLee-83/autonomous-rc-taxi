"""build_places_overlay.py — 시연 장소 표를 지도 위에 그린다.

`map/places.yaml` (rc_car 형제)(단일 소스)에서 좌표를 읽어 `map/places_overlay.png`를 만든다.
그림을 손으로 그리지 않는 이유는 맵 YAML과 같다 — 좌표가 바뀌었는데 그림이 옛날 값이면
그림이 거짓말을 한다. 좌표를 고치면 이 스크립트를 다시 돌린다.

    python tools/build_places_overlay.py

빨강 = 승객 호출 위치(블록 위) / 하늘색 = 차량 정차점 + 진행 방향.
정차점 옆 괄호는 그 차선 조각의 끝까지 남은 거리다 — 도착 반경(0.10m)보다 커야 한다
(작으면 이음매 위 정지로 도착 판정이 안 난다. tests/unit/test_places.py 가 강제한다).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

MAP_DIR = ROOT.parent / "map"
BASE_IMAGE = ROOT.parents[1] / "자료" / "자율주행무인택시_시연지도최종안.png"
OUT = MAP_DIR / "places_overlay.png"

MAP_W_M, MAP_H_M = 5.0, 3.0
RED = (235, 70, 90)
CYAN = (70, 200, 255)
WHITE = (245, 245, 245)


def main() -> None:
    import yaml
    from PIL import Image, ImageDraw, ImageFont

    from mapping.lane_map import load_lane_map
    from navigation.destination_resolver import DestinationResolver

    with open(MAP_DIR / "places.yaml", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    lane_map = load_lane_map(MAP_DIR / "main_track_map.yaml")
    resolver = DestinationResolver(lane_map, warn_snap_distance_m=0.30)

    img = Image.open(BASE_IMAGE).convert("RGB")
    w, h = img.size
    draw = ImageDraw.Draw(img)

    def px(x: float, y: float) -> tuple[float, float]:
        return (x / MAP_W_M * w, h - y / MAP_H_M * h)

    def font(size: int):
        for name in ("malgunbd.ttf", "malgun.ttf"):
            try:
                return ImageFont.truetype(rf"C:\Windows\Fonts\{name}", size)
            except OSError:
                continue
        return ImageFont.load_default()

    f_label, f_small = font(46), font(30)

    for pid, p in doc["places"].items():
        cx, cy = px(*p["call_point"])
        sx, sy = px(*p["stop_point"])
        sp = resolver.resolve(*p["stop_point"])
        remain = lane_map.lanes[sp.lane_id].length_m - sp.progress_s

        # 호출 위치 -> 정차점 연결선
        draw.line([cx, cy, sx, sy], fill=WHITE, width=4)

        # 호출 위치 (블록 위)
        r = 16
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=RED, outline=(20, 20, 20), width=3)

        # 정차점 + 진행 방향 화살표
        draw.ellipse([sx - r, sy - r, sx + r, sy + r], fill=(20, 20, 20),
                     outline=CYAN, width=6)
        rad = math.radians(p["heading_deg"])
        ax, ay = sx + math.cos(rad) * 120, sy - math.sin(rad) * 120
        draw.line([sx, sy, ax, ay], fill=CYAN, width=9)
        nx, ny = math.cos(rad), -math.sin(rad)
        draw.polygon([(ax + nx * 34, ay + ny * 34),
                      (ax - ny * 22, ay + nx * 22),
                      (ax + ny * 22, ay - nx * 22)], fill=CYAN)

        # 라벨 — 정차점 반대편에 둔다 (마커·화살표와 겹치지 않게)
        label = p["label"]
        if sy < cy:      # 정차점이 위쪽 -> 라벨은 아래
            draw.text((cx, cy + 40), label, fill=WHITE, font=f_label, anchor="ma",
                      stroke_width=6, stroke_fill=(20, 20, 20))
        else:
            draw.text((cx, cy - 78), label, fill=WHITE, font=f_label, anchor="ms",
                      stroke_width=6, stroke_fill=(20, 20, 20))
        draw.text((sx, sy + 34), f"정차 ({p['stop_point'][0]:.2f}, {p['stop_point'][1]:.2f}) "
                                 f"[여유 {remain:.2f}m]",
                  fill=CYAN, font=f_small, anchor="ma",
                  stroke_width=5, stroke_fill=(20, 20, 20))

    # 범례
    lx, ly = 40, 40
    draw.rectangle([lx, ly, lx + 780, ly + 140], fill=(20, 20, 20))
    draw.ellipse([lx + 30, ly + 28, lx + 58, ly + 56], fill=RED)
    draw.text((lx + 76, ly + 22), "승객 호출 위치 (블록 위)", fill=WHITE, font=f_small)
    draw.ellipse([lx + 30, ly + 84, lx + 58, ly + 112], outline=CYAN, width=5)
    draw.text((lx + 76, ly + 78), "차량 정차점 + 진행 방향", fill=WHITE, font=f_small)

    img.save(OUT)
    print(f"saved: {OUT}  ({w}x{h})")
    for pid, p in doc["places"].items():
        sp = resolver.resolve(*p["stop_point"])
        remain = lane_map.lanes[sp.lane_id].length_m - sp.progress_s
        print(f"  {p['label']:5s} stop=({p['stop_point'][0]:.2f}, {p['stop_point'][1]:.2f}) "
              f"lane={sp.lane_id:16s} end_margin={remain:.3f}m")


if __name__ == "__main__":
    main()
