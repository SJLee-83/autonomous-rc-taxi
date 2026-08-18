"""fit_frame_transform — 맵 실측 기준점으로 서버→맵 아핀 변환 적합 (워크로그 §0-32).

기준점 절차: 프로브 마커를 도색 특징점(맵 좌표를 아는 곳)에 놓고 서버 보고 좌표를
읽어 대응쌍을 만든다. 3쌍 이상(권장 4쌍+검증 1쌍)을 넣으면 p_map = A·p_server + t 를
최소자승으로 적합하고, network.yaml 의 frame_transform 블록을 그대로 출력한다.

    python tools/fit_frame_transform.py \
        --pair 3.01,2.01,3.2729,1.9739 \
        --pair 1.99,2.01,2.2367,1.9678 \
        --pair 3.01,0.99,3.2543,0.9209 \
        --pair 1.99,0.99,2.3624,0.9133

pair 형식: map_x,map_y,server_x,server_y (m). 잔차(residual)가 크면(>3cm) 해당
기준점의 마커 배치나 카메라 정합(같은-x 교차검증)을 의심한다.
"""
import argparse
import math
import sys


def _solve3(mat, vec):
    """3x3 연립방정식 가우스 소거 (numpy 없이 — 보드·PC 어디서든 실행)."""
    m = [row[:] + [v] for row, v in zip(mat, vec)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("기준점이 한 직선 위 - 퍼진 3점 이상 필요")
        m[col], m[piv] = m[piv], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    return [m[i][3] / m[i][i] for i in range(3)]


def _fit_axis(s_pts, targets):
    """targets ≈ c0·s_x + c1·s_y + c2 최소자승 (정규방정식)."""
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for (sx, sy), v in zip(s_pts, targets):
        row = (sx, sy, 1.0)
        for i in range(3):
            for j in range(3):
                ata[i][j] += row[i] * row[j]
            atb[i] += row[i] * v
    return _solve3(ata, atb)


def main() -> int:
    ap = argparse.ArgumentParser(description="서버→맵 아핀 변환 적합")
    ap.add_argument("--pair", action="append", required=True,
                    metavar="map_x,map_y,server_x,server_y",
                    help="기준점 대응쌍 (반복 지정, 3개 이상)")
    args = ap.parse_args()

    pairs = []
    for raw in args.pair:
        v = [float(s) for s in raw.split(",")]
        if len(v) != 4:
            print(f"pair 형식 오류: {raw!r} (map_x,map_y,server_x,server_y)")
            return 2
        pairs.append(v)
    if len(pairs) < 3:
        print(f"대응쌍 {len(pairs)}개 - 아핀 적합에는 3개 이상 필요 (권장 4+)")
        return 2

    s_pts = [(p[2], p[3]) for p in pairs]                # 서버 좌표 (입력)
    cx = _fit_axis(s_pts, [p[0] for p in pairs])         # map_x = cx·[s_x, s_y, 1]
    cy = _fit_axis(s_pts, [p[1] for p in pairs])
    a = [[cx[0], cx[1]], [cy[0], cy[1]]]
    t = [cx[2], cy[2]]

    res_mm = []
    for (sx, sy), p in zip(s_pts, pairs):
        fx = a[0][0] * sx + a[0][1] * sy + t[0]
        fy = a[1][0] * sx + a[1][1] * sy + t[1]
        res_mm.append(math.hypot(p[0] - fx, p[1] - fy) * 1000.0)

    scale_x = math.hypot(a[0][0], a[1][0])               # 서버 x축이 맵에서 갖는 길이
    scale_y = math.hypot(a[0][1], a[1][1])
    rot_deg = math.degrees(math.atan2(a[1][0], a[0][0]))

    print(f"대응쌍 {len(pairs)}개 적합 결과")
    for i, (p, r) in enumerate(zip(pairs, res_mm), 1):
        print(f"  P{i} map({p[0]:.3f},{p[1]:.3f}) <- server({p[2]:.3f},{p[3]:.3f})"
              f"  잔차 {r:.1f}mm")
    print(f"  잔차 최대 {max(res_mm):.1f}mm / 평균 {sum(res_mm) / len(res_mm):.1f}mm")
    print(f"  스케일 x {scale_x:.4f} / y {scale_y:.4f}   회전 {rot_deg:+.2f} deg")
    if max(res_mm) > 30.0:
        print("  [경고] 잔차 3cm 초과 - 기준점 재확인 또는 카메라 정합(같은-x 교차검증) 필요")

    print("\nnetwork.yaml localization.frame_transform 에 붙여넣기:")
    print("  frame_transform:")
    print(f"    a: [[{a[0][0]:.6f}, {a[0][1]:.6f}], [{a[1][0]:.6f}, {a[1][1]:.6f}]]")
    print(f"    t: [{t[0]:.6f}, {t[1]:.6f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
