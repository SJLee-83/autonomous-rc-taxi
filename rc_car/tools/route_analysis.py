# -*- coding: utf-8 -*-
"""노선·회전 분석 도구 (2026-08-06 신설 — 알고리즘 전면 재설계 논의 산출물).

배경: 주행을 "가상 라인 트레이싱"에서 **차선 번호 노선 + 비전 회전 트리거**로 전환하면서,
      ① 번호 노선이 맵 그래프에서 성립하는지 ② 어떤 회전을 몇 번 튜닝해야 하는지
      ③ 후보 경로를 무엇으로 잡아야 튜닝 부담이 최소인지를 숫자로 답하기 위해 만들었다.

사용법 (rc_car 폴더 기준 어디서든):
    python tools/route_analysis.py route 2 17 11 24 16 31 5   # 번호 노선 검증
    python tools/route_analysis.py pairs                      # 시연 장소쌍 회전 구성
    python tools/route_analysis.py turns                      # 회전 데이터 재고(로그 기준)

핵심 상수 (2026-08-06 확정):
    좌회전 커넥터 24개 = 전부 R0.35  → 차량 최소회전반경 0.243m 대비 여유 44% (안전)
    우회전 커넥터 24개 = 전부 R0.26  → 여유 7% (위험). 시연 장소쌍 28/30이 이걸 요구한다.
"""
from __future__ import annotations

import collections
import glob
import heapq
import itertools
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]        # rc_car 의 부모 (map 과 형제)
MAP_YAML = ROOT / "map/main_track_map.yaml"
DRIVE_DATA = ROOT / "자료/주행데이터"

MIN_TURN_RADIUS_M = 0.243        # 차량 실측 최소 회전반경
RISK_RADIUS_M = 0.30             # 이보다 작은 반경 = 위험 회전

# 차선 번호 ↔ ID (자료/차선번호맵_0805.html 과 같은 순번 — 도로별 그룹, 방향 무관)
NUM2ID = {
    1: "top_outer_wb_e", 2: "top_outer_wb_w", 3: "top_inner_eb_w", 4: "top_inner_eb_e",
    5: "mid_wb1_e", 6: "mid_wb1_w", 7: "mid_wb2_e", 8: "mid_wb2_w",
    9: "mid_eb1_w", 10: "mid_eb1_e", 11: "mid_eb2_w", 12: "mid_eb2_e",
    13: "bot_inner_wb_e", 14: "bot_inner_wb_w", 15: "bot_outer_eb_w", 16: "bot_outer_eb_e",
    17: "left_outer_sb_n", 18: "left_outer_sb_s", 19: "left_inner_nb_s", 20: "left_inner_nb_n",
    21: "vert_sb1_n", 22: "vert_sb1_s", 23: "vert_sb2_n", 24: "vert_sb2_s",
    25: "vert_nb1_s", 26: "vert_nb1_n", 27: "vert_nb2_s", 28: "vert_nb2_n",
    29: "right_inner_sb_n", 30: "right_inner_sb_s", 31: "right_outer_nb_s", 32: "right_outer_nb_n",
}
ID2NUM = {v: k for k, v in NUM2ID.items()}

# 시연 장소 → 정차 차선 (map/places.yaml)
PLACES = {
    "우리집": "top_inner_eb_w", "보건소": "mid_wb1_e", "경로당": "mid_eb2_w",
    "시장": "mid_eb2_e", "면사무소": "bot_inner_wb_w", "은행": "bot_inner_wb_e",
}


# ---------- 맵 파싱 (pyyaml 비의존 — 필요한 필드만) ----------

def parse_map():
    """(lanes, connectors). lanes[id]['succ'] = 커넥터 이름들, connectors[id]['successor'] = 도착 차선."""
    lanes, conns = {}, {}
    section = cur = None
    for ln in MAP_YAML.read_text(encoding="utf-8").split("\n"):
        if re.match(r"^[a-z_]+:", ln):
            section, cur = ln.split(":")[0], None
            continue
        m = re.match(r"^  ([a-z_0-9]+):\s*$", ln)
        if m:
            cur = m.group(1)
            if section == "lanes":
                lanes[cur] = {"succ": [], "cl": []}
            elif section == "connectors":
                conns[cur] = {"cl": []}
            continue
        if cur is None:
            continue
        d = lanes.get(cur) if section == "lanes" else conns.get(cur)
        if d is None:
            continue
        if re.match(r"^    (successors|centerline):", ln):
            continue
        m = re.match(r"^      - (.+)$", ln)
        if m:
            v = m.group(1).strip()
            if v.startswith("["):
                n = [float(z) for z in re.findall(r"-?\d+\.?\d*", v)]
                d["cl"].append((n[0], n[1]))
            else:
                d.setdefault("succ", []).append(v)
            continue
        for key in ("successor", "maneuver", "radius_m", "kind", "speed_limit_mps"):
            m = re.match(rf"^    {key}:\s*(.+)$", ln)
            if m:
                d[key] = m.group(1).strip()
    return lanes, conns


def build_adj(lanes, conns):
    """차선 → [(도착차선, 커넥터명, 기동, 반경)]"""
    adj = {}
    for lid, l in lanes.items():
        for cn in l.get("succ", []):
            c = conns.get(cn)
            if c and c.get("successor"):
                adj.setdefault(lid, []).append(
                    (c["successor"], cn, c.get("maneuver", "?"), float(c.get("radius_m") or 0)))
    return adj


def is_risky(radius_m: float) -> bool:
    return 0 < radius_m < RISK_RADIUS_M


# ---------- ① 번호 노선 검증 ----------

def cmd_route(nums):
    lanes, conns = parse_map()
    adj = build_adj(lanes, conns)
    nums = [int(n) for n in nums] or [2, 17, 11, 24, 16, 31, 5]
    print("검증 노선:", " → ".join(map(str, nums)), "\n")
    ok = True
    for a, b in zip(nums, nums[1:]):
        ida, idb = NUM2ID[a], NUM2ID[b]
        hop = next((h for h in adj.get(ida, []) if h[0] == idb), None)
        if hop:
            _, cn, man, rad = hop
            mark = "⚠" if is_risky(rad) else " "
            print(f"  {a:2d} {ida:18s} → {b:2d} {idb:18s}  ✅ {man:9s} R={rad or '-':<6}{mark} ({cn})")
        else:
            ok = False
            outs = sorted({ID2NUM[h[0]] for h in adj.get(ida, []) if h[0] in ID2NUM})
            print(f"  {a:2d} {ida:18s} → {b:2d} {idb:18s}  ❌ 직접 연결 없음 (갈 수 있는 곳: {outs})")
    print("\n" + ("노선 성립" if ok else "노선 불성립 — 중간 차선 누락"))
    man = collections.Counter(c.get("maneuver", "?") for c in conns.values())
    print("커넥터 기동 분포:", dict(man))


# ---------- ② 장소쌍 회전 구성 ----------

def best_route(adj, a, b):
    """위험 회전 최소화 우선, 그 다음 회전 수 최소."""
    pq, best = [(0.0, 0, a, [])], {}
    while pq:
        cost, _, cur, path = heapq.heappop(pq)
        if cur == b:
            return path
        if cur in best and best[cur] <= cost:
            continue
        best[cur] = cost
        for nxt, cn, man, rad in adj.get(cur, []):
            c = 0.2 if man == "straight" else (10.0 if is_risky(rad) else 1.0)
            heapq.heappush(pq, (cost + c, len(path) + 1, nxt, path + [(cn, man, rad, cur, nxt)]))
    return None


def cmd_pairs():
    lanes, conns = parse_map()
    adj = build_adj(lanes, conns)
    print("=" * 84)
    print("시연 장소쌍별 최적 경로 (위험 우회전 R<0.30 최소화)")
    print("=" * 84)
    print(f"{'출발':<7}{'도착':<7}{'차선 노선':<32}{'좌':>4}{'우(위험)':>9}")
    results = {}
    for a, b in itertools.permutations(PLACES, 2):
        p = best_route(adj, PLACES[a], PLACES[b])
        if p is None:
            print(f"{a:<7}{b:<7}  경로 없음")
            continue
        turns = [(ID2NUM[s], ID2NUM[e], man) for cn, man, rad, s, e in p
                 if man in ("left", "right")]
        nl = sum(1 for t in turns if t[2] == "left")
        nr = sum(1 for t in turns if t[2] == "right")
        seq = [ID2NUM[PLACES[a]]] + [ID2NUM[e] for cn, man, rad, s, e in p]
        results[(a, b)] = (set(turns), nl, nr)
        print(f"{a:<7}{b:<7}{'→'.join(map(str, seq)):<32}{nl:>4}{nr:>9}")

    for k in (2, 3):
        print(f"\n--- 경로 {k}개 조합: 튜닝할 회전 종류가 최소인 것 ---")
        combos = []
        for ps in itertools.combinations(results, k):
            union = set()
            for p in ps:
                union |= results[p][0]
            if not union or len({x for p in ps for x in p}) < k + 1:
                continue
            combos.append((sum(1 for t in union if t[2] == "right"), len(union), ps, union))
        combos.sort()
        seen, shown = set(), 0
        for nr, n, ps, union in combos:
            key = tuple(sorted(union))
            if key in seen:
                continue
            seen.add(key)
            td = " ".join(f"{s}→{e}{'우' if m == 'right' else '좌'}" for s, e, m in sorted(union))
            print(f"  회전 {n}종 (위험 {nr}) | " + " / ".join(f"{a}→{b}" for a, b in ps))
            print(f"       {td}")
            shown += 1
            if shown >= 5:
                break


# ---------- ③ 회전 데이터 재고 ----------

def cmd_turns():
    lanes, conns = parse_map()
    adj = build_adj(lanes, conns)

    def shortest(a, b):
        q, seen = collections.deque([(a, [])]), {a}
        while q:
            cur, path = q.popleft()
            for nxt, cn, man, rad in adj.get(cur, []):
                if nxt in seen:
                    continue
                if nxt == b:
                    return path + [(cn, man, cur, nxt)]
                seen.add(nxt)
                q.append((nxt, path + [(cn, man, cur, nxt)]))
        return None

    routes = {}
    for f in sorted(glob.glob(str(DRIVE_DATA / "*/veh_*.log"))):
        txt = Path(f).read_text(encoding="utf-8", errors="replace")
        for m in re.finditer(r"(\d\d:\d\d:\d\d).*?경로 확정: (\S+?)\(s=[\d.]+\) → (\S+?)\(s=[\d.]+\)", txt):
            tm, a, b = m.groups()
            routes[(Path(f).parent.name, tm)] = (re.sub(r"_[ab]$", "", a), re.sub(r"_[ab]$", "", b))

    print("=" * 84)
    print("경로 확정 로그 → 계획된 회전 (완주 여부는 별개)")
    print("=" * 84)
    count = collections.Counter()
    for (day, tm), (a, b) in sorted(routes.items()):
        p = shortest(a, b)
        if p is None:
            continue
        turns = [(ID2NUM.get(s), ID2NUM.get(e), man) for cn, man, s, e in p
                 if man in ("left", "right")]
        desc = " ".join(f"{s}→{e}({'좌' if m == 'left' else '우'})" for s, e, m in turns) or "회전없음"
        print(f"  {day} {tm}  {ID2NUM.get(a, '?'):>2}→{ID2NUM.get(b, '?'):<2}  {desc}")
        count.update(turns)
    print("\n  회전별 누적:")
    for (s, e, m), n in count.most_common():
        print(f"    {s:>2} → {e:<2}  {'좌회전' if m == 'left' else '우회전'}  {n}회")
    print("\n  ⚠ 비전 트레이스가 함께 있는 회전은 11→22(우)·22→14(우)·7→22(좌) 3종뿐")
    print("     (0805 drive3 18:20~18:34 / drive4 19:00~19:49 구간에서만 녹화)")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "pairs"
    {"route": lambda: cmd_route(sys.argv[2:]),
     "pairs": cmd_pairs,
     "turns": cmd_turns}.get(cmd, cmd_pairs)()
