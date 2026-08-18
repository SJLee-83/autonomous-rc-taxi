"""TurnTable — 차선별 회전 트리거 표 (2026-08-06 재설계 §0-45).

config/turn_table.yaml 을 읽어 회전 하나마다 "언제 꺾고, 얼마나 꺾고, 언제 그만두나"를
결정된 값(ResolvedTurn)으로 내놓는다. 근거·값의 지위는 yaml 주석에 있다.

핵심 설계
    - 트리거 시점은 **픽셀 비율**로 정의한다. 세로 캘리브레이션(픽셀행→전방거리 m)이
      아직 없기 때문이다(§0-45 ③). 캘리브가 생겨도 이 정의는 그대로 쓸 수 있다.
    - 조향각 기본값은 커넥터 반경 기하에서 온다: atan(축거/R). 표에서 덮어쓸 수 있다.
      (원호를 **추종**하지는 않는다 — 반경은 초기 조향각 산출에만 쓴다)
    - 표에 없는 회전은 defaults + 좌표 폴백으로 떨어진다. 즉 표가 비어도 주행은 된다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from core.exceptions import ConfigError
from navigation import lane_route

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "turn_table.yaml"

_TRIGGER_CLASSES = ("crosswalk", "stop_line", "direction_arrow", "dashed_line")


@dataclass(frozen=True)
class TriggerSpec:
    """비전 2차 트리거 — 어떤 노면표시가 얼마나 가까이 오면 꺾기 시작하나."""
    cls: str | None                 # None = 비전 트리거 없음 (좌표 폴백만)
    min_conf: float
    near_row_frac: float            # 낮을수록 일찍 발화 (표식이 멀 때)
    max_lateral_m: float

    @property
    def uses_vision(self) -> bool:
        return self.cls is not None


@dataclass(frozen=True)
class ResolvedTurn:
    """회전 하나에 대해 완전히 결정된 파라미터."""
    key: str
    maneuver: str                   # left | right | straight
    arm_distance_m: float
    start_offset_m: float
    wheel_deg: float                # 부호 포함 (+우 / −좌). straight 은 0
    exit_align_deg: float
    min_turn_m: float
    max_turn_m: float
    arc_complete_m: float | None     # 이 거리를 돌면 정렬과 무관하게 정상 종료 (None = 사용 안 함)
    fallback_at_lane_end: bool
    fallback_lead_m: float          # 좌표 폴백 개시를 차선 끝보다 이만큼 앞당긴다 (0 = 차선 끝)
    fuse_next: bool                 # True = **다음 회전까지 한 번에 이어 돈다** (중간 종료 판정 없음)
    trigger: TriggerSpec
    from_table: bool                # True = yaml turns 에서 값을 얻었다 (직접 or 대칭 상속)
    source_key: str | None = None   # 값을 실제로 가져온 키. key 와 다르면 **대칭 상속**이다

    @property
    def inherited(self) -> bool:
        """대칭 상대의 튜닝값을 물려받았는가 — 로그에서 구분하기 위한 것."""
        return self.from_table and self.source_key is not None and self.source_key != self.key

    @property
    def is_turn(self) -> bool:
        """교차로 직진(straight)은 회전 상태기계를 타지 않는다 — 그냥 다음 다리로 이어 달린다."""
        return self.maneuver in ("left", "right")


def _merge(base: dict, over: dict | None) -> dict:
    """turns 항목을 defaults 위에 한 겹만 덮는다 (trigger 는 중첩이라 따로)."""
    out = dict(base)
    if not over:
        return out
    for k, v in over.items():
        if k == "trigger" and isinstance(v, dict):
            merged = dict(base.get("trigger") or {})
            merged.update(v)
            out["trigger"] = merged
        else:
            out[k] = v
    return out


class TurnTable:
    def __init__(self, raw: dict, wheelbase_m: float, max_wheel_deg: float):
        block = raw.get("turn_table")
        if not isinstance(block, dict):
            raise ConfigError("turn_table.yaml: 'turn_table' 섹션 없음")
        self._defaults = block.get("defaults") or {}
        self._turns = block.get("turns") or {}
        if not isinstance(self._turns, dict):
            raise ConfigError("turn_table.yaml: 'turn_table.turns' 는 매핑이어야 함")
        self._wheelbase = float(wheelbase_m)
        self._max_wheel = float(max_wheel_deg)
        self._validate()

    # ---------- 생성 ----------

    @classmethod
    def load(cls, wheelbase_m: float, max_wheel_deg: float,
             path: Path | None = None) -> "TurnTable":
        p = path or CONFIG_PATH
        if not p.exists():
            raise ConfigError(f"설정 파일 없음: {p}")
        with open(p, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict):
            raise ConfigError(f"{p.name}: 최상위가 매핑이 아님")
        return cls(raw, wheelbase_m, max_wheel_deg)

    # ---------- 조회 ----------

    @property
    def tuned_keys(self) -> tuple[str, ...]:
        """개별 항목이 있는 회전 키 — 나머지는 defaults + 좌표 폴백으로 간다."""
        return tuple(str(k) for k in self._turns)

    def resolve(self, turn) -> ResolvedTurn:
        """navigation.lane_route.Turn → ResolvedTurn.

        조회 순서: ① 이 회전의 직접 항목 → ② **대칭 회전의 항목**(상속) → ③ defaults.

        ②가 사용자가 제시한 핵심이다 — 맵이 180° 점대칭이라 대칭 쌍은 도로 형태가 합동이고,
        따라서 **한쪽을 튜닝하면 반대쪽이 공짜로 따라온다**. 회전 48개 → 튜닝 클래스 24종.
        """
        key = turn.key
        entry = self._turns.get(key)
        source_key = key if entry is not None else None
        if entry is None:
            mirror = lane_route.symmetric_key(key)
            if mirror is not None:
                entry = self._turns.get(mirror)
                if entry is not None:
                    source_key = mirror
        cfg = _merge(self._defaults, entry)
        trig_cfg = cfg.get("trigger") or {}

        wheel = cfg.get("wheel_deg")
        if wheel is None:
            wheel = self._geometric_wheel_deg(turn.radius_m)
        else:
            wheel = abs(float(wheel))
        if turn.maneuver == "right":
            wheel = +wheel                      # ControlCommand 부호: + = 우측
        elif turn.maneuver == "left":
            wheel = -wheel
        else:
            wheel = 0.0
        wheel = max(-self._max_wheel, min(self._max_wheel, wheel))

        return ResolvedTurn(
            key=key,
            maneuver=turn.maneuver,
            arm_distance_m=float(cfg.get("arm_distance_m", 0.6)),
            start_offset_m=float(cfg.get("start_offset_m", 0.0)),
            wheel_deg=wheel,
            exit_align_deg=float(cfg.get("exit_align_deg", 12.0)),
            min_turn_m=float(cfg.get("min_turn_m", 0.10)),
            max_turn_m=float(cfg.get("max_turn_m", 1.20)),
            arc_complete_m=(None if cfg.get("arc_complete_m") is None
                            else float(cfg["arc_complete_m"])),
            fuse_next=bool(cfg.get("fuse_next", False)),
            fallback_at_lane_end=bool(cfg.get("fallback_at_lane_end", True)),
            fallback_lead_m=float(cfg.get("fallback_lead_m", 0.0)),
            trigger=TriggerSpec(
                cls=trig_cfg.get("class"),
                min_conf=float(trig_cfg.get("min_conf", 0.45)),
                near_row_frac=float(trig_cfg.get("near_row_frac", 0.80)),
                max_lateral_m=float(trig_cfg.get("max_lateral_m", 0.30))),
            from_table=entry is not None,
            source_key=source_key)

    def _geometric_wheel_deg(self, radius_m: float | None) -> float:
        """호 반경에서 필요한 바퀴각 — atan(축거/R). 반경 없으면 풀락 직전으로 둔다."""
        if not radius_m or radius_m <= 0:
            return self._max_wheel
        return math.degrees(math.atan(self._wheelbase / float(radius_m)))

    # ---------- 검증 ----------

    def _validate(self) -> None:
        for name, cfg in [("defaults", self._defaults)] + list(self._turns.items()):
            trig = (cfg or {}).get("trigger") or {}
            cls = trig.get("class")
            if cls is not None and cls not in _TRIGGER_CLASSES:
                raise ConfigError(
                    f"turn_table.yaml: '{name}.trigger.class' 는 {_TRIGGER_CLASSES} 중 하나 "
                    f"또는 null (받은 값: {cls!r}) — 모델 클래스 4종과 일치해야 한다")
            frac = trig.get("near_row_frac")
            if frac is not None and not (0.0 < float(frac) <= 1.0):
                raise ConfigError(
                    f"turn_table.yaml: '{name}.trigger.near_row_frac' 는 0 초과 1 이하 "
                    f"(버드아이 높이 대비 비율, 1.0 = 하단 끝). 받은 값: {frac!r}")
        for key in self._turns:
            if "->" not in str(key):
                raise ConfigError(
                    f"turn_table.yaml: turns 키는 '출발번호->도착번호' 형식이어야 함: {key!r}")
