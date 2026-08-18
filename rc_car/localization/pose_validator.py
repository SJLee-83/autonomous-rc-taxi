"""PoseValidator — 위치 수신 검증 (protocol_2 §3.3·§3.4).

순수 판정기다. 네트워크도 스레드도 모르고 직전 채택 pose만 기억한다.
통과하면 LocalizationPose를, 아니면 거부 사유를 돌려준다 — 사유별 후속 동작은
LocalizationService가 정한다 (여기서 상태를 바꾸지 않는다).

검증 순서가 곧 §3.4 규칙 순서다:
  마커 → timestamp → found → 키·타입 → 순서 역전 → 맵 범위 → 점프

큐브 모드 (network.yaml localization.cube — 워크로그 §0-31):
  실차는 정육면체(한 변 12cm) 수직 4면에 마커 6/7/8/9를 붙이고 서버는 **보이는 면을
  그대로** 보고한다. enabled=true면 ① 4면 ID 전부 수용 ② 면별 yaw 오프셋으로 차량
  전방 환산 ③ 면 좌표를 바깥 법선 반대 방향으로 center_offset_m 이동해 차량 중심으로
  보정 ④ 보이는 면이 바뀐 직후 1회는 점프 판정을 건너뛴다(면 전환 불연속 흡수).
  ⚠️ 서버가 수직 면 마커의 heading으로 "면의 바깥 법선 방향"을 준다고 가정한다 —
  부호·오프셋 값은 현장 큐브 회전 테스트로 확정한다.
"""
import json
import math
from dataclasses import dataclass

from core.models import LocalizationPose

# 거부 사유. OTHER_MARKER만 "우리 메시지가 아님"이고, 나머지는 전부 "유효 pose 없음"이다.
PARSE = "parse"
OTHER_MARKER = "other_marker"
NOT_FOUND = "not_found"
MALFORMED = "malformed"
OUT_OF_ORDER = "out_of_order"
OUT_OF_MAP = "out_of_map"
JUMP = "jump"


@dataclass(frozen=True)
class PoseResult:
    pose: "LocalizationPose | None" = None
    reason: "str | None" = None

    @property
    def accepted(self) -> bool:
        return self.pose is not None


def _is_finite_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _heading_delta_deg(a: float, b: float) -> float:
    """0~360 순환을 고려한 최단 각 차이 (359° ↔ 1° = 2°)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class PoseValidator:
    def __init__(self, cfg: dict, map_x_max: float, map_y_max: float):
        self._marker_id = cfg["marker_id"]
        self._yaw_offset = cfg["marker_yaw_offset_deg"]
        self._max_jump_m = cfg["max_jump_m"]
        self._max_heading_jump_deg = cfg["max_heading_jump_deg"]
        self._nominal_dt = cfg["interval_ms"] / 1000.0
        self._map_x = map_x_max
        self._map_y = map_y_max
        self._prev: "LocalizationPose | None" = None
        cube = cfg.get("cube") or {}
        self._cube = bool(cube.get("enabled"))
        if self._cube:
            self._faces = {int(k): float(v)
                           for k, v in cube["face_yaw_offset_deg"].items()}
            self._center_off = float(cube["center_offset_m"])
        self._prev_face: "int | None" = None
        # GPS 서버 좌표 → 맵 좌표 아핀 변환 (맵 실측 §0-32 — 기본 항등 = 무변환).
        # p_map = a·p_server + t. heading은 아핀의 회전 성분만 더한다 (이방성 스케일이
        # 만드는 각 왜곡은 1° 미만이라 무시 — 상수 편차는 cube/yaw 오프셋이 흡수).
        ft = cfg.get("frame_transform") or {}
        a = ft.get("a") or [[1.0, 0.0], [0.0, 1.0]]
        t = ft.get("t") or [0.0, 0.0]
        self._ft_a = ((float(a[0][0]), float(a[0][1])),
                      (float(a[1][0]), float(a[1][1])))
        self._ft_t = (float(t[0]), float(t[1]))
        self._ft_identity = (self._ft_a == ((1.0, 0.0), (0.0, 1.0))
                             and self._ft_t == (0.0, 0.0))
        self._ft_rot_deg = math.degrees(math.atan2(self._ft_a[1][0], self._ft_a[0][0]))

    @property
    def previous(self) -> "LocalizationPose | None":
        return self._prev

    def reset(self) -> None:
        """유실 확정 시 호출 — 다음 pose를 기준점으로 다시 잡는다.

        유실 동안 차량이 밀렸거나 마커를 다시 잡은 위치가 달라졌을 때, 정상 복귀분을
        점프로 오인해 영구히 거부하는 상황을 막는다.
        """
        self._prev = None
        self._prev_face = None

    def validate(self, raw: str, received_monotonic_s: float) -> PoseResult:
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
            return PoseResult(reason=PARSE)
        if not isinstance(msg, dict):
            return PoseResult(reason=PARSE)

        mid = msg.get("marker_id")
        if self._cube:
            if mid not in self._faces:
                return PoseResult(reason=OTHER_MARKER)  # 큐브 4면이 아닌 마커
        elif mid != self._marker_id:
            return PoseResult(reason=OTHER_MARKER)      # 다른 차량 — 유실 판정과 무관 (§3.4)

        ts = msg.get("timestamp")
        if not _is_finite_number(ts):
            return PoseResult(reason=MALFORMED)
        ts = float(ts)

        if not msg.get("found"):
            # found=false면 position·heading 키 자체가 없다 — 좌표를 읽지 않는다 (§3.4)
            return PoseResult(reason=NOT_FOUND)

        pos = msg.get("position")
        heading = msg.get("heading")
        if (not isinstance(pos, dict)
                or not _is_finite_number(pos.get("x"))
                or not _is_finite_number(pos.get("y"))
                or not _is_finite_number(heading)):
            return PoseResult(reason=MALFORMED)         # z는 읽지 않는다 (§3.3 — 주행 미사용)

        prev = self._prev
        if prev is not None and ts <= prev.source_timestamp_s:
            return PoseResult(reason=OUT_OF_ORDER)      # 순서 역전은 timestamp로 판정 (§3.4)

        x, y = float(pos["x"]), float(pos["y"])
        if self._cube:
            # 면 좌표 → 차량 중심: 보고 heading = 면의 바깥 법선 방향 가정 (모듈 docstring)
            rad = math.radians(float(heading))
            x -= self._center_off * math.cos(rad)
            y -= self._center_off * math.sin(rad)
        if not self._ft_identity:
            # 서버 좌표 → 맵 좌표 (실측 아핀). 맵 범위 검사는 변환 후 = 맵 좌표계에서
            (a00, a01), (a10, a11) = self._ft_a
            x, y = (a00 * x + a01 * y + self._ft_t[0],
                    a10 * x + a11 * y + self._ft_t[1])
        if not (0.0 <= x <= self._map_x and 0.0 <= y <= self._map_y):
            return PoseResult(reason=OUT_OF_MAP)        # §2.6 유효 범위 밖

        # 마커 부착 방향 보정 후 0~360 정규화 (§2.7). 오프셋은 A3 실측으로 확정한다
        offset = self._faces[mid] if self._cube else self._yaw_offset
        heading_deg = (float(heading) + offset + self._ft_rot_deg) % 360.0

        same_face = (not self._cube) or (mid == self._prev_face)
        if prev is not None and same_face:
            # 허용치를 경과 시간에 비례시킨다 — 프레임이 몇 장 빠진 뒤의 정상 이동을
            # 점프로 오인하지 않게 한다 (10Hz 기준 실이동 2.1cm / 회전 4.9° vs 허용 8cm / 15°)
            scale = max(1.0, (ts - prev.source_timestamp_s) / self._nominal_dt)
            if math.hypot(x - prev.x, y - prev.y) > self._max_jump_m * scale:
                return PoseResult(reason=JUMP)
            if _heading_delta_deg(heading_deg, prev.heading_deg) > self._max_heading_jump_deg * scale:
                return PoseResult(reason=JUMP)

        pose = LocalizationPose(x=x, y=y, heading_deg=heading_deg,
                                source_timestamp_s=ts,
                                received_monotonic_s=received_monotonic_s)
        self._prev = pose
        self._prev_face = mid if self._cube else None
        return PoseResult(pose=pose)
