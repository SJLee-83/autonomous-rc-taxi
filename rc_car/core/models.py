"""핵심 데이터 모델 (명세서 §8, protocol_1 §6.2 ActiveMission).

전부 불변(frozen) dataclass — StateStore가 원자적으로 교체한다.
"""
from dataclasses import dataclass, field

from .enums import DrivingState, ServiceKind


@dataclass(frozen=True)
class LocalizationPose:
    """GPS 서버에서 받은 채택 pose (protocol_2 §3.3·§3.4 검증 통과분).

    - heading_deg: 월드 +x 기준 반시계 0~360 (§2.7). 내부 계산은 라디안 변환 후 사용.
    - source_timestamp_s: 위치 서버가 찍은 시각 (소수 그대로 — §2.5)
    - received_monotonic_s: 수신 시점 time.monotonic() — 안전 판단은 이 값 우선 (§3.4)
    """
    x: float
    y: float
    heading_deg: float
    source_timestamp_s: float
    received_monotonic_s: float


@dataclass(frozen=True)
class ControlCommand:
    """상위 로직 → 하드웨어. 조향은 **바퀴 각도**다. 서보 변환은 드라이버 계층만 안다 (§4.4)."""
    throttle: float = 0.0          # 0.0 ~ 1.0 (후진 없음)
    steering_wheel_deg: float = 0.0  # -30(좌) ~ +30(우), 0 = 중앙


STOP_COMMAND = ControlCommand(throttle=0.0, steering_wheel_deg=0.0)


@dataclass(frozen=True)
class ActiveMission:
    """진행 중 운행. resume에 좌표가 없으므로 차량이 목적지를 기억한다 (protocol_1 §6.2).

    - 주행 중에만 존재. 대기 중 오류에서는 None → 복구 시 '복귀만' (protocol_2 §7.2)
    - 오류 정지에 진입해도 지우지 않는다. stop 수락(도착 확인) 시 폐기.
    """
    dest_x: float
    dest_y: float
    kind: ServiceKind
    accepted_at_s: float


@dataclass(frozen=True)
class VehicleSnapshot:
    """StateStore 원자적 스냅샷 (명세서 §9.1)."""
    driving_state: DrivingState = DrivingState.BOOT
    prev_state_for_recovery: DrivingState | None = None   # 오류 진입 직전 state (§7 복귀)
    service_kind: ServiceKind | None = None
    pose: LocalizationPose | None = None
    pose_stale: bool = True            # 유효 최신 pose 보유 여부 (protocol_2 §4.2)
    mission: ActiveMission | None = None
    control_command: ControlCommand = field(default_factory=lambda: STOP_COMMAND)
    estimated_speed_mps: float = 0.0   # pose 차분 이동평균 (protocol_2 §4.3)
