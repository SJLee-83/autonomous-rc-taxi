"""내부 상태 enum. 명세서 §22.1 기반, 필수 범위로 축소.

- 정지선/장애물/교차로 상태는 카메라 기반 추가 기능이므로 **자리만 남기고 사용하지 않는다**
  (명세서 §33-11, protocol_2 §11).
- 관제로 나가는 한글 `state` 5종과의 대응은 `to_service_state()` 참조 (protocol_2 §4.5).
"""
from enum import Enum


class DrivingState(Enum):
    BOOT = "BOOT"
    WAITING = "WAITING"                  # 유효 pose 대기 포함 (첫 pose 전 모터 금지)
    PLANNING = "PLANNING"
    FOLLOWING_ROUTE = "FOLLOWING_ROUTE"
    APPROACHING_DESTINATION = "APPROACHING_DESTINATION"
    ARRIVED = "ARRIVED"                  # complete 재전송 중 (stop 수신 대기)
    ERROR = "ERROR"
    # --- 추가 기능 자리 (구현하지 않음) ---
    APPROACHING_STOP_LINE = "APPROACHING_STOP_LINE"
    STOPPED_AT_LINE = "STOPPED_AT_LINE"
    CROSSING_INTERSECTION = "CROSSING_INTERSECTION"
    OBSTACLE_WAIT = "OBSTACLE_WAIT"


#: 주행 계열 (protocol_2 §6.4 매트릭스의 "이동 중 — 주행 중")
DRIVING_STATES = frozenset({
    DrivingState.PLANNING,
    DrivingState.FOLLOWING_ROUTE,
    DrivingState.APPROACHING_DESTINATION,
})


class ServiceKind(Enum):
    """운행 목적 — move가 픽업인지 하차인지. 직전 상태가 결정한다 (protocol_2 §4.5)."""
    PICKUP = "호출 응답 이동 중"
    CARRY = "고객 탑승 이동 중"


def to_service_state(driving: DrivingState, kind: "ServiceKind | None") -> str:
    """내부 DrivingState → 관제 `info.state` (한글 5종, protocol_2 §4.5)."""
    if driving == DrivingState.ERROR:
        return "오류 정지"
    if driving in DRIVING_STATES or driving == DrivingState.ARRIVED:
        # ARRIVED = 도착 후 complete 재전송 중 — stop 수신 전까지 이동 중 상태 유지 (§4.5)
        return kind.value if kind else "대기 중"
    if kind == ServiceKind.PICKUP:
        # PICKUP 완료 후 stop 수신 → 탑승 대기 중 (전이는 CommandPolicy가 수행)
        return "탑승 대기 중"
    return "대기 중"


class ErrorCode(Enum):
    """내부 로그·디버깅용 (명세서 §31). report:error에는 실리지 않는다."""
    CONFIG_INVALID = "CONFIG_INVALID"
    LOCALIZATION_CONNECTION_LOST = "LOCALIZATION_CONNECTION_LOST"
    LOCALIZATION_TIMEOUT = "LOCALIZATION_TIMEOUT"
    LOCALIZATION_MARKER_LOST = "LOCALIZATION_MARKER_LOST"
    LOCALIZATION_POSE_OUT_OF_MAP = "LOCALIZATION_POSE_OUT_OF_MAP"
    CONTROL_SERVER_CONNECTION_LOST = "CONTROL_SERVER_CONNECTION_LOST"
    CONTROL_LOOP_FAILED = "CONTROL_LOOP_FAILED"
    MOTOR_DRIVER_FAILED = "MOTOR_DRIVER_FAILED"
    STEERING_DRIVER_FAILED = "STEERING_DRIVER_FAILED"
    INTERNAL_UNHANDLED_EXCEPTION = "INTERNAL_UNHANDLED_EXCEPTION"
