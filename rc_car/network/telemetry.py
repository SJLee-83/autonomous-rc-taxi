"""TelemetryWorker — info 5Hz 송신 (protocol_2 §4).

- 위치 수신과 무관한 독립 타이머 (§4.0) — GPS가 끊겨도 info는 계속 나간다.
- 첫 유효 pose 전에는 송신하지 않는다 (§4.0 게이트).
- steer는 서보 각도(중립 108)로 변환해 보낸다 (§4.4). 변환 상수는 config에서 온다.
"""
import threading
import time

from core.enums import to_service_state
from core.state_store import StateStore
from network.messages import build_info
from workers.periodic_worker import PeriodicWorker


class TelemetryWorker(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, client, store: StateStore,
                 steering_cfg: dict, rate_hz: float, on_error=None):
        super().__init__("TelemetryWorker", 1.0 / rate_hz, stop_event, on_error)
        self._client = client
        self._store = store
        self._servo_center = steering_cfg["center_deg"]
        self._wheel_ratio = steering_cfg["wheel_angle_ratio"]

    def tick(self) -> None:
        snap = self._store.snapshot()
        if snap.pose is None:
            return   # 첫 유효 pose 게이트 (§4.0) — 그 전까지 관제는 오프라인으로 표시
        # 서보각 = 중립 + 바퀴각 / 링키지 비율 (§4.4)
        steer = self._servo_center + snap.control_command.steering_wheel_deg / self._wheel_ratio
        self._client.send_json(build_info(
            timestamp=time.time(),
            x=snap.pose.x, y=snap.pose.y, heading=snap.pose.heading_deg,
            stale=snap.pose_stale,          # 유실 시 마지막 유효값 유지 + stale (§4.2)
            speed=snap.estimated_speed_mps,
            steer=steer,
            state=to_service_state(snap.driving_state, snap.service_kind),
        ))
