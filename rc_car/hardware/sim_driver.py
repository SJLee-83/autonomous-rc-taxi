"""Sim 드라이버 — 제어 명령을 UDP로 시뮬 월드에 중계한다 (⑦ 통합 시뮬 전용).

mock과 동일하게 하드웨어가 없지만, 명령을 버리는 대신 `tools/sim_world.py`(운동학
적분 + 합성 카메라)로 보낸다. 그러면 pose가 실제 GPS 경로(카메라 프레임 → GPS 서버
→ wss)를 통해 되돌아온다 — 차량 코드는 자신이 시뮬레이션인 것을 모른다.

driver_mode "sim"은 config 파일이 아니라 main.py CLI(--driver-mode sim)로만 지정한다
(로컬 검증 시 config를 건드리지 않는다 — 워크로그 교훈).
"""
import json
import logging
import socket
import threading

from .motor_driver import MotorDriver
from .steering_driver import SteeringDriver

log = logging.getLogger("hw.sim")


class SimCommandLink:
    """최신 제어 명령을 UDP(localhost)로 push. 무연결·손실 허용 — 최신 값만 의미 있다."""

    def __init__(self, port: int):
        self._addr = ("127.0.0.1", port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self._throttle = 0.0
        self._wheel = 0.0
        log.info("sim 명령 링크: udp://127.0.0.1:%d", port)

    def set_throttle(self, value: float) -> None:
        with self._lock:
            self._throttle = value
            self._send()

    def set_wheel(self, wheel_deg: float) -> None:
        with self._lock:
            self._wheel = wheel_deg
            self._send()

    def _send(self) -> None:
        payload = json.dumps({"throttle": self._throttle, "wheel_deg": self._wheel})
        try:
            self._sock.sendto(payload.encode(), self._addr)
        except OSError:      # 시뮬 월드가 아직/이미 없음 — 손실 무해
            pass

    def close(self) -> None:
        self._sock.close()


class SimMotorDriver(MotorDriver):
    def __init__(self, link: SimCommandLink):
        self._link = link

    def set_throttle(self, value: float) -> None:
        if value < 0.0:
            raise ValueError("후진 없음 — throttle은 0 이상 (protocol_2 §4.3)")
        self._link.set_throttle(min(value, 1.0))

    def stop(self) -> None:
        self._link.set_throttle(0.0)

    def cleanup(self) -> None:
        self._link.set_throttle(0.0)
        self._link.close()
        log.info("sim 모터 드라이버 cleanup")


class SimSteeringDriver(SteeringDriver):
    def __init__(self, steering_cfg: dict, link: SimCommandLink):
        super().__init__(steering_cfg)
        self._link = link
        self._servo_deg = self.center

    def set_angle_deg(self, wheel_deg: float) -> None:
        self._servo_deg = self.wheel_to_servo(wheel_deg)   # telemetry 서보각 (§4.4)
        self._link.set_wheel(wheel_deg)

    def center_steering(self) -> None:
        self._servo_deg = self.center
        self._link.set_wheel(0.0)

    def cleanup(self) -> None:
        log.info("sim 조향 드라이버 cleanup")

    @property
    def current_servo_deg(self) -> float:
        return self._servo_deg
