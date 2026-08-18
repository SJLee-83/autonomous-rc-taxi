"""LocalizationService — 위치 수신 처리의 단일 지점 (명세서 §35 · protocol_2 §3.4·§7.2).

- 네트워크 스레드가 넘긴 원문을 PoseValidator에 태우고, 채택분만 StateStore에 반영한다.
- 유실·정상화 판정을 여기서 내려 CommandPolicy 훅(on_gps_lost / on_gps_recovered)을 부른다.
- **유실 판정 기준은 마지막 채택 pose의 나이다.** 미수신·미인식·검증 실패를 구분하지
  않는다 — 차량이 자기 위치를 모른다는 결과가 같기 때문 (§7.2). 연결 끊김만 예외로
  즉시 판정한다 (§7.4).

두 시점을 구분한다:
  - **주행 차단**: found=false를 받는 즉시 pose_stale — 이전 pose를 계속 쓰지 않는다 (§3.4)
  - **오류 정지 전이**: 유실이 pose_timeout_sec 지속된 뒤 (§7.4 "미인식은 0.6초")
  깜빡임 한 장에 바퀴는 멈추지만 state는 흔들리지 않는다.

터널 모드 (2026-08-06 — 동편 측위 유실 보험, config control.yaml `tunnel`):
  지정 구역(8/4·8/5 유실 실측 동편) 안에서 주행 중 pose가 끊기면, §3.4 정지 대신
  DeadReckoner(명령 기반 추측 항법)가 합성 pose를 store에 공급해 강제 노선 주행을
  잇는다. 예외는 좁게 연다 — ①구역 안 ②주행 상태 ③맹목 거리·시간 한도 안에서만.
  한도를 넘으면 공급을 멈추고 기존 유실 사슬(§7.2 정지)로 떨어진다.
  덤: 구역 안에서 검증을 통과한 pose도 DR 예측과 크게 어긋나면 의심 거부한다 —
  8/5 유실 직전의 "y 떠오름"(수락된 틀린 pose)에 대한 유일한 방어선이다.
"""
import logging
import threading
import time
from dataclasses import replace

from core.enums import DrivingState
from core.models import LocalizationPose
from core.state_store import StateStore
from localization.dead_reckoner import DeadReckoner
from localization.heading_estimator import HeadingEstimator
from localization.motion_estimator import MotionEstimator
from localization.pose_validator import NOT_FOUND, OTHER_MARKER, OUT_OF_ORDER, PoseValidator
from workers.periodic_worker import PeriodicWorker

log = logging.getLogger("localization")

_MAX_CHECK_INTERVAL_SEC = 0.1   # 유실 감시 주기 상한 — 판정 지연을 timeout의 일부로 묶는다
_TUNNEL_STATES = (DrivingState.FOLLOWING_ROUTE, DrivingState.APPROACHING_DESTINATION)
_HEADING_WARN_DEG = 25.0        # 마커 heading 과 추정값이 이 이상 벌어지면 1초 1회 경고.
                                # 임계를 넘어도 **동작은 안 바뀐다** — 추정값을 계속 쓴다.
                                # 기존 heading_guard 의 이진 전환과 다른 점이 여기다.
_TUNNEL_LOG_EVERY_SEC = 1.0     # DR 공급·의심 거부 로그 간격


class LocalizationService(PeriodicWorker):
    def __init__(self, stop_event: threading.Event, store: StateStore,
                 validator: PoseValidator, cfg: dict,
                 on_lost, on_recovered, on_error=None, estimator=None,
                 tunnel_cfg=None, wheelbase_m: float = 0.14,
                 heading_cfg=None):
        timeout = cfg["pose_timeout_sec"]
        # 미수신(pose_timeout_sec)과 미인식 지속(lost_hold_sec)은 같은 값이다 —
        # 원인 무관 동일 동작이므로 "마지막 채택 이후 경과" 하나로 감시한다
        hold = cfg["lost_hold_sec"]
        super().__init__("LocalizationService",
                         min(_MAX_CHECK_INTERVAL_SEC, min(timeout, hold) / 2.0),
                         stop_event, on_error)
        self._store = store
        self._validator = validator
        # 속도 추정기 — 없으면 estimated_speed_mps가 0으로 남는다 (§4.3)
        self._estimator = estimator or MotionEstimator(
            window_poses=cfg.get("speed_average_window", 5),
            max_gap_sec=min(timeout, hold))
        self._timeout = min(timeout, hold)
        self._on_lost = on_lost
        self._on_recovered = on_recovered
        self._lock = threading.RLock()
        self._had_pose = False               # 첫 유효 pose 수신 여부 (§4.0 게이트와 같은 기준)
        self._lost = False
        self._last_accept_monotonic = 0.0
        # 터널 모드 — enabled 아니면 전부 비활성 (기존 동작 그대로)
        self._dr = None
        if tunnel_cfg and tunnel_cfg.get("enabled"):
            self._dr = DeadReckoner(tunnel_cfg, wheelbase_m)
            self._zones = [tuple(float(v) for v in z) for z in tunnel_cfg["zones"]]
            self._distrust_m = float(tunnel_cfg["distrust_deviation_m"])
            log.info("터널 모드 활성: 구역 %d개, 맹목 한도 %.2fm / %.0fs, 의심 임계 %.2fm",
                     len(self._zones), float(tunnel_cfg["max_blind_distance_m"]),
                     float(tunnel_cfg["max_blind_time_s"]), self._distrust_m)
        self._dr_feeding = False             # 현재 합성 pose 공급 중인가 (전이 로그용)
        self._dr_last_log = 0.0
        self._suspect_count = 0
        # 🔴 연속 헤딩 추정 (2026-08-08 신설 — localization/heading_estimator.py 참조).
        #   마커 heading 을 그대로 쓰지 않고, 자전거 모델 + 위치 코스각으로 이어 간다.
        #   자전거 모델 수치는 터널 모드(tunnel)와 같은 실측값을 공유하므로 그 설정이
        #   있어야 켤 수 있다. enabled=false 이거나 키가 없으면 **기존 동작 그대로**
        #   (마커 heading 통과) — 되돌릴 때 코드를 손댈 필요가 없다.
        self._heading_est = None
        if heading_cfg and heading_cfg.get("enabled") and tunnel_cfg:
            self._heading_est = HeadingEstimator(heading_cfg, tunnel_cfg, wheelbase_m)
            log.info("연속 헤딩 추정 활성: 코스각 혼합 %.2f (최소 이동 %.3fm) / "
                     "마커 heading 반영 %.2f — 마커 값은 초기화와 약보정에만 쓴다",
                     float(heading_cfg.get("course_alpha", 0.45)),
                     float(heading_cfg.get("course_min_distance_m", 0.04)),
                     float(heading_cfg.get("marker_alpha", 0.03)))
        self._heading_log_at = 0.0           # 추정↔마커 차이 요약 로그 (1초 1회)

    # ---------- 수신 (네트워크 스레드) ----------

    def on_raw(self, raw: str) -> None:
        result = self._validator.validate(raw, time.monotonic())
        with self._lock:
            if result.accepted:
                self._accept(result.pose)
                return
            if result.reason == OTHER_MARKER:
                return                        # 다른 마커 — 우리 메시지가 아니다
            if result.reason == NOT_FOUND:
                # 마커 미인식 — 이전 pose를 주행에 쓰지 않는다 (§3.4).
                # 오류 정지 전이는 tick()이 지속 시간을 보고 판정한다
                self._store.mark_pose_stale()
            elif result.reason == OUT_OF_ORDER:
                log.debug("pose 순서 역전 — 무시 (§3.4)")
            else:
                log.warning("pose 거부 (%s) — 유효 pose 갱신 없음", result.reason)

    def on_disconnected(self) -> None:
        """위치 서버 연결 끊김 — 지속 시간을 기다리지 않고 즉시 유실 (§7.4)."""
        with self._lock:
            self._declare_lost("위치 서버 연결 끊김")

    # ---------- 유실 감시 ----------

    def tick(self) -> None:
        with self._lock:
            if self._lost or not self._had_pose:
                return
            now = time.monotonic()
            age = now - self._last_accept_monotonic
            snap = self._store.snapshot()
            if snap.pose_stale or age >= self._timeout:
                # 터널 모드 — 조건이 맞으면 합성 pose로 유실을 유예한다
                if self._tunnel_feed(snap, now):
                    return
            if age >= self._timeout:
                self._declare_lost(f"{age:.2f}초간 유효 pose 없음")

    # ---------- 내부 ----------

    def _accept(self, pose) -> None:
        if self._suspect(pose):
            return                            # 구역 내 왜곡 의심 — DR이 잇는다 (터널 모드)
        # 🔴 마커 heading → 연속 추정값으로 치환 (2026-08-08). 비활성이면 그대로 통과한다.
        #   여기서 바꿔 두면 이후 전부(DR 앵커·store·회전 판정·도착 판정·차선 매칭)가
        #   같은 값을 쓴다 — 소비자마다 다른 heading 을 보는 상태를 만들지 않는다.
        pose = self._estimate_heading(pose)
        if not self._store.update_pose(pose):
            return                            # StateStore 순서 방어와 이중 안전장치
        self._store.set_estimated_speed(self._estimator.update(pose))   # §4.3
        self._last_accept_monotonic = pose.received_monotonic_s
        if self._dr is not None:
            self._dr.anchor(pose.x, pose.y, pose.heading_deg,
                            pose.received_monotonic_s, pose.source_timestamp_s)
            if self._dr_feeding:
                self._dr_feeding = False
                log.info("터널 모드 종료 — 실측 pose 복귀 (%.3f, %.3f)", pose.x, pose.y)
        # 수신 주기(10Hz, network.yaml interval_ms: 100) pose+heading 시계열 —
        # 세로 캘리브·트리거 역산의 전제 (§0-45: 기존에는
        # 「차선 매칭 실패」 WARNING에만 pose가 있어 정상 주행 heading이 로그에 없었다)
        log.info("pose (%.3f, %.3f) heading=%.1f°", pose.x, pose.y, pose.heading_deg)
        if not self._had_pose:
            self._had_pose = True
            log.info("첫 유효 pose — (%.2f, %.2f) heading %.1f°",
                     pose.x, pose.y, pose.heading_deg)
        if self._lost:
            self._lost = False
            log.info("위치 수신 정상화 — recovered 보고 후 직전 state 복귀 (§7.2)")
            self._on_recovered()

    def _declare_lost(self, why: str) -> None:
        if self._lost or not self._had_pose:
            # 첫 pose 이전(기동 직후)은 오류로 격상하지 않는다 — 아직 운행 전이고
            # SafetySupervisor가 이미 출력을 막고 있다. ControlClient와 같은 원칙
            return
        self._lost = True
        self._store.mark_pose_stale()
        self._validator.reset()               # 복귀분을 점프로 오인하지 않도록 기준점 해제
        self._estimator.reset()               # 유실 전후를 이어 붙인 가짜 속도 방지
        self._store.set_estimated_speed(0.0)  # 정지 상태 — 마지막 속도를 유지하지 않는다
        if self._dr is not None:
            self._dr.reset()                  # DR도 앵커 해제 — 유실 중 예측은 신뢰 불가
        if self._heading_est is not None:
            # 유실 확정 = 위치 자취가 끊긴 것이라 코스각 기준점이 무의미해진다.
            # 다음 유효 pose 에서 마커 heading 으로 재초기화한다.
            self._heading_est.reset()
            self._dr_feeding = False
        log.warning("위치 유실 — 즉시 정지 (%s)", why)
        self._on_lost()

    # ---------- 터널 모드 (동편 유실 보험) ----------

    def _in_zone(self, x: float, y: float) -> bool:
        return any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0, x1, y0, y1 in self._zones)

    def _estimate_heading(self, pose):
        """마커 heading 을 연속 추정값으로 치환. 추정기가 없으면 pose 를 그대로 돌려준다.

        추정기 입력은 **적용된** 제어 명령이다 (SafetySupervisor 가 store 에 남긴 값) —
        지령이 아니라 실제로 하드웨어에 나간 값이라야 모델이 실물을 따라간다.
        """
        if self._heading_est is None:
            return pose
        cmd = self._store.snapshot().control_command
        est = self._heading_est.update(
            pose.x, pose.y, pose.heading_deg,
            cmd.throttle, cmd.steering_wheel_deg, pose.received_monotonic_s)
        gap = abs((pose.heading_deg - est + 180.0) % 360.0 - 180.0)
        if (gap >= _HEADING_WARN_DEG
                and pose.received_monotonic_s - self._heading_log_at >= 1.0):
            self._heading_log_at = pose.received_monotonic_s
            log.warning("마커 heading %.1f° vs 추정 %.1f° (차이 %.1f°) — 추정값 사용",
                        pose.heading_deg, est, gap)
        return replace(pose, heading_deg=est)

    def _suspect(self, pose) -> bool:
        """검증 통과 pose의 왜곡 의심 판정 — 구역 안 + 주행 중 + DR 신뢰 구간에서만.

        8/5 유실 직전 "y 떠오름"은 §3.4 전 검사를 통과한 틀린 pose였다. DR 예측과
        임계 이상 어긋나면 버린다. DR 한도가 다하면 판정을 멈춘다(그 뒤는 기존 동작) —
        DR이 무한정 실측을 눌러 이기는 것을 막는 안전핀.
        """
        if self._dr is None or not self._dr.anchored:
            return False
        if not self._in_zone(pose.x, pose.y):
            return False
        snap = self._store.snapshot()
        if snap.driving_state not in _TUNNEL_STATES:
            return False                     # 정차·수동 재배치 중에는 실측이 항상 이긴다
        if not self._dr.within_budget:
            return False
        cmd = snap.control_command
        self._dr.propagate(cmd.throttle, cmd.steering_wheel_deg,
                           pose.received_monotonic_s)
        dev = self._dr.deviation_m(pose.x, pose.y)
        if dev <= self._distrust_m:
            return False
        self._suspect_count += 1
        now = time.monotonic()
        if now - self._dr_last_log >= _TUNNEL_LOG_EVERY_SEC:
            self._dr_last_log = now
            log.warning("pose 의심 거부 — DR 예측과 %.2fm 어긋남 (구역 내, 누적 %d건): "
                        "수신 (%.3f, %.3f)", dev, self._suspect_count, pose.x, pose.y)
        return True

    def _tunnel_feed(self, snap, now: float) -> bool:
        """합성 pose 공급 시도. 성공하면 True — 호출측은 유실 선언을 유예한다."""
        if self._dr is None or not self._dr.anchored:
            return False
        if snap.driving_state not in _TUNNEL_STATES:
            return False
        cmd = snap.control_command
        self._dr.propagate(cmd.throttle, cmd.steering_wheel_deg, now)
        x, y, heading, ts = self._dr.predict()
        if not self._in_zone(x, y):
            return False                     # 구역 밖 유실은 기존 규칙대로 정지 (§3.4)
        if not self._dr.within_budget:
            if self._dr_feeding:
                self._dr_feeding = False
                log.warning("터널 모드 한도 초과 — 맹목 %.2fm/%.1fs, 공급 중단 (유실 사슬로)",
                            self._dr.blind_distance_m, self._dr.blind_time_s)
            return False
        pose = LocalizationPose(x=x, y=y, heading_deg=heading,
                                source_timestamp_s=ts, received_monotonic_s=now)
        if not self._store.update_pose(pose):
            return False
        self._store.set_estimated_speed(self._estimator.update(pose))
        if not self._dr_feeding:
            self._dr_feeding = True
            log.warning("터널 모드 진입 — DR 주행 시작 (%.3f, %.3f), 한도 %.2fm",
                        x, y, self._dr.max_blind_distance_m)
        if now - self._dr_last_log >= _TUNNEL_LOG_EVERY_SEC:
            self._dr_last_log = now
            log.info("pose (%.3f, %.3f) heading=%.1f° [DR 맹목 %.2fm]",
                     x, y, heading, self._dr.blind_distance_m)
        return True

    # ---------- 상태 조회 (테스트·디버깅) ----------

    @property
    def lost(self) -> bool:
        with self._lock:
            return self._lost
