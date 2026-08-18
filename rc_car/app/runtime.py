"""Runtime — initialize / start / shutdown (명세서 §25).

main.py는 이 클래스를 호출만 한다 (SW 3원칙 1 — main에 비즈니스 로직 금지).

단계 1 범위: config → StateStore → mock driver → SafetySupervisor → worker 기동/안전 종료.
단계 2(위치 서버)·단계 4(관제 서버) 조립이 이어져 있다. 두 네트워크 클라이언트는
worker보다 먼저 기동하고, 종료 시퀀스에서는 worker join 뒤에 닫는다 (§25.2·§25.3).
"""
import logging
import math
import threading
from pathlib import Path

from app.stage1_workers import HeartbeatWorker
from behavior.driving_worker import DrivingWorker
from control.lane_follower import LaneFollower
from control.legacy import LEGACY_ENV, legacy_arc_enabled
from core.config import Config, load_config
from core.exceptions import ConfigError
from mapping.lane_map import load_lane_map
from mapping.map_matcher import MapMatcher
from perception.mock_seg_model import MockSegModel
from perception.perception_worker import PerceptionWorker
from perception.seg_adapter import SegAdapter
from perception.vision_marks import FileMarkSource, MarkAdapter, NullMarkSource
from navigation.arrival_checker import ArrivalChecker
from navigation.destination_resolver import DestinationResolver
from navigation.lane_route import from_lane_numbers
from navigation.route_planner import RoutePlanner
from navigation.turn_table import TurnTable
from core.enums import DrivingState
from core.state_store import StateStore
from hardware.mock_motor_driver import MockMotorDriver
from hardware.mock_steering_driver import MockSteeringDriver
from localization.localization_service import LocalizationService
from localization.motion_estimator import MotionEstimator
from localization.pose_validator import PoseValidator
from network.command_policy import CommandPolicy
from network.control_client import ControlClient
from network.localization_client import LocalizationClient
from network.report_manager import ReportManager
from network.telemetry import TelemetryWorker
from safety.safety_supervisor import SafetySupervisor
from safety.watchdog import SafetyWatchdog

log = logging.getLogger("runtime")


class Runtime:
    def __init__(self, overrides: dict | None = None) -> None:
        # CLI 오버라이드 (main.py) — 로컬 검증 시 config 파일을 건드리지 않기 위한 통로
        self._overrides = overrides or {}
        self.config: Config | None = None
        self.lane_map = None
        self.map_matcher = None
        self.destination_resolver = None
        self.route_planner = None
        self.arrival_checker = None
        self.seg_adapter = None
        self.store: StateStore | None = None
        self.supervisor: SafetySupervisor | None = None
        self.control_client: ControlClient | None = None
        self.localization_client: LocalizationClient | None = None
        self.localization: LocalizationService | None = None
        self.policy: CommandPolicy | None = None
        self.stop_event = threading.Event()
        self.workers = []
        self._motor = None
        self._steering = None

    # ---------- §25.1 initialize ----------

    def initialize(self) -> None:
        self.config = load_config()
        app_cfg = self.config.app["runtime"]
        # CLI 오버라이드 적용 — 파일 검증(load_config)은 원본 기준으로 이미 끝났다
        if self._overrides.get("driver_mode"):
            app_cfg["driver_mode"] = self._overrides["driver_mode"]
        if self._overrides.get("seg_mode"):
            self.config.perception["perception"]["seg_mode"] = self._overrides["seg_mode"]
        if self._overrides.get("localization_url"):
            self.config.network["localization"]["websocket_url"] = \
                self._overrides["localization_url"]
        if self._overrides.get("control_url"):
            self.config.network["control_server"]["websocket_url"] = \
                self._overrides["control_url"]

        logging.basicConfig(
            level=getattr(logging, app_cfg["log_level"], logging.INFO),
            # .%(msecs)03d — 10Hz pose 시계열(network.yaml interval_ms: 100)을
            # vision trace(time.time())와 대조하려면 초 단위로는 부족하다 (한 초에 pose 10개)
            format="%(asctime)s.%(msecs)03d %(levelname)-7s [%(name)s] %(message)s",
            datefmt="%H:%M:%S",
        )
        # 콘솔(tee·ssh)이 얼어도 제어·안전 스레드가 로깅에 블록되지 않게 (2026-08-05 사고)
        from core.safe_logging import make_nonblocking
        make_nonblocking()
        log.info("초기화 시작 (driver_mode=%s)", app_cfg["driver_mode"])

        # 단계 5 — 차선 그래프 맵 (D1). 기동 시 로드·검증 = 잘못된 맵은 즉시 거부.
        #   경로 탐색(D2)·차선 매칭(§12)이 이 객체를 쓴다.
        veh = self.config.vehicle["vehicle"]
        map_path = (Path(__file__).resolve().parent.parent
                    / self.config.app["map"]["graph_path"]).resolve()
        self.lane_map = load_lane_map(map_path)
        self.lane_map.validate_turn_radius(
            veh["wheelbase_m"] / math.tan(math.radians(veh["max_steering_deg"])))
        log.info("차선 그래프 로드: lanes=%d connectors=%d (inner %d / outer %d)",
                 len(self.lane_map.lanes), len(self.lane_map.connectors),
                 len(self.lane_map.lanes_in_circuit("inner")),
                 len(self.lane_map.lanes_in_circuit("outer")))

        # 단계 5 — 내비게이션 (D2). 기동 시 구성해 config 오류를 즉시 노출한다.
        #   실제 사용(미션 수립·주행)은 ⑤ 폴백 조향·D4 도착 판정에서 배선된다.
        self.map_matcher = MapMatcher(self.lane_map, self.config.control["matching"])
        self.destination_resolver = DestinationResolver(
            self.lane_map, self.config.control["destination"]["warn_snap_distance_m"])
        self.route_planner = RoutePlanner(self.lane_map)
        # 강제 지정 노선 (config/routes.yaml — 2026-08-06 시나리오 1 안 1 확정).
        # 기동 시 연결성 검증: 단절·오타 노선은 여기서 즉시 기동 거부된다.
        # 기동 로그의 "강제 노선 등록" 줄이 배포 확인 수단이다 — 없으면 Dijkstra(안 2)로 돈다.
        self.forced_routes = []
        for nums in (self.config.routes or {}).get("forced_routes") or []:
            lr = from_lane_numbers(self.lane_map, nums)
            self.forced_routes.append(tuple(nums))
            log.info("강제 노선 등록: %s (%.2fm)", lr.describe(), lr.total_length_m)
        self.arrival_checker = ArrivalChecker(
            self.config.control["control"]["arrival_radius_m"],
            self.config.control["arrival"])

        self.store = StateStore()

        # 드라이버 — real은 팀 구동 코드 어댑터 (hardware/vendor/, 2026-07-24 D2 해소)
        steering_cfg = self.config.vehicle["steering"]
        if app_cfg["driver_mode"] == "mock":
            self._motor = MockMotorDriver()
            self._steering = MockSteeringDriver(steering_cfg)
        elif app_cfg["driver_mode"] == "sim":
            # ⑦ 통합 시뮬 — 명령을 UDP로 sim_world에 중계 (pose는 GPS 경로로 되돌아온다)
            from hardware.sim_driver import SimCommandLink, SimMotorDriver, SimSteeringDriver
            link = SimCommandLink(self._overrides.get("sim_port", 9100))
            self._motor = SimMotorDriver(link)
            self._steering = SimSteeringDriver(steering_cfg, link)
        else:
            # Jetson 전용 — adafruit 라이브러리·I2C 필요. import를 여기로 지연시킨다
            from core.config import load_hardware
            from hardware.real_motor_driver import RealMotorDriver
            from hardware.real_steering_driver import RealSteeringDriver
            hw_cfg = load_hardware()
            self._motor = RealMotorDriver(hw_cfg)
            self._steering = RealSteeringDriver(steering_cfg, hw_cfg)

        self.supervisor = SafetySupervisor(
            self.store, self._motor, self._steering,
            max_wheel_deg=self.config.vehicle["vehicle"]["max_steering_deg"],
        )

        on_err = self._on_worker_error
        map_cfg = self.config.network["map"]

        # 단계 2 — 위치 서버 (protocol_2 ①)
        #   client(수신) → service(검증·유실 판정) → store / policy 훅으로 이어진다
        loc_cfg = self.config.network["localization"]
        self.localization = LocalizationService(
            self.stop_event, self.store,
            PoseValidator(loc_cfg, map_cfg["x_max"], map_cfg["y_max"]), loc_cfg,
            on_lost=lambda: self.policy.on_gps_lost(),
            on_recovered=lambda: self.policy.on_gps_recovered(),
            on_error=on_err,
            # 창 크기는 telemetry 섹션에 있다 — speed는 info로 나가는 값이라서 (§4.3)
            estimator=MotionEstimator(
                window_poses=self.config.network["telemetry"]["speed_average_window"],
                max_gap_sec=loc_cfg["pose_timeout_sec"]),
            # 터널 모드 — 동편 측위 유실 구역 한정 DR (control.yaml tunnel, §3.4 예외)
            tunnel_cfg=self.config.control["tunnel"],
            wheelbase_m=veh["wheelbase_m"],
            # 🔴 연속 헤딩 추정 (2026-08-08) — control.yaml heading_estimator.
            #   자전거 모델 수치는 tunnel 과 공유한다. 키가 없으면 꺼짐(기존 동작).
            heading_cfg=self.config.control.get("heading_estimator"),
        )
        self.localization_client = LocalizationClient(
            loc_cfg,
            on_message=lambda raw: self.localization.on_raw(raw),
            on_disconnected=lambda: self.localization.on_disconnected(),
        )

        # 단계 4 — 관제 클라이언트 (protocol_2 ②③④)
        cs_cfg = self.config.network["control_server"]
        self.control_client = ControlClient(
            cs_cfg,
            on_message=lambda raw: self.policy.on_raw_message(raw),
            on_connected=lambda: self.policy.on_control_connected(),
            on_disconnected=lambda: self.policy.on_control_disconnected(),
        )
        reports = ReportManager(self.stop_event, self.control_client, self.store,
                                cs_cfg, on_err)
        self.policy = CommandPolicy(self.store, self.supervisor, reports,
                                    map_cfg["x_max"], map_cfg["y_max"])
        telemetry = TelemetryWorker(self.stop_event, self.control_client, self.store,
                                    steering_cfg,
                                    self.config.network["telemetry"]["rate_hz"], on_err)

        control_cfg = self.config.control["control"]
        control_hz = control_cfg["loop_hz"]
        heartbeat = self.config.app["runtime"].get("heartbeat_sec", 1.0)

        # 시연 조향 입력원 — seg 계약 §8: config 1줄(seg_mode)로 off/mock/real 교체
        percep_cfg = self.config.perception["perception"]
        perception_worker = None
        self.mark_adapter = None
        if percep_cfg["seg_mode"] != "off":
            if percep_cfg["seg_mode"] == "mock":
                model = MockSegModel(self.lane_map, self.store)
                mark_source = NullMarkSource()
            else:
                from perception.real_seg_model import RealSegModel
                model = RealSegModel(percep_cfg["publish_path"])
                mark_source = FileMarkSource(percep_cfg["publish_path"])
            self.seg_adapter = SegAdapter(model, self.config.perception["seg_correction"])
            self.seg_adapter.load()
            # 노면표시 트리거 — 같은 게시 파일, 다른 소비 (§0-45 2차 회전 트리거)
            self.mark_adapter = MarkAdapter(
                mark_source,
                freshness_max_s=self.config.perception["seg_correction"]["freshness_max_s"],
                axis_px=percep_cfg.get("vehicle_axis_px"))
            perception_worker = PerceptionWorker(self.stop_event, self.seg_adapter,
                                                 percep_cfg["rate_hz"], on_err,
                                                 mark_adapter=self.mark_adapter)
            log.info("seg 조향: %s (보정 활성) / 노면표시 트리거 활성",
                     percep_cfg["seg_mode"])

        # 단계 5 — 주행 판단: 매칭→스냅→경로→추종→도착→complete (stage1 Idle 대체)
        #
        # 🔴 주행 로직 선택 (2026-08-06 §0-45). 기본은 항상 LaneFollower —
        #    차선 번호 point-to-point + 2단 회전(좌표 1차 · 비전 2차).
        #    가상 라인 트레이싱(원호 추종)은 control/legacy 로 격리되어 있고
        #    **환경변수 RC_CAR_LEGACY_ARC=1 로만** 되살아난다. config·yaml 경로는 없다
        #    (app.yaml driver_mode:mock 이 재배포로 3번 되살아난 사고와 같은 길 차단).
        if legacy_arc_enabled():
            from control.legacy.waypoint_follower import WaypointFollower
            log.warning("=" * 72)
            log.warning("🔴 폐기 보류 주행 로직 활성: 가상 라인 트레이싱(커넥터 원호 추종)")
            log.warning("🔴 %s=1 이 설정되어 있다. 비전 회전 트리거는 사용되지 않는다.", LEGACY_ENV)
            log.warning("🔴 의도한 것이 아니면 즉시 내리고 재기동할 것 (기본값 = LaneFollower)")
            log.warning("=" * 72)
            follower = WaypointFollower(control_cfg, veh["max_steering_deg"],
                                        veh["max_speed_mps"], wheelbase_m=veh["wheelbase_m"])
        else:
            self.turn_table = TurnTable.load(veh["wheelbase_m"], veh["max_steering_deg"])
            follower = LaneFollower(control_cfg, self.turn_table, veh["max_steering_deg"],
                                    veh["max_speed_mps"], wheelbase_m=veh["wheelbase_m"],
                                    marks=self.mark_adapter, seg=self.seg_adapter)
            log.info("주행 로직: 차선 번호 point-to-point + 2단 회전 "
                     "(좌표 1차 · 비전 2차, 트리거 표 %d종: %s)",
                     len(self.turn_table.tuned_keys), ", ".join(self.turn_table.tuned_keys))
            sh = control_cfg.get("seg_heading") or {}
            lanes = sh.get("lane_nums") or []
            if lanes and self.seg_adapter is not None:
                log.info("seg 방위 대체: 차선 %s (횡방향 전용 · 불일치 경고 %.0f° · 슬루 %.0f°/s)",
                         ", ".join(str(n) for n in lanes),
                         float(sh.get("disagree_warn_deg", 15.0)),
                         float(sh.get("steer_rate_deg_per_s", 0.0)))
            elif lanes:
                log.warning("seg 방위 대체 설정됐으나 seg 미가동(seg_mode=off) — GPS 방위만 사용")
            else:
                log.info("seg 방위 대체: 꺼짐 (전 차선 GPS 방위)")
            hg = control_cfg.get("heading_guard") or {}
            guard_deg = float(hg.get("disagree_deg", 0.0))
            if guard_deg > 0.0:
                log.info("heading 튐 방어: GPS 방위가 실제 진행 방향과 %.0f° 이상 벌어지면 "
                         "진행 방향으로 대체 (베이스라인 %.2fm, 직선 구간 전용)",
                         guard_deg, float(hg.get("window_m", 0.15)))
            else:
                log.warning("heading 튐 방어: 꺼짐 — GPS 방위를 그대로 믿는다 "
                            "(0806·0807 인도 침범이 이 상태에서 났다)")
        control_worker = DrivingWorker(
            self.stop_event, self.store, self.supervisor, self.policy,
            self.lane_map, self.map_matcher, self.destination_resolver,
            self.route_planner, self.arrival_checker, follower,
            control_hz, control_cfg["destination_slowdown_distance_m"], on_err,
            seg_adapter=self.seg_adapter, forced_routes=self.forced_routes)
        heartbeat_worker = HeartbeatWorker(self.stop_event, self.store, heartbeat, on_err)
        watched = [control_worker, heartbeat_worker, reports, telemetry, self.localization]
        if perception_worker is not None:
            watched.append(perception_worker)
        watchdog = SafetyWatchdog(self.stop_event, self.supervisor, watched=watched)
        # 기동 순서 §25.2: SafetyWatchdog → Localization → Control → Telemetry
        self.workers = [watchdog, self.localization, control_worker,
                        heartbeat_worker, reports, telemetry]
        if perception_worker is not None:
            self.workers.append(perception_worker)

        # 초기 출력 확정: 모터 정지 + 조향 중앙 (§25.1-9)
        self.supervisor.force_stop()
        self.store.set_driving_state(DrivingState.WAITING)
        log.info("초기화 완료 — 모터 정지·조향 중앙(서보 %.0f°) 확인",
                 self._steering.current_servo_deg)

    # ---------- §25.2 start ----------

    def start(self) -> None:
        # 네트워크 client를 worker보다 먼저 시작한다 (§25.2)
        if self.localization_client:
            self.localization_client.start()
        if self.control_client:
            self.control_client.start()
        for w in self.workers:
            w.start()
        log.info("worker %d개 기동", len(self.workers))

    def wait(self, run_seconds: float | None = None) -> None:
        """run_seconds가 None이면 Ctrl+C까지 대기 (테스트용 시간 제한 지원)."""
        try:
            if run_seconds:
                self.stop_event.wait(run_seconds)
            else:
                while not self.stop_event.is_set():
                    self.stop_event.wait(1.0)
        except KeyboardInterrupt:
            log.info("KeyboardInterrupt 수신")

    # ---------- §25.3 shutdown ----------

    def shutdown(self) -> None:
        """어떤 이유로 종료하든 이 순서를 지킨다: 모터 정지 → 조향 중앙 → stop event
        → worker join → WebSocket 종료 → cleanup."""
        log.info("종료 시퀀스 시작")
        try:
            if self.supervisor:
                self.supervisor.force_stop()          # 1·2. 모터 정지 + 조향 중앙
        finally:
            self.stop_event.set()                     # 3. 전역 정지 이벤트
            for w in self.workers:                    # 4. worker join
                w.join()
            if self.control_client:                   # 5. WebSocket 종료
                self.control_client.stop()
            if self.localization_client:
                self.localization_client.stop()
            if self.seg_adapter:                      # 6. 자원 해제 — seg 모델 close (계약 §3.1)
                self.seg_adapter.close()
            if self._motor:
                self._motor.cleanup()
            if self._steering:
                self._steering.cleanup()
            from safety.watchdog import SafetyWatchdog
            SafetyWatchdog.clear_heartbeat()          # 7. guard에 '정상 종료' 신호
            log.info("종료 완료 — 최종 모터 정지 보장됨")

    def _on_worker_error(self, worker_name: str, exc: BaseException) -> None:
        """worker 최상위 예외 → 즉시 정지 래치 (§24.3)."""
        if self.supervisor:
            self.supervisor.latch_stop(f"{worker_name} 예외: {exc!r}")
