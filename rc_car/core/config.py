"""설정 로더 — config/*.yaml 을 읽고 필수 키를 검증한다.

미확정 하드웨어 상수는 코드에 하드코딩하지 않는다 (명세서 §0·§33-13).
"""
from dataclasses import dataclass
from pathlib import Path

import yaml

from .exceptions import ConfigError

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def _load(name: str) -> dict:
    path = CONFIG_DIR / name
    if not path.exists():
        raise ConfigError(f"설정 파일 없음: {path}")
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ConfigError(f"{name}: 최상위가 매핑이 아님")
    return data


def _load_optional(name: str) -> dict:
    """있으면 _load, 없으면 빈 매핑 — 강제 노선처럼 없어도 주행이 되는 설정용."""
    if not (CONFIG_DIR / name).exists():
        return {}
    return _load(name)


def _require(data: dict, section: str, keys: list[str], name: str) -> dict:
    if section not in data:
        raise ConfigError(f"{name}: '{section}' 섹션 없음")
    block = data[section]
    for k in keys:
        if k not in block:
            raise ConfigError(f"{name}: '{section}.{k}' 없음")
    return block


@dataclass(frozen=True)
class Config:
    app: dict
    network: dict
    vehicle: dict
    control: dict
    perception: dict
    routes: dict = None      # routes.yaml (선택) — 강제 지정 번호 노선


def load_config() -> Config:
    app = _load("app.yaml")
    network = _load("network.yaml")
    vehicle = _load("vehicle.yaml")
    control = _load("control.yaml")
    perception = _load("perception.yaml")
    routes = _load_optional("routes.yaml")

    _require(app, "runtime", ["driver_mode", "log_level"], "app.yaml")
    _require(app, "map", ["graph_path"], "app.yaml")
    _require(network, "localization", ["websocket_url", "tls_verify", "marker_id",
                                       "interval_ms", "marker_yaw_offset_deg",
                                       "reconnect_interval_sec", "pose_timeout_sec",
                                       "lost_hold_sec", "max_jump_m",
                                       "max_heading_jump_deg"], "network.yaml")
    _require(network, "control_server", ["websocket_url", "reconnect_interval_sec",
                                         "error_resend_interval_sec",
                                         "complete_resend_interval_sec",
                                         "ping_interval_sec", "ping_timeout_sec"], "network.yaml")
    _require(network, "telemetry", ["rate_hz", "speed_average_window"], "network.yaml")
    _require(network, "map", ["x_max", "y_max"], "network.yaml")
    veh = _require(vehicle, "vehicle", ["wheelbase_m", "max_steering_deg", "max_speed_mps"],
                   "vehicle.yaml")
    _require(vehicle, "steering", ["center_deg", "left_max_deg", "right_max_deg",
                                   "wheel_angle_ratio"], "vehicle.yaml")
    ctl = _require(control, "control", ["loop_hz", "arrival_radius_m", "lookahead_min_m",
                                        "destination_slowdown_distance_m", "waypoint_spacing_m",
                                        "steer_full_lock_error_deg", "align_heading_error_deg",
                                        "align_throttle_ratio", "cruise_throttle",
                                        "turn_throttle", "near_target_throttle_ratio",
                                        "stop_trigger_radius_m", "departure_ramp_distance_m",
                                        "corner_slowdown_distance_m",
                                        "corner_exit_hold_distance_m"], "control.yaml")
    if ctl["stop_trigger_radius_m"] > ctl["arrival_radius_m"]:
        raise ConfigError("control.yaml: stop_trigger_radius_m 는 arrival_radius_m 이하여야 함 — "
                          "트리거가 게이트보다 크면 스로틀 0 전에 도착 반경을 지나친다")
    _require(control, "matching", ["distance_weight", "heading_weight_per_deg",
                                   "max_center_distance_m", "max_heading_error_deg"],
             "control.yaml")
    _require(control, "destination", ["warn_snap_distance_m"], "control.yaml")
    tun = _require(control, "tunnel",
                   ["enabled", "zones", "max_blind_distance_m", "max_blind_time_s",
                    "distrust_deviation_m", "wheel_bias_deg", "servo_rate_dps",
                    "speed_points"], "control.yaml")
    for i, z in enumerate(tun["zones"] or []):
        if not (isinstance(z, list) and len(z) == 4
                and all(isinstance(v, (int, float)) for v in z)):
            raise ConfigError(f"control.yaml: tunnel.zones[{i}] 는 [x_min, x_max, y_min, y_max]")
    if tun["enabled"] and not tun["zones"]:
        raise ConfigError("control.yaml: tunnel.enabled 인데 zones 가 비어 있음")
    _require(control, "arrival", ["max_estimated_speed_mps", "settle_time_sec",
                                  "max_heading_error_deg"], "control.yaml")
    _require(perception, "perception", ["seg_mode", "rate_hz"], "perception.yaml")
    _require(perception, "seg_correction",
             ["offset_gain_wheel_deg_per_m", "heading_gain_wheel_deg_per_deg",
              "max_correction_wheel_deg", "invalid_fallback_after"], "perception.yaml")
    if perception["perception"]["seg_mode"] not in ("off", "mock", "real"):
        raise ConfigError("perception.yaml: seg_mode 는 off|mock|real")

    for i, r in enumerate(routes.get("forced_routes") or []):
        if (not isinstance(r, list) or len(r) < 2
                or not all(isinstance(n, int) for n in r)):
            raise ConfigError(
                f"routes.yaml: forced_routes[{i}] 는 차선 번호(정수) 2개 이상의 목록이어야 함")

    if app["runtime"]["driver_mode"] not in ("mock", "real"):
        raise ConfigError("app.yaml: runtime.driver_mode 는 mock|real")
    if veh["wheelbase_m"] <= 0 or veh["max_speed_mps"] <= 0:
        raise ConfigError("vehicle.yaml: 제원 값은 양수여야 함")

    return Config(app=app, network=network, vehicle=vehicle, control=control,
                  perception=perception, routes=routes)


def load_hardware() -> dict:
    """실물 하드웨어 상수 (config/hardware.yaml → vendor config 브리지).

    real 모드에서만 필요하므로 load_config()와 분리한다. mock 모드는 이 파일이 없어도 된다.
    """
    hw = _load("hardware.yaml")
    block = _require(hw, "orin_car",
                     ["motor_i2c_address", "motor_channel", "motor_sign",
                      "servo_i2c_address", "servo_channel",
                      "servo_left_angle", "servo_right_angle", "servo_center_angle"],
                     "hardware.yaml")
    if block["servo_left_angle"] <= block["servo_right_angle"]:
        raise ConfigError(
            "hardware.yaml: servo_left_angle > servo_right_angle 이어야 함 "
            "(vendor 클램프 전제 — hardware.yaml 주석 참조)")
    return block
