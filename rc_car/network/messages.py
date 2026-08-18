"""관제 메시지 조립·해석 (protocol_2 §2.4·§2.10·§4~§6).

- 송신은 build_*, 수신은 parse_command 한 곳으로 고정한다.
- 해석할 수 없는 메시지는 None을 반환하고 로그만 남긴다 — 연결은 끊지 않는다 (§2.10).
"""
import json
import logging

log = logging.getLogger("network.messages")

PROTOCOL_VERSION = "2.3"
PROTOCOL_MAJOR = "2"

COMMANDS = ("move", "stop", "resume")


def build_info(timestamp: float, x: float, y: float, heading: float, stale: bool,
               speed: float, steer: float, state: str) -> dict:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "header": "info",
        "timestamp": timestamp,
        "loc": {"x": round(x, 4), "y": round(y, 4),
                "heading": round(heading, 1), "stale": stale},
        "speed": round(max(0.0, speed), 4),   # 후진 없음 — 음수 금지 (§4.3)
        "steer": round(steer, 1),
        "state": state,
    }


def build_report(kind: str, timestamp: float) -> dict:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "header": "report",
        "timestamp": timestamp,
        "report": kind,
    }


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def parse_command(raw: str, map_x_max: float, map_y_max: float) -> "tuple[str, dict | None] | None":
    """수신 원문 → (command, loc). 무시해야 하는 메시지는 None (§2.10 — 무시 + 로그).

    move의 loc은 여기서 맵 범위까지 검증한다 (§6.5) — None이면 accept도 나가지 않는다.
    """
    try:
        msg = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError):
        log.warning("파싱 실패 — 무시 (§2.10): %.80r", raw)
        return None
    if not isinstance(msg, dict):
        log.warning("최상위가 객체가 아님 — 무시 (§2.10)")
        return None

    ver = msg.get("protocol_version")
    if isinstance(ver, str) and ver.split(".")[0] != PROTOCOL_MAJOR:
        log.warning("protocol_version major 불일치 (%s) — 무시 (§2.4)", ver)
        return None
    if ver != PROTOCOL_VERSION:
        log.warning("protocol_version minor 불일치 (%s) — 진행 (§2.4)", ver)

    if msg.get("header") != "command":
        log.warning("header=%r — 무시 (§2.10)", msg.get("header"))
        return None

    cmd = msg.get("command")
    if cmd not in COMMANDS:
        log.warning("알 수 없는 command=%r — 무시 (§2.10)", cmd)
        return None

    if cmd == "move":
        loc = msg.get("loc")
        if not isinstance(loc, dict) or not _is_number(loc.get("x")) or not _is_number(loc.get("y")):
            log.warning("move에 유효한 loc 없음 — 무시, accept 미전송 (§2.10·§6.3)")
            return None
        if not (0.0 <= loc["x"] <= map_x_max and 0.0 <= loc["y"] <= map_y_max):
            log.warning("move.loc 맵 범위 밖 (%.2f, %.2f) — 무시, accept 미전송 (§6.5)",
                        loc["x"], loc["y"])
            return None
        return cmd, {"x": float(loc["x"]), "y": float(loc["y"])}

    if "loc" in msg:
        log.warning("%s에 loc 포함 — loc만 무시하고 정상 처리 (§6.3)", cmd)
    return cmd, None
