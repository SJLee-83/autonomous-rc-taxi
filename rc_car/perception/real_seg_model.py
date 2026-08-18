"""RealSegModel — vision 실행체 게시 파일을 읽는 클라이언트 (계약 v0.3 §3, V1).

vision_runner(별도 프로세스)가 /dev/shm/vision_latest.json 에 원자적으로 게시한
최신 결과를 latest()로 반환한다. 카메라·추론은 전부 vision 소유 — 이 클래스는
파일 읽기만 하므로 논블로킹·저비용이다.

폴백 사슬: vision 프로세스가 죽으면 파일이 낡는다 → 게시 timestamp가 그대로
어댑터의 freshness_max_s 판정에 걸려 invalid → GPS 단독 (§6.2). 여기서 따로
신선도를 판정하지 않는 이유다 — 판정 주체는 SegAdapter 하나로 유지한다.

현 단계(V1): 게시 payload의 seg 필드가 null — 차선 중앙 보정(offset/heading)
계산은 vision 쪽 미구현이라 latest()는 valid=False를 반환한다. 이 상태로도
seg_mode=real 기동·폴백 사슬·신선도 판정이 전부 실차에서 살아 있고,
vision이 seg를 채우기 시작하는 순간 코드 무수정으로 보정이 활성화된다.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

_INVALID = {"valid": False, "lateral_offset_m": 0.0, "heading_error_deg": 0.0}


class RealSegModel:
    def __init__(self, publish_path: str):
        self._path = Path(publish_path)

    # ---------- 계약 §3.1 (load/latest/close) ----------

    def load(self, config: dict) -> None:
        """§3: 카메라(=vision 실행체) 실패는 load 예외 → 기동 거부.

        vision_runner가 떠 있으면 게시 파일이 존재한다. 기동 직후일 수 있어
        짧게 대기 후 없으면 예외 — seg_mode=real은 vision 가동이 전제다.
        """
        deadline = time.monotonic() + float(config.get("load_wait_s", 3.0))
        while time.monotonic() < deadline:
            if self._path.exists():
                return
            time.sleep(0.2)
        raise FileNotFoundError(
            f"vision 게시 파일 없음: {self._path} — vision_runner 실행 확인 "
            "(계약 v0.3 §3: 카메라 실패 = 기동 거부)")

    def latest(self) -> dict:
        """게시 파일의 seg 필드를 §5 형태로 반환. 읽기 실패는 invalid (§6.1)."""
        try:
            payload = json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return dict(_INVALID)     # 원자적 게시라 드묾 — 파일 소실·경합 방어
        ts = payload.get("timestamp")
        seg = payload.get("seg")
        if not isinstance(seg, dict):
            out = dict(_INVALID)      # V1: seg 미구현 — invalid + timestamp 전달
        else:
            out = {"valid": seg.get("valid", False),
                   "lateral_offset_m": seg.get("lateral_offset_m", 0.0),
                   "heading_error_deg": seg.get("heading_error_deg", 0.0)}
        if ts is not None:
            out["timestamp"] = ts     # 신선도 판정은 SegAdapter 몫 (§5)
        return out

    def close(self) -> None:
        pass
