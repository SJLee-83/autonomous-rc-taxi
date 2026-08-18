"""MotionEstimator — pose 차분으로 속도를 추정한다 (protocol_2 §4.3).

**차량에 엔코더가 없다.** 위치 변화만으로 속도를 만들어야 하므로 ArUco 위치 노이즈가
그대로 증폭된다 — 위치 오차 ±1cm도 100ms 간격에서는 ±0.1 m/s가 되고, 이는 차량
최대 속도(0.2076 m/s)의 약 50%다. 그래서 순간값을 쓰지 않고 **최근 N개 pose의
이동평균**을 낸다 (§4.3 권고: 3~5개).

정밀 계측이 아니라 **상태 확인용 지표**다 — `info.speed` 보고와 도착 판정 보조에 쓴다.

시간 축은 **촬영 시각(`source_timestamp_s`)** 을 쓴다. 수신 시각을 쓰면 네트워크
지터가 그대로 속도 노이즈로 들어온다. 순서 역전은 PoseValidator가 이미 걸러낸다.
"""
import logging
import math
from collections import deque

log = logging.getLogger("localization.motion")


class MotionEstimator:
    def __init__(self, window_poses: int, max_gap_sec: float):
        """window_poses: 이동평균 창(pose 개수). 구간 속도는 그보다 하나 적게 나온다."""
        if window_poses < 2:
            raise ValueError("speed_average_window는 2 이상이어야 한다 (구간이 생겨야 함)")
        self._max_gap = max_gap_sec
        self._samples: "deque[float]" = deque(maxlen=window_poses - 1)
        self._prev = None

    def reset(self) -> None:
        """위치 유실 확정 시 호출 — 유실 전후를 이어 붙이면 엉뚱한 속도가 나온다."""
        self._samples.clear()
        self._prev = None

    def update(self, pose) -> float:
        """채택된 pose를 넣고 갱신된 추정 속도(m/s)를 돌려준다."""
        prev, self._prev = self._prev, pose
        if prev is None:
            return self.speed_mps            # 첫 pose — 아직 구간이 없다

        dt = pose.source_timestamp_s - prev.source_timestamp_s
        if dt <= 0.0 or dt > self._max_gap:
            # 순서 역전은 검증기가 거르므로 여기 오면 시계 이상이거나 오랜 공백이다.
            # 이어 붙이면 없는 이동을 만들어내므로 창을 비우고 다시 쌓는다
            log.debug("속도 추정 구간 불연속 (dt=%.3f초) — 창 초기화", dt)
            self._samples.clear()
            return self.speed_mps

        self._samples.append(math.hypot(pose.x - prev.x, pose.y - prev.y) / dt)
        return self.speed_mps

    @property
    def speed_mps(self) -> float:
        """후진이 없으므로 항상 0 이상이다 (§4.3)."""
        if not self._samples:
            return 0.0
        return sum(self._samples) / len(self._samples)

    @property
    def sample_count(self) -> int:
        """창에 쌓인 구간 수 — 창이 차기 전에는 평균이 덜 매끄럽다."""
        return len(self._samples)
