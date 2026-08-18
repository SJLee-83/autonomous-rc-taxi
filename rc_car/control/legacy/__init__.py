"""🔴 폐기 보류 구역 — 가상 라인 트레이싱 (2026-08-06 사용자 결정).

이 패키지 안의 코드는 **커넥터 원호 centerline 추종**(= 가상 라인 트레이싱)이다.
2026-08-06 주행 알고리즘 재설계(§0-45)로 **기본 주행에서 완전히 빠졌고**,
"시연 당일 새 알고리즘이 안 될 때 하드코딩으로 쓸 최후 폴백"으로만 남긴다.

🔴 활성화 규칙 (사용자 지시 — 목·금 실주행에서 말하기 전까지 나오면 안 된다)

    ① **yaml·config로는 절대 켜지지 않는다.**
       app.yaml `driver_mode: mock` 이 PC 정본 재배포로 3번 되살아난 사고(§0-43)와
       같은 경로를 원천 차단한다. 이 패키지를 켜는 config 키는 존재하지 않는다.
    ② 켜는 방법은 **환경변수 하나뿐**: RC_CAR_LEGACY_ARC=1
       (보드 기동 명령에 직접 쓰지 않으면 켜질 수 없다)
    ③ 켜지면 기동 로그에 🔴 경고가 찍힌다 — 조용히 켜지는 일은 없다.
    ④ tests/unit/test_legacy_quarantine.py 가 ①②③을 전부 강제한다.
       (말뚝 테스트 — 이 파일을 지우지 않는 한 재유입이 CI에서 걸린다)

왜 지우지 않고 남기나: 시연 당일 비전 트리거가 무너지면 되돌아갈 곳이 필요하다.
맵의 커넥터 원호 좌표(64개)도 같은 이유로 그대로 둔다 — "맵은 손대지 않는다"(§0-45).

현 기본 주행은 control/lane_follower.py (차선 번호 point-to-point + 2단 회전).
"""
from __future__ import annotations

import os

LEGACY_ENV = "RC_CAR_LEGACY_ARC"


def legacy_arc_enabled() -> bool:
    """가상 라인 트레이싱(원호 추종) 활성 여부 — 환경변수 단독 판정.

    config를 읽지 않는다. 읽으면 ①의 차단이 무너진다.
    """
    return os.environ.get(LEGACY_ENV, "").strip() == "1"
