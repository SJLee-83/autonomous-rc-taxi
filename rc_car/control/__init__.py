"""제어.

기본 = LaneFollower (control/lane_follower.py) — 차선 번호 point-to-point 직선 +
2단 회전(좌표 1차 · 비전 2차). 2026-08-06 재설계 §0-45.

가상 라인 트레이싱(커넥터 원호 추종)은 control/legacy/ 로 격리되었다.
환경변수 RC_CAR_LEGACY_ARC=1 로만 되살아나며 config·yaml 경로는 없다.
"""
