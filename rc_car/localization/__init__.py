"""위치 처리 (protocol_2 ①) — 단계 2 A1·A2 구현 완료.

- pose_validator: §3.4 수신 검증 (마커·found·키·순서·맵 범위·점프). 순수 판정기
- motion_estimator: pose 차분 → 추정 속도 이동평균 (§4.3). 엔코더가 없어서 필요하다
- localization_service: 채택분 StateStore 반영 + 유실/정상화 판정 → CommandPolicy 훅

수신 자체(접속·구독·재구독)는 `network/localization_client.py`가 맡는다.
pose_filter(명세서 §7)는 단계 3~5에서 필요해지면 추가한다.
"""
