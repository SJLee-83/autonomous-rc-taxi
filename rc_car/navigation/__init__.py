"""내비게이션 (단계 5 — D2·D4 완료: destination_resolver·route_planner·arrival_checker).

남은 것: trajectory_generator(§15 — 폴백 P-제어에는 Route.waypoints로 충분해 보류).
주행 배선(⑤)이 이 모듈들을 소비한다: 매칭 → 스냅 → 경로 → 추종 → 도착 → complete.
"""
