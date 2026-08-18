"""네트워크 클라이언트 — 위치 서버 ①(단계 2) + 관제 서버 ②③④(단계 4).

- localization_client: 위치 서버 접속·구독·재구독 (§3.1·§3.2, 자체 서명 TLS).
  수신 원문의 검증·판정은 `localization/` 패키지가 맡는다
- control_client: WebSocket 접속·재접속·ping/pong (§2.8, 전용 asyncio 스레드)
- command_policy: 수용 매트릭스·상태 전이·오류/복구의 단일 지점 (§4.5·§6.4·§7)
- report_manager: report 4종 + complete/error 재전송 (§5)
- telemetry: info 5Hz 독립 타이머 + 첫 pose 게이트 (§4)
- messages: 메시지 조립·해석 (§2.10 — 해석 불가는 무시+로그)
"""
