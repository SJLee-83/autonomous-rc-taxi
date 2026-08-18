# Orin Car 임베디드 (rc_car)

> 기준: 통신 규약 `protocol_2.md` v2.8 + `RC_CAR_SYSTEM_DESIGN_SPEC.md` (둘 다 팀 내부 문서 — 이 저장소에 미포함)
> 최종 상태: **2026-08-10 시연 무개입 완주 3회** (첫 실물 완주는 2026-07-31). 테스트 383건(서브 462) PASS, 1 skip(Windows SIGHUP)
> 파트 전체 개요·동작 원리는 **`../README.md`** 부터 읽는다. 이 문서는 실행·구조 상세다.

## 실행

```bash
pip install pyyaml websockets pytest

python main.py --driver-mode mock                  # PC 기동, Ctrl+C 종료
python main.py --driver-mode mock --run-seconds 3  # 자동 테스트용
python -m pytest tests -q       # 전체 테스트 (unit + scenario)

python tools/probe_gps.py                    # 위치 서버 수신 통계 (A3·A4 측정용)
python tools/probe_gps.py --seconds 30       # 노이즈 측정은 길게
python tools/probe_gps.py --url wss://127.0.0.1:8000/ws/v1/localization
```

`tools/fake_camera.py`는 cv2가 필요하므로 **GPS 서버 venv의 파이썬**으로 실행한다.

```powershell
<GPS서버경로>\.venv\Scripts\python.exe tools\fake_camera.py
... tools\fake_camera.py --x 3.0 --y 2.0 --heading 180   # 좌표·방향 지정
... tools\fake_camera.py --seconds 5                     # 5초 뒤 중단 → 유실 경로 검증
```

### 로컬 GPS 서버 (실서버·차량·라즈베리파이 없이 A트랙 검증)

위치 서버는 `<GPS서버경로>`에서 **PC 단독 실행**할 수 있다 (2026-07-27 검증).

```powershell
cd <GPS서버경로>
.\.venv\Scripts\python.exe scripts\setup_https.py   # 최초 1회 — 없으면 ws://로 뜬다
.\.venv\Scripts\python.exe -m gps_server.main       # https://0.0.0.0:8000
```

`certs/`가 비어 있으면 TLS가 자동으로 꺼져 `ws://`가 되므로, `wss://` 경로를 검증하려면
인증서를 먼저 만든다. 차량은 `config/network.yaml`의 `websocket_url`을 `wss://127.0.0.1:8000/...`
으로 바꾸거나 `probe_gps.py --url`로 덮어쓴다.

카메라 없이도 `found=false` 흐름·재구독·close 4404까지 검증되고, 카메라가 붙으면
`found=true`까지 전 경로가 돈다.

**Jetson 불필요** — `--driver-mode mock` 플래그로 전체 로직이 PC에서 돈다. config 기본값은
`real`(보드 재배포 때 mock이 되살아나는 회귀 방지용 의도 설정)이므로 파일 수정 대신 플래그로
덮는다. 실물 드라이버는 `hardware/`의 Real* 클래스(vendor 원본 무수정 + 브리지)다.

### 통합 시뮬 — 전 시나리오 가상 주행

GPS 서버(원본) + 합성 카메라 되먹임 + 자체 관제 서버 + 차량 전체 스택을 한 번에 띄우고,
정상 사이클 + 오류 시나리오를 자동 구동·검증한다 (**S1~S6 전부 PASS**):

```powershell
python tools\integration_sim.py                 # 전체 (약 4~5분)
python tools\integration_sim.py --scenarios s1  # 개별
```

- 차량은 `--driver-mode sim`으로 떠서 제어 명령을 UDP로 `tools/sim_world.py`(운동학
  적분 + 합성 카메라)에 보낸다 — pose는 **실제 GPS 경로(카메라→서버→wss)** 로 돌아온다
- sim_world는 기동 시 **자가 캘리브레이션**으로 합성 카메라의 계통 왜곡(~10cm)을
  아핀 역보정한다 (잔차 ~2.5cm)
- 하위 프로세스 로그: `tools/sim_logs/` / 판정 근거: 관제 `events.jsonl`

## 구조 (SW 3원칙)

| 원칙 | 구현 |
|---|---|
| main.py는 조립만 | `main.py` → `app/runtime.py` (initialize/start/shutdown §25) |
| 상태는 StateStore 중앙 관리 | `core/state_store.py` — RLock + 불변 스냅샷, pose 순서 거부 |
| 하드웨어 추상화 | `hardware/` — 상위는 `set_throttle`/`set_angle_deg(바퀴각)`만. **서보 변환(108±, 0.526)은 드라이버 계층만 안다** |

핵심 안전 장치: **`safety/safety_supervisor.py` — 모든 모터 출력의 단일 통과점.**
유효 pose 없음 / 주행 상태 아님 / 정지 래치 → 무조건 정지 출력.

## 디렉터리 상태

| 디렉터리 | 상태 |
|---|---|
| `core/` `hardware/` `safety/` `workers/` `app/` `config/` | ✅ 단계 1 구현 완료 |
| `network/` | ✅ **단계 4(관제) + 단계 2 A1 완료** — control_client / command_policy / report_manager / telemetry / messages / localization_client |
| `localization/` | ✅ 완료 — pose_validator / localization_service / **motion_estimator**(pose 차분 → `estimated_speed_mps`) |
| `mapping/` `navigation/` `control/` `behavior/` | ✅ 완료 — 차선 매칭 / 목적지 스냅·Dijkstra 경로 계획·도착 판정 / P-제어 추종 / 주행 워커 |
| `perception/` | ✅ 완료 — `real_seg_model`(비전 게시 파일 pull) + `seg_adapter`(부호 반전·±8° 보정 제한·신선도 판정) + `vision_marks`(노면표시 → 회전 트리거) + `perception_worker`(10Hz). `mock_seg_model`은 통합 시뮬용 정답 seg. `off/mock/real`은 config 또는 `--seg-mode` 한 줄로 교체 |

## 검증 완료

**2026-07-24 (단계 1)**
- `python main.py` — mock 기동 → 초기 모터 정지·조향 중앙(서보 108°) → 종료 시퀀스 exit 0 ✅
- 단위 테스트 — pose 순서 거부 / ERROR 직전 state 보관·복귀 / 미션 유지 / 서보 변환 왕복 / SafetySupervisor 차단 4조건 / 후진 금지 / config 실측값 / state 한글 매핑 ✅

**2026-07-27 (단계 4 — 관제 연동, 총 43건 통과)**
- 단위 19건 — 메시지 검증(§2.10·§6.5) / 수용 매트릭스(§6.4) / 전이표(§4.5) / 오류·복귀(§7, 동시 유실 §7.5 양순서) / 재전송 timestamp 고정(§5.2) / telemetry 게이트·서보 변환(§4)
- **C5 시나리오 6건** (실제 WebSocket 서버 상대) — 정상 사이클 + 오류 5종(GPS 유실 자체복귀 / 관제 끊김 resume / 동시 유실 / 대기 중 오류 / complete 재전송 중 오류) ✅
- 자체 관제 서버(control_server — 개발용 검증 서버, 이 저장소 미포함)와 실통신 — 접속·move 수락·accept·상태 전이 확인 ✅

**2026-07-27 (단계 2 A1·A2 — 위치 연동, 총 74건 통과)**
- 단위 25건 — §3.4 검증 전 규칙(마커·found·키 부재·NaN·순서 역전·맵 범위·점프·heading 순환) / 유실 판정(즉시 stale vs 0.6초 전이) / 복귀 1회 / 첫 pose 전 격상 안 함
- **시나리오 6건** (실제 WebSocket 위치 서버 상대) — 접속·구독 / pose 반영 / **강제 절단 후 재구독** / 무수신 재접속 / 미인식 지속→복귀 / 자체 서명 TLS 설정 ✅
- `python main.py --run-seconds 4` — worker 6개 기동, GPS 없어도 exit 0 ✅
- **실 GPS 서버(로컬 실행) 통합 검증** ✅ — wss 자체 서명 접속·구독 / `found=true` 실페이로드 채택
  (`첫 유효 pose (2.04, 0.91) heading 88.2°`) / 중복 프레임 `out_of_order` 제거 / 카메라 중단 →
  **0.6초 후 오류 정지** → 재개 → **recovered·직전 state 자체 복귀** / 미등록 마커 `close 4404` 사유 로그
- 당시 실서버 미기동으로 남겨 둔 A3(마커 오프셋)·A5(노이즈)는 이후 실차 반입 단계(7/30~)에서 실측으로 처리

**2026-07-28 이후 (Phase 1 완결 → 실물 주행)**
- 통합 시뮬 S1~S6 전부 PASS — 4프로세스 실통신 폐루프 (GPS 서버 + 합성 카메라 + 관제 + 차량)
- 실차 무선 E2E 전 시나리오 PASS — 젯슨 ↔ PC(웹팀 Java 백엔드 + GPS 서버), WiFi 양축
- ⭐ **2026-07-31 첫 실물 자율주행 완주** — 관제 명령 → 경로 계획 → 주행 → 정차 → 완료 보고
- ⭐ **2026-08-10 시연 — 무개입 완주 3회**, 관제 왕복(complete↔stop) 6회 전건 성공 (상세는 `../README.md` §5)
- 최종 **테스트 383건(서브테스트 462) PASS, 1 skip**

> 파트 전체 개요·동작 원리는 상위 폴더의 **`../README.md`** 를 본다.
