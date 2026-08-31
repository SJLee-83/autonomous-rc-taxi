# Orin Car 임베디드 (rc_car)

> 기준: 통신 규약 `protocol_2.md` v2.8 + `RC_CAR_SYSTEM_DESIGN_SPEC.md` (둘 다 팀 내부 문서, 이 저장소에 미포함)
> 최종 상태: 2026-08-10 시연에서 완주 (첫 실물 완주는 2026-07-31). 테스트 383건(서브 462) PASS, 1 skip(Windows SIGHUP)
> 파트 전체 개요와 동작 원리는 `../README.md` 부터 읽음. 이 문서는 실행·구조 상세임.

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

`tools/fake_camera.py` 는 cv2가 필요하므로 GPS 서버 venv의 파이썬으로 실행함.

```powershell
<GPS서버경로>\.venv\Scripts\python.exe tools\fake_camera.py
... tools\fake_camera.py --x 3.0 --y 2.0 --heading 180   # 좌표·방향 지정
... tools\fake_camera.py --seconds 5                     # 5초 뒤 중단 → 유실 경로 검증
```

### 로컬 GPS 서버 (실서버·차량·라즈베리파이 없이 A트랙 검증)

위치 서버는 `<GPS서버경로>` 에서 PC 단독 실행이 가능함 (2026-07-27 검증).

```powershell
cd <GPS서버경로>
.\.venv\Scripts\python.exe scripts\setup_https.py   # 최초 1회. 없으면 ws:// 로 뜸
.\.venv\Scripts\python.exe -m gps_server.main       # https://0.0.0.0:8000
```

- `certs/` 가 비어 있으면 TLS가 자동으로 꺼져 `ws://` 가 되므로, `wss://` 경로를 검증하려면 인증서를 먼저 만듦
- 차량 쪽은 `config/network.yaml` 의 `websocket_url` 을 `wss://127.0.0.1:8000/...` 으로 바꾸거나 `probe_gps.py --url` 로 덮어씀
- 카메라 없이도 `found=false` 흐름과 재구독, `close 4404` 까지 검증됨. 카메라가 붙으면 `found=true` 까지 전 경로가 돎

Jetson 은 불필요함. `--driver-mode mock` 플래그로 전체 로직이 PC에서 돎.

- config 기본값은 `real` 임. 보드 재배포 때 mock 이 되살아나는 회귀를 막으려는 의도 설정이므로, 파일을 고치는 대신 플래그로 덮음
- 실물 드라이버는 `hardware/` 의 Real\* 클래스임 (vendor 원본 무수정 + 브리지)

### 통합 시뮬 (전 시나리오 가상 주행)

GPS 서버(원본)와 합성 카메라 되먹임, 자체 관제 서버, 차량 전체 스택을 한 번에 띄우고 정상 사이클과 오류 시나리오를 자동 구동·검증함. S1\~S6 전부 PASS.

```powershell
python tools\integration_sim.py                 # 전체 (약 4\~5분)
python tools\integration_sim.py --scenarios s1  # 개별
```

- 차량은 `--driver-mode sim` 으로 떠서 제어 명령을 UDP로 `tools/sim_world.py`(운동학 적분 + 합성 카메라)에 보냄. pose 는 실제 GPS 경로(카메라 → 서버 → wss)로 돌아옴
- `sim_world` 는 기동 시 자가 캘리브레이션으로 합성 카메라의 계통 왜곡(약 10cm)을 아핀 역보정함 (잔차 약 2.5cm)
- 하위 프로세스 로그: `tools/sim_logs/`
- 판정 근거: 관제 `events.jsonl`

## 구조

### SW 3원칙

| 원칙 | 구현 |
| --- | --- |
| `main.py` 는 조립만 | `main.py` → `app/runtime.py` (initialize / start / shutdown §25) |
| 상태는 StateStore 중앙 관리 | `core/state_store.py`. RLock + 불변 스냅샷, pose 순서 거부 |
| 하드웨어 추상화 | `hardware/`. 상위는 `set_throttle` 과 `set_angle_deg`(바퀴각)만 씀. 서보 변환(108±, 0.526)은 드라이버 계층만 앎 |

핵심 안전 장치는 `safety/safety_supervisor.py` 이며, 모든 모터 출력의 단일 통과점임.

- 유효 pose 없음 / 주행 상태 아님 / 정지 래치 중 하나라도 걸리면 무조건 정지를 출력함

### 모듈 구성

| 디렉터리 | 내용 |
| --- | --- |
| `app/` `core/` `workers/` `config/` | 런타임 조립(initialize 16단계), StateStore, 주기 워커 기반 클래스, 설정 로더 |
| `hardware/` | 모터·서보 드라이버 3종(real / mock / sim), vendor 원본 브리지 |
| `safety/` | SafetySupervisor(단일 통과점), watchdog |
| `network/` | `control_client`(관제) · `command_policy` · `report_manager` · `telemetry` · `messages` · `localization_client`(위치 서버) |
| `localization/` | `pose_validator` · `localization_service` · `heading_estimator` · `motion_estimator`(pose 차분 → `estimated_speed_mps`) · `dead_reckoner` |
| `mapping/` `navigation/` | 차선 매칭 / 목적지 스냅 · Dijkstra 경로 계획 · 도착 판정 |
| `control/` `behavior/` | P 제어 추종 / 주행 워커 |
| `perception/` | `real_seg_model`(비전 게시 파일 pull) · `seg_adapter`(부호 반전 · ±8° 보정 제한 · 최신 여부 판정) · `vision_marks`(노면표시 → 회전 트리거) · `perception_worker`(10Hz) |

- `perception/mock_seg_model` 은 통합 시뮬용 정답 seg 임
- seg 모드 `off / mock / real` 은 config 또는 `--seg-mode` 한 줄로 교체함

## 검증 현황

- 테스트 383건(서브테스트 462) PASS, 1 skip (Windows SIGHUP)
- 통합 시뮬 S1\~S6 전부 PASS. GPS 서버 + 합성 카메라 + 관제 + 차량 4프로세스 실통신 구성
- 실차 무선 E2E 전 시나리오 PASS. 젯슨과 PC(웹팀 Java 백엔드 + GPS 서버) 사이 WiFi 양축
- 2026-07-31 첫 실물 자율주행 완주. 관제 명령 → 경로 계획 → 주행 → 정차 → 완료 보고
- 2026-08-10 시연 결과는 `../README.md` §5 참조

### 검증 범위

코어·안전

- 기동 후 초기 상태: 모터 정지, 조향 중앙(서보 108°), 종료 시퀀스 `exit 0`
- GPS 서버가 없어도 워커 6개가 기동하고 질서 있게 종료함
- 단위 검증: pose 순서 거부 / ERROR 직전 state 보관과 복귀 / 미션 유지 / 서보 변환 왕복 / SafetySupervisor 차단 4조건 / 후진 금지 / config 실측값 / state 한글 매핑

관제 통신 (`protocol_2.md`)

- 메시지 검증 §2.10 · §6.5 / 수용 매트릭스 §6.4 / 상태 전이표 §4.5
- 오류·복귀 §7. 동시 유실 §7.5 는 양쪽 순서를 모두 검증함
- 재전송 timestamp 고정 §5.2 / telemetry 게이트와 서보 변환 §4
- 시나리오: 정상 사이클 + 오류 5종(GPS 유실 자체복귀 / 관제 끊김 후 resume / 동시 유실 / 대기 중 오류 / `complete` 재전송 중 오류)
- 자체 관제 서버(`control_server`, 개발용 검증 서버로 이 저장소에 미포함)와 실통신으로 접속·`move` 수락·`accept`·상태 전이를 확인함

위치 서버 연동

- §3.4 검증 전 규칙: 마커 · `found` · 키 부재 · NaN · 순서 역전 · 맵 범위 · 점프 · heading 순환
- 유실 판정(즉시 stale 과 0.6초 전이의 구분), 복귀 1회, 첫 pose 전에는 격상하지 않음
- 시나리오: 접속·구독 / pose 반영 / 강제 절단 후 재구독 / 무수신 재접속 / 미인식 지속 후 복귀 / 자체 서명 TLS 설정
- 실 GPS 서버 통합에서 확인한 것
  - wss 자체 서명 접속과 구독, `found=true` 실페이로드 채택 (첫 유효 pose `(2.04, 0.91) heading 88.2°`)
  - 중복 프레임 `out_of_order` 제거
  - 카메라 중단 시 0.6초 후 오류 정지, 재개 시 `recovered` 와 직전 state 자체 복귀
  - 미등록 마커 `close 4404` 사유 로그

> A3(마커 오프셋)과 A5(노이즈)는 당시 실서버 미기동으로 남겨 두었고, 이후 실차 반입 단계(7/30 이후)에서 실측으로 처리함.

기능별 구현 경위와 시행착오는 `../DEVELOPMENT_HISTORY.md` 를 봄.
