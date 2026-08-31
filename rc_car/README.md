# Orin Car 임베디드 (rc_car)

> 기준 문서: 통신 규약 `protocol_2.md` v2.8, `RC_CAR_SYSTEM_DESIGN_SPEC.md` (팀 내부 문서, 저장소 미포함)
> 최종 상태: 2026-08-10 시연 완주 (첫 실물 완주 2026-07-31). 테스트 383건(서브 462) PASS, 1 skip(Windows SIGHUP)
> 파트 개요·동작 원리는 `../README.md`. 이 문서는 실행·구조·검증 범위

## 실행

```bash
pip install pyyaml websockets pytest

python main.py --driver-mode mock --seg-mode off                  # PC 기동, Ctrl+C 종료
python main.py --driver-mode mock --seg-mode off --run-seconds 3  # 자동 테스트용
python -m pytest tests -q                                         # 전체 테스트 (unit + scenario)

python tools/probe_gps.py                    # 위치 서버 수신 통계
python tools/probe_gps.py --seconds 30       # 노이즈 측정
python tools/probe_gps.py --url wss://127.0.0.1:8000/ws/v1/localization
```

- Jetson 불필요. `--driver-mode mock` 으로 전체 로직이 PC에서 구동
- config 기본값은 `driver_mode: real`, `seg_mode: real`. 보드 재배포 시 mock 회귀 방지용 설정이므로 파일 수정 대신 플래그로 덮어씀
- `seg_mode: real` 은 비전 게시 파일이 없으면 기동 거부. PC 에서는 `--seg-mode off` 필수
- 실물 드라이버는 `hardware/` 의 Real\* 클래스 (vendor 원본 무수정 + 브리지)

`tools/fake_camera.py` 는 cv2 의존이므로 GPS 서버 venv 의 파이썬으로 실행

```powershell
<GPS서버경로>\.venv\Scripts\python.exe tools\fake_camera.py
... tools\fake_camera.py --x 3.0 --y 2.0 --heading 180   # 좌표·방향 지정
... tools\fake_camera.py --seconds 5                     # 5초 뒤 중단, 유실 경로 검증
```

### 로컬 GPS 서버 (실서버·차량·카메라 없이 위치 연동 검증)

위치 서버(팀 코드, 저장소 미포함)는 PC 단독 실행 가능

```powershell
cd <GPS서버경로>
.\.venv\Scripts\python.exe scripts\setup_https.py   # 최초 1회. 미실행 시 ws://
.\.venv\Scripts\python.exe -m gps_server.main       # https://0.0.0.0:8000
```

- `certs/` 가 비어 있으면 TLS 자동 해제(`ws://`). `wss://` 경로 검증 시 인증서 선생성
- 차량 쪽은 `config/network.yaml` 의 `websocket_url` 을 `wss://127.0.0.1:8000/...` 으로 변경하거나 `probe_gps.py --url` 로 덮어씀
- 카메라 없이 `found=false` 흐름·재구독·`close 4404` 까지 검증 가능. 카메라 연결 시 `found=true` 전 경로 구동

### 통합 시뮬 (전 시나리오 가상 주행)

GPS 서버(원본) + 합성 카메라 되먹임 + 자체 관제 서버 + 차량 전체 스택을 한 번에 기동해 정상 사이클과 오류 시나리오를 자동 구동·판정. S1\~S6 전부 PASS

```powershell
python tools\integration_sim.py                 # 전체 (약 4\~5분)
python tools\integration_sim.py --scenarios s1  # 개별
```

- 차량은 `--driver-mode sim` 으로 기동해 제어 명령을 UDP 로 `tools/sim_world.py`(운동학 적분 + 합성 카메라)에 송신. pose 는 실제 GPS 경로(카메라 → 서버 → wss)로 회신
- `sim_world` 는 기동 시 자가 캘리브레이션으로 합성 카메라의 계통 왜곡(약 10cm)을 아핀 역보정 (잔차 약 2.5cm)
- 하위 프로세스 로그: `tools/sim_logs/`. 판정 근거: 관제 `events.jsonl`

## 구조

### SW 3원칙

| 원칙 | 구현 |
| --- | --- |
| `main.py` 는 조립만 | `main.py` → `app/runtime.py` (initialize / start / shutdown §25) |
| 상태는 StateStore 중앙 관리 | `core/state_store.py`. RLock + 불변 스냅샷, pose 순서 역전 거부 |
| 하드웨어 추상화 | `hardware/`. 상위는 `set_throttle` 과 `set_angle_deg`(바퀴각)만 사용. 서보 변환(108±, 0.526)은 드라이버 계층만 보유 |

`safety/safety_supervisor.py` 가 모든 모터 출력의 단일 통과점. 정지 래치 / 유효 pose 없음 / 비주행 상태 중 하나라도 걸리면 정지 출력

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

- `perception/mock_seg_model` 은 통합 시뮬용 정답 seg
- seg 모드 `off / mock / real` 은 config 또는 `--seg-mode` 로 교체

## 검증 현황

- 테스트 383건(서브테스트 462) PASS, 1 skip (Windows SIGHUP)
- 통합 시뮬 S1\~S6 전부 PASS. GPS 서버 + 합성 카메라 + 관제 + 차량 4프로세스 실통신 구성
- 실차 무선 E2E 전 시나리오 PASS. Jetson 과 PC(웹팀 Java 백엔드 + GPS 서버) 사이 WiFi 양축
- 2026-07-31 첫 실물 자율주행 완주 (관제 명령 → 경로 계획 → 주행 → 정차 → 완료 보고)
- 2026-08-10 시연 결과는 `../README.md` §5

### 검증 범위

코어·안전

- 기동 후 초기 상태: 모터 정지, 조향 중앙(서보 108°), 종료 시퀀스 `exit 0`
- GPS 서버 없이도 워커 6개 기동·질서 종료
- 단위 검증: pose 순서 거부 / ERROR 직전 state 보관·복귀 / 미션 유지 / 서보 변환 왕복 / SafetySupervisor 차단 3조건 / 후진 금지 / config 실측값 / state 한글 매핑

관제 통신 (`protocol_2.md`)

- 메시지 검증 §2.10 · §6.5 / 수용 매트릭스 §6.4 / 상태 전이표 §4.5
- 오류·복귀 §7. 동시 유실 §7.5 는 양쪽 순서 모두 검증
- 재전송 timestamp 고정 §5.2 / telemetry 게이트와 서보 변환 §4
- 시나리오: 정상 사이클 + 오류 5종(GPS 유실 자체 복귀 / 관제 단절 후 resume / 동시 유실 / 대기 중 오류 / `complete` 재전송 중 오류)
- 자체 관제 서버(`control_server`, 개발용 검증 서버로 저장소 미포함)와 실통신으로 접속 · `move` 수락 · `accept` · 상태 전이 확인

위치 서버 연동

- §3.4 검증 규칙: 마커 · `found` · 키 부재 · NaN · 순서 역전 · 맵 범위 · 점프 · heading 순환
- 유실 판정(즉시 stale 과 0.6초 전이의 구분), 복귀 1회, 첫 pose 전 미격상
- 시나리오: 접속·구독 / pose 반영 / 강제 절단 후 재구독 / 무수신 재접속 / 미인식 지속 후 복귀 / 자체 서명 TLS 설정
- 실 GPS 서버 통합 확인 항목
  - wss 자체 서명 접속·구독, `found=true` 실페이로드 채택 (첫 유효 pose `(2.04, 0.91) heading 88.2°`)
  - 중복 프레임 `out_of_order` 제거
  - 카메라 중단 시 0.6초 후 오류 정지, 재개 시 `recovered` 와 직전 state 자체 복귀
  - 미등록 마커 `close 4404` 사유 로그

기능별 구현 경위와 시행착오는 `../DEVELOPMENT_HISTORY.md`
