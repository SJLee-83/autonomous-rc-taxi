# RC카 자율주행 — 차량 파트 제출본 (2026-08-10)

2026-08-10 시연에서 **무개입 완주 3회**를 낸 차량 주행 코드와, 그 차량에 비전 신호를
공급하는 **비전 실행체**를 한 묶음으로 정리한 것.

| 폴더 | 무엇 | 실행 위치 |
| --- | --- | --- |
| `rc_car/` | 차량 주행 프로세스 (측위·경로·조향·관제 통신·안전) | Jetson Orin Nano |
| `vision_runner/` | 비전 프로세스 (CSI 캡처 → 버드아이 → YOLO seg → 결과 게시) | 같은 보드, **별도 프로세스** |
| `map/` | 차선 그래프·장소 정의 (`rc_car` 가 `../map` 으로 참조) | — |

```
. (repo 루트)
├── rc_car/            # 주행 (python3 main.py)
├── vision_runner/     # 비전 (python3 vision_runner.py)
├── map/               # main_track_map.yaml · places.yaml
└── README.md          # 이 문서
```

---

## 1. 두 프로세스가 어떻게 만나는가

카메라는 **비전이 소유**함. 주행 코드는 카메라를 열지 않고, 비전이 tmpfs에 원자적으로
게시한 최신 결과를 **논블로킹으로 당겨 읽음**(pull). 두 프로세스는 파일 하나로만 만남.

```
 [카메라 CSI]
      │
      ▼
 vision_runner (5 FPS)                        rc_car (주행 50Hz / 인지 10Hz)
   BirdseyeExtractor  워프                       PerceptionWorker  10Hz pull
   extract_yellow()   황색선 = 색 휴리스틱          ├─ SegAdapter   → 차선 보조 (횡보정)
   YOLO best.engine   점선·정지선·횡단보도·화살표      └─ MarkAdapter  → 조향 트리거 (회전 개시)
   compute_seg()      차선 중앙 offset/heading                │
      │                                                     ▼
      └──▶ /dev/shm/vision_latest.json ──────────────▶ LaneFollower (조향 명령)
              (timestamp 포함, 원자적 교체)
```

게시 payload(`vision_runner.py`)와 소비 측 계약:

| payload 필드 | 읽는 쪽 | 쓰임 |
| --- | --- | --- |
| `timestamp` | `SegAdapter` / `MarkAdapter` | 신선도 판정 — 0.5초보다 낡으면 **invalid** |
| `seg.{valid, lateral_offset_m, heading_error_deg}` | `RealSegModel` → `SegAdapter` | 차선 중앙 보정량 |
| `model.detections[].{cls, conf, xyxy_px}` | `FileMarkSource` → `MarkAdapter` | 회전 트리거 판정 |
| `pixels_per_meter`, `vehicle_axis_px`, `birdseye_size` | `MarkAdapter` | 픽셀 → 미터 환산·횡거리 게이트 |

**무중단 폴백이 설계의 핵심.** 비전 프로세스가 죽으면 게시가 낡고 → 신선도 판정이
invalid → 차선 보정은 0, 트리거는 미발화 → 주행은 GPS 단독 + 좌표 폴백으로 **계속됨**.
비전은 주행의 전제가 아니라 가산 신호.

---

## 2. 비전 기반 차선 보조 (직진 중 횡보정)

차선 중앙에서 얼마나 벗어났는지(`lateral_offset_m`)와 차선과 얼마나 틀어졌는지
(`heading_error_deg`)를 받아 **바퀴각에 보정을 더함**. 기저 조향은 항상 GPS 경로 추종이고,
비전은 그 위에 얹히는 제한된 보정 (명세서 §18-3: *보정값은 반드시 제한한다*).

| 파일 | 역할 |
| --- | --- |
| `vision_runner/vision_runner.py` `compute_seg()` | **생산자.** 행별로 황색선 단면 + 모델 점선 상자를 모아 차량 축을 감싸는 차선 폭 쌍의 중점을 구하고, 그 점들에 직선을 적합해 offset/heading 산출. 쌍이 없는 행은 편측 추정(경계선 + 공칭 반폭)으로 보충 |
| `rc_car/perception/real_seg_model.py` | 게시 파일 클라이언트. `latest()` 로 최신 seg 반환 |
| `rc_car/perception/mock_seg_model.py` | 계약 정답 seg (통합 시뮬용, 실차 무관) |
| `rc_car/perception/seg_adapter.py` | **소비자.** 필수 4필드 검증 · 신선도 판정 · 부호 반전 · 보정 계산 및 클램프 · 연속 invalid 시 폴백 로그 |
| `rc_car/perception/perception_worker.py` | 10Hz 관측 스레드 (주행 50Hz 루프와 분리) |
| `rc_car/config/perception.yaml` | 모드·게시 경로·게인·제한 |

보정식 (`seg_adapter.py:correction_wheel_deg`)

```
correction = -( k_off · lateral_offset_m + k_head · heading_error_deg )
             클램프 ±max_correction_wheel_deg
```

`config/perception.yaml`

```yaml
perception:
  seg_mode: "real"                        # off | mock | real
  rate_hz: 10
  publish_path: "/dev/shm/vision_latest.json"
seg_correction:
  offset_gain_wheel_deg_per_m: 30.0       # 횡오프셋 1m 당 바퀴각 30°
  heading_gain_wheel_deg_per_deg: 0.3     # 방위오차 1° 당 바퀴각 0.3°
  max_correction_wheel_deg: 8.0           # 보정 상한
  invalid_fallback_after: 3               # 연속 invalid 3회 → 폴백 로그
  freshness_max_s: 0.5                    # 이보다 낡으면 stale = invalid
```

> ⚠️ **게인 이력**: 두 게인은 8/6 실주행에서 강한 좌편향(부호 반전 또는 편측 추정 계통
> 오차로 추정)이 나와 `0.0` 으로 무력화한 적이 있고, **8/10 시연은 보정 0 상태로 완주함.**
> 이 제출본은 원복값(30.0 / 0.3)으로 되돌려 두었으나 **그 값의 실주행 재검증은 하지 않았음.**
> 보정을 끄려면 두 게인을 `0.0` 으로 되돌리면 됨 — 트리거는 게인과 무관하게 계속 동작함.

---

## 3. 조향 트리거 (회전 개시 시점 결정)

차선 보조와 **같은 게시 파일을 읽지만 쓰임이 완전히 다른 두 번째 소비자**.
회전을 "좌표로 언제 꺾을지" 대신 **"노면표시가 보이면 꺾는다"** 로 바꾼 것이 핵심.
비전 트리거의 가치는 정확도가 아니라 **좌표계 독립성** — 8/7 실측에서 좌표계가 세 번
바뀌는 동안 `22->14` 트리거는 7/7 발화하며 매번 같은 지점에서 꺾었음.

| 파일 | 역할 |
| --- | --- |
| `rc_car/perception/vision_marks.py` | `MarkAdapter` — 검출 목록에서 트리거 조건 판정 |
| `rc_car/navigation/turn_table.py` + `config/turn_table.yaml` | 회전별 트리거 사양·조향각·종료 조건 표 |
| `rc_car/control/lane_follower.py` | 상태 기계 `FOLLOW → ARMED → TURNING → 정렬 종료` |

**상태 기계**

```
FOLLOW ──(차선 끝까지 arm_distance)──▶ ARMED ──(① 비전 트리거 발화 or ② 좌표 폴백)──▶ TURNING
                                                                                      │
                                            (방위 정렬 or 최대 거리 상한) ◀────────────┘
```

**트리거 판정 4조건** (`vision_marks.py`)

| # | 조건 | 뜻 |
| --- | --- | --- |
| ① | 클래스 일치 (`crosswalk` / `stop_line` / `direction_arrow` / `dashed_line`) | 어떤 표식으로 꺾을지 |
| ② | `conf >= min_conf` | 오검출 배제 |
| ③ | 상자 **하단** y ≥ 높이 × `near_row_frac` | 표식이 차에 그만큼 가까워짐 = **발화 시점 조절 손잡이** |
| ④ | 상자 중심의 횡거리 ≤ `max_lateral_m` | 남의 차선 표식 기각 (실측에서 정지선이 차선 5~6개 밖까지 잡힘) |

`near_row_frac` 이 낮을수록 표식이 멀 때 발화 = **일찍 꺾음.** 회전 튜닝은 이 값 하나로 함.

**비전이 죽어도 회전은 함** — 트리거가 영원히 안 뜨면 `fallback_at_lane_end` 로 차선 끝에서
좌표 단독 개시. 시연 로그의 `트리거 좌표단독` 이 그 경로.

---

## 4. 실행

### 보드 (실차)

```bash
# ① 비전 — 주행 프로필 (무선 0, 온보드 녹화)
cd vision_runner
python3 vision_runner.py --record-dir ~/vision_rec/run1

# ② 주행 — 비전 게시가 있어야 seg_mode:real 로 뜸
cd rc_car
python3 main.py >> ~/veh_MMDD.log 2>&1
```

`vision_runner.py --stream-port 8090` 은 브라우저 실시간 확인용이며 **정차 중에만** 사용
(주행 중 무선 전송이 WiFi 동결을 유발한 사고가 있었음).

### PC (차·보드·카메라 없이)

```bash
cd rc_car
python3 main.py --driver-mode mock --seg-mode off    # 비전 없이 전 로직 구동
python3 main.py --driver-mode mock --seg-mode mock   # 계약 정답 seg 로 통합 시뮬
python3 -m pytest tests -q
```

`seg_mode` 는 **config 한 줄 또는 `--seg-mode` 한 플래그**로 `off / mock / real` 이 갈림.
`real` 은 비전 게시 파일이 없으면 **기동 거부**(계약: 카메라 실패 = 기동 거부).

### 의존

- 주행: `pyyaml`, `websockets` (+ 보드에서 `Adafruit_PCA9685`)
- 비전: `vision_runner/requirements.txt` — `opencv-python`, `numpy`, `ultralytics`(TensorRT `best.engine`)

### ⚠️ 실행 전 채워야 하는 값 (공개 제출본이라 비워 둠)

서버 주소·차량 토큰·개발 PC 경로는 `<...>` 플레이스홀더로 치환해 둠. 실제로 돌리려면
`rc_car/config/network.yaml` 의 두 줄을 환경에 맞게 채워야 함.

```yaml
localization:
  websocket_url: ws://<gps-server-host>:8100/ws/v1/localization   # 위치(ArUco) 서버
control_server:
  websocket_url: wss://<control-server-host>/ws/vehicle?token=<VEHICLE_TOKEN>   # 관제 서버
```

`tools/` 의 일부 개발 도구(`t4_wireless_e2e.py`, `sim_world.py`, `fake_camera.py`,
`integration_sim.py`)와 `vision_runner/seg_replay.py` 에도 개발 PC 경로가 플레이스홀더로
남아 있음. **주행·비전 본체 실행에는 영향 없음.**

---

## 5. 시연 결과 (2026-08-10)

정차점 — 면사무소 (1.20, 0.39) / 우리집 (0.39, 2.25)

| # | 시각 | 면사무소 도착 오차 | 우리집 도착 오차 |
| --- | --- | --- | --- |
| 1 | 07:45 | 16.3 cm | 19.3 cm |
| 2 | 08:59 | 16.8 cm | 18.0 cm |
| 3 | 09:16 | **12.2 cm** | **8.7 cm** |

- 3회 모두 **사람 개입 없이** 호출 → 배차 → 이동 → 도착 → `complete` → 다음 배차까지 완주
- 관제 왕복(`complete` ↔ `stop`) 6회 전건 성공
- 기동 시 등록된 회전 트리거 표 7종: `11->22, 4->29, 29->7, 7->22, 20->3, 22->14, 14->19`

---

## 6. 이 제출본의 상태 (알려진 것)

정직하게 남김.

1. **시연 당일 비전은 꺼져 있었음.** 완주 3회는 `--seg-mode off` (GPS 단독 + 좌표 폴백
   회전)로 달성. 비전 경로는 8/5~8/8 실주행에서 검증된 코드이고 이 제출본에 전부 포함돼
   있으나, **8/10 완주 기록 자체는 비전 없이 낸 것.**
2. **차선 보정 게인은 재검증 전** (§2 경고 참조).
3. **단위 테스트는 전량 통과** — 383건 수집: 382 passed, 1 skipped(Windows에 없는
   SIGHUP), 서브테스트 462. 2026-08-10 제출 원본에서는 15건이 실패했었음 — 원인은 코드
   결함이 아니라 **버전 짝**(`rc_car` 는 8/8 스냅샷, `map` 은 8/9 갱신본)으로, 실차 튜닝으로
   바뀐 값을 테스트 말뚝이 따라가지 못한 것. 이 정리본은 말뚝을 시연 확정값
   (`turn_table` 말뚝, 도착 반경 0.40 정합)으로 갱신해 해소. **주행 동작에는 원래부터
   영향이 없었음** — 시연 3회 완주가 그 조합 그대로였음.

---

## 7. 코드 지도 (읽는 순서)

| 관심사 | 파일 |
| --- | --- |
| 기동·조립 | `rc_car/main.py` → `app/runtime.py` (worker 6개 구성) |
| 주행 판단 | `behavior/driving_worker.py` (50Hz) → `control/lane_follower.py` |
| 경로 | `navigation/route_planner.py` · `lane_route.py` · `turn_table.py` |
| 측위 | `localization/localization_service.py` · `heading_estimator.py` · `pose_validator.py` |
| 비전 | `perception/` 전체 + `vision_runner/vision_runner.py` |
| 안전 | `safety/safety_supervisor.py` (모든 구동 명령의 단일 통과점) · `watchdog.py` |
| 통신 | `network/control_client.py` (관제) · `localization_client.py` (위치 서버) |
| 맵 | `map/main_track_map.yaml` (차선 32 · 커넥터 64) · `map/places.yaml` |

세부 실행·검증 절차는 `rc_car/README.md`, 맵 정의는 `map/README.md` 참조.
