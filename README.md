# RC카 자율주행 - 차량 파트 제출본 (2026-08-10)

2026-08-10 시연에서 **완주**한 차량 주행 코드와, 그 차량에 비전 신호를
공급하는 **비전 실행체**를 한 묶음으로 정리한 것.

| 폴더 | 무엇 | 실행 위치 |
| --- | --- | --- |
| `rc_car/` | 차량 주행 프로세스 (측위·경로·조향·관제 통신·안전) | Jetson Orin Nano |
| `vision_runner/` | 비전 프로세스 (CSI 캡처 → 버드아이 → YOLO seg → 결과 게시) | 같은 보드, **별도 프로세스** |
| `map/` | 차선 그래프·장소 정의 (`rc_car` 가 `../map` 으로 참조) | - |

```
. (repo 루트)
├── rc_car/            # 주행 (python3 main.py)
├── vision_runner/     # 비전 (python3 vision_runner.py)
├── map/               # main_track_map.yaml · places.yaml
└── README.md
```

---

## 1. 두 프로세스가 어떻게 만나는가

주행 코드는 카메라에 직접 접근하지 않고 비전이 인식 결과를 tmpfs 파일에 계속 덮어쓰면,
주행 쪽이 자기 주기(10Hz)로 그 파일을 읽어감(pull). 주행과 비전 프로세스는 파일 단위로 통신.

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

게시 파일(`vision_latest.json`)의 필드와 읽는 쪽의 사용처:

| 필드 | 읽는 모듈 (주행 프로세스) | 기능 |
| --- | --- | --- |
| `timestamp` | `SegAdapter` / `MarkAdapter` | 최신 여부 판정<br>- 기록 후 0.5초가 지난 데이터는 **invalid** 처리 |
| `seg.{valid, lateral_offset_m, heading_error_deg}` | `RealSegModel` → `SegAdapter` | 차선 중앙 보정량 |
| `model.detections[].{cls, conf, xyxy_px}` | `FileMarkSource` → `MarkAdapter` | 회전 트리거 판정 |
| `pixels_per_meter`, `vehicle_axis_px`, `birdseye_size` | `MarkAdapter` | 픽셀 → 미터 환산·횡거리 게이트 |

비전 프로세스에 문제가 생겼을 때의 처리 순서:

1. 게시 파일 갱신이 멈춤
2. 기록 후 0.5초가 지난 데이터는 invalid 처리
3. 차선 보정량 0, 회전 트리거 미작동
4. 주행은 GPS 단독으로 계속, 회전은 좌표 기준으로 개시

---

## 2. 비전 기반 차선 보조 (직진 중 횡보정)

차선 중앙에서 얼마나 벗어났는지(`lateral_offset_m`)와 차선과 얼마나 틀어졌는지
(`heading_error_deg`)를 받아 **바퀴각에 보정을 더함**. 기저 조향은 항상 GPS 경로 추종이고,
비전은 그 위에 얹히는 제한된 보정.

| 파일 | 역할 |
| --- | --- |
| `vision_runner/vision_runner.py` `compute_seg()` | 행별로 황색선 단면 + 모델 점선 상자를 모아 차량 축을 감싸는 차선 폭 쌍의 중점을 구하고, 그 점들에 직선을 적합해 offset/heading 산출. 쌍이 없는 행은 편측 추정(경계선 + 공칭 반폭)으로 보충 |
| `rc_car/perception/real_seg_model.py` | 게시 파일 클라이언트. `latest()` 로 최신 seg 반환 |
| `rc_car/perception/mock_seg_model.py` | 시뮬레이션용 대체 모델<br>- 실제 비전 없이 차량 위치와 차선 그래프에서 offset/heading 을 계산해 공급 (실차에서는 미사용) |
| `rc_car/perception/seg_adapter.py` | - 4개의 필드 검증<br>- 최신 여부 판정<br>- 부호 반전<br>- 보정 계산 및 클램프<br>- 연속 invalid 시 GPS 단독 전환을 로그로 기록 |
| `rc_car/perception/perception_worker.py` | 10Hz 관측 스레드 |
| `rc_car/config/perception.yaml` | 설정 파일<br>- seg 동작 모드(off/mock/real)<br>- 게시 파일 경로<br>- 보정 강도(게인)<br>- 보정 상한 |

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
  invalid_fallback_after: 3               # 연속 invalid 3회면 GPS 단독 전환 로그
  freshness_max_s: 0.5                    # 기록 후 이 시간이 지나면 invalid
```

---

## 3. 조향 트리거 (회전 개시 시점 결정)

노면표시(횡단보도·정지선·점선·방향화살표)가 화면 하단에 충분히 가까워지는 시점에
회전을 시작함.

| 파일 | 역할 |
| --- | --- |
| `rc_car/perception/vision_marks.py` | `MarkAdapter`: 검출 목록에서 트리거 조건 판정 |
| `rc_car/navigation/turn_table.py` + `config/turn_table.yaml` | 회전별 트리거 사양·조향각·종료 조건 표 |
| `rc_car/control/lane_follower.py` | 상태 기계 `FOLLOW → ARMED → TURNING → 정렬 종료` |

**상태 기계**

```
FOLLOW ──(차선 끝까지 arm_distance)──▶ ARMED ──(① 비전 트리거 작동 or ② 좌표 기준)──▶ TURNING
                                                                                      │
                                            (방위 정렬 or 최대 거리 상한) ◀────────────┘
```

**트리거 판정 4조건** (`vision_marks.py`)

| # | 조건 | 뜻 |
| --- | --- | --- |
| ① | 클래스 일치 (`crosswalk` / `stop_line` / `direction_arrow` / `dashed_line`) | 어떤 표식으로 꺾을지 |
| ② | `conf >= min_conf` | 오검출 배제 |
| ③ | 상자 **하단** y ≥ 높이 × `near_row_frac` | 표식이 차에 그만큼 가까워짐<br>- 이 값으로 회전 시작 시점을 조절 |
| ④ | 상자 중심의 횡거리 ≤ `max_lateral_m` | 다른 차선의 표식 제외 (실측에서 정지선이 차선 5~6개 밖까지 검출됨) |

`near_row_frac` 이 낮을수록 표식이 멀리 있을 때 트리거가 작동해 **일찍 꺾음.**

**비전이 멈춰도 회전은 진행됨**
- 트리거가 끝까지 작동하지 않으면 `fallback_at_lane_end` 설정에 따라 차선 끝 좌표에서 회전을 시작함.

---

## 4. 실행

### 보드 (실차)

```bash
# ① 비전: 주행 프로필 (무선 전송 없음, 온보드 녹화)
cd vision_runner
python3 vision_runner.py --record-dir ~/vision_rec/run1

# ② 주행: seg_mode:real 은 비전 게시 파일이 있어야 기동함
cd rc_car
python3 main.py >> ~/veh_MMDD.log 2>&1
```

`vision_runner.py --stream-port 8090` 은 브라우저 실시간 확인용이며 **정차 중에만** 사용
(주행 중 무선 전송이 WiFi 동결을 유발한 사고가 있었음).

### PC (차·보드·카메라 없이)

```bash
cd rc_car
python3 main.py --driver-mode mock --seg-mode off    # 비전 없이 전 로직 구동
python3 main.py --driver-mode mock --seg-mode mock   # 가상 seg 데이터로 통합 시뮬
python3 -m pytest tests -q
```

`seg_mode` 는 **config 한 줄 또는 `--seg-mode` 플래그**로 `off / mock / real` 을 전환.
`real` 은 비전 게시 파일이 없으면 **기동하지 않음**.

### 의존

- 주행: `pyyaml`, `websockets` (+ 보드에서 `Adafruit_PCA9685`)
- 비전: `vision_runner/requirements.txt` 의 `opencv-python`, `numpy`, `ultralytics`(TensorRT `best.engine`)

### 실행 전 채울 값

실제 주행 시 `rc_car/config/network.yaml` 의 두 줄을 환경에 맞게 채움.

```yaml
localization:
  websocket_url: ws://<gps-server-host>:8100/ws/v1/localization   # 위치(ArUco) 서버
control_server:
  websocket_url: wss://<control-server-host>/ws/vehicle?token=<VEHICLE_TOKEN>   # 관제 서버
```

---

## 5. 시연 결과 (2026-08-10)

정차점: 면사무소 (1.20, 0.39) / 우리집 (0.39, 2.25)

| # | 시각 | 면사무소 도착 오차 | 우리집 도착 오차 |
| --- | --- | --- | --- |
| 1 | 07:45 | 16.3 cm | 19.3 cm |
| 2 | 08:59 | 16.8 cm | 18.0 cm |
| 3 | 09:16 | **12.2 cm** | **8.7 cm** |

- 3회 모두 **사람 개입 없이** 호출 → 배차 → 이동 → 도착 → `complete` → 다음 배차까지 완주
- 관제 왕복(`complete` ↔ `stop`) 6회 전건 성공
- 기동 시 등록된 회전 트리거 표 7종: `11->22, 4->29, 29->7, 7->22, 20->3, 22->14, 14->19`

---

## 6. 주요 코드 구성 (읽는 순서)

| 관심사 | 파일 |
| --- | --- |
| 기동·초기화 | `rc_car/main.py` → `app/runtime.py` (워커 구성) |
| 주행 판단 | `behavior/driving_worker.py` (50Hz) → `control/lane_follower.py` |
| 경로 | `navigation/route_planner.py` · `lane_route.py` · `turn_table.py` |
| 측위 | `localization/localization_service.py` · `heading_estimator.py` · `pose_validator.py` |
| 비전 | `perception/` 전체 + `vision_runner/vision_runner.py` |
| 안전 | `safety/safety_supervisor.py` (모든 구동 명령의 단일 통과점) · `watchdog.py` |
| 통신 | `network/control_client.py` (관제) · `localization_client.py` (위치 서버) |
| 맵 | `map/main_track_map.yaml` (차선 32 · 커넥터 64) · `map/places.yaml` |

세부 실행·검증 절차는 `rc_car/README.md`, 맵 정의는 `map/README.md` 참조.

---

## 7. 트러블슈팅과 개선 방향

### 개발 중 해결한 주요 문제

| 문제 | 원인 | 해결 |
| --- | --- | --- |
| 주행 프로세스가 통째로 멈춘 상태에서 모터가 켜진 채 트랙 이탈 | WiFi 연결이 잠깐 끊김 → 로그 출력 정체 → 로깅 락에 전 스레드가 함께 멈춤 | 로깅 비블로킹화 + 독립 프로세스 `guard.py` 신설<br>- 하트비트 파일이 0.7초 이상 갱신되지 않으면 I2C 로 모터 전원을 직접 차단 |
| 직선 주행 중 차선 이탈·인도 침범 | GPS heading 이 직선에서 +17\~80° 튐. 임계값을 넘을 때만 다른 값으로 대체하는 방식은 값이 왔다 갔다 하는 진동 유발 | 연속 헤딩 추정기로 교체<br>- 자전거 모델로 방위를 이어가며 이동 방향을 조금씩 혼합<br>- 회전 후 정렬 오차 158\~174° → 7.7\~11.9° |
| 회전 반경이 계산값보다 큼 | 조향이 지령각의 약 40%만 반영되는 하드웨어 편차 | 원 주행으로 실제 회전 반경을 직접 실측, 실측값 기반 고정 조향으로 전환 |
| 정차 후 도착 보고(complete)가 나가지 않는 교착 | 정차 지점이 차선 끝에 가까우면 조금만 지나쳐 서도 목표 차선을 벗어나 매칭 실패 | 차선과 차선을 잇는 연결 구간 위에 서면 앞뒤 차선을 모두 도착으로 인정 |
| I2C 오류 반복으로 모터 제어 불능 | 정지 상태 최대 조향 시 전류 급증 → 전압 강하 → 모터 드라이버 칩(PCA9685) 리셋 | 재시도 대신 칩 재초기화(리셋되면 PWM 주파수 설정이 사라짐), 정지 중 조향 동작 제한 |
