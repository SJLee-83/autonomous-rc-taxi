# vision_runner/ (비전 실행체, 주행 코드와 분리된 별도 프로세스)

카메라를 소유하고 차선 관측을 게시하는 프로세스. 주행 코드(`rc_car`)와 프로세스 완전 분리, 전달은 `/dev/shm/vision_latest.json` 원자적 게시 단일 경로, 차량 쪽이 필요할 때 읽는 pull 방식 (통신 규약 v0.3 §4·§5)

| 파일 | 역할 |
|---|---|
| `vision_runner.py` | 실행 진입점. 캡처 → 버드아이 워프 → 노란선 색 검출 + 세그 추론 → 차선 중심 계산 → JSON 게시 |
| `birdseye_extractor.py` | **Vision 파트 제공 모듈.** IMX708 렌즈 왜곡 보정 + 버드아이 투영. `calibration/` 의 JSON 2종 사용 |
| `run_csi_direct.py` | **Vision 파트 제공 모듈.** CSI 카메라 프레임 공급(GStreamer 파이프라인 구성·프레임 이터레이터). 단독 실행 시 게시·추론 없이 추출기만 도는 점검용 |
| `seg_replay.py` | 오프라인 재처리 하네스. 녹화 오버레이만으로 `compute_seg` 변형의 가동률 재측정 |
| `calibration/calibration.json` | 렌즈 보정 계수 (`camera_matrix`·`dist_coeffs`, RMS·채택 이미지 수 포함) |
| `calibration/birdseye.json` | 버드아이 변환 (호모그래피·`pixels_per_meter`·출력 크기) |
| `requirements.txt` | numpy 만 명시. OpenCV 는 JetPack 제공본 사용 |

> 출처: `birdseye_extractor.py` 와 `run_csi_direct.py` 는 Vision 파트에서 받은 실행 의존성. 나머지 파일과 이 문서는 차량 파트 작성

## 검출 역할 분담

- **황색선**: 색 휴리스틱 전담 (HSV·LAB 임계). 노란색은 색만으로 유일하게 구분 가능한 대상이라 모델 제외
- **점선·정지선·횡단보도·화살표**: 세그 모델 단독. 흰색 페인트는 색 구분 불가라 초기의 흰색 색검출 경로는 혼동으로 제거

## 게시 형식

- 경로: `/dev/shm/vision_latest.json` (`--publish` 로 변경 가능)
- 방식: 임시 파일 기록 후 rename 원자적 교체 (읽는 쪽의 미완성 파일 관측 차단)

| 키 | 내용 |
|---|---|
| `timestamp` | 게시 시각. 차량 쪽 최신 여부 판정의 유일한 근거 |
| `frame` · `proc_ms` | 프레임 번호, 단계별 소요(`extract`·`infer`) |
| `birdseye_size` · `pixels_per_meter` | 버드아이 출력 크기, 픽셀당 미터 환산 계수 |
| `vehicle_axis_px` | 차량 진행축 열. 회전 트리거 횡거리 게이트의 입력이며, 게시해 두면 축 재보정에도 차량 코드가 추종 |
| `model` | `conf`, 클래스별 검출 수, 검출 상자 목록 |
| `heuristic` | 노란색 픽셀 수 |
| `seg` | 차선 중심 계산 결과 (아래) |

`seg` 유효 시 `lateral_offset_m`·`heading_error_deg`·`rows`·`pair_rows`, 무효 시 `valid: false` 와 `rows`·`pair_rows`, 임계 초과 기각 시 `reject` 동반

## 차선 중심 계산 (`compute_seg`)

1. 지정된 행마다 후보 수집: 노란선 단면 중심 + 그 행을 지나는 모델 점선 상자의 중심
2. 차량 축을 사이에 두면서 차선 폭 범위(0.25m ± 30%)에 드는 쌍의 중점 = 그 행의 차선 중심
3. 쌍이 없는 행은 편측 추정으로 보충. 축에서 0.25m 안의 후보가 **정확히 하나**일 때만 경계선으로 인정하고 공칭 반폭(0.125m) 보정, 후보가 여럿이면 경계 모호로 그 행 포기
4. 확보 행 5개 미만이면 invalid
5. 모인 점에 1차 피팅, 근거리 x 로 횡오차 · 기울기로 헤딩오차 산출
6. 횡오차 0.20m 또는 헤딩오차 10° 초과 시 invalid. 차선 보정은 차선 정렬 근처에서만 유효하므로 코너 시야의 큰 값은 보정 대상에서 제외

주요 상수는 `vision_runner.py` 상단에 집약 (`PPM` 250, `VEHICLE_AXIS_PX` 477, `SEG_ROWS` 140\~235, `MIN_ROWS` 5)

## 차량 쪽 연결

- `rc_car/perception/real_seg_model.py` 가 이 JSON 을 읽어 어댑터에 전달
- **최신 여부 판정 주체는 `SegAdapter` 단일** (`freshness_max_s` 0.5s). 게시 측·모델 측의 중복 판정 배제
- 무중단 대체 동작: 이 프로세스 중단 시 게시가 낡고 어댑터가 자동 invalid 처리, 차량은 GPS 단독 주행 지속

## 운용 프로필

| 프로필 | 인자 | 용도 |
|---|---|---|
| 주행용 | `--record-dir ~/vision_rec/<run>` | 온보드 녹화. 무선 트래픽 0, 사후 재생 확인 |
| 점검용 | `--stream-port 8090` | 브라우저 실시간 확인. **정차 중 한정** |

프로필 분리 사유는 2026-08-05 주행 중 스트리밍에 의한 WiFi 동결 사고

## 실행

```
python3 vision_runner.py \
    --engine ~/vision/best.engine \
    --fps 5 --conf 0.4 \
    --publish /dev/shm/vision_latest.json \
    --record-dir ~/vision_rec/run1
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--engine` | `~/vision/best.engine` | TensorRT 세그 엔진 경로 |
| `--fps` | 5 | 처리 목표 주기 |
| `--conf` | 0.4 | 모델 신뢰도 임계 |
| `--calibration-dir` | `calibration/` | 보정 JSON 위치 |
| `--publish` | `/dev/shm/vision_latest.json` | 게시 경로 |
| `--record-dir` · `--record-every` | 없음 · 2 | 오버레이 녹화 경로, 저장 간격(프레임) |
| `--stream-port` · `--stream-fps` | 0 · 2.0 | 점검 스트림 포트, 전송 주기 |
| `--sensor-id` · `--sensor-mode` | 0 · 1 | CSI 센서 선택·모드 |
| `--max-frames` | 0 | 0 이면 무제한 |

`seg_replay.py` 는 보드 없이 실행 가능

```
python seg_replay.py [프레임수]
```

## 저장소에 없는 것

| 대상 | 이유 |
|---|---|
| `best.engine` | TensorRT 엔진은 보드·JetPack 버전 종속. 같은 보드에서 재변환 필요 |
| 학습 데이터셋·가중치 원본 | 용량 |
| 녹화본(`vision_rec/`)·`trace.jsonl` | 용량. 재현은 `seg_replay.py` |

## 알려진 한계

- `VEHICLE_AXIS_PX` 는 정차 실측 보정값. 카메라 재장착 시 재보정 필요
- `seg_replay.py` 는 **가동률만** 측정. 값의 부호·스케일 검증 불가
  - 0806_run1 의 GPS 는 구 변환이라 편향 15\~18cm 로 차선 반폭 초과, 실제 횡변동은 1\~3cm 라 기준 부적합
  - 부호 검증은 정차 3점 실측으로 수행
