# vision_runner/ (비전 실행체, 주행 코드와 분리된 별도 프로세스)

카메라를 소유하고 차선 관측을 게시하는 프로세스임. 주행 코드(`rc_car`)와 프로세스가 완전히 분리돼 있고, 결과는 `/dev/shm/vision_latest.json` 원자적 게시로만 전달됨. 차량 쪽이 필요할 때 읽어 가는 pull 방식임 (통신 규약 v0.3 §4·§5).

| 파일 | 역할 |
|---|---|
| `vision_runner.py` | 실행 진입점. 캡처 → 버드아이 워프 → 노란선 색 검출 + 세그 추론 → 차선 중심 계산 → JSON 게시 |
| `birdseye_extractor.py` | IMX708 렌즈 왜곡 보정 + 버드아이 투영. `calibration/` 의 JSON 두 개를 읽음 |
| `run_csi_direct.py` | CSI 카메라 프레임 공급(GStreamer 파이프라인 구성·프레임 이터레이터). 단독 실행 시 게시·추론 없이 추출기만 돌리는 점검용 |
| `seg_replay.py` | 오프라인 재처리 하네스. 녹화 오버레이만으로 `compute_seg` 변형의 가동률을 재측정함 |
| `calibration/calibration.json` | 렌즈 보정 계수 (`camera_matrix`·`dist_coeffs`, RMS·채택 이미지 수 포함) |
| `calibration/birdseye.json` | 버드아이 변환 (호모그래피·`pixels_per_meter`·출력 크기) |
| `requirements.txt` | numpy만 명시. OpenCV 는 JetPack 제공본을 그대로 씀 |

## 검출 역할 분담 (2026-08-05 확정)

- **황색선**: 색 휴리스틱 전담. HSV·LAB 임계로 검출함
  - 노란색은 색만으로 유일하게 구분 가능한 대상이라 모델에 맡기지 않음
- **점선·정지선·횡단보도·화살표**: 세그 모델 단독
  - 흰색 페인트는 색으로 구분이 안 됨. 초기의 흰색 색검출 경로는 혼동 때문에 제거했음

## 게시 형식

- 경로: `/dev/shm/vision_latest.json` (`--publish` 로 변경 가능)
- 방식: 임시 파일 기록 후 rename 으로 원자적 교체. 읽는 쪽이 반쯤 쓰인 파일을 보는 상황을 막음

| 키 | 내용 |
|---|---|
| `timestamp` | 게시 시각. 차량 쪽 신선도 판정의 유일한 근거임 |
| `frame` · `proc_ms` | 프레임 번호, 단계별 소요(`extract`·`infer`) |
| `birdseye_size` · `pixels_per_meter` | 버드아이 출력 크기, 픽셀당 미터 환산 계수 |
| `vehicle_axis_px` | 차량 진행축 열. 회전 트리거의 횡거리 게이트가 이 값을 씀. 게시해 두면 축을 재보정해도 차량 코드가 따라옴 |
| `model` | `conf`, 클래스별 검출 수, 검출 상자 목록 |
| `heuristic` | 노란색 픽셀 수 |
| `seg` | 차선 중심 계산 결과 (아래) |

`seg` 유효 시 `lateral_offset_m`·`heading_error_deg`·`rows`·`pair_rows` 를 실음. 무효 시 `valid: false` 와 `rows`·`pair_rows`, 임계 초과로 버린 경우 `reject` 를 실음.

## 차선 중심 계산 (`compute_seg`)

1. 지정된 행마다 후보를 모음: 노란선 단면 중심 + 그 행을 지나는 모델 점선 상자의 중심
2. 차량 축을 사이에 두면서 차선 폭 범위(0.25m ± 30%)에 드는 쌍의 중점을 그 행의 차선 중심으로 씀
3. 쌍이 없는 행은 편측 추정으로 보충함
   - 축에서 0.25m 안의 후보가 **정확히 하나**일 때만 경계선으로 인정하고 공칭 반폭(0.125m)을 보정함
   - 후보가 여럿이면 어느 쪽 경계인지 모호해 그 행을 포기함
4. 확보된 행이 5개 미만이면 invalid
5. 모인 점에 1차 피팅. 근거리 x 로 횡오차, 기울기로 헤딩오차를 산출함
6. 횡오차 0.20m 또는 헤딩오차 10° 를 넘으면 invalid
   - 차선 보정은 차선과 정렬된 근처에서만 의미가 있음. 코너 시야에서 나오는 큰 값은 보정 대상에서 제외함

주요 상수는 `vision_runner.py` 상단에 모아 뒀음 (`PPM` 250, `VEHICLE_AXIS_PX` 477, `SEG_ROWS` 140\~235, `MIN_ROWS` 5).

## 차량 쪽 연결

- `rc_car/perception/real_seg_model.py` 가 이 JSON 을 읽어 어댑터에 넘김
- **신선도 판정은 `SegAdapter` 한 곳에서만 함** (`freshness_max_s` 0.5s). 판정 주체를 하나로 유지하려고 게시 측·모델 측에서 중복 판정하지 않음
- 무중단 폴백: 이 프로세스가 죽으면 게시가 낡고 어댑터가 자동으로 invalid 처리함. 차량은 GPS 단독 주행을 계속함

## 운용 프로필

| 프로필 | 인자 | 용도 |
|---|---|---|
| 주행용 | `--record-dir ~/vision_rec/<run>` | 온보드 녹화. 무선 트래픽 0, 사후 재생으로 확인함 |
| 점검용 | `--stream-port 8090` | 브라우저 실시간 확인. **정차 중에만 사용** |

2026-08-05 주행 중 스트리밍으로 WiFi 가 동결되는 사고가 있어 두 프로필을 분리했음.

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

`seg_replay.py` 는 보드 없이 돌아감.

```
python seg_replay.py [프레임수]
```

## 저장소에 없는 것

| 대상 | 이유 |
|---|---|
| `best.engine` | TensorRT 엔진은 보드·JetPack 버전에 종속됨. 같은 보드에서 다시 변환해야 함 |
| 학습 데이터셋·가중치 원본 | 용량 |
| 녹화본(`vision_rec/`)·`trace.jsonl` | 용량. 재현은 `seg_replay.py` 로 함 |

## 알려진 한계

- `VEHICLE_AXIS_PX` 는 정차 실측으로 보정하는 값임. 카메라를 다시 달면 재보정이 필요함
- `seg_replay.py` 는 **가동률만** 측정함. 값의 부호·스케일 검증은 못 함
  - 0806_run1 의 GPS 는 구 변환이라 편향이 15\~18cm 로 차선 반폭을 넘고, 실제 횡변동은 1\~3cm 라 기준으로 쓸 수 없음
  - 부호 검증은 정차 3점 실측으로 함
