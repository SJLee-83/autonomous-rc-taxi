"""팀 구동 코드 사본 (2026-07-24 수신) — ⚠️ 이 디렉터리의 파일은 수정 금지.

- 원본: 프로젝트 루트 `motor_controller.py` / `servo_controller.py` (팀 기준 코드)
- 이 사본은 Jetson 배포 시 rc_car 폴더만 복사하면 되도록 동봉한 것이다.
- 원본과의 동일성은 `tests/unit/test_hardware_bridge.py`가 검사한다.
- 이 코드들은 최상위 `config` 모듈(상수 8개)을 기대한다
  → `hardware/vendor_bridge.py`가 `config/hardware.yaml`에서 생성·주입한다.
"""
