"""시연 조향 입력원 — vision segmentation (D3 결정: seg 승격 / GPS 폴백).

⑥ 완료: 계약 §8 mock(정답 seg) + SegAdapter(검증·부호 반전·보정 제한) + 10Hz worker.
seg_mode(off/mock/real) 교체는 config/perception.yaml 1줄. 실모델은 Phase 4.
정지선·장애물 인지는 여전히 추가 기능 자리 (protocol_2 §11).
"""
