"""합성 카메라 — 실물 카메라·라즈베리파이·마커·차량 없이 GPS 서버를 돌린다.

ArUco 마커를 그린 이미지를 만들어 GPS 서버의 `/ws/v1/camera`로 밀어 넣는다.
라즈베리파이 카메라 클라이언트 자리를 대신하는 것이므로 **GPS 서버가 먼저 떠 있어야 한다.**

    1) 기준 마커 0~3을 맵 네 꼭짓점에 놓은 프레임을 보내고 `initialize` 수행
    2) 마커 4를 지정 좌표·각도에 놓고 계속 push → `/ws/v1/localization`이 found=true를 낸다

실행 (cv2가 필요하므로 **GPS 서버 venv의 파이썬**으로 돌린다):

    <GPS서버경로>\\.venv\\Scripts\\python.exe tools/fake_camera.py
    ... tools/fake_camera.py --seconds 5            # 5초 뒤 멈춤 → 유실 경로 검증
    ... tools/fake_camera.py --x 3.0 --y 2.0 --heading 180

좌표 정답을 코드가 알고 있으므로, 수신된 pose와 비교하면 좌표계·heading **방향**을
실측 없이 회귀 검증할 수 있다 (2026-07-27: heading 90° 지정 → 88.2° 수신 확인).

⚠️ **거리 정확도는 믿지 말 것.** 카메라 내부 파라미터가 근사값이라 좌표에 수~십 cm
오차가 있고 중심에서 멀수록 커진다. 이 도구의 용도는 **프로토콜·상태 흐름 검증**이지
캘리브레이션 기준이 아니다. 실제 정밀도는 실환경 ChArUco 보정 후 측정한다.

⚠️ 원본 GPS 서버 코드(`Desktop\\GPS`)는 건드리지 않는다 — vendor 무수정 원칙.
"""
import argparse
import asyncio
import json
import math
import ssl
import sys
import time

import cv2
import numpy as np

# 가상 천장 카메라 — 맵 전체를 위에서 내려다본다
PX_PER_M = 384.0
SPACE_W_M, SPACE_H_M = 5.0, 3.0        # protocol_2 §2.6 유효 범위와 같게 잡는다
MARKER_M = 0.105                        # GPS config.json의 marker_sizes_m
MARKER_PX = int(MARKER_M * PX_PER_M)

IMG_W, IMG_H = int(SPACE_W_M * PX_PER_M), int(SPACE_H_M * PX_PER_M)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def world_to_px(x_m: float, y_m: float) -> "tuple[int, int]":
    """월드(원점=왼쪽 아래, +y 위) → 이미지 픽셀(원점=왼쪽 위)."""
    return int(x_m * PX_PER_M), int((SPACE_H_M - y_m) * PX_PER_M)


def draw(markers: dict, rot_deg: "dict | None" = None):
    """markers: {marker_id: (x_m, y_m)} / rot_deg: {marker_id: 반시계 각도(도)}

    회전은 √2 패딩 캔버스에서 수행한다 — 타일 자기 크기 안에서 돌리면 90° 배수가
    아닌 각도에서 모서리가 잘려, 검출 pose 왜곡(드리프트)과 검출 실패(유실)를 만든다
    (⑦ 통합 시뮬에서 실측 발견 — 폐루프 주행은 연속 각도를 쓴다).
    """
    img = np.full((IMG_H, IMG_W), 255, dtype=np.uint8)
    for mid, (x_m, y_m) in markers.items():
        tile = cv2.aruco.generateImageMarker(DICT, mid, MARKER_PX)
        angle = (rot_deg or {}).get(mid, 0.0)
        if angle:
            pad = int(MARKER_PX * 0.25) + 2      # (√2−1)/2 ≈ 0.207 — 여유 있게
            tile = cv2.copyMakeBorder(tile, pad, pad, pad, pad,
                                      cv2.BORDER_CONSTANT, value=255)
            c = tile.shape[0] / 2
            # 이미지 y축이 아래로 증가하므로 월드 반시계 = 이미지상 음의 회전
            m = cv2.getRotationMatrix2D((c, c), -angle, 1.0)
            tile = cv2.warpAffine(tile, m, tile.shape[::-1],
                                  borderValue=255, flags=cv2.INTER_LINEAR)
        size = tile.shape[0]
        cx, cy = world_to_px(x_m, y_m)
        x0, y0 = cx - size // 2, cy - size // 2
        img[y0:y0 + size, x0:x0 + size] = tile
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


async def send_frame(ws, frame, frame_id: int) -> dict:
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError("JPEG 압축 실패")
    await ws.send(json.dumps({"type": "frame_meta", "frame_id": frame_id,
                              "captured_at": time.time()}))
    await ws.send(jpeg.tobytes())
    return json.loads(await ws.recv())


async def run(args) -> int:
    from websockets.asyncio.client import connect

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False              # 자체 서명 인증서 (GPS 서버 §3.1)
    ctx.verify_mode = ssl.CERT_NONE
    ssl_ctx = ctx if args.url.startswith("wss://") else None

    margin = MARKER_M                        # 마커가 이미지 밖으로 나가지 않게 안쪽으로
    refs = {0: (margin, margin),                                  # 왼쪽 아래 = 원점
            1: (SPACE_W_M - margin, margin),                      # 오른쪽 아래 → +x
            2: (SPACE_W_M - margin, SPACE_H_M - margin),
            3: (margin, SPACE_H_M - margin)}

    async with connect(args.url, ssl=ssl_ctx, max_size=16 * 1024 * 1024) as ws:
        hello = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        if hello.get("type") != "hello":
            print(f"카메라 소켓이 아니다: {hello}")
            return 1

        if not args.skip_init:
            ref_frame = draw(refs)
            result = {}
            for i in range(5):
                result = await send_frame(ws, ref_frame, i)
            print(f"기준 마커 검출: {result.get('detected_ids')}")
            await ws.send(json.dumps({"type": "initialize"}))
            res = json.loads(await ws.recv())
            if res.get("type") != "space_built":
                print(f"Initialize 실패: {res}")
                return 1
            sp = res["space"]
            print(f"공간 확정: width={sp['width_m']:.3f}m depth={sp['depth_m']:.3f}m")

        # 마커 이미지를 그대로 놓으면(rot=0) 위쪽(local +y)이 월드 +y라 heading 90°가 된다.
        # draw()가 이미 이미지 좌표계로 부호를 뒤집으므로 여기서 한 번 더 뒤집는다
        rot = {4: 90.0 - args.heading}
        frame = draw({4: (args.x, args.y)}, rot_deg=rot)
        started = time.monotonic()
        deadline = started + args.seconds if args.seconds else None
        print(f"마커 4 push: ({args.x}, {args.y}) heading {args.heading}deg, "
              f"speed {args.speed}m/s, {args.fps}fps, {args.seconds or 'infinite'}s")

        frame_id = 100
        limit = SPACE_W_M - MARKER_M * 1.5
        while deadline is None or time.monotonic() < deadline:
            if args.speed:
                # heading 방향으로 등속 이동 — 차량이 추정한 speed와 대조할 정답값이다
                d = args.speed * (time.monotonic() - started)
                x = args.x + d * math.cos(math.radians(args.heading))
                y = args.y + d * math.sin(math.radians(args.heading))
                if not (MARKER_M * 1.5 <= x <= limit
                        and MARKER_M * 1.5 <= y <= SPACE_H_M - MARKER_M * 1.5):
                    print(f"맵 경계 도달 ({x:.2f}, {y:.2f}) - push 종료")
                    break
                frame = draw({4: (x, y)}, rot_deg=rot)
            await send_frame(ws, frame, frame_id)
            frame_id += 1
            await asyncio.sleep(1.0 / args.fps)
        print("push 종료 (차량은 0.6초 뒤 위치 유실로 판정해야 한다)")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="GPS 서버용 합성 카메라")
    ap.add_argument("--url", default="wss://127.0.0.1:8000/ws/v1/camera")
    ap.add_argument("--x", type=float, default=2.0, help="마커 4 x 좌표(m)")
    ap.add_argument("--y", type=float, default=1.0, help="마커 4 y 좌표(m)")
    ap.add_argument("--heading", type=float, default=90.0,
                    help="전방 각도(도). protocol_2 §2.7 — +x=0, +y=90, 반시계")
    ap.add_argument("--speed", type=float, default=0.0,
                    help="heading 방향 등속 이동(m/s). 0이면 정지. "
                         "차량 max_speed 0.2076 참고 — 추정 속도 검증용 정답값")
    ap.add_argument("--fps", type=float, default=12.0,
                    help="라즈베리파이 클라이언트 기본값과 동일")
    ap.add_argument("--seconds", type=float, default=0.0, help="0이면 무한")
    ap.add_argument("--skip-init", action="store_true",
                    help="이미 Initialize된 공간을 재사용")
    args = ap.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
