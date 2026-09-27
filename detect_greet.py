#!/usr/bin/env python3
"""카메라에 손 든 사람이 잡히면 xArm6 가 인사한다.

행동분류 모델을 학습시키지 않는다. YOLO11-pose 로 사람 키포인트를 뽑고,
"손목이 어깨보다 위에 있다" 는 규칙으로 손 든 것을 판정한다. 전시 환경에서는
학습 모델보다 이 방식이 덜 깨지고 조명/복장 변화에도 강하다.

먼저 로봇 없이 감지만 확인한다 (팔이 움직이지 않는다):
  .venv/bin/python detect_greet.py --show

확인이 끝나면 로봇을 붙인다:
  .venv/bin/python detect_greet.py --show --robot --speed 20

종료는 q 또는 Ctrl+C.
"""
import argparse, collections, sys, time

COCO = {'nose': 0, 'l_shoulder': 5, 'r_shoulder': 6,
        'l_elbow': 7, 'r_elbow': 8, 'l_wrist': 9, 'r_wrist': 10}
KP_CONF = 0.5          # 키포인트를 믿을 최소 신뢰도


def hand_raised(kp, conf):
    """손목이 어깨보다 위에 있으면 손을 든 것으로 본다.

    이미지 좌표는 y 가 아래로 커지므로 '위' 는 y 가 작은 쪽이다.
    반환: (들었는지, 어느 손, 손목 x 정규화 전 픽셀좌표)
    """
    for side in ('l', 'r'):
        w, s = COCO['%s_wrist' % side], COCO['%s_shoulder' % side]
        if conf[w] < KP_CONF or conf[s] < KP_CONF:
            continue
        if kp[w][1] < kp[s][1]:
            return True, side, kp[w]
    return False, None, None


class WaveWatcher:
    """손목 x 좌표의 좌우 진동을 보고 흔드는지 판정한다. 모델이 필요 없다."""

    def __init__(self, window=1.5, min_amp=0.03, min_turns=2):
        self.window = window        # 초
        self.min_amp = min_amp      # 화면 폭 대비 진폭
        self.min_turns = min_turns  # 방향 전환 횟수
        self.hist = collections.defaultdict(collections.deque)

    def update(self, tid, x_norm, now):
        h = self.hist[tid]
        h.append((now, x_norm))
        while h and now - h[0][0] > self.window:
            h.popleft()
        if len(h) < 5:
            return False
        xs = [x for _, x in h]
        if max(xs) - min(xs) < self.min_amp:
            return False
        turns, prev = 0, None
        for a, b in zip(xs, xs[1:]):
            d = b - a
            if abs(d) < 1e-4:
                continue
            sign = d > 0
            if prev is not None and sign != prev:
                turns += 1
            prev = sign
        return turns >= self.min_turns

    def forget(self, tid):
        self.hist.pop(tid, None)


def main():
    ap = argparse.ArgumentParser(description='사람이 손을 들면 인사')
    ap.add_argument('--camera', type=int, default=1, help='카메라 인덱스 (Arducam)')
    ap.add_argument('--model', default='yolo11n-pose.pt')
    ap.add_argument('--conf', type=float, default=0.4, help='사람 검출 신뢰도')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--show', action='store_true', help='영상 창 표시')
    ap.add_argument('--robot', action='store_true',
                    help='실제로 팔을 움직인다. 없으면 감지만 하고 출력만 한다')
    ap.add_argument('--speed', type=float, default=20.0, help='관절 속도 deg/s')
    ap.add_argument('--name', help='poses/<이름>.json 으로 인사 (생략하면 내장 인사)')
    ap.add_argument('--require-wave', action='store_true',
                    help='손만 든 게 아니라 좌우로 흔들어야 반응')
    ap.add_argument('--hold', type=float, default=0.4,
                    help='이 시간(초) 이상 조건이 유지되면 인사 (오검출 방지)')
    ap.add_argument('--cooldown', type=float, default=8.0,
                    help='같은 사람에게 다시 인사하기까지 최소 간격(초)')
    ap.add_argument('--j1-span', type=float, default=45.0,
                    help='화면 좌우 끝에 대응하는 J1 각도')
    ap.add_argument('--flip-j1', action='store_true',
                    help='좌우가 반대로 돌면 이 옵션을 준다')
    args = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit('카메라 %d 를 열 수 없음. 인덱스를 바꿔보거나 '
                 '시스템 설정 > 개인정보 보호 및 보안 > 카메라 에서 터미널을 허용해줘.'
                 % args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    model = YOLO(args.model)

    greeter = None
    if args.robot:
        from greet import Greeter
        greeter = Greeter(speed=args.speed)
        print('로봇 연결됨. 대기 자세.')
    else:
        print('감지 전용 모드. 팔은 움직이지 않는다. 실제로 움직이려면 --robot 을 붙여줘.')

    watcher = WaveWatcher()
    since = {}          # track id -> 조건이 만족되기 시작한 시각
    last_greet = {}     # track id -> 마지막 인사 시각
    fps_t, fps_n, fps = time.time(), 0, 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('프레임 읽기 실패'); break
            now = time.time()
            h, w = frame.shape[:2]

            res = model.track(frame, persist=True, verbose=False,
                              conf=args.conf, classes=[0])[0]

            target = None       # (면적, track id, j1 각도, 어느 손)
            seen = set()
            if res.keypoints is not None and res.boxes is not None:
                kps = res.keypoints.xy.cpu().numpy()
                kcf = res.keypoints.conf
                kcf = kcf.cpu().numpy() if kcf is not None else None
                boxes = res.boxes.xyxy.cpu().numpy()
                ids = (res.boxes.id.int().cpu().numpy()
                       if res.boxes.id is not None else range(len(boxes)))

                for i, tid in enumerate(ids):
                    tid = int(tid)
                    seen.add(tid)
                    conf = kcf[i] if kcf is not None else [1.0] * len(kps[i])
                    up, side, wrist = hand_raised(kps[i], conf)
                    if not up:
                        since.pop(tid, None)
                        continue
                    if args.require_wave:
                        if not watcher.update(tid, wrist[0] / w, now):
                            continue
                    since.setdefault(tid, now)
                    if now - since[tid] < args.hold:
                        continue
                    if now - last_greet.get(tid, 0) < args.cooldown:
                        continue
                    x1, y1, x2, y2 = boxes[i]
                    area = (x2 - x1) * (y2 - y1)
                    cx = (x1 + x2) / 2 / w - 0.5
                    j1 = (cx if args.flip_j1 else -cx) * 2 * args.j1_span
                    if target is None or area > target[0]:
                        target = (area, tid, j1, side)

            for tid in list(since):
                if tid not in seen:
                    since.pop(tid, None)
                    watcher.forget(tid)

            if target:
                _, tid, j1, side = target
                last_greet[tid] = now
                since.pop(tid, None)
                print('[%s] id=%d %s손 들었음 -> J1=%+.0f 도로 인사'
                      % (time.strftime('%H:%M:%S'), tid,
                         '왼' if side == 'l' else '오른', j1))
                if greeter:
                    greeter.greet(j1_deg=j1, name=args.name)
                    # 인사 동안 쌓인 프레임을 버려 지연을 없앤다
                    for _ in range(5):
                        cap.read()

            fps_n += 1
            if now - fps_t >= 1.0:
                fps, fps_n, fps_t = fps_n / (now - fps_t), 0, now

            if args.show:
                vis = res.plot()
                cv2.putText(vis, '%.1f fps   %s' % (fps, '로봇 ON' if greeter else '감지 전용'),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow('detect_greet  (q=종료)', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print('\n중단 요청')
        if greeter:
            greeter.arm.emergency_stop()
            print('비상 정지. 다시 움직이려면 greet.py rest 로 대기 자세부터.')
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        if greeter:
            greeter.arm.disconnect()
    print('종료')


if __name__ == '__main__':
    main()
