#!/usr/bin/env python3
"""카메라에 손 든 사람이 잡히면 xArm6 가 인사한다.

행동분류 모델을 학습시키지 않는다. YOLO11-pose 로 사람 키포인트를 뽑고,
"손목이 어깨보다 위에 있다" 는 규칙으로 손 든 것을 판정한다. 전시 환경에서는
학습 모델보다 이 방식이 덜 깨지고 조명/복장 변화에도 강하다.

먼저 로봇 없이 감지만 확인한다 (팔이 움직이지 않는다):
  .venv/bin/python detect_greet.py --list-cameras   # 어느 인덱스가 IMX477 인지 확인
  .venv/bin/python detect_greet.py --show

확인이 끝나면 로봇을 붙인다:
  .venv/bin/python detect_greet.py --list-cameras   # 어느 인덱스가 IMX477 인지 확인
  .venv/bin/python detect_greet.py --show --robot --speed 20

종료는 q 또는 Ctrl+C.
"""
import argparse, collections, os, sys, time

COCO = {'nose': 0, 'l_shoulder': 5, 'r_shoulder': 6,
        'l_elbow': 7, 'r_elbow': 8, 'l_wrist': 9, 'r_wrist': 10}
KP_CONF = 0.5          # 키포인트를 믿을 최소 신뢰도

FONT_PATH = '/System/Library/Fonts/AppleSDGothicNeo.ttc'


def _up(kp, conf, side):
    """해당 쪽 손목이 어깨보다 위에 있는지. 신뢰도가 낮으면 None."""
    w, sh = COCO['%s_wrist' % side], COCO['%s_shoulder' % side]
    if conf[w] < KP_CONF or conf[sh] < KP_CONF:
        return None
    return kp[w][1] < kp[sh][1]


def classify(kp, conf, waving):
    """키포인트로 포즈를 분류한다. (한글 라벨, 화면용 ASCII 라벨) 를 돌려준다."""
    l, r = _up(kp, conf, 'l'), _up(kp, conf, 'r')
    if l is None and r is None:
        return '판정불가', 'UNKNOWN'
    if l and r:
        return '양손 들기', 'BOTH HANDS UP'
    if l or r:
        if waving:
            return '손 흔들기', 'WAVING'
        return '손 들기', 'HAND UP'
    return '서 있음', 'STANDING'


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


_font_cache = {}


def draw_label(frame, text, xy, color=(0, 255, 0), size=22):
    """OpenCV 는 한글을 못 그리므로 PIL 로 그려 넣는다."""
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    if size not in _font_cache:
        try:
            _font_cache[size] = ImageFont.truetype(FONT_PATH, size)
        except OSError:
            _font_cache[size] = None
    font = _font_cache[size]
    if font is None:
        cv2.putText(frame, text.encode('ascii', 'replace').decode(), xy,
                    cv2.FONT_HERSHEY_SIMPLEX, size / 30, color, 2)
        return frame
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(img)
    x, y = xy
    d.text((x + 1, y + 1), text, font=font, fill=(0, 0, 0))
    d.text((x, y), text, font=font, fill=color[::-1])
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser(description='사람이 손을 들면 인사')
    ap.add_argument('--camera', type=int, default=0,
                    help='카메라 인덱스. Arducam IMX477 은 보통 0, 맥북 내장은 1')
    ap.add_argument('--list-cameras', action='store_true',
                    help='연결된 카메라의 인덱스와 해상도를 출력하고 종료')
    ap.add_argument('--model', default='yolo11n-pose.pt')
    ap.add_argument('--device', default='auto',
                    help="추론 장치. auto 면 애플 실리콘 GPU(mps)를 쓴다")
    ap.add_argument('--imgsz', type=int, default=640, help='추론 입력 크기')
    ap.add_argument('--tracker', default='bytetrack.yaml',
                    help='트래커. 기본 ByteTrack. 카메라가 고정이라 BoT-SORT 의 '
                         'GMC(카메라 움직임 보정)가 필요 없고, OpenCV 5 에서 GMC 가 '
                         '깨져 경고를 쏟아낸다')
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
    ap.add_argument('--every', type=float, default=0.0,
                    help='분류값을 이 간격(초)마다 계속 출력. 0 이면 바뀔 때만 출력')
    args = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    if args.list_cameras:
        # 없는 인덱스를 열면 OpenCV 가 stderr 로 경고를 뱉는다. 잠시 막아둔다.
        import contextlib
        devnull = os.open(os.devnull, os.O_WRONLY)
        saved = os.dup(2)
        os.dup2(devnull, 2)
        print('index  기본 해상도    최대 해상도     추정')
        misses = 0
        for i in range(8):
            if misses >= 2:      # 연속 2개가 없으면 더 볼 필요가 없다
                break
            c = cv2.VideoCapture(i)
            if not c.isOpened():
                misses += 1
                c.release()
                continue
            misses = 0
            ok, f = c.read()
            c.set(cv2.CAP_PROP_FRAME_WIDTH, 4056)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, 3040)
            mw, mh = c.get(cv2.CAP_PROP_FRAME_WIDTH), c.get(cv2.CAP_PROP_FRAME_HEIGHT)
            guess = 'Arducam IMX477' if mw * mh > 6e6 else '내장 카메라로 보임'
            print('%5d  %4dx%-4d     %4dx%-4d     %s'
                  % (i, f.shape[1] if ok else 0, f.shape[0] if ok else 0,
                     mw, mh, guess))
            c.release()
        os.dup2(saved, 2)
        os.close(saved)
        os.close(devnull)
        return

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit('카메라 %d 를 열 수 없음. 인덱스를 바꿔보거나 '
                 '시스템 설정 > 개인정보 보호 및 보안 > 카메라 에서 터미널을 허용해줘.'
                 % args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    device = args.device
    if device == 'auto':
        import torch
        device = 'mps' if torch.backends.mps.is_available() else 'cpu'
    print('추론 장치: %s' % device)
    model = YOLO(args.model)

    greeter = None
    if args.robot:
        from greet import Greeter
        greeter = Greeter(speed=args.speed)
        print('로봇 연결됨. 대기 자세.')
    else:
        print('감지 전용 모드. 팔은 움직이지 않는다. 실제로 움직이려면 --robot 을 붙여줘.')

    watcher = WaveWatcher()
    last_label = {}     # track id -> 마지막으로 출력한 분류값
    last_print = 0.0
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
                              conf=args.conf, classes=[0],
                              device=device, imgsz=args.imgsz,
                              tracker=args.tracker)[0]

            target = None       # (면적, track id, j1 각도, 어느 손)
            seen = set()
            labels = {}         # track id -> (한글 라벨, ASCII 라벨, 박스)
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

                    # 손을 들었을 때만 흔들림을 본다 (손목 x 진동)
                    waving = (watcher.update(tid, wrist[0] / w, now)
                              if up else False)
                    label_ko, label_en = classify(kps[i], conf, waving)
                    labels[tid] = (label_ko, label_en, boxes[i])

                    # 터미널 출력: 바뀔 때만, 또는 --every 간격마다
                    if args.every > 0:
                        if now - last_print >= args.every:
                            show_line = True
                        else:
                            show_line = False
                    else:
                        show_line = last_label.get(tid) != label_ko
                    if show_line:
                        lw, ls = COCO['l_wrist'], COCO['l_shoulder']
                        rw, rs = COCO['r_wrist'], COCO['r_shoulder']
                        print('[%s] id=%-2d %-10s  왼손목y=%4.0f(어깨%4.0f) '
                              '오른손목y=%4.0f(어깨%4.0f) 흔듦=%s'
                              % (time.strftime('%H:%M:%S'), tid, label_ko,
                                 kps[i][lw][1], kps[i][ls][1],
                                 kps[i][rw][1], kps[i][rs][1], waving))
                    last_label[tid] = label_ko

                    if not up:
                        since.pop(tid, None)
                        continue
                    if args.require_wave and not waving:
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

            if args.every > 0 and now - last_print >= args.every:
                last_print = now
                if not labels:
                    print('[%s] 사람 없음' % time.strftime('%H:%M:%S'))

            for tid in set(list(since) + list(last_label)):
                if tid not in seen:
                    since.pop(tid, None)
                    last_label.pop(tid, None)
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
                for tid, (ko, en, box) in labels.items():
                    x1, y1 = int(box[0]), int(box[1])
                    color = (0, 255, 255) if ko in ('손 흔들기', '손 들기',
                                                    '양손 들기') else (0, 255, 0)
                    vis = draw_label(vis, 'id%d %s' % (tid, ko),
                                     (x1, max(0, y1 - 28)), color)
                vis = draw_label(vis, '%.1f fps   %s' %
                                 (fps, '로봇 ON' if greeter else '감지 전용'),
                                 (10, 8), (0, 255, 0))
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
