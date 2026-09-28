#!/usr/bin/env python3
"""카메라에 손 든 사람이 잡히면 xArm6 가 인사한다.

행동분류 모델을 학습시키지 않는다. YOLO11-pose 로 사람 키포인트를 뽑고,
"손목이 어깨보다 위에 있다" 는 규칙으로 손 든 것을 판정한다. 전시 환경에서는
학습 모델보다 이 방식이 덜 깨지고 조명/복장 변화에도 강하다.

먼저 로봇 없이 감지만 확인한다 (팔이 움직이지 않는다):
  .venv/bin/python detect_greet.py --list-scenarios
  .venv/bin/python detect_greet.py --scenario hello --show --robot
  .venv/bin/python detect_greet.py --list-cameras   # 어느 인덱스가 IMX477 인지 확인
  .venv/bin/python detect_greet.py --show

확인이 끝나면 로봇을 붙인다:
  .venv/bin/python detect_greet.py --list-scenarios
  .venv/bin/python detect_greet.py --scenario hello --show --robot
  .venv/bin/python detect_greet.py --list-cameras   # 어느 인덱스가 IMX477 인지 확인
  .venv/bin/python detect_greet.py --show --robot --dry-run   # 연결만, 안 움직임
  .venv/bin/python detect_greet.py --show --robot --speed 20

종료는 q 또는 Ctrl+C.
"""
import argparse, collections, math, os, sys, threading, time

COCO = {'nose': 0, 'l_eye': 1, 'r_eye': 2, 'l_shoulder': 5, 'r_shoulder': 6,
        'l_elbow': 7, 'r_elbow': 8, 'l_wrist': 9, 'r_wrist': 10}
KP_CONF = 0.5          # 키포인트를 믿을 최소 신뢰도
HEAD_TOP = 0.5         # 정수리 ≈ 코 위로 (어깨 폭 x 이 값). 손목이 이보다 위면 '머리 위'
UP_GRACE = 0.5         # 손 든 게 이 시간(초) 넘게 안 보여야 유지 시간을 초기화
WAVE_SWING = 0.15      # 흔들기로 셀 최소 되돌림 (어깨 폭 대비). 작을수록 덜 흔들어도 인식

FONT_PATH = '/System/Library/Fonts/AppleSDGothicNeo.ttc'


def _shoulders(kp, conf):
    """(어깨 중심 x, 어깨 폭 px). 양 어깨 신뢰도가 낮으면 None."""
    ls, rs = COCO['l_shoulder'], COCO['r_shoulder']
    if conf[ls] < KP_CONF or conf[rs] < KP_CONF:
        return None
    sw = math.hypot(kp[ls][0] - kp[rs][0], kp[ls][1] - kp[rs][1])
    if sw < 1e-3:
        return None
    return (kp[ls][0] + kp[rs][0]) / 2, sw


def _head_top_y(kp, conf, sw):
    """정수리 y 추정: 코(없으면 눈)에서 어깨 폭 x HEAD_TOP 만큼 위. 얼굴이 안 보이면 None."""
    for names in (('nose',), ('l_eye', 'r_eye')):
        ys = [kp[COCO[n]][1] for n in names if conf[COCO[n]] >= KP_CONF]
        if ys:
            return min(ys) - HEAD_TOP * sw
    return None


def hand_zone(kp, conf, side):
    """한쪽 손의 높이 구간: 'down' / 'up'(어깨~머리) / 'over_head' / None(판정불가).

    이미지 좌표는 y 가 아래로 커지므로 '위' 는 y 가 작은 쪽이다.
    인사(hello)는 'up', 하이파이브는 'over_head' 로 가른다. 손을 들었는데 얼굴이나
    양 어깨가 안 보이면 머리 위인지 가를 수 없으므로 None 이다.
    """
    w, sh = COCO['%s_wrist' % side], COCO['%s_shoulder' % side]
    if conf[w] < KP_CONF or conf[sh] < KP_CONF:
        return None
    if kp[w][1] >= kp[sh][1]:
        return 'down'
    s = _shoulders(kp, conf)
    head = _head_top_y(kp, conf, s[1]) if s else None
    if head is None:
        return None
    return 'over_head' if kp[w][1] < head else 'up'


def metrics(kp, conf):
    """제스처를 가르는 기하 지표. 모두 어깨 폭으로 정규화해 거리에 무관하게 만든다.

    forearm: 전완(팔꿈치-손목) 길이 / 어깨 폭.
        카메라 쪽으로 손을 내밀면 전완이 카메라 축과 나란해져 화면상 짧아진다.
        좌우로 흔드는 손은 카메라와 수직이라 제 길이로 보인다.
    wrist_up: (어깨 y - 손목 y) / 어깨 폭. 클수록 손이 머리 쪽으로 높다.
    over_nose: (코 y - 손목 y) / 어깨 폭. HEAD_TOP 보다 크면 '머리 위'.
    """
    import numpy as np
    ls, rs = COCO['l_shoulder'], COCO['r_shoulder']
    if conf[ls] < KP_CONF or conf[rs] < KP_CONF:
        return None
    sw = float(np.linalg.norm(kp[ls] - kp[rs]))
    if sw < 1e-3:
        return None
    nose = COCO['nose']
    out = {}
    for side in ('l', 'r'):
        e, w, sh = (COCO['%s_elbow' % side], COCO['%s_wrist' % side],
                    COCO['%s_shoulder' % side])
        if conf[e] < KP_CONF or conf[w] < KP_CONF:
            continue
        out[side] = {
            'forearm': float(np.linalg.norm(kp[w] - kp[e])) / sw,
            'wrist_up': float(kp[sh][1] - kp[w][1]) / sw,
            'over_nose': (float(kp[nose][1] - kp[w][1]) / sw
                          if conf[nose] >= KP_CONF else None),
        }
    return {'shoulder_px': sw, 'sides': out} if out else None


def classify(zones, waving):
    """손 높이 구간으로 포즈를 분류한다. (한글 라벨, 화면용 ASCII 라벨) 를 돌려준다."""
    vals = [z for z in zones.values() if z is not None]
    if not vals:
        return '판정불가', 'UNKNOWN'
    if 'over_head' in vals:
        return '머리 위로 들기', 'HAND OVER HEAD'
    if vals.count('up') == 2:
        return '양손 들기', 'BOTH HANDS UP'
    if 'up' in vals:
        return ('손 흔들기', 'WAVING') if waving else ('손 들기', 'HAND UP')
    return '서 있음', 'STANDING'


class WaveWatcher:
    """손목의 좌우 왕복을 보고 흔드는지 판정한다. 모델이 필요 없다.

    x 는 어깨 중심 기준, 어깨 폭 단위로 받는다. 그래서 몸이 통째로 움직이는 건
    빠지고 사람까지의 거리와도 무관하다. 키포인트 떨림을 방향 전환으로 세지 않도록
    직전 극값에서 min_swing 이상 되돌아올 때만 한 번으로 센다.
    """

    def __init__(self, window=2.0, min_swing=WAVE_SWING, min_turns=2):
        self.window = window        # 초
        self.min_swing = min_swing  # 방향 전환으로 셀 최소 되돌림 (어깨 폭 대비)
        self.min_turns = min_turns  # 방향 전환 횟수. 2 = 좌-우-좌
        self.hist = collections.defaultdict(collections.deque)

    def update(self, key, x, now):
        h = self.hist[key]
        h.append((now, x))
        while h and now - h[0][0] > self.window:
            h.popleft()
        turns, direction = 0, 0
        lo = hi = ext = h[0][1]
        for _, v in h:
            if direction == 0:
                lo, hi = min(lo, v), max(hi, v)
                if hi - lo >= self.min_swing:
                    direction = 1 if v == hi else -1
                    ext = v
            elif (v - ext) * direction > 0:
                ext = v
            elif abs(v - ext) >= self.min_swing:
                turns += 1
                direction, ext = -direction, v
        return turns >= self.min_turns

    def forget(self, key):
        self.hist.pop(key, None)

    def forget_track(self, tid):
        for k in [k for k in self.hist if k[0] == tid]:
            del self.hist[k]


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
    ap.add_argument('--conf', type=float, help='사람 검출 신뢰도 (기본 0.4)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--show', action='store_true', help='영상 창 표시')
    ap.add_argument('--robot', action='store_true',
                    help='실제로 팔을 움직인다. 없으면 감지만 하고 출력만 한다')
    ap.add_argument('--speed', type=float, help='관절 속도 deg/s (기본 20)')
    ap.add_argument('--name', help='<이름>.json 으로 인사 (생략하면 내장 인사)')
    ap.add_argument('--cycles', type=int,
                    help='--name 동작의 repeat_from 구간 반복 횟수 (기본 1)')
    ap.add_argument('--repeat', type=int, nargs=2, metavar=('시작', '끝'),
                    help='--name 동작에서 반복할 구간. 0 부터 세고 끝은 제외. '
                         '예: --repeat 4 6 이면 4,5번 자세를 반복')
    ap.add_argument('--no-highfive', action='store_true',
                    help='시나리오에 highfive 가 있어도 머리 위 손에 반응하지 않는다')
    ap.add_argument('--repeat-speed', type=float,
                    help='--repeat 구간(흔들기)만 이 속도 deg/s. 없으면 --motion-speed')
    ap.add_argument('--motion-speed', type=float,
                    help='--name 인사 동작만 이 속도 deg/s. 하이파이브는 --speed 를 쓴다. '
                         '없으면 --speed')
    ap.add_argument('--no-rest', action='store_true', default=None,
                    help='시작/인사 후 내장 대기 자세(REST)로 가지 않는다. '
                         '저장한 좌표로만 움직이게 할 때 쓴다')
    ap.add_argument('--scenario', help='scenarios/<이름>/scenario.json 을 읽어 '
                                       '트리거와 동작, 파라미터를 적용한다')
    ap.add_argument('--list-scenarios', action='store_true',
                    help='시나리오 목록을 출력하고 종료')
    ap.add_argument('--trigger', choices=['wave', 'handup'],
                    help='반응 조건. wave=손을 좌우로 흔들 때(기본), '
                         'handup=손만 들어도')
    ap.add_argument('--dry-run', action='store_true',
                    help='--robot 과 함께. 로봇에 연결은 하되 실제로 움직이지 않는다')
    ap.add_argument('--hold', type=float,
                    help='이 시간(초) 이상 조건이 유지되면 인사 (기본 0.4)')
    ap.add_argument('--cooldown', type=float,
                    help='같은 사람에게 다시 인사하기까지 최소 간격(초). 기본 8')
    ap.add_argument('--j1-span', type=float,
                    help='화면 좌우 끝에 대응하는 J1 각도. 기본 45')
    ap.add_argument('--flip-j1', action='store_true',
                    help='좌우가 반대로 돌면 이 옵션을 준다')
    ap.add_argument('--metrics', action='store_true',
                    help='제스처 구분용 기하 지표를 출력한다. 하이파이브와 손 흔들기가 '
                         '수치로 갈리는지 확인할 때 쓴다')
    ap.add_argument('--every', type=float, default=0.0,
                    help='분류값을 이 간격(초)마다 계속 출력. 0 이면 바뀔 때만 출력')
    args = ap.parse_args()

    import json
    root = os.path.dirname(os.path.abspath(__file__))
    sdir = os.path.join(root, 'scenarios')

    if args.list_scenarios:
        if not os.path.isdir(sdir):
            print('scenarios/ 가 없음')
            return
        for n in sorted(os.listdir(sdir)):
            f = os.path.join(sdir, n, 'scenario.json')
            if not os.path.exists(f):
                continue
            c = json.load(open(f))
            print('%-12s %-22s 트리거=%-7s 동작=%s'
                  % (n, c.get('title', ''), c.get('trigger', ''),
                     c.get('motion', {}).get('name') or
                     c.get('motion', {}).get('kind', '내장')))
        return

    # 시나리오 -> 내장 기본값 순으로 빈 값을 채운다 (CLI 로 준 값이 항상 이긴다)
    cfg, params = {}, {}
    if args.scenario:
        f = os.path.join(sdir, args.scenario, 'scenario.json')
        if not os.path.exists(f):
            sys.exit('시나리오 %s 없음. --list-scenarios 로 확인해줘.' % args.scenario)
        cfg = json.load(open(f))
        params = cfg.get('params', {})
        print('시나리오: %s — %s' % (cfg.get('name'), cfg.get('title', '')))
        motion = cfg.get('motion', {})
        if args.name is None and motion.get('type') == 'file':
            args.name = motion.get('name')
        if args.cycles is None:
            args.cycles = motion.get('cycles')
        if args.repeat is None:
            args.repeat = motion.get('repeat')
        if args.repeat_speed is None:
            args.repeat_speed = motion.get('repeat_speed')
        if args.motion_speed is None:
            args.motion_speed = motion.get('speed')
        if args.no_rest is None and motion.get('rest') is False:
            args.no_rest = True
        if args.trigger is None:
            args.trigger = cfg.get('trigger')
    # 머리 위로 손을 들면 하는 하이파이브. 시나리오에 highfive 가 있을 때만 켠다
    highfive = None if args.no_highfive else cfg.get('highfive')

    for key, default in (('conf', 0.4), ('speed', 20.0), ('hold', 0.4),
                         ('cooldown', 8.0), ('j1_span', 45.0)):
        if getattr(args, key) is None:
            setattr(args, key, params.get(key, default))
    if args.trigger is None:
        args.trigger = 'wave'
    if args.cycles is None:
        args.cycles = 1
    args.no_rest = bool(args.no_rest)

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
        from greet import Greeter, resolve_pose, HF_TIMEOUT, HF_CURRENT, HF_DISTAL
        if highfive:
            # 자세 파일을 로봇이 움직이기 전에 읽어 둔다. 없으면 여기서 멈춘다
            hf_start = resolve_pose(highfive['start']) if highfive.get('start') else None
            resolve_pose(highfive['name'] + ':0')
        greeter = Greeter(speed=args.speed, dry_run=args.dry_run,
                          go_rest=not args.no_rest)
        print('로봇 연결됨%s' % (' (dry-run: 움직이지 않음)' if args.dry_run else
                               '. 현재 자세 유지.' if args.no_rest else '. 대기 자세.'))
    else:
        print('감지 전용 모드. 팔은 움직이지 않는다. 실제로 움직이려면 --robot 을 붙여줘.')

    need_wave = (args.trigger == 'wave')
    print('트리거: %s' % ('손을 좌우로 흔들 때' if need_wave else '손을 들 때'))
    print('하이파이브: %s' % ('머리 위로 손을 들 때 (%s, 시작 %s)'
                             % (highfive['name'], highfive.get('start'))
                             if highfive else '꺼짐'))

    # 인사는 몇 초가 걸린다. 별도 스레드로 돌려 카메라 루프가 멈추지 않게 한다.
    busy = threading.Event()
    worker = [None]

    def do_greet(kind, j1):
        try:
            if kind == 'highfive':
                greeter.highfive(highfive['name'], start=hf_start,
                                 timeout=highfive.get('timeout', HF_TIMEOUT),
                                 threshold=highfive.get('current', HF_CURRENT),
                                 distal=highfive.get('distal', HF_DISTAL))
            else:
                greeter.greet(j1_deg=j1, name=args.name, cycles=args.cycles,
                              repeat=args.repeat, repeat_speed=args.repeat_speed,
                              speed=args.motion_speed,
                              back_to_rest=not args.no_rest)
        except Exception as e:
            print('%s 중 오류: %s' % ('하이파이브' if kind == 'highfive' else '인사', e))
        finally:
            busy.clear()

    watcher = WaveWatcher()
    last_label = {}     # track id -> 마지막으로 출력한 분류값
    last_print = 0.0
    since = {}          # track id -> 조건이 만족되기 시작한 시각
    last_up = {}        # track id -> 마지막으로 손 든 게 보인 시각
    hf_since = {}       # track id -> 머리 위로 손 든 게 시작된 시각
    last_over = {}      # track id -> 마지막으로 머리 위 손이 보인 시각
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

            target = None       # (면적, track id, j1 각도, 어느 손, 'hello'|'highfive')
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
                    zones = {s: hand_zone(kps[i], conf, s) for s in ('l', 'r')}
                    sh = _shoulders(kps[i], conf)

                    # 어깨~머리 높이로 든 손만 흔들림을 본다 (어깨 기준 손목 x 왕복).
                    # 머리 위로 든 손은 하이파이브용이라 인사하지 않는다.
                    # 판정불가(None) 프레임은 건너뛰기만 한다. 키포인트 신뢰도가 프레임마다
                    # 깜빡이므로 여기서 기록을 지우면 흔들기가 쌓이지 않는다.
                    wave_side = None
                    for s in ('l', 'r'):
                        if zones[s] in ('down', 'over_head'):
                            watcher.forget((tid, s))
                        if zones[s] != 'up':
                            continue
                        x = (kps[i][COCO['%s_wrist' % s]][0] - sh[0]) / sh[1]
                        if watcher.update((tid, s), x, now) and wave_side is None:
                            wave_side = s
                    waving = wave_side is not None
                    up_side = next((s for s in ('l', 'r') if zones[s] == 'up'), None)
                    up = up_side is not None
                    side = wave_side or up_side
                    label_ko, label_en = classify(zones, waving)
                    labels[tid] = (label_ko, label_en, boxes[i])

                    # 터미널 출력: 바뀔 때만, 또는 --every 간격마다
                    if args.every > 0:
                        if now - last_print >= args.every:
                            show_line = True
                        else:
                            show_line = False
                    else:
                        show_line = last_label.get(tid) != label_ko
                    if show_line and args.metrics:
                        m = metrics(kps[i], conf)
                        if m is None:
                            print('[%s] id=%-2d %-10s  어깨/팔꿈치 키포인트 신뢰도 낮음'
                                  % (time.strftime('%H:%M:%S'), tid, label_ko))
                        else:
                            parts = ['%s: 전완%.2f 손높이%+.2f 코위%s' %
                                     ('왼' if k == 'l' else '오른',
                                      v['forearm'], v['wrist_up'],
                                      '%+.2f' % v['over_nose']
                                      if v['over_nose'] is not None else '?')
                                     for k, v in m['sides'].items()]
                            print('[%s] id=%-2d %-10s  어깨폭%4.0fpx  %s  흔듦=%s'
                                  % (time.strftime('%H:%M:%S'), tid, label_ko,
                                     m['shoulder_px'], '  '.join(parts), waving))
                    elif show_line:
                        lw, ls = COCO['l_wrist'], COCO['l_shoulder']
                        rw, rs = COCO['r_wrist'], COCO['r_shoulder']
                        print('[%s] id=%-2d %-10s  왼손목y=%4.0f(어깨%4.0f) '
                              '오른손목y=%4.0f(어깨%4.0f) 흔듦=%s'
                              % (time.strftime('%H:%M:%S'), tid, label_ko,
                                 kps[i][lw][1], kps[i][ls][1],
                                 kps[i][rw][1], kps[i][rs][1], waving))
                    last_label[tid] = label_ko

                    over_side = next((s for s in ('l', 'r')
                                      if zones[s] == 'over_head'), None)

                    # 판정불가가 한두 프레임 끼는 건 봐준다. 손 든 상태가 UP_GRACE 초 넘게
                    # 끊겼을 때만 유지 시간을 초기화한다.
                    if up:
                        last_up[tid] = now
                    elif now - last_up.get(tid, 0) > UP_GRACE:
                        since.pop(tid, None)
                    if over_side:
                        last_over[tid] = now
                    elif now - last_over.get(tid, 0) > UP_GRACE:
                        hf_since.pop(tid, None)

                    # 한 손이라도 머리 위면 하이파이브 자세다. 다른 손이 흔들어도 인사하지 않는다.
                    if over_side:
                        since.pop(tid, None)
                        if not highfive:
                            continue
                        kind, side = 'highfive', over_side
                        started = hf_since.setdefault(tid, now)
                    else:
                        if not up or (need_wave and not waving):
                            continue
                        kind = 'hello'
                        started = since.setdefault(tid, now)
                    if now - started < args.hold:
                        continue
                    if now - last_greet.get(tid, 0) < args.cooldown:
                        continue
                    x1, y1, x2, y2 = boxes[i]
                    area = (x2 - x1) * (y2 - y1)
                    cx = (x1 + x2) / 2 / w - 0.5
                    j1 = (cx if args.flip_j1 else -cx) * 2 * args.j1_span
                    if target is None or area > target[0]:
                        target = (area, tid, j1, side, kind)

            if args.every > 0 and now - last_print >= args.every:
                last_print = now
                if not labels:
                    print('[%s] 사람 없음' % time.strftime('%H:%M:%S'))

            for tid in set(list(since) + list(last_label)):
                if tid not in seen:
                    since.pop(tid, None)
                    last_label.pop(tid, None)
                    last_up.pop(tid, None)
                    hf_since.pop(tid, None)
                    last_over.pop(tid, None)
                    watcher.forget_track(tid)

            if target and not busy.is_set():
                _, tid, j1, side, kind = target
                last_greet[tid] = now
                since.pop(tid, None)
                hf_since.pop(tid, None)
                if kind == 'highfive':
                    print('[%s] id=%d %s손 머리 위 -> 하이파이브'
                          % (time.strftime('%H:%M:%S'), tid,
                             '왼' if side == 'l' else '오른'))
                else:
                    print('[%s] id=%d %s손 %s -> J1=%+.0f 도로 인사'
                          % (time.strftime('%H:%M:%S'), tid,
                             '왼' if side == 'l' else '오른',
                             '흔듦' if need_wave else '들었음', j1))
                if greeter:
                    busy.set()
                    worker[0] = threading.Thread(target=do_greet, args=(kind, j1),
                                                 daemon=True)
                    worker[0].start()

            fps_n += 1
            if now - fps_t >= 1.0:
                fps, fps_n, fps_t = fps_n / (now - fps_t), 0, now

            if args.show:
                vis = res.plot()
                for tid, (ko, en, box) in labels.items():
                    x1, y1 = int(box[0]), int(box[1])
                    color = (0, 255, 255) if ko in ('손 흔들기', '손 들기', '양손 들기',
                                                    '머리 위로 들기') else (0, 255, 0)
                    vis = draw_label(vis, 'id%d %s' % (tid, ko),
                                     (x1, max(0, y1 - 28)), color)
                state = '감지 전용'
                if greeter:
                    state = '동작 중' if busy.is_set() else '대기 중'
                vis = draw_label(vis, '%.1f fps   %s   트리거=%s' %
                                 (fps, state, '흔들기' if need_wave else '손들기'),
                                 (10, 8),
                                 (0, 128, 255) if busy.is_set() else (0, 255, 0))
                cv2.imshow('detect_greet  (q=종료)', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print('\n중단 요청')
        if greeter:
            greeter.arm.emergency_stop()
            print('비상 정지. 다시 움직이려면 greet.py rest 로 대기 자세부터.')
    finally:
        if worker[0] is not None and worker[0].is_alive():
            print('인사 동작이 끝나기를 기다리는 중...')
            worker[0].join(timeout=15)
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        if greeter:
            greeter.arm.disconnect()
    print('종료')


if __name__ == '__main__':
    main()
