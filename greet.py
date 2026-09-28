#!/usr/bin/env python3
"""xArm6 인사 동작 모듈.

처음 실행할 때는 반드시 이 순서로:
  .venv/bin/python greet.py wave --dry-run   # 안 움직임. 지나갈 자세만 검증
  .venv/bin/python greet.py wave --step      # 이동마다 Enter 확인
  .venv/bin/python greet.py wave             # 확인 끝나면 연속 실행

단독 실행 (동작 확인용):
  .venv/bin/python greet.py wave            # 손 흔들기
  .venv/bin/python greet.py wave --j1 25    # 왼쪽 사람을 보고 인사
  .venv/bin/python greet.py wave --smooth   # 사인파 스트리밍(부드러움)
  .venv/bin/python greet.py bow
  .venv/bin/python greet.py random
  .venv/bin/python greet.py rest            # 대기 자세로

저장해 둔 동작을 재생 (jog.py 의 `w <이름>` 으로 만든 파일):
  .venv/bin/python greet.py --list                        # 저장된 동작 목록
  .venv/bin/python greet.py play --name hello --dry-run
  .venv/bin/python greet.py play --name hello --cycles 3

감지 코드에서 쓸 때:
  from greet import Greeter
  g = Greeter()                 # 연결 + 대기 자세
  g.greet(j1_deg=15)            # 코드에 내장된 인사
  g.greet(name='hello')         # 내가 저장해 둔 동작으로 인사
  g.rest()
  g.close()
"""
import argparse, json, math, os, random, time
from xarm.wrapper import XArmAPI

IP = os.environ.get('XARM_IP', '192.168.1.221')

# 기준 자세 (deg). jog.py 로 조정한 값을 여기에 반영하면 됨.
REST  = [0, -10, -35, 0, 45, 0]   # 대기: 팔을 내린 자세 (xyz 375,0,273)
RAISE = [0, -60, -30, 0,  0, 0]   # 인사: 툴 Z축이 정면 수평, 손바닥이 관객을 봄

J1_LIMIT = 60.0                   # 사람 쪽으로 돌릴 수 있는 최대 각도
BLEND = 20.0                      # 흔들기 코너 블렌딩 반경(mm). 클수록 부드럽고 진폭이 줄어든다
HF_CURRENT = 0.5                  # 하이파이브 충격으로 볼 관절 전류 변화(A). 정지 중 떨림은 0.02A 안팎
HF_TIMEOUT = 5.0                  # 손을 내민 채 하이파이브를 기다리는 최대 시간(초)
HF_SETTLE = 1.0                   # 손 내민 뒤 기준 전류를 재기 전 안정화 대기(초). 멈춘 직후엔 J2 전류가 흔들린다
HF_DISTAL = 0.15                  # 손목 관절 J5 전류 변화가 이 이상이어야 손을 친 것으로 본다(A).
                                  # 실측: 테이블 흔들림 J5 0.05~0.09 (J4 는 0.15 까지 올라 못 쓴다),
                                  # 손 치기 J5 0.19~0.37
HF_WINDOW = 0.4                   # 관절별 변화를 묶어 보는 시간(초). 전류 보고가 초당 5회라 관절마다
                                  # 튀는 샘플이 다를 수 있다


ROOT = os.path.dirname(os.path.abspath(__file__))
POSE_DIR = os.path.join(ROOT, 'poses')
SCENARIO_DIR = os.path.join(ROOT, 'scenarios')


def pose_dirs():
    """자세 파일을 찾을 디렉터리. 시나리오 폴더가 우선이다."""
    dirs = []
    if os.path.isdir(SCENARIO_DIR):
        for n in sorted(os.listdir(SCENARIO_DIR)):
            d = os.path.join(SCENARIO_DIR, n, 'poses')
            if os.path.isdir(d):
                dirs.append(d)
    if os.path.isdir(POSE_DIR):
        dirs.append(POSE_DIR)
    return dirs


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def load_waypoints(name):
    """poses/<name>.json 에서 관절각 목록과 반복 시작점을 읽는다.

    jog.py 의 `w <이름>` 과 xarm_pose.py 가 저장하는 형식을 모두 읽는다.
    """
    if os.path.isabs(name) or name.endswith('.json'):
        path = name
    else:
        path = None
        for d in pose_dirs():
            cand = os.path.join(d, name + '.json')
            if os.path.exists(cand):
                path = cand
                break
    if path is None or not os.path.exists(path):
        have = []
        for d in pose_dirs():
            have += [f[:-5] for f in os.listdir(d) if f.endswith('.json')]
        raise FileNotFoundError('%s 없음. 저장된 동작: %s'
                                % (name, ', '.join(sorted(set(have))) or '(없음)'))
    data = json.load(open(path))
    angles = [wp['angles'] for wp in data['waypoints']]
    if not angles:
        raise ValueError('%s 에 웨이포인트가 없음' % path)
    return angles, data.get('repeat_from')


def resolve_pose(ref):
    """'<이름>:<인덱스>' 를 관절각으로 바꾼다. 예: 'gh_hello2:0' (인덱스는 0 부터)."""
    name, idx = ref.rsplit(':', 1)
    angles, _ = load_waypoints(name)
    return angles[int(idx)]


class Greeter:
    def __init__(self, ip=IP, speed=60.0, go_rest=True, dry_run=False, step=False):
        self.speed = speed
        self.dry_run = dry_run      # True 면 계산만 하고 로봇을 움직이지 않는다
        self.step = step            # True 면 이동마다 Enter 확인을 받는다
        arm = XArmAPI(ip, is_radian=False)
        arm.clean_warn()
        arm.clean_error()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.3)
        self.arm = arm
        if go_rest:
            self.rest()

    # ---------- 기본 이동 ----------
    def _move(self, angles, speed=None, wait=True, radius=None):
        angles = list(angles)
        sp = speed or self.speed
        code, pose = self.arm.get_forward_kinematics(angles)
        if code != 0:
            raise RuntimeError('도달 불가한 자세라 중단: %s (code=%s)' % (angles, code))
        desc = ('J = ' + ' '.join('%6.1f' % a for a in angles)
                + '   xyz = %5.0f %5.0f %5.0f' % tuple(pose[:3])
                + '   %3.0f deg/s' % sp)
        if self.dry_run:
            print('  [계산만] ' + desc)
            return 0
        if self.step:
            ans = input('  이동? [Enter=실행, s=건너뜀, q=중단]  ' + desc + '\n  > ').strip().lower()
            if ans == 'q':
                raise KeyboardInterrupt('사용자 중단')
            if ans == 's':
                print('  건너뜀')
                return 0
        else:
            print('  ' + desc)
        return self.arm.set_servo_angle(angle=angles, speed=sp, wait=wait,
                                        radius=radius)

    def _wait(self, timeout=30.0):
        """큐에 쌓인 동작이 끝날 때까지 기다린다."""
        if self.dry_run:
            return
        time.sleep(0.2)
        t0 = time.time()
        while time.time() - t0 < timeout:
            moving = self.arm.get_is_moving
            if callable(moving):
                moving = moving()
            if not moving:
                return
            time.sleep(0.05)
        print('  경고: %.0f초 안에 동작이 끝나지 않음' % timeout)

    def rest(self):
        """대기 자세로 복귀."""
        return self._move(REST)

    def raise_hand(self, j1_deg=0.0):
        """사람 방향(j1_deg)으로 돌려서 손을 든다."""
        base = list(RAISE)
        base[0] = clamp(j1_deg, -J1_LIMIT, J1_LIMIT)
        self._move(base)
        return base

    # ---------- 인사 동작 ----------
    def wave(self, j1_deg=0.0, cycles=3, amp=30.0, speed=None, smooth=False):
        """손을 들고 손목(J6)을 좌우로 흔든다."""
        base = self.raise_hand(j1_deg)
        if smooth:
            self._wave_smooth(base, cycles, amp)
        else:
            sp = speed or self.speed
            for _ in range(cycles):
                for sign in (+1, -1):
                    a = list(base)
                    a[5] = base[5] + sign * amp
                    self._move(a, speed=sp, wait=False, radius=BLEND)
            self._move(base, speed=sp, wait=False, radius=BLEND)
            self._wait()
        return 0

    def _wave_smooth(self, base, cycles=3, amp=30.0, hz=1.2, dt=0.01):
        """mode 1 스트리밍으로 사인파 흔들기. 끊김 없이 부드럽다."""
        if self.dry_run:
            print('  [계산만] 사인파 흔들기 %d회, 진폭 %.0f도' % (cycles, amp))
            return
        arm = self.arm
        arm.set_mode(1)
        arm.set_state(0)
        time.sleep(0.2)
        try:
            duration = cycles / hz
            t0 = time.time()
            while True:
                t = time.time() - t0
                if t >= duration:
                    break
                a = list(base)
                a[5] = base[5] + amp * math.sin(2 * math.pi * hz * t)
                arm.set_servo_angle_j(angles=a)
                time.sleep(dt)
        finally:
            arm.set_mode(0)
            arm.set_state(0)
            time.sleep(0.2)
            self._move(base, wait=True)

    def play(self, name, cycles=1, speed=None, back_to_rest=False, repeat=None,
             repeat_speed=None):
        """poses/<name>.json 에 저장해 둔 자세를 순서대로 재생한다.

        파일에 repeat_from 이 있으면 그 지점부터 끝까지만 cycles 번 반복한다.
        (jog.py 로 저장한 파일에는 repeat_from 이 없어 전체를 cycles 번 반복한다.)
        repeat=(시작, 끝) 을 주면 파일 값 대신 angles[시작:끝] 만 cycles 번 반복하고
        나머지 자세로 이어간다. 번호는 0 부터, 끝은 포함하지 않는다.
        repeat_speed 를 주면 반복 구간 자세로 가는 이동만 그 속도로 한다 (흔들기).
        """
        angles, repeat_from = load_waypoints(name)
        repeat_to = len(angles)
        if repeat is not None:
            repeat_from, repeat_to = repeat
        if repeat_from is not None and not 0 <= repeat_from < repeat_to <= len(angles):
            raise ValueError('반복 구간 %s~%s 가 웨이포인트 %d개 범위를 벗어남'
                             % (repeat_from, repeat_to, len(angles)))
        if repeat_from is None:
            head, loop, after = angles, [], []
        else:
            head = angles[:repeat_from]
            loop = angles[repeat_from:repeat_to]
            after = angles[repeat_to:]
        print('%s: %d개 웨이포인트%s' %
              (name, len(angles),
               ', %d~%d번 %d회 반복' % (repeat_from, repeat_to - 1, cycles) if loop else
               (', 전체 %d회 반복' % cycles if cycles > 1 else '')))
        for a in head:
            self._move(a, speed=speed)
        if loop:
            for _ in range(cycles):
                for a in loop:
                    self._move(a, speed=repeat_speed or speed)
            for a in after:
                self._move(a, speed=speed)
        else:
            for _ in range(cycles - 1):
                for a in angles:
                    self._move(a, speed=speed)
        if back_to_rest:
            self.rest()
        return 0

    def _recover(self):
        """충돌 감지 등으로 멈춰 있으면 에러를 지우고 다시 움직일 수 있게 한다."""
        arm = self.arm
        if self.dry_run or (not arm.error_code and arm.state != 4):
            return False
        print('  로봇 정지 상태 (error=%s, state=%s) -> 복구' % (arm.error_code, arm.state))
        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.5)
        return True

    def _currents(self, duration=0.6):
        """duration 초 동안 관절 전류(A) 평균. 컨트롤러 보고가 초당 5회라 짧게 잡지 않는다."""
        samples, t0 = [], time.time()
        while time.time() - t0 < duration:
            samples.append(list(self.arm.currents[:6]))
            time.sleep(0.05)
        return [sum(c) / len(c) for c in zip(*samples)]

    def _wait_impact(self, timeout, threshold, distal=HF_DISTAL):
        """지금 자세에서 손을 쳐 주기를 기다린다.

        충격 = 컨트롤러 충돌 감지로 멈춤, 또는 최근 HF_WINDOW 초 안에 관절 전류가 대기
        기준보다 threshold(A) 넘게 변하고 손목 관절 J5 도 distal(A) 넘게 변함.
        J5 가 거의 안 변하면 테이블 흔들림으로 보고 무시한다.
        반환: 'collision' / 'current' / None(시간 초과).
        """
        if self.dry_run:
            print('  [계산만] 충격 대기 최대 %.0f초' % timeout)
            return None
        time.sleep(HF_SETTLE)                 # 멈춘 직후 전류가 가라앉기를 기다린다
        base = self._currents()
        peak = [0.0] * 6
        recent = []                           # (시각, 관절별 변화)
        ignoring = False                      # 무시한 흔들림이 아직 이어지는 중
        fmt = lambda d: ' '.join('J%d %.2f' % (i + 1, v) for i, v in enumerate(d))
        print('  하이파이브 대기 (최대 %.0f초, 전류 변화 %.2fA 이상 + J5 %.2fA 이상이면 충격)'
              % (timeout, threshold, distal))
        t0 = time.time()
        while time.time() - t0 < timeout:
            now = time.time()
            if self.arm.error_code or self.arm.state == 4:
                print('  충격: 컨트롤러 충돌 감지 (error=%s), 대기 %.1f초째'
                      % (self.arm.error_code, now - t0))
                return 'collision'
            dev = [abs(c - b) for c, b in zip(self.arm.currents[:6], base)]
            peak = [max(p, d) for p, d in zip(peak, dev)]
            recent = [(t, d) for t, d in recent if now - t <= HF_WINDOW] + [(now, dev)]
            win = [max(d[i] for _, d in recent) for i in range(6)]
            if max(win) >= threshold:
                if win[4] >= distal:
                    print('  충격: 대기 %.1f초째, 전류 변화 %s' % (now - t0, fmt(win)))
                    return 'current'
                if not ignoring:
                    print('  무시: 대기 %.1f초째, J5 변화가 %.2fA 미만 '
                          '(테이블 흔들림으로 봄) %s' % (now - t0, distal, fmt(win)))
                    ignoring = True
            else:
                ignoring = False
            time.sleep(0.02)
        print('  시간 초과. 최대 전류 변화 ' + fmt(peak))
        return None

    def highfive(self, name, start=None, speed=None, timeout=HF_TIMEOUT,
                 threshold=HF_CURRENT, distal=HF_DISTAL):
        """start -> <name> 자세들로 손을 내밀고, 하이파이브(충격)가 오면 왔던 길을
        거꾸로 되짚어 start 로 돌아간다. timeout 초 안에 충격이 없어도 돌아간다.

        저장한 좌표로만 움직인다. 반환: 'collision' / 'current' / None(시간 초과).
        """
        angles, _ = load_waypoints(name)
        path = ([list(start)] if start is not None else []) + angles
        print('하이파이브 %s: %d개 자세로 손 내밀기' % (name, len(path)))
        reached = 0
        for a in path:
            if self._move(a, speed=speed) != 0:
                break
            reached += 1
        if reached == len(path):
            hit = self._wait_impact(timeout, threshold, distal)
            back = path[:-1]                  # 지금 path[-1] 에 있다
        else:
            print('  손 내미는 중 멈춤 (충돌 감지 등)')
            hit = 'collision'
            back = path[:reached]             # path[reached-1] 과 path[reached] 사이에 있다
        self._recover()
        print('  원래 자리로 복귀')
        for a in reversed(back):
            if self._move(a, speed=speed) != 0:
                print('  복귀 중 멈춤. greet.py rest 대신 jog.py 로 자세를 확인해줘')
                break
        return hit

    def bow(self, j1_deg=0.0, depth=20.0):
        """고개 숙이는 느낌의 인사. 손을 든 뒤 팔 전체를 앞으로 굽힌다."""
        base = self.raise_hand(j1_deg)
        down = list(base)
        down[1] = base[1] + depth        # J2 를 앞으로
        down[4] = base[4] + depth        # 손목도 같이 숙임
        self._move(down, wait=False)
        self._move(base, wait=False)
        self._wait()
        return 0

    def big_wave(self, j1_deg=0.0, cycles=2):
        """팔을 더 높이 들고 크게 흔든다."""
        return self.wave(j1_deg, cycles=cycles, amp=45.0)

    def small_wave(self, j1_deg=0.0, cycles=4):
        """작게 빠르게 흔든다."""
        return self.wave(j1_deg, cycles=cycles, amp=15.0)

    def greet(self, j1_deg=0.0, kind=None, back_to_rest=True, name=None, cycles=1,
              repeat=None, repeat_speed=None):
        """감지 코드에서 호출하는 진입점.

        name 을 주면 poses/<name>.json 에 저장해 둔 동작을 재생한다 (cycles/repeat 는
        play() 와 같고, j1_deg 는 쓰지 않는다).
        생략하면 코드에 내장된 인사를 쓰고, kind 도 생략하면 그중 랜덤.
        """
        if name:
            code = self.play(name, cycles=cycles, repeat=repeat,
                             repeat_speed=repeat_speed)
            if back_to_rest:
                self.rest()
            return code, name
        kinds = {'wave': self.wave, 'big': self.big_wave,
                 'small': self.small_wave, 'bow': self.bow}
        if kind is None:
            kind = random.choice(list(kinds))
        fn = kinds.get(kind)
        if fn is None:
            raise ValueError('알 수 없는 인사 종류: %s' % kind)
        code = fn(j1_deg)
        if back_to_rest:
            self.rest()
        return code, kind

    def close(self):
        try:
            self.rest()
        finally:
            self.arm.disconnect()


def main():
    ap = argparse.ArgumentParser(description='xArm6 인사 동작')
    ap.add_argument('kind', nargs='?', default='wave',
                    choices=['wave', 'big', 'small', 'bow', 'random', 'rest', 'raise', 'play',
                             'highfive'])
    ap.add_argument('--name', help="play 할 때 poses/<이름>.json 지정")
    ap.add_argument('--list', action='store_true', help='저장된 동작 목록만 보고 종료')
    ap.add_argument('--j1', type=float, default=0.0, help='사람 방향 각도 (deg)')
    ap.add_argument('--cycles', type=int, default=3)
    ap.add_argument('--repeat', type=int, nargs=2, metavar=('시작', '끝'),
                    help='play 할 때 반복 구간. 0 부터 세고 끝은 제외. 예: --repeat 4 6')
    ap.add_argument('--start', help="highfive 시작 자세 '<이름>:<인덱스>'. 예: gh_hello2:0")
    ap.add_argument('--timeout', type=float, default=HF_TIMEOUT,
                    help='highfive 에서 충격을 기다리는 최대 시간(초)')
    ap.add_argument('--current', type=float, default=HF_CURRENT,
                    help='highfive 충격으로 볼 관절 전류 변화(A)')
    ap.add_argument('--distal', type=float, default=HF_DISTAL,
                    help='highfive 충격으로 볼 손목 관절 J5 최소 전류 변화(A). '
                         '이보다 작으면 테이블 흔들림으로 보고 무시')
    ap.add_argument('--repeat-speed', type=float,
                    help='play 할 때 반복 구간(흔들기)만 이 속도 deg/s. 없으면 --speed')
    ap.add_argument('--amp', type=float, default=30.0)
    ap.add_argument('--speed', type=float, default=20.0,
                    help='관절 속도 deg/s (기본 20, 안전하게 낮춰둠)')
    ap.add_argument('--smooth', action='store_true', help='사인파 스트리밍으로 흔들기')
    ap.add_argument('--stay', action='store_true', help='끝나고 대기 자세로 안 돌아감')
    ap.add_argument('--no-start-rest', action='store_true',
                    help='시작할 때 대기 자세로 모으지 않고 현재 자세에서 바로 시작')
    ap.add_argument('--dry-run', action='store_true',
                    help='로봇을 움직이지 않고 지나갈 자세만 검증해서 출력')
    ap.add_argument('--step', action='store_true',
                    help='이동마다 Enter 확인을 받는다. 처음 실행할 때 권장')
    args = ap.parse_args()

    if args.list:
        found = False
        for d in pose_dirs():
            names = sorted(f[:-5] for f in os.listdir(d) if f.endswith('.json'))
            if not names:
                continue
            found = True
            print('[%s]' % os.path.relpath(d, ROOT))
            for n in names:
                a, rf = load_waypoints(os.path.join(d, n + '.json'))
                print('  %-16s %2d개  repeat_from=%s' % (n, len(a), rf))
        if not found:
            print('저장된 동작이 없음. jog.py 에서 s 로 저장하고 w <이름> 으로 파일로 만들어줘.')
        return

    g = Greeter(speed=args.speed,
                go_rest=(args.kind not in ('rest', 'highfive') and not args.no_start_rest),
                dry_run=args.dry_run, step=args.step)
    try:
      try:
        if args.kind == 'play':
            if not args.name:
                ap.error('play 는 --name 이 필요해. 예: greet.py play --name hello')
            print('재생 code=%s' % g.play(args.name, cycles=args.cycles,
                                         repeat=args.repeat,
                                         repeat_speed=args.repeat_speed))
        elif args.kind == 'highfive':
            if not args.name:
                ap.error('highfive 는 --name 이 필요해. 예: greet.py highfive --name ighfive '
                         '--start gh_hello2:0')
            start = resolve_pose(args.start) if args.start else None
            print('하이파이브 결과=%s' % g.highfive(args.name, start=start,
                                               timeout=args.timeout,
                                               threshold=args.current,
                                               distal=args.distal))
        elif args.kind == 'rest':
            print('대기 자세 code=%s' % g.rest())
        elif args.kind == 'raise':
            print('손 들기 -> %s' % g.raise_hand(args.j1))
        elif args.kind == 'wave':
            print('흔들기 code=%s' % g.wave(args.j1, args.cycles, args.amp,
                                           smooth=args.smooth))
        elif args.kind == 'random':
            print('인사 code=%s kind=%s' % g.greet(args.j1))
        else:
            print('인사 code=%s kind=%s' % g.greet(args.j1, kind=args.kind))
        if not args.stay and args.kind not in ('rest', 'raise', 'highfive'):
            g.rest()
      except KeyboardInterrupt:
        # Ctrl+C 만으로는 컨트롤러 큐에 쌓인 동작이 계속 실행된다.
        # 반드시 emergency_stop 을 보내야 실제로 멈춘다.
        print('\n중단 요청 -> 비상 정지')
        if not args.dry_run:
            g.arm.emergency_stop()
        print('정지. 다시 움직이려면 greet.py rest 로 대기 자세부터 잡아줘.')
    finally:
        g.arm.disconnect()


if __name__ == '__main__':
    main()
