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

    def play(self, name, cycles=1, speed=None, back_to_rest=False):
        """poses/<name>.json 에 저장해 둔 자세를 순서대로 재생한다.

        파일에 repeat_from 이 있으면 그 지점부터 끝까지만 cycles 번 반복한다.
        (jog.py 로 저장한 파일에는 repeat_from 이 없어 전체를 cycles 번 반복한다.)
        """
        angles, repeat_from = load_waypoints(name)
        head = angles if repeat_from is None else angles[:repeat_from]
        tail = [] if repeat_from is None else angles[repeat_from:]
        print('%s: %d개 웨이포인트%s' %
              (name, len(angles),
               ', %d번째부터 %d회 반복' % (repeat_from + 1, cycles) if tail else
               (', 전체 %d회 반복' % cycles if cycles > 1 else '')))
        for a in head:
            self._move(a, speed=speed)
        loops = tail if tail else (angles if cycles > 1 else [])
        if tail:
            for _ in range(cycles):
                for a in tail:
                    self._move(a, speed=speed)
        else:
            for _ in range(cycles - 1):
                for a in angles:
                    self._move(a, speed=speed)
        if back_to_rest:
            self.rest()
        return 0

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

    def greet(self, j1_deg=0.0, kind=None, back_to_rest=True, name=None):
        """감지 코드에서 호출하는 진입점.

        name 을 주면 poses/<name>.json 에 저장해 둔 동작을 재생한다.
        생략하면 코드에 내장된 인사를 쓰고, kind 도 생략하면 그중 랜덤.
        """
        if name:
            code = self.play(name)
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
                    choices=['wave', 'big', 'small', 'bow', 'random', 'rest', 'raise', 'play'])
    ap.add_argument('--name', help="play 할 때 poses/<이름>.json 지정")
    ap.add_argument('--list', action='store_true', help='저장된 동작 목록만 보고 종료')
    ap.add_argument('--j1', type=float, default=0.0, help='사람 방향 각도 (deg)')
    ap.add_argument('--cycles', type=int, default=3)
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
                go_rest=(args.kind != 'rest' and not args.no_start_rest),
                dry_run=args.dry_run, step=args.step)
    try:
      try:
        if args.kind == 'play':
            if not args.name:
                ap.error('play 는 --name 이 필요해. 예: greet.py play --name hello')
            print('재생 code=%s' % g.play(args.name, cycles=args.cycles))
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
        if not args.stay and args.kind not in ('rest', 'raise'):
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
