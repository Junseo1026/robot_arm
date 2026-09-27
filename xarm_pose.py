#!/usr/bin/env python3
"""xArm6 자세 티칭 / 재생 도구.

사용 예:
  .venv/bin/python xarm_pose.py here                 # 현재 자세 출력
  .venv/bin/python xarm_pose.py record wave          # 손으로 움직이며 자세 저장
  .venv/bin/python xarm_pose.py list
  .venv/bin/python xarm_pose.py play wave --loop 3
"""
import argparse, json, os, time
from xarm.wrapper import XArmAPI

IP = os.environ.get('XARM_IP', '192.168.1.221')
DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'poses')


def jpath(name):
    return os.path.join(DIR, name + '.json')


def connect():
    arm = XArmAPI(IP, is_radian=False)
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)
    return arm


def snapshot(arm):
    c1, pose = arm.get_position()
    c2, ang = arm.get_servo_angle()
    if c1 != 0 or c2 != 0:
        raise RuntimeError('자세 읽기 실패 code=%s/%s' % (c1, c2))
    return {'pose': [round(v, 2) for v in pose],
            'angles': [round(v, 2) for v in ang]}


def fmt(wp):
    x, y, z, r, p, yw = wp['pose']
    return ('xyz=(%7.1f %7.1f %7.1f) rpy=(%6.1f %6.1f %6.1f)' % (x, y, z, r, p, yw)
            + '  J=' + ' '.join('%6.1f' % a for a in wp['angles']))


def cmd_here(args):
    arm = connect()
    print(fmt(snapshot(arm)))
    arm.disconnect()


def cmd_list(args):
    names = sorted(f[:-5] for f in os.listdir(DIR) if f.endswith('.json'))
    if not names:
        print('저장된 동작이 없음. record 로 만들어줘.')
        return
    for n in names:
        d = json.load(open(jpath(n)))
        print('%-16s %2d points  repeat_from=%s' % (n, len(d['waypoints']), d.get('repeat_from')))


def cmd_record(args):
    data = {'waypoints': [], 'repeat_from': None}
    if args.append and os.path.exists(jpath(args.name)):
        data = json.load(open(jpath(args.name)))
        print('기존 %d개에 이어서 기록' % len(data['waypoints']))

    arm = connect()
    if args.manual:
        arm.set_mode(2)   # 수동(프리드라이브) 모드: 손으로 팔을 움직일 수 있음
        arm.set_state(0)
        time.sleep(0.5)
        print('>> 수동 모드. 팔을 손으로 잡고 원하는 자세로 옮겨줘.')
    else:
        print('>> 일반 모드. UFactory Studio 나 조그로 움직인 뒤 저장해줘.')

    print('   [Enter] 현재 자세 저장   d 마지막 삭제   r 여기서부터 반복   q 저장하고 종료')
    try:
        while True:
            k = input('(%d개) > ' % len(data['waypoints'])).strip().lower()
            if k == 'q':
                break
            if k == 'd':
                if data['waypoints']:
                    data['waypoints'].pop()
                    print('   마지막 삭제')
                continue
            if k == 'r':
                data['repeat_from'] = len(data['waypoints'])
                print('   반복 시작점 = %d' % data['repeat_from'])
                continue
            wp = snapshot(arm)
            data['waypoints'].append(wp)
            print('   저장 #%d  %s' % (len(data['waypoints']), fmt(wp)))
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.3)

    if data['waypoints']:
        json.dump(data, open(jpath(args.name), 'w'), indent=1)
        print('저장 완료: %s (%d points)' % (jpath(args.name), len(data['waypoints'])))
    else:
        print('저장할 자세가 없어서 파일은 안 만들었음.')
    arm.disconnect()


def goto(arm, wp, speed, mode):
    if mode == 'angle':
        return arm.set_servo_angle(angle=wp['angles'], speed=speed, wait=True)
    return arm.set_position(*wp['pose'], speed=speed, wait=True)


def cmd_play(args):
    data = json.load(open(jpath(args.name)))
    wps = data['waypoints']
    rf = data.get('repeat_from')
    if args.repeat_from is not None:
        rf = args.repeat_from

    arm = connect()
    speed = args.speed if args.mode == 'angle' else args.speed * 3

    head = wps if rf is None else wps[:rf]
    tail = [] if rf is None else wps[rf:]

    def run(seq, tag):
        for i, wp in enumerate(seq):
            code = goto(arm, wp, speed, args.mode)
            print('%s #%d code=%s' % (tag, i + 1, code))
            if code != 0:
                raise RuntimeError('동작 실패 code=%s' % code)

    try:
        run(head, 'go')
        if tail:
            for n in range(args.loop):
                run(tail, 'loop%d' % (n + 1))
        elif args.loop > 1:
            for n in range(args.loop - 1):
                run(wps, 'loop%d' % (n + 2))
        if args.home:
            arm.move_gohome(wait=True)
    finally:
        arm.disconnect()
    print('완료')


def main():
    os.makedirs(DIR, exist_ok=True)
    ap = argparse.ArgumentParser(description='xArm6 자세 티칭/재생')
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('here', help='현재 자세 출력').set_defaults(func=cmd_here)
    sub.add_parser('list', help='저장된 동작 목록').set_defaults(func=cmd_list)

    p = sub.add_parser('record', help='자세 기록')
    p.add_argument('name')
    p.add_argument('--manual', action='store_true', help='수동(프리드라이브) 모드로 기록')
    p.add_argument('--append', action='store_true', help='기존 파일에 이어서')
    p.set_defaults(func=cmd_record)

    p = sub.add_parser('play', help='동작 재생')
    p.add_argument('name')
    p.add_argument('--speed', type=float, default=40, help='관절 deg/s (기본 40)')
    p.add_argument('--loop', type=int, default=1)
    p.add_argument('--repeat-from', type=int, default=None)
    p.add_argument('--mode', choices=['angle', 'pose'], default='angle',
                   help='angle=관절각 재생(권장), pose=xyz+rpy 재생')
    p.add_argument('--home', action='store_true', help='끝나고 홈으로')
    p.set_defaults(func=cmd_play)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
