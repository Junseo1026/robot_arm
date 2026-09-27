#!/usr/bin/env python3
"""xArm6 관절값 직접 입력 / 티칭 REPL.

  .venv/bin/python jog.py

입력 방법:
  0 -60 -30 0 0 0   6개 관절값(deg)을 절대값으로 이동
  +6 15             6번 관절만 +15도 (상대 이동).  -2 10 은 2번 관절 -10도
  Enter             현재 자세 다시 출력
  s                 현재 자세를 웨이포인트로 저장
  p                 저장한 웨이포인트를 순서대로 재생
  d                 마지막 웨이포인트 삭제
  w 이름            poses/이름.json 으로 저장
  h                 홈으로
  speed 40          이동 속도(deg/s) 변경
  q                 종료
"""
import json, os, sys, time
from xarm.wrapper import XArmAPI

IP = os.environ.get('XARM_IP', '192.168.1.221')
DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'poses')


def connect():
    arm = XArmAPI(IP, is_radian=False)
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)
    return arm


def show(arm):
    c1, ang = arm.get_servo_angle()
    c2, pose = arm.get_position()
    if c1 or c2:
        print('  읽기 실패 %s/%s' % (c1, c2))
        return None
    print('  J = ' + ' '.join('%7.2f' % a for a in ang[:6]))
    print('  xyz = %7.1f %7.1f %7.1f   rpy = %6.1f %6.1f %6.1f' % tuple(pose[:6]))
    return ang[:6]


def main():
    os.makedirs(DIR, exist_ok=True)
    arm = connect()
    speed = 30.0
    wps = []
    print(__doc__)
    print('현재 자세:')
    show(arm)

    while True:
        try:
            line = input('jog[%d저장,%.0fdeg/s]> ' % (len(wps), speed)).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        low = line.lower()
        if low == 'q':
            break
        if line == '':
            show(arm)
            continue
        if low == 'h':
            print('  홈으로 이동...')
            print('  code=%s' % arm.move_gohome(speed=speed, wait=True))
            show(arm)
            continue
        if low == 's':
            ang = show(arm)
            if ang:
                wps.append([round(a, 2) for a in ang])
                print('  저장 #%d' % len(wps))
            continue
        if low == 'd':
            if wps:
                wps.pop()
                print('  마지막 삭제 (남은 %d개)' % len(wps))
            continue
        if low == 'p':
            if not wps:
                print('  저장된 웨이포인트 없음')
                continue
            for i, a in enumerate(wps):
                code = arm.set_servo_angle(angle=a, speed=speed, wait=True)
                print('  #%d code=%s' % (i + 1, code))
                if code:
                    break
            continue
        if low.startswith('speed'):
            try:
                speed = float(line.split()[1])
                print('  속도 = %.0f deg/s' % speed)
            except (IndexError, ValueError):
                print('  예: speed 40')
            continue
        if low.startswith('w'):
            parts = line.split()
            if len(parts) < 2:
                print('  예: w hello')
                continue
            if not wps:
                print('  저장된 웨이포인트 없음')
                continue
            path = os.path.join(DIR, parts[1] + '.json')
            json.dump({'waypoints': [{'angles': a} for a in wps], 'repeat_from': None},
                      open(path, 'w'), indent=1)
            print('  파일 저장: %s (%d개)' % (path, len(wps)))
            continue

        # 상대 이동:  +6 15  /  -2 10
        if line[0] in '+-' and len(line.split()) == 2 and line[1].isdigit():
            j = int(line[1])
            try:
                delta = float(line.split()[1])
            except ValueError:
                print('  예: +6 15')
                continue
            if not 1 <= j <= 6:
                print('  관절 번호는 1~6')
                continue
            if line[0] == '-':
                delta = -delta
            code, ang = arm.get_servo_angle()
            if code:
                print('  읽기 실패 %s' % code)
                continue
            target = [round(a, 2) for a in ang[:6]]
            target[j - 1] += delta
            print('  J%d %+.1f -> %s' % (j, delta, target))
            print('  code=%s' % arm.set_servo_angle(angle=target, speed=speed, wait=True))
            show(arm)
            continue

        # 절대 이동: 관절값 6개
        parts = line.replace(',', ' ').split()
        if len(parts) == 6:
            try:
                target = [float(p) for p in parts]
            except ValueError:
                print('  숫자 6개를 입력해줘')
                continue
            print('  이동 -> %s' % target)
            print('  code=%s' % arm.set_servo_angle(angle=target, speed=speed, wait=True))
            show(arm)
            continue

        print('  인식 못한 입력. 도움말은 파일 상단 주석 참고.')

    arm.disconnect()
    print('종료')


if __name__ == '__main__':
    main()
