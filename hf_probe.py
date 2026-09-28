#!/usr/bin/env python3
"""하이파이브 충격 판정 기준을 정하기 위한 측정.

로봇을 하이파이브 자세(gh_hello2 첫 자세 -> ighfive)로 내밀어 둔 채, 관절 전류를
보고가 올 때마다(약 6Hz) 기록한다. 실시간 보고(포트 30003)의 토크는 모델 추정치라
외력에 반응하지 않아 쓰지 않는다 (v2.7.0 에서 확인). 음성으로 알려주는
순서대로 가만히 / 책상 흔들기 / 살짝 터치 / 세게 치기 를 하면 구간별로 CSV 에 남는다.
끝나면 왔던 길로 첫 자세까지 돌아간다. 저장한 좌표로만 움직인다.

  .venv/bin/python hf_probe.py              # 측정
  .venv/bin/python hf_probe.py --dry-run    # 순서만 출력, 로봇 연결 안 함

중단은 Ctrl+C (비상 정지).
"""
import argparse, csv, os, subprocess, time
from xarm.wrapper import XArmAPI
from greet import IP, ROOT, load_waypoints, resolve_pose

# (구간 이름, 길이 초, 시작할 때 읽어줄 말)
PHASES = [
    ('ready',  5, '5초 뒤에 시작합니다. 준비하세요'),
    ('idle',   8, '측정 시작. 가만히 계세요'),
    ('shake', 15, '책상을 흔드세요'),
    ('rest1',  5, '멈추세요'),
    ('light', 15, '손을 살짝 터치하세요. 여러 번'),
    ('rest2',  5, '멈추세요'),
    ('hard',  15, '하이파이브 하듯 세게 치세요. 여러 번'),
    ('rest3',  4, '멈추세요'),
]


def say(text):
    print('>> ' + text)
    try:
        subprocess.Popen(['say', '-v', 'Yuna', text])
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser(description='하이파이브 충격 측정')
    ap.add_argument('--start', default='gh_hello2:0')
    ap.add_argument('--name', default='ighfive')
    ap.add_argument('--speed', type=float, default=40.0)
    ap.add_argument('--out', help='CSV 경로. 기본 logs/hf_probe_<시각>.csv')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    path = [resolve_pose(args.start)] + load_waypoints(args.name)[0]
    out = args.out or os.path.join(ROOT, 'logs',
                                   time.strftime('hf_probe_%Y%m%d_%H%M%S.csv'))
    if args.dry_run:
        for a in path:
            print('  이동 J =', ' '.join('%6.1f' % v for v in a))
        for name, sec, text in PHASES:
            print('  %-6s %3d초  %s' % (name, sec, text))
        print('  복귀: 역순으로 %d개 자세' % (len(path) - 1))
        return

    arm = XArmAPI(IP, is_radian=False)
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

    def recover():
        arm.clean_error()
        arm.clean_warn()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.5)

    rows, events = [], []
    try:
        say('하이파이브 자세로 이동합니다')
        for a in path:
            code = arm.set_servo_angle(angle=a, speed=args.speed, wait=True)
            if code != 0:
                raise RuntimeError('이동 실패 code=%s' % code)
        time.sleep(2.0)                     # 멈춘 직후 흔들림이 가라앉기를 기다린다

        t0 = time.time()
        last = None
        for name, sec, text in PHASES:
            say(text)
            end = time.time() + sec
            while time.time() < end:
                cur = arm.currents          # 보고가 올 때마다 새 리스트로 바뀐다
                if cur is not last:
                    last = cur
                    rows.append([round(time.time() - t0, 4), name, arm.state]
                                + [round(v, 4) for v in cur[:6]]
                                + [round(v, 3) for v in arm.angles[:6]])
                if arm.state == 4:
                    events.append((round(time.time() - t0, 2), name, arm.error_code))
                    print('  충돌 감지로 멈춤 (%s, error=%s) -> 복구' % (name, arm.error_code))
                    recover()
                time.sleep(0.0005)
        say('측정 끝. 원래 자리로 돌아갑니다')
        if arm.state == 4:
            recover()
        for a in reversed(path[:-1]):
            arm.set_servo_angle(angle=a, speed=args.speed, wait=True)
    except KeyboardInterrupt:
        print('\n중단 요청 -> 비상 정지')
        arm.emergency_stop()
    finally:
        if rows:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'phase', 'state']
                           + ['i%d' % i for i in range(1, 7)]
                           + ['j%d' % i for i in range(1, 7)])
                w.writerows(rows)
            dur = rows[-1][0] - rows[0][0]
            print('저장: %s (%d개 샘플, 약 %.0fHz)' % (out, len(rows),
                                                   len(rows) / dur if dur else 0))
        for e in events:
            print('  충돌 감지: %.2f초 %s error=%s' % e)
        arm.disconnect()


if __name__ == '__main__':
    main()
