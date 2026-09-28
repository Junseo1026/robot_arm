# gh_hello2 — 손 들면 티칭 동작으로 인사 (v2)

관객이 손목을 어깨 위로 올리면 로봇이 저장소 최상위 `poses/gh_hello2.json` 의 좌표로만 인사한다.

```sh
.venv/bin/python detect_greet.py --scenario gh_hello2 --show --robot --dry-run   # 연결만, 안 움직임
.venv/bin/python detect_greet.py --scenario gh_hello2 --show --robot
```

## 동작

번호는 jog.py 에서 저장한 순서(1부터). 괄호는 JSON 배열 인덱스(0부터).

| 저장 순서 | 인덱스 | J4 | 역할 |
|---|---|---|---|
| 1 | 0 | 1.9 | 시작 |
| 2 | 1 | 1.9 | 팔 들기 |
| 3 | 2 | 1.9 | 손목 세우기 |
| 4 | 3 | -30.3 | 흔들기 좌 |
| 5 | 4 | 22.7 | 흔들기 우 |
| 6 | 5 | 0.9 | 손 가운데로 |
| 7 | 6 | 0.9 | 팔 내림, 인사 후 머무는 자리 |

```
1 → 2 → 3 → (4 → 5) × 2 → 6 → 7
```

- `motion.repeat: [3, 5]` — **인덱스**(0부터, 끝 제외) 기준. 저장 순서 4·5번을 반복한다
- `motion.cycles: 2` — 반복 횟수
- 마지막 자세에서 멈추고, 다음 인사는 거기서 첫 자세로 간다
- `motion.rest: false` — 시작할 때도, 인사 후에도 `greet.py` 의 내장 `REST` 로 가지 않는다
- 사람 방향(J1) 회전도 하지 않는다

자세를 다시 찍으려면 `jog.py` 에서 `w gh_hello2` 로 덮어쓰면 바로 반영된다.
자세 개수가 바뀌면 `repeat` 도 맞춰 고친다.
