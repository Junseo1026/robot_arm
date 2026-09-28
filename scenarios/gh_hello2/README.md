# gh_hello2 — 손 흔들면 인사, 머리 위로 들면 하이파이브 (v2)

관객이 손을 어깨~머리 높이로 들고 좌우로 흔들면(`trigger: wave`) 로봇이 저장소 최상위 `poses/gh_hello2.json` 의 좌표로만 인사한다.

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

## 하이파이브

손을 머리 위로 들면(`over_head`, `--hold` 이상 유지) 하이파이브를 한다. 흔들기보다 우선한다.

```
gh_hello2 첫 자세 → ighfive 0 → ighfive 1 (손 내밀기)
  → 충격 대기 (최대 timeout 초)
  → ighfive 0 → gh_hello2 첫 자세 (왔던 길 그대로 복귀)
```

`scenario.json` 의 `highfive`:

- `name`: 자세 파일 (`poses/ighfive.json`)
- `start`: 시작·복귀 자세 `'<이름>:<인덱스>'` (인덱스 0 부터). `gh_hello2:0`
- `timeout`: 손을 내민 채 기다리는 최대 시간(초). 지나면 충격 없이 복귀
- `current`: 충격으로 볼 관절 전류 변화(A). 정지 중 떨림은 0.02A 안팎. 0.35 는 손대기 전에 반응하는 일이 있어 0.5 로 둔다

충격은 둘 중 하나로 본다.

1. 컨트롤러 충돌 감지(민감도는 UFactory Studio 설정)로 팔이 멈춤 → 에러를 지우고 복귀
2. 관절 전류가 손 내민 직후 기준보다 `current` 넘게 변함. 전류는 초당 5회만 보고되므로
   아주 짧은 충격은 놓칠 수 있다

로봇만 따로 시험하려면 (비전 없이):

```sh
.venv/bin/python greet.py highfive --name ighfive --start gh_hello2:0 --speed 40 --dry-run
.venv/bin/python greet.py highfive --name ighfive --start gh_hello2:0 --speed 40
```

충격이 없으면 `시간 초과. 최대 전류 변화 J1 .. J6 ..` 가 찍힌다. 이 값을 보고 `current` 를 정한다.
