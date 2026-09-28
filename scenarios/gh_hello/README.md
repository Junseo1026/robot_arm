# gh_hello — 손 들면 티칭 동작으로 인사

관객이 손목을 어깨 위로 올리면 로봇이 `poses/gh_hello.json` 의 좌표로만 인사한다.

```sh
.venv/bin/python detect_greet.py --scenario gh_hello --show --robot --dry-run   # 연결만, 안 움직임
.venv/bin/python detect_greet.py --scenario gh_hello --show --robot
```

## 동작

원본 기록은 저장소 최상위 `poses/gh_hello.json` (5개). 여기서 첫 번째 자세를 뺀 4개를 쓴다.

```
#1 → #2 → (#3 → #4) × 3
```

- `repeat_from: 2` — 3·4번 자세를 한 세트로 반복, 횟수는 `scenario.json` 의 `motion.cycles`
- `motion.rest: false` — 시작할 때도, 인사 후에도 `greet.py` 의 내장 `REST` 로 가지 않는다.
  인사가 끝나면 마지막 자세(#4)에 멈춰 있고, 다음 인사는 거기서 #1 로 간다
- 사람 방향(J1) 회전도 하지 않는다. 저장된 관절값 그대로 움직인다
