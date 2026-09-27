# hello — 손 흔들면 인사

관객이 손을 좌우로 흔들면 로봇이 그 방향으로 팔을 들어 마주 인사한다.

```sh
.venv/bin/python detect_greet.py --scenario hello --show --robot
```

## 구성

- 트리거: `wave` (손목이 어깨보다 위 + 손목 x 좌우 진동)
- 동작: `poses/hello.json` — jog.py 로 티칭한 5개 포인트, 2번째부터 반복
- 여러 명 중 아무나 흔들면 반응하고, 가장 가까운 사람을 향한다

## 조정

동작을 다시 티칭하려면:

```sh
.venv/bin/python jog.py
#  s 저장 / r 반복 시작점 / w hello 파일로
```

`w hello` 는 저장소 최상위 `poses/` 에 쓴다. 이 시나리오에 반영하려면
`scenarios/hello/poses/hello.json` 으로 옮긴다.
