# robot_arm

xArm6 로봇팔 제어 코드. 전시용으로, 카메라에 사람이 잡히면 로봇팔이 인사한다.

## 전체 순서 요약

| 하려는 것 | 실행 | 키 |
|---|---|---|
| 포인트 찍기 | `.venv/bin/python jog.py` | 관절값 6개 입력 → `s` 저장 → `w 이름` 파일로 |
| 저장한 대로 실행 | `.venv/bin/python greet.py play --name 이름` | — |
| 내장 인사 실행 | `.venv/bin/python greet.py wave` | — |
| 긴급 중단 | 실행 중 `Ctrl+C` | 이후 `reset`(jog) 또는 `greet.py rest` |

`s` 는 메모리에만 담는다. **`w 이름` 을 해야 파일로 남는다.**

## 구성

| 파일 | 역할 |
|---|---|
| `greet.py` | 인사 동작. `Greeter` 클래스 + 단독 실행 CLI |
| `jog.py` | 관절값을 직접 입력해 움직이는 티칭 REPL |
| `xarm_pose.py` | 자세 기록/재생 (수동 프리드라이브 티칭 포함) |
| `poses/*.json` | 저장된 웨이포인트 |

## 하드웨어 / 네트워크

- 로봇: xArm6, 컨트롤러 v2.7.0, `192.168.1.221`
- UFactory Studio: <http://192.168.1.221:18333>
- 카메라: Arducam 12MP (UVC), 모니터 옆 고정. 카메라는 움직이지 않는다.

Mac은 Wi-Fi로 인터넷을 쓰면서 USB 이더넷으로 로봇에 붙는다. **게이트웨이를 비워두는 것이
핵심** — USB LAN이 서비스 순서상 Wi-Fi보다 위라서 게이트웨이를 넣으면 기본 경로를 가로채
인터넷이 끊긴다.

```sh
sudo networksetup -setmanual "USB 10/100 LAN" 192.168.1.100 255.255.255.0 ""
```

## 설치

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

의존성은 반드시 이 venv에 둔다. base conda 환경에 mediapipe/opencv를 넣으면 numpy가 2.x로
올라가 scipy·numba·gensim이 깨진다.

## 사용

### 처음 실행할 때는 이 순서로

```sh
.venv/bin/python greet.py wave --dry-run       # 안 움직인다. 지나갈 자세만 검증해 출력
.venv/bin/python greet.py wave --step          # 이동마다 Enter 확인 (s=건너뜀, q=중단)
.venv/bin/python greet.py wave --speed 15      # 느리게 연속 실행
.venv/bin/python greet.py wave                 # 확인 끝나면 기본 속도로
```

`--speed` 기본값은 20 deg/s로 낮게 잡아뒀다. 흔들기도 이 값을 따른다.

### 멈추는 방법

**Ctrl+C 만으로는 멈추지 않는다.** 컨트롤러 큐에 이미 쌓인 동작이 계속 실행된다.
`greet.py`는 Ctrl+C 를 받으면 `emergency_stop()` 을 보내도록 해 뒀다. 정지 후에는
`greet.py rest` 로 대기 자세부터 다시 잡아야 한다. 물리 비상정지 버튼과 UFactory Studio의
정지 버튼이 항상 최종 수단이다.

### 그 다음

```sh
.venv/bin/python greet.py wave             # 손 흔들기
.venv/bin/python greet.py wave --j1 25     # 왼쪽 사람을 보고 인사
.venv/bin/python greet.py wave --smooth    # 사인파 스트리밍(부드러움)
.venv/bin/python greet.py random           # wave/big/small/bow 중 랜덤
.venv/bin/python greet.py rest             # 대기 자세로
```

감지 코드에서 호출:

```python
from greet import Greeter

g = Greeter()          # 연결 + 대기 자세
g.greet(j1_deg=15)     # 사람 방향으로 인사한 뒤 대기 자세 복귀
g.close()
```

### 내가 저장한 동작으로 인사하기

`greet.py wave` 는 `poses/` 를 읽지 않는다. 코드 상단의 `REST`/`RAISE` 상수로 계산한
내장 동작이다. 직접 티칭한 동작을 쓰려면 `play` 를 쓴다.

```sh
.venv/bin/python greet.py --list                        # 저장된 동작 목록
.venv/bin/python greet.py play --name hello --dry-run   # 검증만
.venv/bin/python greet.py play --name hello --cycles 3
```

감지 코드에서도 마찬가지로 고를 수 있다.

```python
g.greet(j1_deg=15)      # 내장 인사
g.greet(name='hello')   # 티칭해 둔 동작
```

`repeat_from` 이 있는 파일은 그 지점부터 끝까지만 `cycles` 번 반복한다 (흔드는 구간).
`jog.py` 로 저장한 파일에는 `repeat_from` 이 없어 전체를 반복한다. 흔드는 구간만 반복시키려면
`xarm_pose.py record` 에서 `r` 로 지정하거나 JSON 의 `repeat_from` 을 직접 넣으면 된다.

자세를 새로 찾을 때:

```sh
.venv/bin/python jog.py
#  0 -60 -30 0 0 0   관절값 6개 절대 이동
#  +6 15             6번 관절만 +15도
#  s / p / d         현재 자세 저장 / 재생 / 마지막 삭제
#  w hello           poses/hello.json 으로 저장
#  speed 40, h(홈), q(종료)
```

## 동작 설계

`REST`(팔 내림) → `RAISE`(손 들기) → J6 좌우 흔들기 → `REST`.
두 상수는 `greet.py` 상단에 있고, 이것만 바꾸면 전체 동작이 따라 바뀐다.

- `RAISE = [0, -60, -30, 0, 0, 0]` — 툴 Z축이 정면 수평(+X), 높이 608mm.
  이 자세에서는 **J6을 돌려도 xyz가 전혀 변하지 않는다.** 회전축이 정면 방향과 일치하므로
  J6 회전이 곧 사람이 손을 흔드는 동작이 된다. 손가락이 고정된 3D 프린팅 손에 적합하다.
- `REST = [0, -10, -35, 0, 45, 0]` — xyz (375, 0, 273).
- `J1_LIMIT = 60` — 사람 쪽으로 돌리는 최대 각도. 모니터가 팔 근처면 줄인다.

동작 범위는 반경 약 400mm, 높이 610mm. J1이 ±60° 도니 옆 공간도 필요하다.

### 웨이포인트는 자동으로 보간되지 않는다

`set_servo_angle(wait=True)`는 점마다 완전히 멈춘다. 부드럽게 하려면 (1) `wait=False`로
명령을 큐에 쌓아 블렌딩시키거나, (2) `set_position`의 `radius`로 코너를 깎거나, (3) mode 1에서
100Hz로 보간점을 스트리밍해야 한다. 손 흔들기는 양 끝에서 방향을 바꾸므로 큐 방식으로도
자연스럽고, `_wave_smooth()`가 (3)번 방식을 구현해 뒀다.

## 시나리오

동작 하나하나를 `scenarios/<이름>/` 에 모아둔다. 새 시나리오는 폴더를 하나 더 만들면 된다.

```
scenarios/hello/
├── scenario.json      트리거, 사용할 동작, 파라미터
├── poses/hello.json   티칭한 자세
└── README.md
```

```sh
.venv/bin/python detect_greet.py --list-scenarios
.venv/bin/python detect_greet.py --scenario hello --show --robot
```

`scenario.json` 의 값이 기본값이 되고, 명령줄 옵션을 주면 그쪽이 이긴다.
자세 파일은 `scenarios/*/poses/` 와 최상위 `poses/` 에서 찾는다.

## 사람 감지 — detect_greet.py

**행동분류 모델을 학습시키지 않는다.** YOLO11-pose 로 키포인트를 뽑고 "손목이 어깨보다
위에 있다"는 규칙으로 판정한다. 이미지 좌표는 y 가 아래로 커지므로 손을 들면 손목 y 가
어깨 y 보다 작아진다. 학습 데이터도, 모델 관리도 필요 없고 조명·복장 변화에 강하다.
흔드는 동작도 손목 x 좌표의 진동(방향 전환 횟수)으로 보므로 모델이 없다.

분류값은 터미널에 계속 찍히고 화면에도 사람마다 라벨이 붙는다.

| 분류 | 조건 |
|---|---|
| `서 있음` | 양 손목이 어깨보다 아래 |
| `손 들기` | 한쪽 손목이 어깨보다 위 |
| `손 흔들기` | 손 든 상태에서 손목 x 가 좌우로 진동 |
| `양손 들기` | 양쪽 손목이 어깨보다 위 |
| `판정불가` | 손목·어깨 키포인트 신뢰도가 0.5 미만 |

```sh
.venv/bin/python detect_greet.py --show                  # 감지만. 팔이 안 움직인다
.venv/bin/python detect_greet.py --show --every 0.5      # 0.5초마다 분류값 출력
.venv/bin/python detect_greet.py --show --robot          # 로봇 연결
.venv/bin/python detect_greet.py --robot --require-wave   # 흔들어야 반응
```

`--robot` 을 붙이지 않으면 절대 팔이 움직이지 않는다. 먼저 이걸로 판정이 맞는지 본다.

주요 옵션:

- `--camera N` 카메라 인덱스. **Arducam IMX477 은 0, 맥북 내장은 1.**
  `--list-cameras` 로 확인한다 (최대 해상도 4032x3040 이 IMX477)
- `--device auto` 애플 실리콘 GPU(mps) 사용. 1280x720 입력에서 CPU 31fps -> MPS 57fps
- `--tracker bytetrack.yaml` 기본값. ultralytics 기본 트래커인 BoT-SORT 는
  GMC(카메라 움직임 보정)에 OpenCV 의 sparse optical flow 를 쓰는데 OpenCV 5.0 에서
  assertion 으로 깨져 `GMC failed` 경고를 매 프레임 쏟아낸다. 카메라가 고정이라
  GMC 자체가 필요 없으므로 GMC 가 없는 ByteTrack 을 쓴다
- `--hold 0.4` 이 시간 이상 조건이 유지되어야 인사 (오검출 방지)
- `--cooldown 8` 같은 사람에게 다시 인사하기까지 최소 간격. 트래킹 ID 기준
- `--j1-span 45` 화면 좌우 끝에 대응하는 J1 각도. `--flip-j1` 로 좌우 반전
- `--trigger wave|handup` 반응 조건. 기본은 `wave`(좌우로 흔들 때).
  `handup` 은 손만 들어도 반응
- `--dry-run` `--robot` 과 함께 쓰면 로봇에 연결만 하고 움직이지 않는다. 배선 확인용
- `--name hello` 티칭해 둔 동작으로 인사

여러 명이 잡혀도 그중 **아무나** 조건을 만족하면 반응한다. 동시에 여러 명이 만족하면
바운딩박스가 가장 큰 사람(= 가장 가까운 사람)을 향해 인사한다.

인사는 별도 스레드에서 돌아간다. 인사하는 몇 초 동안에도 카메라 루프와 화면이 멈추지
않고, 인사 중에는 새 인사를 받지 않는다(화면 좌상단에 `인사 중` 표시).

### 카메라 위치

어깨와 손목이 **함께** 보여야 판정된다. 어깨 y 좌표가 화면 위쪽 끝(0 근처)에 붙으면
상반신이 잘린 것이고, 손을 들어도 어깨 키포인트 신뢰도가 떨어져 `판정불가` 나 `서 있음`
으로 빠진다. 모니터 옆 눈높이에서 관객 상반신이 화면 가운데 들어오도록 맞춘다.

### 카메라 권한

macOS 는 카메라 접근을 앱 단위로 막는다. 시스템 설정 > 개인정보 보호 및 보안 > 카메라 에서
**터미널**을 허용하고, 터미널을 완전히 종료(⌘Q)한 뒤 다시 열어야 적용된다. 권한이 없으면
`not authorized to capture video` 가 뜨고 모든 인덱스가 열리지 않는다.

## 다음 단계

대기 동작을 붙인다. 워크바이 전시라서 인사 하나만으로는 관객이 없는 대부분의
시간에 로봇이 죽은 것처럼 보인다. 그래서:

- 대기 중 아주 느린 스캔 동작으로 "살아있음"을 보여준다
- 얼굴이 잡히면 그 방향으로 J1을 돌려 쳐다본 뒤 인사한다 (뒤통수에는 반응하지 않는 게이팅)
- 같은 사람에게 반복 인사하지 않도록 트래킹 + 쿨다운
- 인사 종류를 랜덤화해 반복 관람자도 다른 걸 보게 한다
