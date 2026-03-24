# Robot Mode Plan — Atom Echo + SG90 + Waveshare 1.69inch LCD

## 배경/문제
- 현재 로봇 모드 초안은 `SSD1306 I2C OLED + 서보 2개 직결` 전제를 갖고 있다.
- 새 목표 하드웨어는 `SG90 Servo Motor 최대 4개`와 `Waveshare 1.69inch LCD Module (IPS, SPI, ST7789V2)`다.
- 요구사항도 단순 감정 태그 표시가 아니라, 대화 맥락에 따라 제스처를 수행하고, 감정이 한 번의 답변마다 즉시 리셋되지 않는 지속형 로봇 펫 동작이다.
- 이 문서는 공식 문서를 기준으로 현재 Atom Echo 기반 시스템에서 구현 가능한 권장 구조와 실행 계획을 정리한다.

## 범위
- 메인 음성 보드는 계속 `M5Stack Atom Echo (ESP32-PICO-D4)`를 사용한다.
- 로봇 모드는 최대 4개의 SG90와 Waveshare 1.69인치 LCD를 지원한다.
- 동작 범위:
  - 응답과 연결된 의미 있는 제스처
  - 슬립 모드가 아닐 때의 idle 액션
  - 눈 깜빡임, 시선 이동, 하품, 졸림 등 지속형 표정 애니메이션
  - `SOUL + RELATION + MOOD + AFFECT + BODY` 기반 감정 지속 모델
- 비범위:
  - Atom Echo의 마이크/스피커를 제거하는 보드 교체
  - 1차 릴리즈에서의 터치 입력 사용
  - 서보 4개를 Atom Echo에 직접 배선하는 단일 MCU 구조

## 공식 문서 검토 요약

| 대상 | 공식 문서 핵심 내용 | 계획상 의미 |
|---|---|---|
| M5Stack Atom Echo | 공식 문서에 `PinOut G21/G25/5V/GND, 3V3/G22/G19/G23/G33`, Grove는 `G26/G32`, 그리고 `G19/G22/G23/G33` 재사용 금지가 명시됨 | 음성 기능을 유지하면 로봇용으로 실질적으로 쓸 수 있는 외부 GPIO가 매우 제한적이다 |
| ESP32 LEDC | Espressif LEDC 문서는 ESP32가 PWM 신호를 GPIO로 라우팅할 수 있고, 주파수/분해능 조정이 가능하다고 설명함 | SG90 구동 자체는 가능하지만, 병목은 PWM 기능이 아니라 Atom Echo에서 외부로 꺼낼 수 있는 핀 수다 |
| TowerPro SG90 Analog | 공식 페이지에 `Operating voltage 4.8V`, `Dead band width 1us`, `Power Supply: Through External Adapter`가 명시됨 | 서보는 외부 5V 전원 레일을 전제로 계획해야 하며, Atom Echo USB 전원만으로 4개 동시 구동을 목표로 잡으면 안 된다 |
| Waveshare 1.69inch LCD Module | 공식 위키에 `3.3V/5V`, `logic voltage and supply voltage must be the same`, `4-wire SPI`, `DIN/CLK/CS/DC/RST/BL`, `240x280`, `ST7789V2`, `3.3V 90mA`가 명시됨 | ESP32 3.3V 로직에 맞추려면 LCD도 3.3V 구동이 가장 안전하며, 디스플레이는 전용 SPI 제어 핀을 여러 개 요구한다 |

## 결론: Atom Echo 단독 직결은 권장 아키텍처가 아니다
- Atom Echo 공식 문서 기준으로 로봇용으로 안전하게 활용 가능한 외부 GPIO는 사실상 `G21/G25/G26/G32` 중심이다.
- Waveshare LCD는 공식 배선 기준으로 `DIN/CLK/CS/DC/RST/BL`를 요구하고, SG90 4개는 PWM 4채널과 외부 전원을 요구한다.
- 따라서 `Atom Echo 1개 + 공식 노출 핀만 사용 + 음성 기능 유지`라는 조건에서는 유지보수 가능한 단일 MCU 직결 구성이 현실적이지 않다.
- 이 계획의 권장안은 `Atom Echo + 보조 컨트롤러(Companion Controller)` 구조다.

## 권장 하드웨어 아키텍처

### 권장안 A — 듀얼 컨트롤러 구조 (추천)
- `Atom Echo`
  - 마이크/스피커
  - 서버와의 USB/Wi-Fi 통신
  - 음성 인식 결과/LLM 응답 수신
  - 로봇 상태 프레임을 보조 컨트롤러로 전달
- `Companion Controller`
  - SPI 기반 Waveshare LCD 구동
  - SG90 최대 4개 PWM 제어
  - 표정 프레임 루프와 서보 스케줄러를 로컬에서 유지
- `Breadboard + 외부 전원 분배`
  - 서보용 5V 전원 레일
  - 공통 GND
  - LCD용 3.3V 전원 레일

### 비권장안 B — Atom Echo 단일 MCU + 확장 칩 체인
- GPIO expander, PWM expander, 레벨 설계까지 모두 얹으면 이론상 가능성은 있다.
- 그러나 배선 복잡도와 디버깅 비용이 커지고, 현재 저장소의 펌웨어 구조도 크게 흔들린다.
- PoC 실험용으로는 가능하지만 1차 구현 목표로는 비추천한다.

### 대안 C — 메인 보드 교체
- GPIO가 더 많은 ESP32/ESP32-S3 보드로 주 제어 보드를 교체하면 단일 MCU 구성이 쉬워진다.
- 하지만 현재 제품의 Atom Echo 음성 입출력 경로를 크게 바꾸므로 이번 계획에서는 제외한다.

## 권장 배선 계획

### 1) Atom Echo ↔ Companion Controller
- `G26` -> Companion `RX`
- `G32` -> Companion `TX`
- `GND` -> Companion `GND`
- 이유:
  - Grove 포트 2핀으로 UART 브리지를 만들 수 있다
  - Atom Echo의 오디오 관련 금지 핀을 건드리지 않는다
  - 디스플레이/서보의 타이밍 루프를 Companion 쪽으로 완전히 분리할 수 있다

### 2) Companion Controller ↔ Waveshare 1.69 LCD
- `3.3V` -> `VCC`
- `GND` -> `GND`
- SPI GPIO -> `DIN`, `CLK`, `CS`, `DC`, `RST`, `BL`
- 원칙:
  - Waveshare 공식 문서상 전원과 로직 전압을 같게 맞춰야 하므로 3.3V 구동을 기본값으로 한다
  - 5V 구동을 선택할 경우 별도 레벨 시프터 계획이 필요하다

### 3) Companion Controller ↔ SG90 x 1~4
- 각 SG90 signal -> PWM 가능한 GPIO 또는 Companion에 연결된 전용 서보 드라이버
- `V+` -> 외부 5V 전원
- `GND` -> 공통 GND
- 권장:
  - 5V 2A 이상, 가능하면 3A급 외부 전원
  - 서보 전원 입력 근처에 벌크 캐패시터 추가
  - USB 전원 하나에 모든 서보를 직접 매달지 않는다

### 4) 기구/역할 추천
- Servo 0: head pan
- Servo 1: head tilt/pitch
- Servo 2: left arm
- Servo 3: right arm
- 서보가 2개만 장착된 빌드에서는 `head pan + head tilt`만 활성화하고, 팔 동작은 head gesture로 대체한다.

## 감정 모델 설계

### 계층형 Emotion State

| 계층 | 지속 시간 | 저장 위치 | 역할 |
|---|---|---|---|
| `SOUL` | 매우 김 | `server/memory/Soul.md` | 기본 성격. 장난기, 예민함, 회복 속도, 수면 성향 |
| `RELATION` | 김 | `server/memory/Relation.md` | 사용자별 호감도, 신뢰도, 서운함, 친밀감 |
| `MOOD` | 중간 | 런타임 상태 | 최근 사건이 남긴 감정의 잔상 |
| `AFFECT` | 짧음 | 턴별 계산 | 방금 들어온 말에 대한 즉각 반응 |
| `BODY` | 중간 | 런타임 상태 | 졸림, 휴식, 슬립 모드, idle 에너지 |

### 감정 결정 원칙
- 최종 표정/제스처는 `AFFECT`만 보지 않고 `SOUL + RELATION + MOOD + AFFECT + BODY`를 함께 반영해 결정한다.
- 예시:
  - 사용자가 심한 말을 하면 `AFFECT=angry/hurt`, `MOOD`가 음수 방향으로 크게 이동하고, `RELATION.grudge`가 소폭 증가한다.
  - 이후 사용자가 사과하면 즉시 `AFFECT=softened`는 되지만, `MOOD`와 `RELATION.grudge`가 남아 한동안 삐진 표정과 소극적 액션을 유지한다.
  - 반복적으로 다정한 상호작용이 쌓이면 `RELATION.affection`이 올라가고, 같은 중립 문장에도 더 장난스럽고 친근한 idle 액션을 선택한다.

### 저장 전략
- `Soul.md`
  - 성격 파라미터 섹션 추가: `playfulness`, `sensitivity`, `forgiveness`, `curiosity`, `sleep_rhythm`
- `Relation.md`
  - 사용자별 감정 파라미터 섹션 추가: `affection`, `trust`, `grudge`, `comfort`
- 런타임
  - `EmotionSystem` 안에 `mood_vector`, `body_state`, `last_major_event_at`를 둔다
  - `MOOD`는 즉시 neutral로 리셋하지 않고, 회복 곡선과 이벤트 강도에 따라 서서히 이동한다

## 디스플레이 동작 설계

### 목표
- 한 번의 응답마다 얼굴이 딱 바뀌는 구조를 피한다.
- 기본 표정 위에 작은 생체적 움직임을 계속 얹는다.

### 필수 애니메이션 레이어
- `Base Face`
  - happy, sad, angry, sulky, sleepy, curious, surprised, neutral
- `Micro Motion`
  - blink
  - microsaccade
  - eyelid droop
  - subtle breathing/bob
- `Context Overlay`
  - yawn
  - talking pulse
  - sparkle / tear / annoyance mark
- `Transition`
  - 150~300ms easing 기반 전환
  - 표정이 바뀔 때 즉시 스냅 전환 금지

### 타이밍 정책
- blink: 3~7초 랜덤
- gaze shift: 5~12초 랜덤
- yawn: sleepy/bored 상태에서만 확률적으로 실행
- talk overlay: TTS 재생 중 mouth 또는 cheek pulse 활성화
- sleep mode: 눈 감김 + 저빈도 breathing만 유지, 과한 idle 액션 금지

## 액션/서보 동작 설계

### 제스처 계층
- `Semantic Gesture`
  - 대화 의미에 직접 대응하는 동작
  - 예: `"나 나갔다올게"` -> `farewell_wave`
  - 예: `"잔다"` -> `sleep_settle` 또는 `goodnight_wave`
- `Reply Gesture`
  - 응답을 말할 때 보조적으로 붙는 작은 끄덕임/손동작
- `Idle Gesture`
  - 슬립 모드가 아닐 때 확률적으로 나오는 자발 행동

### 우선 제스처 세트

| gesture_id | 트리거 | 2서보 폴백 | 4서보 권장 동작 |
|---|---|---|---|
| `farewell_wave` | 외출/작별 인사 | head tilt + nod | 오른팔 wave + head tilt |
| `goodnight_settle` | 수면/휴식 진입 | slow droop | 양팔 내림 + 눈 감김 + 고개 숙임 |
| `curious_tilt` | 질문/새 정보 | tilt | head tilt + 작은 팔 lift |
| `happy_bounce` | 칭찬/반가움 | nod + wiggle | 양팔 bounce + head bob |
| `hurt_turnaway` | 비난/무례 발화 | head turn away | 팔 고정 + 고개 회피 |
| `idle_stretch` | 장시간 대기 후 깨어있음 | 작은 sway | 팔 스트레치 + 고개 들기 |

### Idle 정책
- awake 상태에서만 활성화
- 과도한 소음/진동을 막기 위해 idle gesture는 쿨다운 기반으로 실행
- 권장 기본값:
  - micro motion: 수초 단위
  - 작은 servo idle: 15~45초 랜덤
  - 큰 idle gesture: 1~3분 랜덤, 최근 사용자 상호작용이 없을 때만

## 서버/펌웨어 역할 분리

### 서버
- 대화 맥락 분석
- `SOUL/RELATION/MOOD/AFFECT/BODY` 계산
- 의미 기반 gesture 추천
- 로봇 상태 페이로드 생성

### Atom Echo 펌웨어
- 서버와 기존 프로토콜 유지
- 로봇 페이로드를 Companion으로 브리지
- 실패 시 로봇 제어를 끊고 음성 경로만 유지

### Companion 펌웨어
- LCD 렌더 루프
- SG90 동작 스케줄링
- idle animation 자율 루프
- 안전 제한:
  - 각도 clamp
  - 서보 동작 쿨다운
  - long hold 최소화

## 제안 프로토콜

### 서버 -> Atom Echo
```json
{
  "action": "ROBOT_STATE",
  "mode": "awake",
  "emotion": {
    "dominant": "sulky",
    "valence": -0.45,
    "arousal": 0.18,
    "persist_sec": 900
  },
  "gesture": {
    "id": "farewell_wave",
    "intensity": 0.72
  },
  "speech": {
    "tts_active": true,
    "text": "다녀와. 조심하고."
  },
  "profile": {
    "servo_count": 4,
    "display": "st7789v2_240x280"
  }
}
```

### Atom Echo -> Companion
- 1차 구현은 디버깅이 쉬운 line-delimited JSON을 사용한다.
- 2차 최적화에서 CBOR 또는 바이너리 프레임을 검토한다.

## 구현 단계

### Phase 0 — 문서/하드웨어 PoC 정렬
- 산출물:
  - `docs/PRD.md`
  - `docs/AGENT_FEATURE_PLANNING.md`
  - `docs/ROBOT_MODE_WAVESHARE_PRD.md`
- 결정 항목:
  - Companion 컨트롤러 종류
  - Atom Echo ↔ Companion transport(UART 우선)
  - 2서보/4서보 프로파일 분리 여부
- 완료 기준:
  - 직결 대신 Companion 구조를 공식 계획으로 확정
  - 배선도 초안과 BOM 초안 작성

### Phase 1 — 프로토콜/설정 스키마 확장
- 대상 파일:
  - `server/config.yaml`
  - `server/src/robot_mode.py`
  - `server/server.py`
  - `arduino/atom_echo_m5stack_esp32_ino/atom_echo_m5stack_esp32_ino.ino`
  - 신규 `arduino/atom_echo_m5stack_esp32_ino/robot_bridge.*`
- 핵심 작업:
  - `robot.controller = legacy_direct | companion_uart`
  - `servo.count = 1..4`
  - `display.type = st7789v2_240x280`
  - 로봇 상태 페이로드 버전 정의
- 완료 기준:
  - 서버 단위 테스트에서 새 payload schema 검증
  - Atom Echo가 Companion으로 payload를 중계할 수 있음

### Phase 2 — Emotion Persistence 엔진
- 대상 파일:
  - `server/emotion_system.py`
  - `server/src/agent_mode.py`
  - `server/src/memory_manager.py`
  - `server/memory/Soul.md`
  - `server/memory/Relation.md`
- 핵심 작업:
  - `SOUL/RELATION/MOOD/AFFECT/BODY` 상태 모델 추가
  - 사과 후 즉시 neutral로 복귀하지 않는 회복 곡선 구현
  - idle 행동 확률에 감정 상태 반영
- 완료 기준:
  - insult -> apology -> lingering sulk 시나리오 테스트 통과
  - sleep mode에서 감정 표현이 억제되는 테스트 통과

### Phase 3 — Companion Display 엔진
- 대상 파일:
  - 신규 `arduino/robot_companion_controller/`
- 핵심 작업:
  - ST7789V2 초기화
  - face sprite/state machine
  - blink, gaze, yawn, talk overlay
- 완료 기준:
  - 부팅 시 neutral face
  - mood 전환 시 부드러운 face transition
  - idle loop 중 blink/gaze/yawn 동작 확인

### Phase 4 — Companion Servo 엔진
- 대상 파일:
  - 신규 `arduino/robot_companion_controller/`
- 핵심 작업:
  - 1~4채널 servo profile
  - gesture preset library
  - idle scheduler와 safety limiter
- 완료 기준:
  - 4서보 각 채널 독립 구동
  - semantic gesture와 idle gesture가 충돌 없이 실행

### Phase 5 — E2E 통합/튜닝
- 대상 파일:
  - `server/tests/*`
  - `docs/*`
  - `arduino/*`
- 핵심 작업:
  - Docker 기반 protocol regression
  - 하드웨어 smoke 시나리오
  - legacy direct path와 fallback 정리
- 완료 기준:
  - 대화 -> emotion -> gesture -> display 전체 흐름 재현
  - 하드웨어 연결 실패 시 음성 모드는 계속 동작

## 테스트 전략

### Docker 검증
- `docker compose -f docker/docker-compose.test.yml run --rm server-test pytest server/tests/test_emotion_system.py server/tests/test_robot_mode_extended.py`
- `docker compose -f docker/docker-compose.test.yml run --rm server-test pytest server/tests/test_protocol.py`
- `docker compose -f docker/docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from server-test`

### 추가 예정 테스트
- `server/tests/test_robot_emotion_persistence.py`
- `server/tests/test_robot_protocol_companion.py`
- `server/tests/test_robot_idle_behavior.py`

### 실기 하드웨어 스모크 체크리스트
1. 전원 투입 후 디스플레이가 neutral face로 부팅된다.
2. SG90 1~4 채널이 개별 sweep 테스트를 통과한다.
3. `"나 나갔다올게"` 발화 시 farewell 계열 동작이 나온다.
4. 부정적 발화 후 사과를 해도 표정이 곧바로 neutral로 돌아가지 않는다.
5. `"잘게"` 또는 sleep intent 후 과한 idle action이 멈춘다.

## 롤백 전략
- `features.robot_mode_enabled = false`로 전체 로봇 모드를 즉시 끌 수 있어야 한다.
- `robot.controller = legacy_direct`를 유지해 기존 SSD1306 실험 경로를 임시 보존할 수 있게 한다.
- Companion 경로가 불안정하면 Atom Echo는 로봇 제어를 생략하고 음성 에이전트 기능만 유지한다.
- 새 감정 상태 계산이 실패하면 `neutral + no gesture + safe display`로 폴백한다.

## 공식 문서 링크
- M5Stack Atom Echo: <https://docs.m5stack.com/en/atom/atomecho>
- Espressif ESP32 LEDC: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html>
- TowerPro SG90 Analog: <https://towerpro.com.tw/product/sg90-analog/>
- Waveshare 1.69inch LCD Module: <https://www.waveshare.com/wiki/1.69inch_LCD_Module>
