# Telegram Channel MVP Guide

## 배경
`ccoli`의 iOS 확장 전 단계로 Telegram Bot 채널 MVP를 제공합니다.

## 범위
- 메시지 수신
- LLM 응답 생성/전송
- 인증(chat id allow-list)
- 레이트리밋(min interval)

## 운영/배포
1. Telegram bot token은 코드/문서에 평문으로 저장하지 말고 `server/.env`로 관리
2. 허용 chat id 목록을 운영자가 관리
3. 전송 실패 시 사용자 메시지와 내부 디버그를 분리

## BotFather로 봇 만들기
1. Telegram 앱에서 `@BotFather`를 연다
2. `/newbot` 입력
3. 봇 이름(display name) 입력
4. 봇 username 입력
   - 반드시 `bot`으로 끝나야 한다
   - 예: `ccoli_home_bot`
5. BotFather가 발급한 HTTP API token을 복사한다

## 서버 설정
`server/.env`에 아래 값을 추가합니다.

```bash
TELEGRAM_ENABLED=true
TELEGRAM_BOT_TOKEN=123456789:replace_with_real_token
TELEGRAM_ALLOWED_CHAT_IDS=
TELEGRAM_MIN_INTERVAL_SEC=0.5
TELEGRAM_POLL_INTERVAL_SEC=1.0
TELEGRAM_LONG_POLL_TIMEOUT_SEC=20
```

설명:
- `TELEGRAM_ENABLED=true`: Telegram polling worker 활성화
- `TELEGRAM_BOT_TOKEN`: BotFather가 발급한 토큰
- `TELEGRAM_ALLOWED_CHAT_IDS`: 허용할 chat id 목록. 쉼표로 여러 개 지정 가능
- `TELEGRAM_MIN_INTERVAL_SEC`: 같은 chat id에 대한 최소 응답 간격
- `TELEGRAM_POLL_INTERVAL_SEC`: 오류 발생 시 재시도 대기 시간
- `TELEGRAM_LONG_POLL_TIMEOUT_SEC`: Telegram `getUpdates` long polling timeout

## 첫 연결 순서
1. 처음에는 `TELEGRAM_ALLOWED_CHAT_IDS`를 비워 둔다
2. 서버를 시작한다

```bash
ccoli start
```

3. Telegram에서 방금 만든 봇을 검색해 대화를 시작한다
4. `/start` 또는 일반 메시지를 보낸다
5. 응답이 오면 연동 성공이다

## allow-list 잠그기
개인용 봇으로 운영할 때는 첫 연결 확인 후 allow-list를 설정하는 것을 권장합니다.

예시:

```bash
TELEGRAM_ALLOWED_CHAT_IDS=123456789,-100987654321
```

- 개인 대화는 보통 양수 chat id
- 그룹/채널은 음수 chat id가 올 수 있다
- allow-list에 없는 사용자가 메시지를 보내면 서버가 거절 응답을 보낸다

## 현재 동작 방식
- Telegram Bot API `getUpdates` long polling 사용
- 들어온 텍스트 메시지를 기존 `AgentMode.generate_response(...)`에 전달
- `@@우선순위 상태` 같은 runtime 텍스트 명령도 기존 runtime controller 경로 재사용
- 사용자별 대화 메모리는 `telegram:{chat_id}` 기준으로 분리

## 장애 대응
- 전송 실패 증가 시 봇 토큰 재발급
- 레이트리밋 알람 빈도 증가 시 `min_interval_sec` 상향

## 롤백
- 채널 기능 비활성화
- allow-list 비우기
