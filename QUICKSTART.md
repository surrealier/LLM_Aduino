# ccoli Quick Start

## 1. Install

```bash
curl -fsSL https://raw.githubusercontent.com/surrealier/LLM_Aduino/main/scripts/install.sh | bash
```

The bootstrap script installs the lightweight CLI first, then opens `ccoli setup` so you can choose:
- `Ollama Local`
- `Cloud API` (`gemini`, `claude`, `chatgpt`)
- `Configure Later`
- `wired` or `wifi` device connection during the same onboarding flow

On macOS, the setup flow defaults STT to `cpu`.
The current STT path (`faster-whisper`) does not use Apple `MPS`, and the default TTS backend (`edge_tts`) also does not use local MPS/GPU acceleration.

If you want to run onboarding again later:

```bash
ccoli setup
```

Non-interactive example:

```bash
ccoli setup --install-target api --provider gemini --connection-mode wired --skip-install --yes
```

Local repo / development path:

```bash
./scripts/install.sh
# or
python3 scripts/install.py
```

## 2. Flash firmware (default: wired USB mode)

Open and upload:
- `arduino/atom_echo_m5stack_esp32_ino/atom_echo_m5stack_esp32_ino.ino`

No extra config is required for wired mode:
- If `arduino/atom_echo_m5stack_esp32_ino/device_secrets.h` is missing, the firmware defaults to `CONNECTION_MODE = "wired"`
- The server auto-detects the Atom Echo over USB serial
- The default wired serial speed is `115200` for broad CP210x stability
- Wired USB audio uses `8kHz G.711 mu-law` in both directions so STT capture and TTS playback fit inside the wired bandwidth budget
- Arduino IDE upload speed can remain `115200`; it does not need to match the runtime protocol settings
- The server temporarily locks the mic while the first greeting is sent, then the firmware reopens it after playback

Optional robot/display mode only:
- Install Arduino libraries `Adafruit SSD1306` and `Adafruit GFX Library`
- Wire the SSD1306 OLED to `G25` (SDA) and `G21` (SCL)

## 3. Start server

```bash
ccoli start
```

Then connect the Atom Echo to your PC with USB-C.
- The server preloads STT and TTS during startup so the first turn avoids the cold-start model penalty.
- If the web dashboard is enabled and its dependencies are installed, startup logs print the dashboard URL(s) and the `/api/docs` link.
- LED status: red before the server link is ready, light green once the server connection is healthy.
- On the first healthy connection, ccoli plays a short time-of-day welcome greeting without calling the LLM.

Optional temporary port override:

```bash
ccoli start --port 5002
```

If startup says the web dashboard dependency is missing, reinstall the runtime extras from the repo root:

```bash
python3 -m pip install -e .[runtime]
```

## 4. Optional Wi-Fi mode

Use this only if you want the ESP32 to connect over Wi-Fi instead of USB:

```bash
ccoli setup
# choose `wifi` during onboarding

# or configure it later:
ccoli config wifi <WiFi Name> password <password> port <port> [mode wifi|wired]
```

Example:

```bash
ccoli config wifi MyHomeWiFi password MySecretPass port 5001
```

Alias (`colli`) is also supported:

```bash
colli config wifi MyHomeWiFi password MySecretPass port 5001
```

After running, check `arduino/atom_echo_m5stack_esp32_ino/device_secrets.h` and set:
- `SERVER_IP` to your PC/server LAN IP

## 5. Adjust runtime priority

These can be spoken to the device or typed in the web chat:

```text
@@우선순위 상태
모델 우선순위 ollama > api > ollama cpu > other
api 우선순위 gemini > claude > chatgpt
연결 우선순위 wired > wifi
프로세서 우선순위 gpu > cpu
```

Notes:
- `Wired > WiFi` is live in `auto` mode, so the server keeps checking both paths and connects to the first healthy link it finds.
- LLM routing is selected once after startup or priority/config reload. If local Ollama is unavailable and Gemini succeeds, later turns keep using Gemini instead of rechecking Ollama every time.
- Runtime LLM thinking is disabled for voice responses. Gemini calls send `thinkingBudget: 0`, and regular agent responses use a larger output token budget to reduce mid-sentence truncation.
- `GPU > CPU` applies directly to STT and local-LLM preference ordering.
- On macOS, `GPU` priority can still matter for local LLM routing, but the current STT/TTS stack does not run on Apple `MPS`.
- `ollama_cpu` is a separate runtime bucket, but it becomes a truly separate physical path only when you provide a CPU-only local Ollama instance.
- Current TTS default is `edge_tts`, so processor priority is recorded and exposed, but Edge TTS itself does not select a local GPU/CPU backend.

## 6. Current mode support

- Agent mode: available
- Robot mode: not available yet (Servo + Display integration in progress)

## 7. Optional Telegram bot channel

If you want to chat with ccoli from Telegram as well as the ESP32/web UI:

1. Create a bot with `@BotFather`
2. Add Telegram settings to `server/.env`
3. Start the server with `ccoli start`
4. Send a message to your bot in Telegram

Reference:
- Telegram setup guide: `docs/TELEGRAM_CHANNEL_GUIDE.md`


## References

- Product requirements: `docs/PRD.md`
- Execution planning: `docs/AGENT_FEATURE_PLANNING.md`

## Docker test entrypoint

Run the standardized test stack with Docker Compose:

```bash
docker compose -f docker/docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from server-test
```

`server-test` 컨테이너는 `server/tests` 전체를 실행해 모듈 단위 테스트, 통합 테스트, CLI 테스트, 시나리오 테스트를 한 번에 검증합니다.

Convenience wrapper:

```bash
./scripts/run_docker_tests.sh
```

If local Docker is not available, run the same pipeline in GitHub Actions:

- Workflow file: `.github/workflows/docker-tests.yml`
- Trigger manually from Actions tab using `workflow_dispatch`


## Ralph PoC evaluation

```bash
python scripts/evaluate_poc.py --tool ralph
```

## Web dashboard

By default the server also serves a multilingual dashboard at `http://localhost:8005` with `English` as the default UI, optional `한국어 / 日本語 / 中文` switching, runtime diagnostics, memory/schedule/chat views, and live logs.
When the server binds to `0.0.0.0`, startup logs also print a LAN URL if one is detected.
