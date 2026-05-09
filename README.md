<div align="center">

<img src="assets/ccoli.png" alt="ccoli logo" />

# 🥦 ccoli

**Talk to your ESP32. Let your PC think.**

Voice-first AI assistant for Arduino makers — speak to an Atom Echo, get intelligent responses powered by local or cloud LLMs.

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](LICENSE)

[Quick Start](#-quick-start) · [Features](#-features) · [Docs](docs/) · [QUICKSTART.md](QUICKSTART.md)

</div>

---

## 💡 What is ccoli?

ccoli turns an **M5Stack Atom Echo (ESP32)** into a voice assistant powered by your PC. You speak → the device sends audio over USB or Wi-Fi → your PC handles speech recognition, LLM reasoning, and text-to-speech → the device plays back the response.

No cloud required. Runs with local Ollama out of the box, or connect to Gemini / Claude / ChatGPT.

<div align="center">
<img src="assets/summary.png" alt="ccoli system overview" width="700" />
</div>

## ✅ What You Need

| | Component | Why |
|---|-----------|-----|
| 🖥️ | **PC** (Windows / Mac / Linux) | Runs the ccoli server (STT + LLM + TTS) |
| 🎤 | **M5Stack Atom Echo** (ESP32) | Captures your voice & plays responses |
| 🔌 | **USB-C cable** | Default wired mode, auto-detected by the server |
| 📶 | **Same Wi-Fi network** | Optional wireless mode |

Official Atom Echo links:
- Product docs: [M5Stack Atom Echo](https://docs.m5stack.com/en/atom/atomecho)
- Official store: [ATOM Echo Smart Speaker Development Kit](https://shop.m5stack.com/products/atom-echo-smart-speaker-dev-kit)

## 🚀 Quick Start

### 1. Install

```bash
curl -fsSL https://raw.githubusercontent.com/surrealier/LLM_Aduino/main/scripts/install.sh | bash
```

The bootstrap script installs the lightweight CLI first, then hands off the rest to `ccoli setup`.

The setup wizard asks whether you want:
- `Ollama Local` for on-device local models
- `Cloud API` for Gemini / Claude / ChatGPT
- `Configure Later` if you only want the runtime installed first

During the same `ccoli setup` flow you can also choose:
- `wired` for USB serial firmware defaults
- `wifi` to write Wi-Fi credentials and `SERVER_IP` into `device_secrets.h`

It keeps the base install lightweight, then installs only the runtime extras this project actually uses. The default runtime no longer pulls `torch` or `transformers`.

If you want to rerun onboarding later:

```bash
ccoli setup
```

If startup later says the web dashboard dependency is missing, reinstall the runtime extras from the repo root:

```bash
python3 -m pip install -e .[runtime]
```

Local repo / development path:

```bash
./scripts/install.sh
# or
python3 scripts/install.py
```

### 2. Flash firmware

Open `arduino/atom_echo_m5stack_esp32_ino/atom_echo_m5stack_esp32_ino.ino` in Arduino IDE and upload to your Atom Echo.

No extra setup is required for default USB wired mode.
- If `device_secrets.h` is missing, the firmware boots in wired mode automatically
- `ccoli start` auto-detects the Atom Echo over USB serial
- Default wired serial speed is `115200` for broad CP210x stability on macOS
- Wired USB audio uses `8kHz G.711 mu-law` in both directions so mic capture and TTS both fit inside the wired bandwidth budget
- Arduino IDE upload speed can stay at `115200`; flashing speed and runtime protocol settings are still separate
- On the first connection, the server temporarily locks mic capture while the welcome TTS is sent, then the firmware reopens it after playback ends

Optional robot/display mode:
- Install Arduino libraries `Adafruit SSD1306` and `Adafruit GFX Library`
- Connect an external SSD1306 OLED to `G25` (SDA) and `G21` (SCL)

<details>
<summary><b>First-Time Arduino IDE Setup (ESP32 + Atom Echo)</b></summary>

If this is your first ESP32 project, use this order:

1. Install Arduino IDE 2.x from the official Arduino site.
2. Open `Arduino IDE -> Settings` and add this Board Manager URL:
   `https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json`
3. Open `Boards Manager` and install `esp32` by `Espressif Systems`.
4. Still in `Boards Manager`, install the `M5Stack` board package so the Atom board profiles appear cleanly in Arduino IDE.
5. Open `Library Manager` and install:
   - `M5Unified`
   - `ESP32Servo`
   - `Adafruit SSD1306` by Adafruit
   - `Adafruit GFX Library` by Adafruit
6. When Arduino IDE asks to install dependent libraries for `M5Unified`, choose `Install All`.
7. Connect the Atom Echo with a USB-C data cable, then check `Tools -> Port` and confirm a serial device appears.
8. In `Tools -> Board`, select an Atom-compatible target. For the original Atom Echo, start with `M5Atom`.
9. Open `arduino/atom_echo_m5stack_esp32_ino/atom_echo_m5stack_esp32_ino.ino`, compile once, then upload.
10. If the board is not recognized:
    - Reconnect with a known data-capable USB cable
    - Try another USB port
    - Restart Arduino IDE after board/library installation
    - Install the official Silicon Labs driver: [CP210x USB to UART Bridge VCP Drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
11. If you only want default wired mode, you can upload without creating `device_secrets.h`.
12. If you want Wi-Fi mode later, run `ccoli config wifi ...` and then set `SERVER_IP` in `arduino/atom_echo_m5stack_esp32_ino/device_secrets.h`.

Notes:
- `M5Unified` is the key firmware dependency for the current sketch.
- `ESP32Servo` is currently included by the firmware, so install it even if you are not using robot mode yet.
- `Adafruit SSD1306` and `Adafruit GFX Library` are only needed for the optional external OLED display flow.

</details>

### 3. Start

```bash
ccoli start
```

Then connect the Atom Echo to your PC with USB-C.
- The server preloads STT and TTS once during startup so the first spoken turn does not pay the full model warmup cost.
- When the web dashboard is enabled, startup logs print the dashboard URL(s) and the `/api/docs` link.
- LED status: red while waiting for the server link, light green when the device is connected and ready.
- On the first healthy connection, ccoli speaks a short welcome line that picks up the recent conversation context when possible.
- On macOS, the current STT path uses `faster-whisper`, so STT stays on `cpu` rather than Apple `MPS`. The default TTS backend `edge_tts` also does not use local MPS/GPU acceleration.

### 4. Optional Wi-Fi mode

```bash
ccoli setup
# choose `wifi` during onboarding

# or update later with:
ccoli config wifi MyHomeWiFi password MySecretPass port 5001
```

Then set `SERVER_IP` in `arduino/atom_echo_m5stack_esp32_ino/device_secrets.h` to your PC's local IP and upload again.

### 5. Optional Telegram bot chat

If you enabled the Telegram channel in `server/.env`, you can start chatting from Telegram as well:

1. Open `@BotFather` and confirm your bot's `username`
2. Search that `username` in Telegram
3. Open the bot chat and tap `Start`, or send `/start`
4. Send a normal message such as `안녕`
5. If the server is running, ccoli replies through the bot

Tips:
- If you changed `server/.env` after starting the server, restart `ccoli start`
- If `TELEGRAM_ALLOWED_CHAT_IDS` is blank, the first chat is not blocked by the allow-list
- Full setup guide: `docs/TELEGRAM_CHANNEL_GUIDE.md`

🎉 That's it — speak to the Atom Echo and hear the response!

## 🏗️ How It Works

```mermaid
flowchart LR
    U["🗣️ You"] --> A["🎤 Atom Echo"]
    A -->|Audio over USB / Wi-Fi| S["🖥️ ccoli server"]
    S -->|Text| L["🧠 LLM\nOllama / Gemini / Claude / ChatGPT"]
    L -->|Response| S
    S -->|TTS audio| A
```

## ✨ Features

- 🗣️ **Voice-first** — speak naturally, get voice responses
- 🧠 **Multi-LLM** — Ollama (local, default), Gemini, Claude, ChatGPT
- 🧭 **Runtime priority routing** — resolves model, network, and processor candidates in priority order, then keeps the selected LLM route until config or priority is reloaded
- 🔌 **Integrations** — weather, calendar, search, maps, notifications
- 🎙️ **Voice ID** — speaker recognition to personalize responses
- 🤖 **Robot mode** *(coming soon)* — servo/display control via voice
- 🐳 **Docker tests** — reproducible test suite out of the box

## 🖥️ Terminal UI Flow

`ccoli` already has a terminal-first onboarding flow with Rich panels and tables. It is not a full-screen ncurses app, but it behaves like a lightweight TUI for install/setup tasks.

Typical flow:
- Run `ccoli setup`
- Pick your AI path: `Ollama Local`, `Cloud API`, or `Configure Later`
- If needed, pick the provider and model
- Pick the STT device (`cpu` on macOS by default)
- Pick the device connection mode (`wired` or `wifi`)
- If needed, enter Wi-Fi SSID/password and the server IP
- Review the generated setup plan in the terminal
- Confirm, then start with `ccoli start`

Example session:

```text
$ ccoli setup
┌─ ccoli Setup ─────────────────────────────────────┐
│ Choose how ccoli should install and configure     │
│ your AI runtime.                                  │
└───────────────────────────────────────────────────┘

Choose your AI path
  1. ollama  - Local model on your machine via Ollama
  2. api     - Gemini / Claude / ChatGPT via API key
  3. manual  - Install runtime now and configure later
Select [1]: 2

Choose your cloud provider
  1. gemini  - Google Gemini
  2. claude  - Anthropic Claude
  3. chatgpt - OpenAI ChatGPT
Select [1]: 1

Model name [gemini-1.5-flash]:
Choose STT device
  1. cpu   - Best default for macOS and general compatibility
  2. cuda  - Use NVIDIA CUDA when available
Select [1]: 1

Choose device connection
  1. wired - USB serial, no Wi-Fi credentials required
  2. wifi  - Write Wi-Fi credentials and server IP to device_secrets.h
Select [1]: 1

Setup Plan
- Install target: api
- Provider: gemini
- Model: gemini-1.5-flash
- STT device: cpu
- Device connection: wired
- Server port: 5001
- Python extras: runtime
Continue with this setup plan? [Y/n]: y

$ ccoli start
...
Runtime warmup: stt_ready=True tts_ready=True
Web dashboard: http://localhost:8005
Web API docs: http://localhost:8005/api/docs
```

Useful terminal commands after setup:
- `ccoli config integration list`
- `ccoli config voice-id status`
- `ccoli start --port 5002`

## ⚙️ Configuration

<details>
<summary><b>LLM Provider</b></summary>

Default is Ollama (local, no API key). Switch anytime:

```bash
ccoli setup
ccoli config llm --provider ollama --model qwen3:8b
ccoli config llm --provider gemini --model gemini-1.5-flash --api-key <GEMINI_API_KEY>
ccoli config llm --provider claude --model claude-3-5-haiku-latest --api-key <ANTHROPIC_API_KEY>
ccoli config llm --provider chatgpt --model gpt-4o-mini --api-key <OPENAI_API_KEY>
```

API key 발급:
| Provider | Get API Key |
|----------|-------------|
| Gemini | [Google AI Studio](https://aistudio.google.com/apikey) |
| Claude | [Anthropic Console](https://console.anthropic.com/settings/keys) |
| ChatGPT | [OpenAI Platform](https://platform.openai.com/api-keys) |

Ollama is auto-installed and auto-started if missing.

</details>

<details>
<summary><b>Runtime Priority</b></summary>

`ccoli` now keeps runtime priority as first-class config:

```yaml
llm:
  priority: [ollama, api, ollama_cpu, other]
  api_priority: [gemini, claude, chatgpt]
connection:
  priority: [wired, wifi]
runtime:
  processor_priority: [gpu, cpu]
```

You can change the same priorities during a conversation or in the web chat:

```text
@@우선순위 상태
모델 우선순위 ollama > api > ollama cpu > other
api 우선순위 gemini > claude > chatgpt
연결 우선순위 wired > wifi
프로세서 우선순위 gpu > cpu
```

When `connection.mode` is `auto`, the server keeps checking both `Wired` and `WiFi` live and binds to the first healthy link that appears while still honoring the current priority order.

LLM priority is resolved on the first LLM request after startup or priority/config reload. If a higher-priority route such as local Ollama is unavailable and Gemini succeeds, later turns go directly to Gemini instead of rechecking Ollama on every message.

`ollama_cpu` is a distinct fallback bucket in runtime policy, but a single shared Ollama server cannot be forced to switch GPU/CPU per request. To make that bucket physically separate, point it at a dedicated CPU-only local Ollama instance.

On macOS, `GPU` priority can still matter for local LLM routing, but the current STT/TTS stack does not run on Apple `MPS`.

</details>

<details>
<summary><b>Integrations</b></summary>

```bash
ccoli config integration list                          # see all integrations
ccoli config integration set weather --api-key <KEY>   # configure
ccoli config integration enable weather                # enable
ccoli config integration test weather                  # verify
```

Google Calendar example:

```bash
ccoli config integration set calendar-google \
  --client-id <ID> --client-secret <SECRET> --refresh-token <TOKEN>
ccoli config integration test calendar-google
```

Missing keys? The `test` command tells you exactly what to set.

Integration API key 발급:
| Integration | Get API Key |
|-------------|-------------|
| Weather | [OpenWeatherMap](https://home.openweathermap.org/api_keys) |
| Search | [Tavily](https://app.tavily.com/home) |
| Maps | [Google Maps Platform](https://console.cloud.google.com/apis/credentials) |
| Calendar (Google) | [Google Cloud Console](https://console.cloud.google.com/apis/credentials) |
| Notify (Slack) | [Slack API — Your Apps](https://api.slack.com/apps) |

</details>

<details>
<summary><b>Voice ID</b></summary>

```bash
ccoli config voice-id enable
ccoli config voice-id threshold --value 0.72
ccoli config voice-id status
```

Or control via voice at runtime:

```
@@<USERNAME> register voice
@@enable voice recognition
```

</details>

## 📋 CLI Reference

| Command | Description |
|---------|-------------|
| `ccoli setup` | Interactive installer / onboarding wizard |
| `ccoli start` | Start the server |
| `ccoli start --port 5002` | Start with port override |
| `ccoli config wifi <SSID> password <PASS> port <PORT>` | Configure optional Wi-Fi mode |
| `ccoli config llm --provider <name> [--model <m>] [--api-key <k>]` | Set LLM provider |
| `ccoli config integration <list\|set\|enable\|disable\|test>` | Manage integrations |
| `ccoli config voice-id <status\|enable\|disable\|delete\|threshold>` | Manage Voice ID |

## 🧪 Testing

```bash
# Docker (recommended)
docker compose -f docker/docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from server-test

# or use the helper script
./scripts/run_docker_tests.sh
```

CI runs the same suite on every PR via GitHub Actions (`.github/workflows/docker-tests.yml`).

## 📁 Project Structure

```
ccoli/
├── arduino/          # Atom Echo ESP32 firmware
├── ccoli/            # CLI entry point
├── server/           # Python server (STT / LLM / TTS)
│   ├── server.py
│   ├── config.yaml
│   └── src/
├── docs/             # API, protocol, PRD docs
├── docker/           # Docker Compose for tests & mocks
└── scripts/          # Helper scripts
```

## 📖 Documentation

| Doc | What's inside |
|-----|---------------|
| [QUICKSTART.md](QUICKSTART.md) | Quick onboarding guide |
| [docs/API.md](docs/API.md) | Server module map |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Binary protocol spec |
| [docs/PRD.md](docs/PRD.md) | Product requirements |

## 🔒 Security

- Never commit real credentials — use `device_secrets.h` only for optional Wi-Fi mode (git-ignored)
- Server secrets go in `server/.env` (see `server/env.example`)

## 📜 License

This project is licensed under the [GNU Affero General Public License v3.0](LICENSE).

## Web dashboard

Open http://localhost:8005 for the multilingual dashboard with `English` as the default UI, optional `한국어 / 日本語 / 中文` switching, a diagnostics-first runtime view, editable memory/schedules/chat, and live logs.
