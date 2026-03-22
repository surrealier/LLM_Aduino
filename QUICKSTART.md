# ccoli Quick Start

## 1. Install

```bash
curl -fsSL https://raw.githubusercontent.com/surrealier/LLM_Aduino/main/scripts/install.sh | bash
```

The bootstrap script installs the lightweight CLI first, then opens `ccoli setup` so you can choose:
- `Ollama Local`
- `Cloud API` (`gemini`, `claude`, `chatgpt`)
- `Configure Later`

On macOS, the setup flow defaults STT to `cpu` and skips unused heavy packages like `torch` / `transformers`.

If you want to run onboarding again later:

```bash
ccoli setup
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
- LED status: red before the server link is ready, light green once the server connection is healthy.
- On the first healthy connection, ccoli plays a short contextual welcome greeting.

Optional temporary port override:

```bash
ccoli start --port 5002
```

## 4. Optional Wi-Fi mode

Use this only if you want the ESP32 to connect over Wi-Fi instead of USB:

```bash
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

## 5. Current mode support

- Agent mode: available
- Robot mode: not available yet (Servo + Display integration in progress)



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
