# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**ccoli** is a voice-first AI assistant that turns an M5Stack Atom Echo (ESP32) into a voice assistant powered by a PC. Audio is streamed over Wi-Fi (TCP) from the device to a Python server, which runs STT → LLM → TTS and sends audio back.

## Commands

### Install & Run

```bash
# Install server dependencies
pip install -r server/requirements.txt

# Install CLI in editable mode
pip install -e .

# Start the server
ccoli start

# Configure Wi-Fi endpoint (written to device firmware or config)
ccoli config wifi <SSID> <PASSWORD> port 5001

# Configure LLM provider
ccoli config llm --provider ollama --model qwen3:8b
ccoli config llm --provider claude --model claude-3-5-haiku-latest --api-key <KEY>
ccoli config llm --provider gemini --model gemini-1.5-flash --api-key <KEY>
```

### Testing

Tests run inside Docker (recommended — avoids GPU/hardware dependencies):

```bash
# Full test suite (server tests + protocol simulation + firmware build check)
docker compose -f docker/docker-compose.test.yml up --build --abort-on-container-exit --exit-code-from server-test

# Convenience wrapper
./scripts/run_docker_tests.sh

# Run specific test markers
pytest -m protocol   # binary protocol regression tests
pytest -m telegram   # Telegram channel tests
pytest -m channel    # channel integration tests
```

## Architecture

### Data Flow

```
ESP32 (mic) → TCP binary packets → server/server.py
    → STT (faster-whisper) → transcription
    → Agent Mode or Robot Mode
    → LLM (Ollama / Gemini / Claude / ChatGPT)
    → TTS (edge-tts) → audio bytes
    → TCP → ESP32 (speaker)
```

### Key Components

| Layer | Location | Purpose |
|---|---|---|
| CLI | `ccoli/cli.py` | Entry point; setup, config, Wi-Fi, integrations |
| TCP Server | `server/server.py` | Accepts ESP32 connections; orchestrates pipeline |
| STT | `server/src/stt_engine.py` | faster-whisper, CUDA/CPU, default Korean |
| LLM Client | `server/src/llm_client.py` | Multi-provider; streaming; token retry |
| Agent Mode | `server/src/agent_mode.py` | Conversation history, integrations, memory |
| Robot Mode | `server/src/robot_mode.py` | Servo + OLED control (feature-gated off by default) |
| Protocol | `server/src/protocol.py` | Binary packet encode/decode (see `docs/PROTOCOL.md`) |
| Audio | `server/src/audio_processor.py` | Silence trimming, normalization, QC |
| Memory | `server/src/memory_manager.py` | Markdown-file-based long-term memory |
| Voice ID | `server/src/voice_id/` | Speaker embedding & recognition |
| Integrations | `server/src/integrations/` | Weather, Search, Maps, Calendar, Slack, Notify |
| Channels | `server/src/channels/` | Telegram remote voice channel |

### Binary Protocol (ESP32 ↔ Server)

Packets: `[1B type][2B length LE][N bytes payload]`

- **ESP32 → Server**: START, AUDIO (PCM16LE 16kHz mono), END, PING, BUFFER_STATUS
- **Server → ESP32**: CMD, AUDIO_OUT, PONG

Full spec: `docs/PROTOCOL.md`

### Configuration

- `server/config.yaml` — runtime config (server port, STT model, LLM provider, TTS voice, integrations, robot mode)
- `server/env.example` — optional env var overrides
- `arduino/.../device_secrets.h.example` — ESP32 Wi-Fi credentials and `SERVER_IP`

### Memory System

Agent uses structured markdown files in `server/memory/`:
- `Soul.md` — agent personality / system prompt
- `User.md` — user personal info
- `Shortterm_Memory.md` — recent conversation summary
- `Longterm_Memory.md` — important facts
- `Relation.md` — user relationships

The memory manager refreshes these via LLM extraction from conversations.

### Integration Pattern

All external service integrations live in `server/src/integrations/`. New integrations should extend `base.py` and register in `registry.py`. The CLI surfaces them via `ccoli config integration`.

## Development Conventions (from AGENTS.md)

- Write tests before code (TDD).
- All tests must pass inside Docker — do not rely on local hardware or GPU.
- Use type hints; keep functions small.
- Separate user-facing error messages from debug logs.
- Robot mode changes require both `server/src/robot_mode.py` and `server/commands.yaml`.
