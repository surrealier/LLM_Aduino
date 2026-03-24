# ccoli Server API Map

This document describes the current runtime entrypoints and key server modules.

## Runtime Entry

- `server/server.py`
  - Main device server process
  - Auto-detects wired USB serial or accepts Wi-Fi/TCP connections
  - Handles ESP32 packet I/O, STT pipeline, Agent mode orchestration
  - Starts the optional web dashboard and logs dashboard URL(s) plus `/api/docs`

## CLI Entry

- `ccoli/cli.py`
  - `ccoli setup` / `ccoli install`
  - `ccoli start`
  - `ccoli config wifi <WiFi Name> password <password> port <port>`

## Core Modules (`server/src`)

- `server/src/protocol.py`
  - Packet type constants
  - Packet encode/decode helpers
  - CMD/AUDIO_OUT send helpers
- `server/src/connection_manager.py`
  - USB serial auto-detect + TCP listen/accept loop
  - Live `Wired > WiFi` priority polling in `auto` mode
- `server/src/stt_engine.py`
  - Whisper model load + transcription wrapper
  - GPU/CPU device-priority fallback
  - Current production path uses `faster-whisper`, so STT runs on `cpu`/`cuda` and does not use Apple `MPS`
- `server/src/audio_processor.py`
  - Audio quality checks, trim, normalization
- `server/src/agent_mode.py`
  - Agent response orchestration (LLM/TTS/services)
- `server/src/robot_mode.py`
  - Robot command parser (currently gated by feature flag)
- `server/src/llm_client.py`
  - Multi-provider LLM wrapper
  - Runtime-verified priority order: `ollama -> api -> ollama_cpu -> other`
- `server/src/runtime_preferences.py`
  - Runtime priority defaults, model resolution, hardware detection
- `server/src/runtime_controller.py`
  - Conversational priority commands such as `우선순위 상태`
  - Live runtime reload for LLM/STT/connection preference changes
- `server/src/input_gate.py`
  - Stream gating for turn-based processing
- `server/src/job_queue.py`
  - Queue utility for STT/TTS command flows

## Configuration Sources

- `server/config.yaml` (primary)
- `server/.env` (optional overrides, see `server/env.example`)

## Web API Notes

- `GET /api/status`
  - Returns the existing dashboard status payload
  - Now also includes `runtime` with current model/network/processor priority state
  - If optional web packages such as `uvicorn` are missing, the device server keeps running and logs an install hint instead of exiting
- `POST /api/chat`
  - Regular agent chat
  - Also accepts runtime-priority commands that would normally be spoken to the device

## Mode Availability

- Agent mode: enabled
- Robot mode: disabled by default via `features.robot_mode_enabled: false`
