# ccoli Device Protocol

## Packet Format

Every packet uses:
- `1 byte` packet type
- `2 bytes` payload length (little-endian)
- `N bytes` payload

## Packet Types

### ESP32 -> Server

- `0x01` `START`
  - Voice stream started
- `0x02` `AUDIO`
  - Wi-Fi/TCP: PCM16LE mono audio chunk (`16kHz`)
  - Wired USB serial: G.711 mu-law mono audio chunk (`8kHz`)
- `0x03` `END`
  - Voice stream ended
- `0x10` `PING`
  - Keepalive heartbeat
- `0x13` `BUFFER_STATUS` (optional)
  - ESP32 buffer telemetry (if enabled)

### Server -> ESP32

- `0x11` `CMD`
  - JSON command payload
- `0x12` `AUDIO_OUT`
  - Wi-Fi/TCP: PCM16LE mono TTS chunk (`16kHz`)
  - Wired USB serial: G.711 mu-law mono TTS chunk (`8kHz`)
  - Wired USB compatibility path keeps chunks at `<= 512 bytes`
- `0x1F` `PONG`
  - Keepalive response

## Audio Format

- Wi-Fi/TCP
  - Encoding: PCM 16-bit little-endian
  - Channel: mono
  - Sample rate: 16kHz
- Wired USB
  - Encoding: G.711 mu-law
  - Channel: mono
  - Sample rate: 8kHz

## Command Payload (`0x11`)

Server sends JSON, for example:

```json
{
  "action": "NOOP",
  "sid": 7,
  "meaningful": false,
  "recognized": true
}
```

Startup control messages may also use the same `CMD` channel, for example `{"action":"MIC_LOCK"}` and `{"action":"MIC_UNLOCK"}`.

## Connection Notes

- ESP32 reconnect logic is handled on firmware side.
- Server answers `PING` with `PONG`.
- Wired USB mode may receive an initial `PONG` immediately after the serial port opens so the device can promote the link to ready before the first keepalive arrives.
- The same packet framing is used over Wi-Fi/TCP and wired USB serial.
- The default wired serial baudrate is `115200`; server and firmware must match.
- Wired USB uses `8kHz mu-law` audio in both directions so speech capture and playback both fit inside `115200` baud on CP210x-class adapters.
- Firmware flashing speed is separate from the runtime protocol speed, so Arduino IDE upload can remain at `115200`.
- Wired USB connections may emit a few boot/reset bytes before the first real packet; the server now skips out-of-sync bytes and re-locks onto the next valid frame automatically.
- Server-side socket timeout is configured in `server/config.yaml` under `connection.socket_timeout`.
- Wired USB mode is auto-detected by the server when `connection.mode` is `auto` or `wired`.
