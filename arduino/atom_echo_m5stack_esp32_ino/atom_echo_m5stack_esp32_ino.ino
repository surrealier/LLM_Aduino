// ============================================================
// atom_echo_m5stack_esp32_ino.ino - main application entry point
// ============================================================
// Role: setup()/loop() entry point for M5Stack Atom Echo.
//       Initializes all submodules (connection, protocol, VAD, audio, LED, servo)
//       and runs cooperative multitasking in the main loop.
//
// Flow:
//   setup() -> hardware initialization -> start WiFi connection
//   loop()  -> connection management -> protocol TX/RX -> TTS playback ->
//             half-duplex mic switching -> VAD voice detection ->
//             LED/servo updates
//
// Hardware pin map (Atom Echo):
//   G22=SPK_DATA, G19=SPK_BCLK, G33=SPK_LRCK/MIC_CLK
//   G23=MIC_DATA, G27=SK6812_LED, G39=BUTTON
//   G25=SERVO (external), G26/G32=Grove
// ============================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <math.h>
#include "config.h"
#include "audio_buffer.h"
#include "connection.h"
#include "led_control.h"
#include "protocol.h"
#include "serial_client.h"
#include "servo_control.h"
#include "vad.h"

// ────────────────────────────────────────────
// Network credentials and server address (define actual values here)
// Definitions corresponding to extern declarations in config.h
// ────────────────────────────────────────────
const char* SSID = "KT_GiGA_3926";
const char* PASS = "fbx7bef119";
const char* SERVER_IP = "172.30.1.20";
const uint16_t SERVER_PORT = 5001;

// ────────────────────────────────────────────
// Global state objects
// ────────────────────────────────────────────
WiFiClient client;               // TCP socket (WiFi 모드)
SerialClient serial_client(Serial);  // USB Serial 래퍼 (Serial 모드)
Client* transport = nullptr;     // 현재 활성 전송 레이어 포인터

// 전송 모드 선택 상태
bool g_serial_mode = false;      // true → Serial 모드, false → WiFi 모드 (protocol.cpp에서 extern)
static bool transport_decided = false;

ConnectionState conn_state;  // WiFi/server connection state machine
VadState vad_state;          // Voice activity detection state
PrerollBuffer preroll;       // Pre-roll audio buffer before VAD start

// Half-duplex control: track mic disable state during TTS playback
// Use a flag instead of M5.Mic.isEnabled() to avoid repeated end/begin calls
static bool mic_disabled = false;
static uint32_t last_play_end_ms = 0;
static bool was_playing_or_buffered = false;
static constexpr uint32_t MIC_REENABLE_COOLDOWN_MS = 450;

// Reinitialize speaker after mic end (Atom Echo shares I2S lines).
// Even if the M5Unified speaker task is alive, the I2S driver can switch via Mic.begin/end
// so call end() first to always reconfigure speaker I2S.
static bool speaker_reinit() {
  M5.Speaker.stop();
  M5.Speaker.end();
  auto spk_cfg = M5.Speaker.config();
  spk_cfg.sample_rate = AUDIO_SAMPLE_RATE;
  M5.Speaker.config(spk_cfg);
  bool ok = M5.Speaker.begin();
  M5.Speaker.setVolume(180);
  if (!ok) {
    Serial.println("[AUDIO] Speaker begin failed");
  }
  return ok;
}

// Reinitialize microphone and release speaker I2S resources first.
static bool mic_reinit() {
  M5.Speaker.stop();
  M5.Speaker.end();
  M5.Mic.end();
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = AUDIO_SAMPLE_RATE;
  M5.Mic.config(mic_cfg);
  bool ok = M5.Mic.begin();
  if (!ok) {
    Serial.println("[AUDIO] Mic begin failed");
  }
  return ok;
}

// ────────────────────────────────────────────
// frame_rms - calculate RMS (Root Mean Square) of an audio frame
// ────────────────────────────────────────────
// Purpose: measure the current frame loudness level for VAD
// Note: ESP32 has no double FPU, so use float math (~10x performance difference)
static inline float frame_rms(const int16_t* x, size_t n) {
  float ss = 0.0f;
  for (size_t i = 0; i < n; i++) {
    float v = (float)x[i];
    ss += v * v;
  }
  return sqrtf(ss / (float)n);
}

// ============================================================
// setup() - one-time run: initialize all hardware and software modules
// ============================================================
void setup() {
  // Initialize M5Unified framework (auto setup for I2S, LED, button, etc.)
  auto cfg = M5.config();
  cfg.internal_mic = true;
  cfg.internal_spk = true;
  M5.begin(cfg);
  Serial.begin(115200);
  delay(500);  // Wait for serial stabilization

  // Initialize LED -> indicate connecting (red)
  led_init();
  servo_init();
  display_init();
  display_show_face(FACE_NEUTRAL);
  led_set_color(LED_COLOR_CONNECTING_R, LED_COLOR_CONNECTING_G, LED_COLOR_CONNECTING_B);

  // Start WiFi connection (async; state tracked in connection_manage)
  connection_init(&conn_state, SSID, PASS);

  auto spk_cfg = M5.Speaker.config();
  auto mic_cfg = M5.Mic.config();
  // g_serial_mode는 아직 false이므로 여기서는 항상 출력 가능
  Serial.printf(
      "[BOOT] board=%d spk(bck=%d ws=%d dout=%d i2s=%d) mic(bck=%d ws=%d din=%d i2s=%d)\n",
      (int)M5.getBoard(),
      (int)spk_cfg.pin_bck,
      (int)spk_cfg.pin_ws,
      (int)spk_cfg.pin_data_out,
      (int)spk_cfg.i2s_port,
      (int)mic_cfg.pin_bck,
      (int)mic_cfg.pin_ws,
      (int)mic_cfg.pin_data_in,
      (int)mic_cfg.i2s_port);
  Serial.printf("[BOOT] WiFi timeout=%dms, Serial baud=%d\n",
      WIFI_CONNECT_TIMEOUT_MS, SERIAL_BAUD_RATE);

  // Default idle state keeps mic enabled (input) and speaker released from I2S resources
  mic_reinit();

  // Initialize protocol RX state machine, VAD, and pre-roll buffer
  protocol_init();
  vad_init(&vad_state);
  preroll_init(&preroll);
}

// ============================================================
// loop() - main loop: cooperative multitasking (about 1ms cycle)
// ============================================================
void loop() {
  // Update M5Unified internal state (button, touch, etc.)
  M5.update();

  // -- Button interrupt: stop immediately when button is pressed during TTS playback --
  #if ENABLE_BUTTON_INTERRUPT
  if (M5.BtnA.wasPressed()) {
    if (protocol_is_audio_playing()) {
      protocol_clear_audio_buffer();
      Serial.println("[BUTTON] TTS interrupted");
    }
  }
  #endif

  // -- 전송 레이어 자동 감지 및 연결 관리 --
  if (!transport_decided) {
    if (WiFi.status() == WL_CONNECTED && conn_state.server_connected) {
      // WiFi 연결 성공 → WiFi 모드 확정
      g_serial_mode = false;
      transport = &client;
      transport_decided = true;
      Serial.println("[TRANSPORT] WiFi mode selected");
    } else if (millis() > WIFI_CONNECT_TIMEOUT_MS) {
      // WiFi 타임아웃 → Serial 모드로 전환
      g_serial_mode = true;
      Serial.end();
      Serial.begin(SERIAL_BAUD_RATE);
      while (!Serial) { delay(10); }
      serial_client.setConnected(true);
      transport = &serial_client;
      transport_decided = true;
      // Cyan LED = Serial 모드 표시
      led_set_color(0, 255, 255);
    } else {
      // WiFi 연결 시도 중
      connection_manage(&conn_state, client);
      delay(100);
      return;
    }
  }

  // WiFi 모드: 연결 상태 유지
  if (!g_serial_mode) {
    connection_manage(&conn_state, client);
    if (!connection_is_server_connected(&conn_state)) {
      delay(100);
      return;
    }
    transport = &client;
  } else {
    // Serial 모드: USB가 끊기면 재연결 대기
    if (!serial_client.connected()) {
      delay(100);
      return;
    }
  }

  // -- Protocol TX/RX --
  protocol_send_ping_if_needed(*transport);  // Keepalive PING every 3 seconds
  protocol_poll(*transport);                  // Receive and dispatch server->ESP32 packets
  // Perform audio playback after mic switching

  // -- Half-duplex mic/speaker switching --
  // Atom Echo shares the I2S bus between mic and speaker
  // disable mic during TTS playback and enable it again after playback ends.
  // Use mic_disabled flag to switch only once (reduces I2S reconfiguration cost)
  bool will_play = protocol_is_audio_playing() || protocol_has_audio_buffered();
  // Record playback end moment (consider ended only when buffer is also empty)
  if (was_playing_or_buffered && !will_play) {
    last_play_end_ms = millis();
  }
  was_playing_or_buffered = will_play;

  if (will_play && !mic_disabled) {
    // TTS playback starts -> disable mic
    M5.Mic.end();
    if (speaker_reinit()) {
      mic_disabled = true;
      Serial.println("[AUDIO] Mic end -> Speaker reinit");
      vad_init(&vad_state);     // Reset VAD state (invalidate leftover voice data)
      preroll_init(&preroll);   // Reset pre-roll buffer
    } else {
      // If speaker init fails, clear buffer and restore input mode
      protocol_clear_audio_buffer();
      mic_reinit();
      mic_disabled = false;
      Serial.println("[AUDIO] Speaker reinit failed -> Mic restored");
    }
  }

  protocol_audio_process();               // Ring buffer -> speaker playback processing

  bool is_playing = protocol_is_audio_playing();
  bool has_buffered_audio = protocol_has_audio_buffered();
  bool cooldown_done = (millis() - last_play_end_ms) >= MIC_REENABLE_COOLDOWN_MS;
  if (!is_playing && !has_buffered_audio && cooldown_done && mic_disabled) {
    // TTS playback completed -> re-enable mic
    mic_reinit();
    mic_disabled = false;
    Serial.println("[AUDIO] Mic begin (after TTS)");
  }

  // -- Voice input processing (only when mic is enabled and TTS is not playing) --
  if (!mic_disabled && !is_playing && !has_buffered_audio && cooldown_done) {
    static int16_t frame_buf[AUDIO_FRAME_SIZE];  // 20ms frame buffer (static: saves stack)

    if (M5.Mic.record(frame_buf, AUDIO_FRAME_SIZE, AUDIO_SAMPLE_RATE)) {
      // Calculate loudness (RMS) of current frame
      float rms = frame_rms(frame_buf, AUDIO_FRAME_SIZE);

      // If not speaking yet, accumulate in pre-roll buffer
      // (At VAD_START, send this buffer first to preserve speech onset)
      if (!vad_state.talking) {
        preroll_push(&preroll, frame_buf, AUDIO_FRAME_SIZE);
      }

      // Update VAD state -> send packets based on event
      VadEvent event = vad_update(&vad_state, rms, AUDIO_FRAME_SIZE, AUDIO_SAMPLE_RATE);

      if (event == VAD_START) {
        // Speech start detected -> LED green + START packet + pre-roll send
        led_set_color(LED_COLOR_RECORDING_R, LED_COLOR_RECORDING_G, LED_COLOR_RECORDING_B);
        if (protocol_send_packet(*transport, PTYPE_START, nullptr, 0)) {
          preroll_send(&preroll, *transport);
        }
      } else if (event == VAD_CONTINUE) {
        // During speech -> send current frame as AUDIO packet
        protocol_send_packet(*transport, PTYPE_AUDIO, (uint8_t*)frame_buf, AUDIO_FRAME_SIZE * sizeof(int16_t));
      } else if (event == VAD_END) {
        // Speech end -> END packet + LED blue (idle)
        protocol_send_packet(*transport, PTYPE_END, nullptr, 0);
        led_set_color(LED_COLOR_IDLE_R, LED_COLOR_IDLE_G, LED_COLOR_IDLE_B);
      }
    }
  }

  // -- Peripheral updates --
  led_update_pattern();  // LED animation pattern
  display_update();      // OLED face animation + blink (currently placeholder)
  servo_update();        // Servo async action (rotate/wiggle) step processing
  delay(1);              // Feed watchdog + yield CPU
}
