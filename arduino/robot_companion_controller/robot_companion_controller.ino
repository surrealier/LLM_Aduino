#include <Arduino.h>

#if defined(ESP32)
HardwareSerial BridgeSerial(1);
static constexpr int BRIDGE_RX_PIN = 16;
static constexpr int BRIDGE_TX_PIN = 17;
static constexpr uint32_t BRIDGE_BAUD = 115200;
#else
#error "robot_companion_controller currently targets ESP32-class companion boards."
#endif

static String incoming_line;
static String last_robot_state;

void setup() {
  Serial.begin(115200);
  BridgeSerial.begin(BRIDGE_BAUD, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);
  Serial.println("[robot-companion] bridge ready");
}

static void handle_robot_line(const String& line) {
  last_robot_state = line;
  Serial.print("[robot-companion] state: ");
  Serial.println(last_robot_state);
}

void loop() {
  while (BridgeSerial.available() > 0) {
    char c = static_cast<char>(BridgeSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (incoming_line.length() > 0) {
        handle_robot_line(incoming_line);
        incoming_line = "";
      }
      continue;
    }
    incoming_line += c;
  }
}
