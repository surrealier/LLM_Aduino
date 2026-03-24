#include "robot_bridge.h"
#include "config.h"

#if ROBOT_BRIDGE_ENABLED

#include <HardwareSerial.h>
#include <string.h>

static HardwareSerial robot_bridge_serial(ROBOT_BRIDGE_UART_PORT);
static bool robot_bridge_started = false;

void robot_bridge_init() {
  robot_bridge_serial.begin(
      ROBOT_BRIDGE_BAUD,
      SERIAL_8N1,
      ROBOT_BRIDGE_RX_PIN,
      ROBOT_BRIDGE_TX_PIN);
  robot_bridge_started = true;
}

void robot_bridge_update() {
  while (robot_bridge_serial.available() > 0) {
    (void)robot_bridge_serial.read();
  }
}

bool robot_bridge_ready() {
  return robot_bridge_started;
}

bool robot_bridge_forward_json(const char* json) {
  if (!robot_bridge_started || !json || !*json) return false;

  size_t wanted = strlen(json);
  size_t written = robot_bridge_serial.write((const uint8_t*)json, wanted);
  if (written != wanted) return false;
  return robot_bridge_serial.write('\n') == 1;
}

#else

void robot_bridge_init() {}

void robot_bridge_update() {}

bool robot_bridge_ready() {
  return false;
}

bool robot_bridge_forward_json(const char* json) {
  (void)json;
  return false;
}

#endif
