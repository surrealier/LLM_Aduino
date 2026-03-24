#ifndef ROBOT_BRIDGE_H
#define ROBOT_BRIDGE_H

#include <Arduino.h>

void robot_bridge_init();
void robot_bridge_update();
bool robot_bridge_ready();
bool robot_bridge_forward_json(const char* json);

#endif
