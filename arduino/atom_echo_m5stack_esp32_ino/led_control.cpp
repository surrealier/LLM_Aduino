// ============================================================
// led_control.cpp — LED 제어 구현
// ============================================================
// 역할: M5Unified API를 통해 Atom Echo의 SK6812 LED 제어.
//       FastLED 라이브러리를 제거하여 M5Unified와의 GPIO 충돌 방지.
//
// 색상 매핑:
//   happy   → 노랑 (255,200,0)   — 밝고 따뜻한 느낌
//   sad     → 파랑 (0,100,255)   — 차갑고 우울한 느낌
//   excited → 핑크 (255,50,200)  — 화려하고 활발한 느낌
//   sleepy  → 회보라 (100,100,150) — 어둡고 차분한 느낌
//   angry   → 빨강 (255,0,0)     — 강렬하고 격한 느낌
//   neutral → 연초록 (100,255,100) — 평온하고 안정적
// ============================================================

#include "led_control.h"
#include "config.h"
#include <string.h>

static uint8_t s_last_r = 0;
static uint8_t s_last_g = 0;
static uint8_t s_last_b = 0;

// led_init — LED 초기화
// M5Unified가 M5.begin()에서 LED를 자동 초기화하므로 추가 작업 불필요
void led_init() {
}

// led_set_color — RGB 색상을 LED에 적용
// Atom Echo의 내장 RGB LED는 M5Unified LED API로 제어한다.
// 디스플레이가 있는 보드/확장에 대해서만 화면 fill fallback을 유지한다.
void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
  s_last_r = r;
  s_last_g = g;
  s_last_b = b;
  if (M5.Led.isEnabled()) {
    M5.Led.setAllColor(r, g, b);
    return;
  }

  if (M5.getDisplayCount() > 0) {
    M5.Display.fillScreen(((uint16_t)(r & 0xF8) << 8) |
                          ((uint16_t)(g & 0xFC) << 3) |
                          ((uint16_t)(b >> 3)));
  }
}

void led_show_connecting() {
  led_set_color(LED_COLOR_CONNECTING_R, LED_COLOR_CONNECTING_G, LED_COLOR_CONNECTING_B);
}

void led_show_connected() {
  led_set_color(LED_COLOR_IDLE_R, LED_COLOR_IDLE_G, LED_COLOR_IDLE_B);
}

// led_show_emotion — 감정 문자열에 따른 LED 색상 설정
// null 또는 미지원 감정은 neutral(연초록)으로 표시
void led_show_emotion(const char* emotion) {
  if (!emotion) {
    led_show_connected();
    return;
  }

  if (strcmp(emotion, "happy") == 0)        led_set_color(255, 200, 0);
  else if (strcmp(emotion, "sad") == 0)     led_set_color(0, 100, 255);
  else if (strcmp(emotion, "excited") == 0) led_set_color(255, 50, 200);
  else if (strcmp(emotion, "sleepy") == 0)  led_set_color(100, 100, 150);
  else if (strcmp(emotion, "angry") == 0)   led_set_color(255, 0, 0);
  else                                      led_show_connected();
}

void led_get_last_color(uint8_t* r, uint8_t* g, uint8_t* b) {
  if (r) *r = s_last_r;
  if (g) *g = s_last_g;
  if (b) *b = s_last_b;
}

// led_update_pattern — 애니메이션 패턴 업데이트
// 현재 placeholder. 향후 감정별 점멸/페이드 패턴 구현 예정.
void led_update_pattern() {
}
