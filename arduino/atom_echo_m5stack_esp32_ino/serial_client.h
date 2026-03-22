// ============================================================
// serial_client.h — USB 시리얼을 WiFiClient 호환 인터페이스로 래핑
// ============================================================
// Arduino의 Client 추상 클래스를 구현하여 protocol.cpp의
// send/recv 함수가 WiFiClient & SerialClient 모두에서 동작.
// ============================================================

#ifndef SERIAL_CLIENT_H
#define SERIAL_CLIENT_H

#include <Client.h>
#include <HardwareSerial.h>

class SerialClient : public Client {
public:
  explicit SerialClient(HardwareSerial& s) : _ser(s), _connected(false) {}

  void setConnected(bool v) { _connected = v; }
  void setNoDelay(bool) {}  // no-op (TCP 전용 옵션)

  // ── Client 인터페이스 구현 ──
  int connect(IPAddress, uint16_t) override { _connected = true; return 1; }
  int connect(const char*, uint16_t) override { _connected = true; return 1; }

  size_t write(uint8_t b) override { return _ser.write(b); }
  size_t write(const uint8_t* buf, size_t n) override { return _ser.write(buf, n); }

  int available() override { return _ser.available(); }
  int read() override { return _ser.read(); }
  int read(uint8_t* buf, size_t n) override { return (int)_ser.readBytes(buf, n); }
  int peek() override { return _ser.peek(); }
  void flush() override { _ser.flush(); }

  void stop() override { _connected = false; }
  uint8_t connected() override { return _connected ? 1 : 0; }
  operator bool() override { return _connected; }

private:
  HardwareSerial& _ser;
  bool _connected;
};

#endif  // SERIAL_CLIENT_H
