#pragma once
#if defined(USBCON) && defined(USBD_USE_CDC)

#include <Arduino.h>
#include <HardwareTimer.h>

/*
 * This is essentially a workaround for the arduino CDC implementation being
 * not very configurable, or tolerant of large burst transfers.
 *
 * Drains the USB CDC receive queue into a large ring buffer from a timer ISR,
 * decoupling KlipperCommander's read cadence from the USB packet arrival rate.
 *
 * The USB ISR fills CDC's ReceiveQueue.
 * This timer ISR fires at ~200µs, drains ReceiveQueue into _buf via
 * Serial.read(), which also calls CDC_resume_receive() to re-arm the USB
 * hardware immediately. loop() reads from _buf at leisure with no timing
 * constraints. loop() must run often enough to empty this, but doesn't need to
 * outrun USB.
 *
 * Threading model: single writer (_drain, timer ISR), single reader (loop).
 * _head written only in ISR; _tail written only in loop. Cortex-M 32-bit
 * aligned loads/stores are atomic, so volatile uint32_t is sufficient without a
 * mutex.
 */
class USBSerialStream : public Stream {
public:
  static constexpr uint32_t BUFFER_SIZE = 2048;

  USBSerialStream(Stream &usb_serial, TIM_TypeDef *tim_instance)
      : _usb(usb_serial), _timer(tim_instance) {}

  void begin(uint32_t interval_us = 200) {
    _head = 0;
    _tail = 0;
    _timer.setOverflow(interval_us, MICROSEC_FORMAT);
    _timer.attachInterrupt([this]() { _drain(); });
    // Priority must be lower (higher number) than the USB ISR (typically 5)
    // so the USB ISR can pre-empt this drain, never the reverse.
    _timer.setInterruptPriority(7, 0);
    _timer.resume();
  }

  int available() override {
    uint32_t h = _head, t = _tail;
    return (int)((h - t + BUFFER_SIZE) % BUFFER_SIZE);
  }

  int read() override {
    uint32_t t = _tail;
    if (_head == t)
      return -1;
    uint8_t b = _buf[t];
    _tail = (t + 1) % BUFFER_SIZE;
    return b;
  }

  int peek() override {
    uint32_t t = _tail;
    if (_head == t)
      return -1;
    return _buf[t];
  }

  size_t write(uint8_t b) override { return _usb.write(b); }

  size_t write(const uint8_t *buf, size_t size) override {
    return _usb.write(buf, size);
  }

  void flush() override { _usb.flush(); }

private:
  void _drain() {
    while (_usb.available()) {
      uint32_t next = (_head + 1) % BUFFER_SIZE;
      if (next == _tail)
        break; // ring full; USB will NAK until loop() consumes
      _buf[_head] = (uint8_t)_usb.read();
      _head = next;
    }
  }

  Stream &_usb;
  HardwareTimer _timer;
  uint8_t _buf[BUFFER_SIZE];
  volatile uint32_t _head = 0;
  volatile uint32_t _tail = 0;
};

#endif // USBCON && USBD_USE_CDC
