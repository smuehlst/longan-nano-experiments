#ifndef NOSERIAL_H
#define NOSERIAL_H

#include <Arduino.h>

class Serial_ : public Stream
{
public:
  Serial_() {}

  virtual int available() { return 0; }
  virtual int read() { return 0; }
  virtual int peek() { return 0; }
  virtual void flush() {}

  virtual size_t write(uint8_t) { return 0; }
};

static Serial_ Serial;

#endif // NOSERIAL_H