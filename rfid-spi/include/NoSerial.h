#ifndef NOSERIAL_H
#define NOSERIAL_H

#include <Arduino.h>
#include "gd32vf103_libopt.h"

class Serial_ : public Stream
{
public:
  Serial_() {}

  virtual int available() { return 0; }
  virtual int read() { return 0; }
  virtual int peek() { return 0; }
  virtual void flush() {}

  virtual size_t write(uint8_t c) { return fwrite(&c, 1, 1, stdout); }
};

extern Serial_ Serial;

#endif // NOSERIAL_H