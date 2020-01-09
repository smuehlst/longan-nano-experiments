#ifndef LCDLOG_H
#define LCDLOG_H

extern "C" {
#include "lcd/lcd.h"
}

class LcdLog
{
public:
    LcdLog() : current_line(0) {}

    void logString(const char *p, u16 color = WHITE);
    void logBytes(const u8 *p, size_t n, u16 color = WHITE);

    static LcdLog lcdLog;
    
private:
    static constexpr unsigned int lines = 5;
    static constexpr unsigned int chars_per_line = 20;

    unsigned int current_line;

    void showLine(const char *p, u16 color = WHITE);
};



#endif