#include "lcdlog.h"
#include <string.h>
#include <stdio.h>

LcdLog LcdLog::lcdLog;

void LcdLog::logString(const char *p, u16 color)
{
    unsigned int logline = current_line % 10;
    char buf[chars_per_line + 1];

    sprintf(buf, "%1u ", logline);
    strncpy(buf + 2, (char *) p, 18);
    buf[20] = 0;

    showLine(buf, color);
}

void LcdLog::logBytes(const u8 *p, size_t n, u16 color)
{
    unsigned int logline = current_line % 10;
    char buf[chars_per_line + 1];
    char *cp = buf;
    size_t i;

    cp += sprintf(cp, "%1u ", logline);

    for (i = 0;
            i < n
                && static_cast<unsigned int>((cp + 2) - buf) < chars_per_line;
            i += 1) {
        cp += sprintf(cp, "%02x", *(p + i));
    }

    showLine(buf, color);
}

void LcdLog::showLine(const char *p, u16 color)
{
    LCD_ShowString(0, 16 * (current_line % lines), (const u8 *) p, color);
    current_line += 1;
}