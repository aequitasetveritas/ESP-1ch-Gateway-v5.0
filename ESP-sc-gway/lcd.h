#ifndef LCD_H
#define LCD_H

#include <stdint.h>

void lcd_init(void);
void lcd_update(uint32_t rx, uint32_t ok, uint32_t f, uint8_t sf);
void lcd_line3(const char * line);
#endif