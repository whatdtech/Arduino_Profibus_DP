#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#define SSD1306_I2C_ADDR         0x78 // 0x3C << 1

#define SSD1306_WIDTH            128
#define SSD1306_HEIGHT           64

void SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_Putc(char ch, uint8_t color);
void SSD1306_Puts(const char* str, uint8_t color);

#endif /* SSD1306_H */
