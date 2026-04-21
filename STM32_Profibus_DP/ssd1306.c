#include "ssd1306.h"
#include "i2c.h"
#include "font.h"

// Display buffer (128x64 pixels = 1024 bytes)
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Current cursor position
static uint8_t SSD1306_X = 0;
static uint8_t SSD1306_Y = 0;

static void SSD1306_WriteCommand(uint8_t cmd) {
    I2C1_WriteMem(SSD1306_I2C_ADDR, 0x00, &cmd, 1);
}

void SSD1306_Init(void) {
    // Init sequence
    SSD1306_WriteCommand(0xAE); // Display OFF
    SSD1306_WriteCommand(0x20); // Set Memory Addressing Mode
    SSD1306_WriteCommand(0x00); // Horizontal Addressing Mode
    SSD1306_WriteCommand(0xB0); // Set Page Start Address
    SSD1306_WriteCommand(0xC8); // Set COM Output Scan Direction
    SSD1306_WriteCommand(0x00); // Set low column address
    SSD1306_WriteCommand(0x10); // Set high column address
    SSD1306_WriteCommand(0x40); // Set start line address
    SSD1306_WriteCommand(0x81); // Set contrast control register
    SSD1306_WriteCommand(0xFF);
    SSD1306_WriteCommand(0xA1); // Set segment re-map 0 to 127
    SSD1306_WriteCommand(0xA6); // Set normal display
    SSD1306_WriteCommand(0xA8); // Set multiplex ratio
    SSD1306_WriteCommand(0x3F); // 64
    SSD1306_WriteCommand(0xA4); // Output follows RAM content
    SSD1306_WriteCommand(0xD3); // Set display offset
    SSD1306_WriteCommand(0x00); // Not offset
    SSD1306_WriteCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    SSD1306_WriteCommand(0xF0); // Set divide ratio
    SSD1306_WriteCommand(0xD9); // Set pre-charge period
    SSD1306_WriteCommand(0x22);
    SSD1306_WriteCommand(0xDA); // Set com pins hardware configuration
    SSD1306_WriteCommand(0x12);
    SSD1306_WriteCommand(0xDB); // Set vcomh
    SSD1306_WriteCommand(0x20); // 0.77xVcc
    SSD1306_WriteCommand(0x8D); // Enable charge pump regulator
    SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(0xAF); // Display ON

    SSD1306_Fill(0);
    SSD1306_UpdateScreen();
}

void SSD1306_Fill(uint8_t color) {
    uint32_t i;
    uint8_t fill = (color == 0) ? 0x00 : 0xFF;
    for(i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = fill;
    }
}

void SSD1306_UpdateScreen(void) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        SSD1306_WriteCommand(0xB0 + i);
        SSD1306_WriteCommand(0x00);
        SSD1306_WriteCommand(0x10);
        I2C1_WriteMem(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    if(color) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_GotoXY(uint8_t x, uint8_t y) {
    SSD1306_X = x;
    SSD1306_Y = y;
}

void SSD1306_Putc(char ch, uint8_t color) {
    if(ch < 32 || ch > 127) {
        ch = '?';
    }
    
    // Check if enough space on current line, if not wrap to next line
    if(SSD1306_WIDTH - SSD1306_X < 6) {
        SSD1306_X = 0;
        SSD1306_Y += 10; // Font height + padding
    }
    
    // Draw character 5x7 plus 1 column spacing
    for(uint8_t i = 0; i < 5; i++) {
        uint8_t line = Font5x7[ch - 32][i];
        for(uint8_t j = 0; j < 7; j++) {
            if(line & 0x01) {
                SSD1306_DrawPixel(SSD1306_X + i, SSD1306_Y + j, color);
            } else {
                SSD1306_DrawPixel(SSD1306_X + i, SSD1306_Y + j, !color);
            }
            line >>= 1;
        }
    }
    
    // 1 column spacing
    for(uint8_t j = 0; j < 7; j++) {
        SSD1306_DrawPixel(SSD1306_X + 5, SSD1306_Y + j, !color);
    }
    
    SSD1306_X += 6;
}

void SSD1306_Puts(const char* str, uint8_t color) {
    while(*str) {
        SSD1306_Putc(*str, color);
        str++;
    }
}
