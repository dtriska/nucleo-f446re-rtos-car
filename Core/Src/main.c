#include "spi.h"
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdbool.h>

#define TFT_CS_PORT          GPIOB
#define TFT_CS_PIN           6      // D10

#define TFT_DC_PORT          GPIOA
#define TFT_DC_PIN           8      // D7

#define TFT_RST_PORT         GPIOB
#define TFT_RST_PIN          10     // D8

/* ---------- Helpers ---------- */
static void delay_cycles(volatile uint32_t c) { while (c--) __NOP(); }
static void delay_ms(uint32_t ms)
{
    // crude delay for ~16MHz core
    while (ms--) delay_cycles(16000);
}

static inline void pin_high(GPIO_TypeDef* port, uint32_t pin) { port->BSRR = (1u << pin); }
static inline void pin_low (GPIO_TypeDef* port, uint32_t pin) { port->BSRR = (1u << (pin + 16u)); }

static inline void tft_select(void)   { pin_low (TFT_CS_PORT, TFT_CS_PIN); }
static inline void tft_deselect(void) { pin_high(TFT_CS_PORT, TFT_CS_PIN); }

static void tft_cmd(uint8_t c)
{
    pin_low(TFT_DC_PORT, TFT_DC_PIN);
    tft_select();
    uint32_t err = 0;
    spi1_write(&c, 1, &err);
    tft_deselect();
}

static void tft_data(const uint8_t* d, uint32_t n)
{
    pin_high(TFT_DC_PORT, TFT_DC_PIN);
    tft_select();
    uint32_t err = 0;
    spi1_write(d, n, &err);
    tft_deselect();
}

static void tft_data8(uint8_t v)
{
    pin_high(TFT_DC_PORT, TFT_DC_PIN);
    tft_select();
    uint32_t err = 0;
    spi1_write(&v, 1, &err);
    tft_deselect();
}

static void tft_reset(void)
{
    pin_low(TFT_RST_PORT, TFT_RST_PIN);
    delay_ms(20);
    pin_high(TFT_RST_PORT, TFT_RST_PIN);
    delay_ms(120);
}

static void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t buf[4];

    tft_cmd(0x2A); // CASET
    buf[0] = (x0 >> 8) & 0xFF; buf[1] = x0 & 0xFF;
    buf[2] = (x1 >> 8) & 0xFF; buf[3] = x1 & 0xFF;
    tft_data(buf, 4);

    tft_cmd(0x2B); // RASET
    buf[0] = (y0 >> 8) & 0xFF; buf[1] = y0 & 0xFF;
    buf[2] = (y1 >> 8) & 0xFF; buf[3] = y1 & 0xFF;
    tft_data(buf, 4);

    tft_cmd(0x2C); // RAMWR
}

static void tft_fill_color_565(uint16_t color, uint16_t w, uint16_t h)
{
    tft_set_addr_window(0, 0, (uint16_t)(w - 1), (uint16_t)(h - 1));

    pin_high(TFT_DC_PORT, TFT_DC_PIN);
    tft_select();

    uint8_t hi = (color >> 8) & 0xFF;
    uint8_t lo = color & 0xFF;
    uint32_t err = 0;
    uint32_t px = (uint32_t)w * (uint32_t)h;
    while (px--)
    {
        spi1_write(&hi, 1, &err);
        spi1_write(&lo, 1, &err);
    }

    tft_deselect();
}

static void tft_min_init(void)
{
    tft_cmd(0x01); // SWRESET
    delay_ms(150);

    tft_cmd(0x11); // SLPOUT
    delay_ms(150);

    tft_cmd(0x3A); // COLMOD
    tft_data8(0x05); // 16-bit color
    delay_ms(10);

    tft_cmd(0x36); // MADCTL
    tft_data8(0x00); // basic orientation

    tft_cmd(0x29); // DISPON
    delay_ms(100);
}

int main(void)
{
    spi1_init();

    // quick pin toggle sanity for el scope
    for (int i = 0; i < 2; i++) {
        pin_low(TFT_CS_PORT, TFT_CS_PIN); delay_ms(50);
        pin_high(TFT_CS_PORT, TFT_CS_PIN); delay_ms(50);
        pin_low(TFT_DC_PORT, TFT_DC_PIN); delay_ms(50);
        pin_high(TFT_DC_PORT, TFT_DC_PIN); delay_ms(50);
        pin_low(TFT_RST_PORT, TFT_RST_PIN); delay_ms(50);
        pin_high(TFT_RST_PORT, TFT_RST_PIN); delay_ms(50);
    }

    tft_reset();
    tft_min_init();

    // ST7735 modules are 128x160
    const uint16_t W = 128, H = 160;

    while (1)
    {
        tft_fill_color_565(0xF800, W, H); delay_ms(500); // red
        tft_fill_color_565(0x07E0, W, H); delay_ms(500); // green
        tft_fill_color_565(0x001F, W, H); delay_ms(500); // blue
        tft_fill_color_565(0xFFFF, W, H); delay_ms(500); // white
        tft_fill_color_565(0x0000, W, H); delay_ms(500); // black
    }
}

