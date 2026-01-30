/**
  * @file    main.c
  */

#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "i2c_int.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---------- local helpers ---------- */

static void delay(volatile uint32_t cnt)
{
    while (cnt--) { __asm volatile ("nop"); }
}

static void usart2_write_cstr(const char *s)
{
    const uint8_t *p = (const uint8_t *)s;
    size_t n = 0;
    while (p[n] != 0) n++;
    usart2_write_string(p, n);
}

static void usart2_write_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t out[2];
    out[0] = (uint8_t)hex[(v >> 4) & 0xF];
    out[1] = (uint8_t)hex[(v >> 0) & 0xF];
    usart2_write_string(out, 2);
}

/* ---------- MPU6050 test config ---------- */

#define IMU_ADDR7        0x68u
#define UART_RX_MAX      64u

/* Shared state for async completion */
static volatile bool g_i2c_done = false;
static volatile i2c_status_t g_i2c_status = I2C_STATUS_ERR;
static volatile uint32_t g_i2c_err = 0;

static uint8_t g_rx6[6] = {0};
static uint8_t g_wake2[2] = { 0x6B, 0x00 }; /* PWR_MGMT_1 <- 0 */
static uint8_t g_reg_accel = 0x3B;         /* ACCEL_XOUT_H */

static void i2c_done_cb(i2c_status_t status, uint32_t err_flags, void *user)
{
    (void)user;
    g_i2c_status = status;
    g_i2c_err = err_flags;
    g_i2c_done = true;
}

static bool streq_cmd(const uint8_t *buf, const char *cmd)
{
    size_t i = 0;
    while (cmd[i] != 0)
    {
        if (buf[i] != (uint8_t)cmd[i]) return false;
        i++;
    }
    /* allow trailing CR/LF or 0 */
    return (buf[i] == 0 || buf[i] == '\r' || buf[i] == '\n');
}

/* ---------- main ---------- */

int main(void)
{
    // SystemClock_Config(); 

    /* LED (PA5) */
    gpio_enable_clock(GPIOA);
    gpio_config_output(GPIOA, 5, GPIO_OUTPUT_PP, GPIO_PULL_NONE, GPIO_SPEED_MEDIUM);

    /* UART */
    usart2_init(115200, true, true);
    usart2_write_cstr("\r\nBoot\r\nType: i2c\r\n");

    /* I2C IRQ driver */
    i2c1_init_irq(16000000u, 100000u);

    uint8_t rx[UART_RX_MAX] = {0};

    while (1)
    {
        gpio_set(GPIOA, 5);

        /* Clear RX buffer */
        for (size_t i = 0; i < UART_RX_MAX; i++) rx[i] = 0;

        /* Blocking read line */
        usart2_read_string(rx);

        /* Echo */
        {
            size_t n = 0;
            while (n < UART_RX_MAX && rx[n] != 0) n++;
            usart2_write_string(rx, n);
            usart2_write_cstr("\r\n");
        }

        if (streq_cmd(rx, "i2c") || streq_cmd(rx, "probe"))
        {
            /* 1) Wake MPU6050: write {0x6B,0x00} */
            {
                i2c_xfer_t x = {0};
                x.addr7  = IMU_ADDR7;
                x.kind   = I2C_XFER_TX;
                x.tx     = g_wake2;
                x.tx_len = sizeof(g_wake2);
                x.opts   = I2C_OPT_SEND_STOP;
                x.done   = i2c_done_cb;
                x.user   = NULL;

                g_i2c_done = false;
                if (!i2c1_submit(&x))
                {
                    usart2_write_cstr("i2c1_submit(wake) failed (busy/bad param)\r\n");
                    goto loop_end;
                }

                while (!g_i2c_done) { /* spin me round baby */ }

                if (g_i2c_status != I2C_STATUS_OK)
                {
                    usart2_write_cstr("WAKE ERR flags=0x");
                    usart2_write_hex8((uint8_t)((g_i2c_err >> 24) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >> 16) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >>  8) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >>  0) & 0xFF));
                    usart2_write_cstr("\r\n");
                    goto loop_end;
                }
            }

            delay(100000); 

            /* 2) Read accel 6 bytes: TXRX {0x3B} then read 6 bytes */
            {
                for (size_t i = 0; i < sizeof(g_rx6); i++) g_rx6[i] = 0;

                i2c_xfer_t x = {0};
                x.addr7  = IMU_ADDR7;
                x.kind   = I2C_XFER_TXRX;
                x.tx     = &g_reg_accel;
                x.tx_len = 1;
                x.rx     = g_rx6;
                x.rx_len = sizeof(g_rx6);
                x.opts   = I2C_OPT_SEND_STOP; /* fine to request STOP at end */
                x.done   = i2c_done_cb;
                x.user   = NULL;

                g_i2c_done = false;
                if (!i2c1_submit(&x))
                {
                    usart2_write_cstr("i2c1_submit(txrx) failed (busy/bad param)\r\n");
                    goto loop_end;
                }

                while (!g_i2c_done) { /* spinning... but in i2c */ }

                usart2_write_cstr("MPU6050 accel bytes: ");
                if (g_i2c_status == I2C_STATUS_OK)
                {
                    for (size_t i = 0; i < sizeof(g_rx6); i++)
                    {
                        usart2_write_hex8(g_rx6[i]);
                        usart2_write_cstr(" ");
                    }
                    usart2_write_cstr("\r\n");
                }
                else
                {
                    usart2_write_cstr("ERR flags=0x");
                    usart2_write_hex8((uint8_t)((g_i2c_err >> 24) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >> 16) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >>  8) & 0xFF));
                    usart2_write_hex8((uint8_t)((g_i2c_err >>  0) & 0xFF));
                    usart2_write_cstr("\r\n");
                }
            }
        }
        else
        {
            usart2_write_cstr("Commands: i2c | probe\r\n");
        }

loop_end:
        delay(200000);
        gpio_clear(GPIOA, 5);
        delay(200000);
    }
}

/* Stubs */
void SystemClock_Config(void) {}
void Error_Handler(void) { while (1) {} }

