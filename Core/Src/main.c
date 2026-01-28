/**
  * @file           : main.c
  * @brief          : Main program body
  */
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void delay(volatile uint32_t cnt);
static void delay(volatile uint32_t cnt)
{
    while (cnt > 0) { cnt--; }
}

static void usart2_write_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t out[2];
    out[0] = (uint8_t)hex[(v >> 4) & 0xF];
    out[1] = (uint8_t)hex[(v >> 0) & 0xF];
    usart2_write_string(out, 2);
}

int main(void)
{
    // SystemClock_Config();

    // LED (PA5)
    gpio_enable_clock(GPIOA);
    gpio_config_output(GPIOA, 5, GPIO_OUTPUT_PP, GPIO_PULL_NONE, GPIO_SPEED_MEDIUM);

    // UART
    usart2_init(115200, true, true);

    i2c1_init(16000000u, 100000u);

    const uint8_t imu_addr7 = 0x68; 
    uint8_t rx[MAX_LENGTH];

    while (1)
    {
        gpio_set(GPIOA, 5);

        // Clear RX buffer
        for (size_t i = 0; i < sizeof(rx); i++) rx[i] = 0;

        // Wait for a command over UART
        usart2_read_string(rx);

        // Echo back
        size_t n = 0;
        while (n < sizeof(rx) && rx[n] != 0) n++;
        usart2_write_string(rx, n);

        // Send i2c
        bool do_probe = false;
        if (n >= 3 && rx[0] == 'i' && rx[1] == '2' && rx[2] == 'c') do_probe = true;

        if (do_probe)
        {
            uint32_t err = 0;
            uint8_t out[6];
            //bool ok = i2c1_probe(imu_addr7, &err);
            uint8_t data = 0x3B;
            uint8_t wake[2] = { 0x6B, 0x00 };
            i2c1_write(imu_addr7, wake, 2, &err, true);
                        
            /*
            bool okW = i2c1_write(imu_addr7, &data, 1, &err);
            if (!okW)
            {
                const char failure[] = "\r\nOK_WRITE FAILURE\r\n";
                usart2_write_string((const uint8_t *)failure, sizeof(failure) - 1);
                usart2_write_hex8(err);
                while (1) {}
            }
            //bool ok = i2c1_who_am_i(imu_addr7, &out);
            bool ok = i2c1_read(imu_addr7, &out, 1, &err);
*/
            bool ok = i2c1_write_read(imu_addr7, &data, 1, out, 6, &err);
            const char prefix[] = "\r\nMPU6050 Accel: ";
            usart2_write_string((const uint8_t*)prefix, sizeof(prefix) - 1);
            usart2_write_hex8((uint8_t)(imu_addr7 << 1)); // show 8-bit address

            if (ok)
            {
                //const char okmsg[] = " : ACK\r\n";
                //usart2_write_string((const uint8_t*)okmsg, sizeof(okmsg) - 1);
                const char outmsg[] = "ACCEL: \r\n";
                const char space[] = " ";
                usart2_write_string((const uint8_t*)outmsg, sizeof(outmsg) - 1);
                //usart2_write_string(&out, 1);
                for (size_t i = 0; i < 6; i++)
                {
                    usart2_write_hex8(out[i]);
                    usart2_write_string((const uint8_t*)space, sizeof(space) - 1);
                }
                const char crlf[] = "\r\n";
                usart2_write_string((const uint8_t*)crlf, sizeof(crlf) - 1);
            }
            else
            {
                const char badmsg[] = " : NACK  err=";
                usart2_write_string((const uint8_t*)badmsg, sizeof(badmsg) - 1);
                const char crlf[] = "\r\n";
                usart2_write_string((const uint8_t*)crlf, sizeof(crlf) - 1);
            }
        }
        else
        {
            const char help[] =
                "\r\nType 'probe' or 'i2c'\r\n";
            usart2_write_string((const uint8_t*)help, sizeof(help) - 1);
        }

        delay(500000);
        gpio_clear(GPIOA, 5);
        delay(500000000);
    }
}

void SystemClock_Config(void)
{
}

void Error_Handler(void)
{
    while (1) { }
}

#ifdef USE_FULL_ASSERT
#endif

