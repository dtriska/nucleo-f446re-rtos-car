/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */
#include "gpio.h"
#include "uart.h"
#include <stdint.h>
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void delay(volatile uint32_t cnt);
static void delay(volatile uint32_t cnt)
{
    while (cnt > 0)
    {
        cnt--;
    }

}

int main(void)
{
 // SystemClock_Config();
    gpio_enable_clock(GPIOA);
    gpio_config_output(GPIOA, 5, GPIO_OUTPUT_PP, GPIO_PULL_NONE, GPIO_SPEED_MEDIUM);

    usart2_init(115200, true, true);
    uint8_t rx[MAX_LENGTH];
//    const char msg[] = "LED\r\n";

    while (1)
    {
        gpio_set(GPIOA, 5);
        for (size_t i = 0; i < sizeof(rx); i++) rx[i] = 0;
        usart2_read_string(rx);
        size_t n = 0;
        while (n < sizeof(rx) && rx[n] != 0) n++;
        usart2_write_string(rx, n);
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
  while (1)
  {

  }
}

#ifdef USE_FULL_ASSERT
#endif 
