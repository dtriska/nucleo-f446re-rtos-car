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

    usart2_init_tx(115200);
    const char msg[] = "LED\r\n";

    while (1)
    {
        gpio_set(GPIOA, 5);
        usart2_write_string(msg, 3);
        delay(500000);
        gpio_clear(GPIOA, 5);
        delay(500000);
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
