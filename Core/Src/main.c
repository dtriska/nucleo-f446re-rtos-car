/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */
#include "stm32f446xx.h"
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

static void GPIOA_Config(void);
static void GPIOA_Config(void)
{
    // Enable GPIOA Periph clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set Mode to GPIO Output for PA5
    GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER5_0;

    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5_Msk;

    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_1; 

    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5_Msk;
}

int main(void)
{
 // SystemClock_Config();
    GPIOA_Config();

    while (1)
    {
        GPIOA->BSRR = (1UL << GPIO_BSRR_BS5_Pos);
        delay(500000);
        GPIOA->BSRR = (1UL << GPIO_BSRR_BR5_Pos);
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
