#include "uart.h"
#include "stm32f446xx.h"
#include <stdint.h>

static uint32_t usart_brr_oversample16(uint32_t pclk_hz, uint32_t baud)
{
    if (baud == 0u) return 0u;

    /* Compute USARTDIV * 16 */
    uint32_t usartdiv16 = (pclk_hz + (baud / 2u)) / baud;

    uint32_t mantissa = usartdiv16 / 16u;
    uint32_t fraction = usartdiv16 % 16u;

    /* Fraction must be 0..15 */
    if (fraction > 15u) fraction = 15u;

    return (mantissa << 4) | fraction;
}

void usart2_init_tx(uint32_t baud)
{ 
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    /* Config pin type */
    GPIOA->MODER    &= ~GPIO_MODER_MODER2_Msk;
    GPIOA->MODER    |= (0x2u << GPIO_MODER_MODER2_Pos);   
    GPIOA->AFR[0]   &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOA->AFR[0]   |= (0x7u << GPIO_AFRL_AFSEL2_Pos);
    GPIOA->OTYPER   &= ~GPIO_OTYPER_OT2_Msk;
    GPIOA->OSPEEDR  &= ~GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOA->OSPEEDR  |= (0x1u << GPIO_OSPEEDR_OSPEED2_Pos);
    GPIOA->PUPDR    &= ~GPIO_PUPDR_PUPD2_Msk;
    GPIOA->PUPDR    |= (0x1U << GPIO_PUPDR_PUPD2_Pos);

    USART2->CR1 &= ~USART_CR1_UE_Msk;

    USART2->CR1 &= ~USART_CR1_OVER8_Msk;
    uint32_t pclk1_hz = 16000000u;
    USART2->BRR = usart_brr_oversample16(pclk1_hz, baud);

    USART2->CR1 |= (0x1u << USART_CR1_TE_Pos);
    USART2->CR1 |= (0x1u << USART_CR1_UE_Pos);
}

bool usart2_write_byte(uint8_t b)
{
    while (!(USART2->SR & (0x1u << USART_SR_TXE_Pos))) {}

    USART2->DR = b;
    return true;
}

bool usart2_write_string(const char* s, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        usart2_write_byte(s[i]);
    }
    return true;
}
