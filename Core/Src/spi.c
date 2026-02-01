#include "spi.h"

static inline void pin_high(GPIO_TypeDef* port, uint32_t pin) { port->BSRR = (1u << pin); }
static inline void pin_low (GPIO_TypeDef* port, uint32_t pin) { port->BSRR = (1u << (pin + 16u)); }

static void spi1_gpio_init()
{
    // ---------- SPI1 SCK: PA5 (AF5) ----------
    GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER5_1;                 // AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5_Msk;               // push-pull
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD5_Msk;              // no pull
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5_Msk;
    GPIOA->AFR[0] |= (0x5u << GPIO_AFRL_AFSEL5_Pos);     // AF5
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED5_Pos);// very high speed

    // ---------- SPI1 MOSI: PA7 (AF5) ----------
    GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER7_1;                 // AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7_Msk;               // push-pull
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD7_Msk;              // no pull
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
    GPIOA->AFR[0] |= (0x5u << GPIO_AFRL_AFSEL7_Pos);     // AF5
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk;
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED7_Pos);// very high speed

    // ---------- SPI1 MISO: PA6 (AF5) ----------
    GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER6_1;                 // AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6_Msk;               // push-pull
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD6_Msk;              // no pull (or pull-down if you prefer)
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
    GPIOA->AFR[0] |= (0x5u << GPIO_AFRL_AFSEL6_Pos);     // AF5
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk;
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED6_Pos);// very high speed

    // ---------- CS: PB6 (GPIO output), idle HIGH ----------
    GPIOB->MODER &= ~GPIO_MODER_MODER6_Msk;
    GPIOB->MODER |= GPIO_MODER_MODER6_0;                 // output
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT6_Msk;               // push-pull
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD6_Msk;              // no pull
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk;
    GPIOB->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED6_Pos);// very high speed
    pin_high(GPIOB, 6);                                  // CS inactive

    // ---------- D/C: PA8 (GPIO output), default HIGH (data) ----------
    GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER8_0;                 // output
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT8_Msk;               // push-pull
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD8_Msk;              // no pull
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8_Msk;
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED8_Pos);// very high speed
    pin_high(GPIOA, 8);

    // ---------- RST: PA9 (GPIO output), default HIGH (not in reset) ----------
    GPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER9_0;                 // output
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT9_Msk;               // push-pull
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD9_Msk;              // no pull
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9_Msk;
    GPIOA->OSPEEDR |= (0x3u << GPIO_OSPEEDR_OSPEED9_Pos);// very high speed
    pin_high(GPIOA, 9);
}

bool spi1_init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    spi1_gpio_init();

    // Disable SPI before config
    SPI1->CR1 &= ~SPI_CR1_SPE_Msk;

    // Mode 0: CPOL=0, CPHA=0
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    // 8-bit frames (DFF=0), MSB-first 
    SPI1->CR1 &= ~(SPI_CR1_DFF | SPI_CR1_LSBFIRST);

    // Master, software NSS management, internal NSS high
    SPI1->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI);

    // Baud rate prescaler
    SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);

    // Default CR2 (no interrupts, no DMA)
    SPI1->CR2 = 0;

    // Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;

    return true;
}

bool spi1_write(const uint8_t* data, size_t len, uint32_t* err_flags)
{
    (void)err_flags; // impl later 

    for (size_t i = 0; i < len; i++)
    {
        while (!(SPI1->SR & SPI_SR_TXE)) {}
        SPI1->DR = data[i];

        // Full-duplex: wait for receive to drain RXNE (prevents OVR)
        while (!(SPI1->SR & SPI_SR_RXNE)) {}
        (void)SPI1->DR;
    }

    // Wait for transfer fully complete
    while (SPI1->SR & SPI_SR_BSY) {}

    // Extra drain
    (void)SPI1->SR;
    (void)SPI1->DR;

    return true;
}

