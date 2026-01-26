#include "i2c.h"
#include "stm32f446xx.h"
#include <stdint.h>

void i2c1_init(uint32_t pclk1_hz, uint32_t i2c_hz)
{
    // Enable the Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;
    // Disable peripheral
    I2C1->CR1 &= ~I2C_CR1_PE_Msk;
    // Program periph clock in cr2 to generate correct timing
    // I2C1 PB8 & PB9 for SCL and SDA 
    // AF4
    GPIOB->MODER    &= ~GPIO_MODER_MODER8_Msk;
    GPIOB->MODER    |= (0x2u << GPIO_MODER_MODER8_Pos);
    GPIOB->AFR[1]   &= ~GPIO_AFRH_AFSEL8_Msk;
    GPIOB->AFR[1]   |= (0x4u << GPIO_AFRH_AFSEL8_Pos);
    GPIOB->OTYPER   &= ~GPIO_OTYPER_OT8_Msk;
    GPIOB->OTYPER   |= (0x1u << GPIO_OTYPER_OT8_Pos);
    GPIOB->OSPEEDR  &= ~GPIO_OSPEEDR_OSPEED8_Msk;
    GPIOB->OSPEEDR  |= (0x2u << GPIO_OSPEEDR_OSPEED8_Pos);
    GPIOB->PUPDR    &= ~GPIO_PUPDR_PUPD8_Msk;
    GPIOB->PUPDR    |= (0x1u << GPIO_PUPDR_PUPD8_Pos);

    GPIOB->MODER    &= ~GPIO_MODER_MODER9_Msk;
    GPIOB->MODER    |= (0x2u << GPIO_MODER_MODER9_Pos);
    GPIOB->AFR[1]   &= ~GPIO_AFRH_AFSEL9_Msk;
    GPIOB->AFR[1]   |= (0x4u << GPIO_AFRH_AFSEL9_Pos);
    GPIOB->OTYPER   &= ~GPIO_OTYPER_OT9_Msk;
    GPIOB->OTYPER   |= (0x1u << GPIO_OTYPER_OT9_Pos);
    GPIOB->OSPEEDR  &= ~GPIO_OSPEEDR_OSPEED9_Msk;
    GPIOB->OSPEEDR  |= (0x2u << GPIO_OSPEEDR_OSPEED9_Pos);
    GPIOB->PUPDR    &= ~GPIO_PUPDR_PUPD9_Msk;
    GPIOB->PUPDR    |= (0x1u << GPIO_PUPDR_PUPD9_Pos);

    uint32_t freq_mhz = pclk1_hz / 1000000u;
    I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ_Msk) | (freq_mhz << I2C_CR2_FREQ_Pos);

    uint32_t ccr = pclk1_hz / (2u * i2c_hz);
    if (ccr < 4u) ccr = 4u;
    I2C1->CCR = ccr;   
        
    I2C1->TRISE = freq_mhz + 1;   

    I2C1->CR1 |= I2C_CR1_PE;
}

bool i2c1_who_am_i(uint8_t addr7, uint8_t* out)
{
    uint32_t err;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}

    I2C1->DR  = (uint8_t)(addr7 << 1u);

    while(1)
    {
        uint32_t sr = I2C1->SR1;
        if (sr & I2C_ERR_MASK)
        {
            err = sr & I2C_ERR_MASK;
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->SR1 &= ~I2C_SR1_AF;
            return false;
        }

        if (sr & I2C_SR1_ADDR)
        {
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            break;
        }
    }

    while(!(I2C1->SR1 & I2C_SR1_TXE)) {}
    I2C1->DR = 0x75;
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {}

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}

    I2C1->DR  = (uint8_t)((addr7 << 1u) | 1u);
    while (1) 
    {
        uint32_t sr = I2C1->SR1;
        if (sr & I2C_ERR_MASK)
        {
            err = sr & I2C_ERR_MASK;
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->SR1 &= ~I2C_SR1_AF;
            return false;
        }
        if (sr & I2C_SR1_ADDR)
        {
            break;
        }
    }

    I2C1->CR1 &= ~I2C_CR1_ACK;
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;

    while(!(I2C1->SR1 & I2C_SR1_RXNE)) {}
    *out = I2C1->DR;

    return true;
}

/* Probe: generates START + address(W). Returns true if ACK. */
bool i2c1_probe(uint8_t addr7, uint32_t* err_flags)
{
    if (err_flags) *err_flags = 0;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}

    I2C1->DR  = (uint8_t)(addr7 << 1u);
    while (1) 
    {
        uint32_t sr = I2C1->SR1;

        if (sr & I2C_ERR_MASK)
        {
            *err_flags = sr & I2C_ERR_MASK;
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->SR1 &= ~I2C_SR1_AF;
            return false;
        }

        if (sr & I2C_SR1_ADDR)
        {
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            return true;
        }
    } 
}

/* Write raw bytes */
bool i2c1_write(uint8_t addr7,
                const uint8_t* data,
                size_t len,
                uint32_t* err_flags)
{

}

/* Read raw bytes */
bool i2c1_read(uint8_t addr7,
               uint8_t* data,
               size_t len,
               uint32_t* err_flags);

bool i2c1_write_read(uint8_t addr7,
                     const uint8_t* wdata,
                     size_t wlen,
                     uint8_t* rdata,
                     size_t rlen,
                     uint32_t* err_flags)
{

}

/* --------- Convenience helpers (typical sensors) --------- */

/* Write one 8-bit register */
bool i2c1_write_reg8(uint8_t addr7,
                     uint8_t reg,
                     uint8_t val,
                     uint32_t* err_flags)
{

}

/* Read one 8-bit register */
bool i2c1_read_reg8(uint8_t addr7,
                    uint8_t reg,
                    uint8_t* val,
                    uint32_t* err_flags)
{

}

/* Burst read starting at 8-bit register */
bool i2c1_read_reg_n(uint8_t addr7,
                     uint8_t reg,
                     uint8_t* buf,
                     size_t n,
                     uint32_t* err_flags)
{

}


