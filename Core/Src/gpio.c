#include "gpio.h"
#include <stdint.h>

void gpio_enable_clock(GPIO_TypeDef* port)
{
    if (port == NULL) 
    {
        return;
    }

    if (port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (port == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else if (port == GPIOE) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    } else if (port == GPIOF) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    } else if (port == GPIOG) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    } else if (port == GPIOH) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    } else {
        /* Unknown/unsupported port pointer */
        return;
    } 
}

void gpio_config_output(GPIO_TypeDef* port,
                        uint8_t pin,
                        gpio_output_type_t type,
                        gpio_pull_t pull,
                        gpio_speed_t speed)
{
    /* Safety Check */
    if (pin > 15u) {
        return;
    }

    const uint32_t shift2 = (uint32_t)pin * 2u;
    const uint32_t mask2  = (0x3u << shift2);   /* 2-bit field mask */
    const uint32_t mask1  = (1u << pin);        /* 1-bit field mask */

    /* MODER: set to general purpose output (01) */
    port->MODER = (port->MODER & ~mask2) | (0x1u << shift2);

    /* OTYPER: 0 = push-pull, 1 = open-drain */
    if (type == GPIO_OUTPUT_OD) {
        port->OTYPER |= mask1;
    } else {
        port->OTYPER &= ~mask1;
    }

    /* OSPEEDR: 2-bit field encoding matches gpio_speed_t (0..3) */
    port->OSPEEDR = (port->OSPEEDR & ~mask2) | (((uint32_t)speed & 0x3u) << shift2);

    /* PUPDR: 2-bit field encoding matches gpio_pull_t (0..2) */
    port->PUPDR = (port->PUPDR & ~mask2) | (((uint32_t)pull & 0x3u) << shift2);
}

void gpio_set(GPIO_TypeDef* port, uint8_t pin)
{
   if (pin > 15U)
   {
       return;
   }
    
    const uint32_t mask1  = (1u << pin);
    
    /* BSRR: set to 1 */
    port->BSRR = mask1;
}

void gpio_clear(GPIO_TypeDef* port, uint8_t pin)
{
   if (pin > 15U)
   {
       return;
   }
    
    const uint32_t shift  = pin + 16;
    const uint32_t mask1  = (1u << shift);
    
    /* BSRR: reset pin set to 1 */
    port->BSRR = mask1;
}

void gpio_write(GPIO_TypeDef* port, uint8_t pin, bool value)
{

}

bool gpio_read(GPIO_TypeDef* port, uint8_t pin)
{
    return false;
}

