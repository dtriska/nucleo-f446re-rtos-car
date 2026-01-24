/* gpio.h - minimal GPIO driver interface for STM32F446 (CMSIS register-level)
 *
 * Design choices:
 *  - pin is an index 0..15 (NOT a bitmask)
 *  - enums map 1:1 to the hardware encoding where possible
 *  - BSRR is used for atomic set/clear
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "stm32f446xx.h"  // for GPIO_TypeDef, RCC, GPIOA... definitions

#ifdef __cplusplus
extern "C" {
#endif

/* Output type maps to OTYPER bit: 0 = push-pull, 1 = open-drain */
typedef enum {
    GPIO_OUTPUT_PP = 0u,
    GPIO_OUTPUT_OD = 1u,
} gpio_output_type_t;

/* Pull maps to PUPDR encoding: 00 none, 01 pull-up, 10 pull-down */
typedef enum {
    GPIO_PULL_NONE = 0u,
    GPIO_PULL_UP   = 1u,
    GPIO_PULL_DOWN = 2u,
} gpio_pull_t;

/* Speed maps to OSPEEDR encoding: 00 low, 01 medium, 10 high, 11 very high */
typedef enum {
    GPIO_SPEED_LOW       = 0u,
    GPIO_SPEED_MEDIUM    = 1u,
    GPIO_SPEED_HIGH      = 2u,
    GPIO_SPEED_VERY_HIGH = 3u,
} gpio_speed_t;

/* Enables the AHB1 peripheral clock for the given GPIO port (GPIOA..GPIOH).
 * Precondition: port is a valid GPIOx base (GPIOA, GPIOB, ...).
 */
void gpio_enable_clock(GPIO_TypeDef* port);

/* Configures a single pin as a general-purpose output.
 * pin: 0..15
 * type: push-pull or open-drain
 * pull: none/up/down
 * speed: low/medium/high/very-high
 *
 * Precondition: gpio_enable_clock(port) has been called.
 */
void gpio_config_output(GPIO_TypeDef* port,
                        uint8_t pin,
                        gpio_output_type_t type,
                        gpio_pull_t pull,
                        gpio_speed_t speed);

/* Atomic output control using BSRR.
 * Precondition: pin configured as output.
 */
void gpio_set(GPIO_TypeDef* port, uint8_t pin);
void gpio_clear(GPIO_TypeDef* port, uint8_t pin);

void gpio_write(GPIO_TypeDef* port, uint8_t pin, bool value);
bool gpio_read(GPIO_TypeDef* port, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */

