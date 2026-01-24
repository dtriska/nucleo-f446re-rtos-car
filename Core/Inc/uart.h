#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f446xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USART2 TX-only driver (polling)
 * TX pin: PA2 (AF7)
 */

void usart2_init_tx(uint32_t baud);

bool usart2_write_byte(uint8_t b);

bool usart2_write_string(const char* s, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */

