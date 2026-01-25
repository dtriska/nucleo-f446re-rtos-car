#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f446xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_LENGTH 32
#define UART_RX_ERR_MASK (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)

void usart2_init(uint32_t baud, bool tx, bool rx);

void usart2_init_tx();

void usart2_init_rx();

bool usart2_write_byte(uint8_t b);

bool usart2_write_string(const uint8_t* s, size_t len);

void usart2_read_byte(uint8_t* out, uint32_t* err_flags);

bool usart2_read_string(uint8_t* c);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */

