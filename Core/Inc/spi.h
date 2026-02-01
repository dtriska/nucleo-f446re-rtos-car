#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f446xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_ERR_MASK (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRCERR)

bool spi1_init();

bool spi1_write(const uint8_t* data, size_t len, uint32_t* err_flags);


#ifdef __cplusplus
}
#endif

#endif /* SPI1_H */

