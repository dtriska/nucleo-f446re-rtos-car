#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f446xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Common errors:
 *  - BERR: Bus error
 *  - ARLO: Arbitration lost
 *  - AF: Acknowledge failure (NACK)
 *  - OVR: Overrun/Underrun
 *  - TIMEOUT: Timeout/Tlow error
 */
#define I2C_ERR_MASK (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT)

/* --------- Init / basic control --------- */

/* Initialize I2C1 as master.
 * pclk1_hz: APB1 clock frequency feeding I2C1 (16000000)
 * i2c_hz:   desired bus speed (100000 or 400000)
 */
void i2c1_init(uint32_t pclk1_hz, uint32_t i2c_hz);

bool i2c1_who_am_i(uint8_t addr7, uint8_t* out);

bool i2c1_probe(uint8_t addr7, uint32_t* err_flags);

/* Write raw bytes */
bool i2c1_write(uint8_t addr7,
                const uint8_t* data,
                size_t len,
                uint32_t* err_flags,
                bool send_stop);

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
                     uint32_t* err_flags);

/* Write one 8-bit register */
bool i2c1_write_reg8(uint8_t val,
                     uint32_t* err_flags);

/* Read one 8-bit register */
bool i2c1_read_reg8(uint8_t addr7,
                    uint8_t reg,
                    uint8_t* val,
                    uint32_t* err_flags);

/* Burst read starting at 8-bit register */
bool i2c1_read_reg_n(uint8_t addr7,
                     uint8_t reg,
                     uint8_t* buf,
                     size_t n,
                     uint32_t* err_flags);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */

