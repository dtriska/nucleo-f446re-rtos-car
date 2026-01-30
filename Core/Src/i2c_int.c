#include "i2c_int.h"
#include "stm32f446xx.h"

static i2c1_ctx_t g_i2c1;

static inline void i2c1_request_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static inline void i2c1_clear_addr(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static void i2c1_finish_ok(void)
{
    if (g_i2c1.xfer.done)
    {
        g_i2c1.xfer.done(I2C_STATUS_OK, g_i2c1.err_flags, g_i2c1.xfer.user);
    }
    g_i2c1.st = I2C_ST_IDLE;
}


static inline void i2c1_set_ack(bool on)
{
    if (on) I2C1->CR1 |= I2C_CR1_ACK;
    else    I2C1->CR1 &= ~I2C_CR1_ACK;
}

static inline void i2c1_set_pos(bool on)
{
    if (on) I2C1->CR1 |= I2C_CR1_POS;
    else    I2C1->CR1 &= ~I2C_CR1_POS;
}

static void i2c1_finish_err(uint32_t flags)
{
    g_i2c1.err_flags |= flags;

    i2c1_request_stop();

    if (I2C1->SR1 & I2C_SR1_AF) I2C1->SR1 &= ~I2C_SR1_AF;

    i2c1_set_ack(true);
    i2c1_set_pos(false);

    if (g_i2c1.xfer.done)
        g_i2c1.xfer.done(I2C_STATUS_ERR, g_i2c1.err_flags, g_i2c1.xfer.user);

    if (I2C1->SR1 & I2C_SR1_ADDR) i2c1_clear_addr();

    g_i2c1.st = I2C_ST_IDLE;
}

/* Initialize driver context + NVIC + enable I2C interrupts (EV/ER) */
void i2c1_init_irq(uint32_t pclk1_hz, uint32_t i2c_hz)
{
    g_i2c1.st = I2C_ST_IDLE;
    g_i2c1.err_flags = 0;
    i2c1_init(pclk1_hz, i2c_hz);
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;
    NVIC_SetPriority(I2C1_EV_IRQn, 12);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 13);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* Submit an async transfer */
bool i2c1_submit(const i2c_xfer_t *xfer)
{
    if (!xfer) return false;
    if (xfer->addr7 > 0x7F) return false;

    if (xfer->tx_len && !xfer->tx) { g_i2c1.err_flags |= I2C_ERR_SW_BAD_PARAM; return false; }
    if (xfer->rx_len && !xfer->rx) { g_i2c1.err_flags |= I2C_ERR_SW_BAD_PARAM; return false; }

    switch (xfer->kind) 
    {
      case I2C_XFER_TX:   if (!xfer->tx_len) return false; break;
      case I2C_XFER_RX:   if (!xfer->rx_len) return false; break;
      case I2C_XFER_TXRX: if (!xfer->tx_len || !xfer->rx_len) return false; break;
      default: return false;
    }

    if (g_i2c1.st != I2C_ST_IDLE) return false;
    if (xfer->tx_len == 0 && xfer->rx_len == 0) return false;
    g_i2c1.xfer = *xfer;
    g_i2c1.tx_idx = 0;
    g_i2c1.rx_idx = 0;
    g_i2c1.rx_remaining = xfer->rx_len;
    g_i2c1.err_flags = 0;

    if (xfer->kind == I2C_XFER_RX) 
    {
        g_i2c1.phase_is_read = true;
    }
    else 
    {
        g_i2c1.phase_is_read = false;
    }

    g_i2c1.addr_byte = (xfer->addr7 << 1) | (g_i2c1.phase_is_read ? 1 : 0);

    if (I2C1->SR2 & I2C_SR2_BUSY) 
    { 
        g_i2c1.err_flags |= I2C_ERR_SW_BUSY; return false; 
    }
    
    i2c1_set_ack(true);
    i2c1_set_pos(false);


    g_i2c1.st = I2C_ST_START; 
    I2C1->CR1 |= I2C_CR1_START;
    return true;
}

void i2c1_abort(uint32_t reason_flags)
{
    g_i2c1.err_flags |= reason_flags;
    i2c1_request_stop();
    if (I2C1->SR1 & I2C_SR1_AF) I2C1->SR1 &= ~I2C_SR1_AF;
    if (I2C1->SR1 & I2C_SR1_ADDR) i2c1_clear_addr();
    i2c1_set_ack(true);
    i2c1_set_pos(false);
    g_i2c1.st = I2C_ST_IDLE;
}

bool i2c1_busy(void)
{
    return (g_i2c1.st != I2C_ST_IDLE);
}

void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    uint32_t errs = sr1 & I2C_ERR_MASK;
    if (!errs) return;

    g_i2c1.err_flags |= errs;

    // STOP to release bus
    i2c1_request_stop();

    // Clear AF specifically -> NACK
    if (errs & I2C_SR1_AF) I2C1->SR1 &= ~I2C_SR1_AF;

    i2c1_finish_err(0); // err_flags already has HW bits
}

void I2C1_EV_IRQHandler(void)
{
    // If nothing active -> just drain flags 
    if (g_i2c1.st == I2C_ST_IDLE || g_i2c1.st == I2C_ST_ERROR)
    {
        // Clear ADDR if sticking
        if (I2C1->SR1 & I2C_SR1_ADDR) i2c1_clear_addr();
        return;
    }

    const uint32_t sr1 = I2C1->SR1;

    // ---- 1) START bit (SB) handling ----
    if (sr1 & I2C_SR1_SB)
    {
        // Decide which address byte to send based on current phase
        // (addr_byte already set in submit; update it for repeated-start read phase)
        if (g_i2c1.st == I2C_ST_RESTART)
        {
            g_i2c1.phase_is_read = true;
            g_i2c1.addr_byte = (uint8_t)((g_i2c1.xfer.addr7 << 1u) | 1u);
            g_i2c1.st = I2C_ST_START;
        }

        // Writing DR clears SB (after SR1 read)
        I2C1->DR = g_i2c1.addr_byte;

        // Wait for ADDR
        g_i2c1.st = I2C_ST_ADDR;
        return; // IRQ handles ADDR -> Leave now
    }

    // ---- 2) Address sent/matched (ADDR) handling ----
    if (sr1 & I2C_SR1_ADDR)
    {
        // Receiver mode if phase_is_read -> else tx
        if (g_i2c1.phase_is_read)
        {
            // Configure ACK/POS depending on rx length
            const size_t n = g_i2c1.xfer.rx_len;
            g_i2c1.rx_remaining = n;

            if (n == 0)
            {
                // Bad Day 
                i2c1_clear_addr();
                i2c1_finish_err(I2C_ERR_SW_BAD_PARAM);
                return;
            }
            else if (n == 1)
            {
                // 1 byte: ACK=0 before clearing ADDR, STOP immediately
                i2c1_set_pos(false);
                i2c1_set_ack(false);
                i2c1_clear_addr();
                i2c1_request_stop();
                g_i2c1.st = I2C_ST_RX_LAST1;
                return;
            }
            else if (n == 2)
            {
                // 2 bytes: POS=1, ACK=0 before clearing ADDR; wait BTF
                i2c1_set_pos(true);
                i2c1_set_ack(false);
                i2c1_clear_addr();
                g_i2c1.st = I2C_ST_RX_LAST2;
                return;
            }
            else
            {
                // >=3 bytes: POS=0, ACK=1 for streaming
                i2c1_set_pos(false);
                i2c1_set_ack(true);
                i2c1_clear_addr();
                g_i2c1.st = I2C_ST_RX;
                return;
            }
        }
        else
        {
            // Transmitter: clear ADDR, then proceed to TX
            i2c1_set_pos(false);
            i2c1_set_ack(true);
            i2c1_clear_addr();
            g_i2c1.st = I2C_ST_TX;
        }
    }

    // Re-read SR1 for combined flag situations (TXE+BTF, RXNE+BTF)
    const uint32_t sr1b = I2C1->SR1;

    // ---- 3) TX path (TXE/BTF) ----
    if (g_i2c1.st == I2C_ST_TX)
    {
        // TXE: data register empty -> write next byte
        if ((sr1b & I2C_SR1_TXE) && (g_i2c1.tx_idx < g_i2c1.xfer.tx_len))
        {
            I2C1->DR = g_i2c1.xfer.tx[g_i2c1.tx_idx++];
            return;
        }

        // If all bytes written, wait for BTF to ensure last byte shifted out
        if ((g_i2c1.tx_idx >= g_i2c1.xfer.tx_len) && (sr1b & I2C_SR1_BTF))
        {
            if (g_i2c1.xfer.kind == I2C_XFER_TXRX)
            {
                // Repeated start into read phase
                g_i2c1.phase_is_read = true;
                g_i2c1.addr_byte = (uint8_t)((g_i2c1.xfer.addr7 << 1u) | 1u);
                g_i2c1.st = I2C_ST_RESTART;
                I2C1->CR1 |= I2C_CR1_START;
                return;
            }
            else
            {
                // TX only complete
                if (g_i2c1.xfer.opts & I2C_OPT_SEND_STOP)
                {
                    i2c1_request_stop();
                }
                g_i2c1.st = I2C_ST_DONE;
                i2c1_finish_ok();
                return;
            }
        }

        return;
    }

    // ---- 4) RX path ----
    if (g_i2c1.st == I2C_ST_RX_LAST1)
    {
        // Wait for RXNE then read single byte
        if (sr1b & I2C_SR1_RXNE)
        {
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;
            g_i2c1.st = I2C_ST_DONE;

            // restore ACK for next transaction
            i2c1_set_ack(true);
            i2c1_set_pos(false);

            i2c1_finish_ok();
        }
        return;
    }

    if (g_i2c1.st == I2C_ST_RX_LAST2)
    {
        // For 2 bytes, wait BTF then STOP then read DR twice
        if (sr1b & I2C_SR1_BTF)
        {
            i2c1_request_stop();
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;

            g_i2c1.st = I2C_ST_DONE;

            // restore defaults
            i2c1_set_ack(true);
            i2c1_set_pos(false);

            i2c1_finish_ok();
        }
        return;
    }

    if (g_i2c1.st == I2C_ST_RX)
    {
        // We stream until 3 bytes remain
        const size_t remaining = g_i2c1.xfer.rx_len - g_i2c1.rx_idx;

        if (remaining > 3)
        {
            if (sr1b & I2C_SR1_RXNE)
            {
                g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;
            }
            return;
        }

        // remaining == 3: switch to last-3 sequence (BTF-driven)
        g_i2c1.st = I2C_ST_RX_LAST3;
        // fall through on purpose
    }

    if (g_i2c1.st == I2C_ST_RX_LAST3)
    {
        // Wait BTF (one in DR one in shift)
        if (sr1b & I2C_SR1_BTF)
        {
            // Prepare to NACK final bytes
            i2c1_set_ack(false);

            // Read N-2
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;

            // Now we need another BTF for the last two bytes
            g_i2c1.st = I2C_ST_STOP;
        }
        return;
    }

    if (g_i2c1.st == I2C_ST_STOP)
    {
        const uint32_t sr1c = I2C1->SR1;
        if (sr1c & I2C_SR1_BTF)
        {
            // STOP, then drain last two bytes
            i2c1_request_stop();
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;
            g_i2c1.xfer.rx[g_i2c1.rx_idx++] = (uint8_t)I2C1->DR;

            // restore defaults
            i2c1_set_ack(true);
            i2c1_set_pos(false);

            g_i2c1.st = I2C_ST_DONE;
            i2c1_finish_ok();
        }
        return;
    }

}
