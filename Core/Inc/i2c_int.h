#include "i2c.h"

/* ===================== Interrupt-driven I2C ===================== */

typedef enum
{
    I2C_DIR_WRITE = 0,
    I2C_DIR_READ  = 1,
} i2c_dir_t;

/* High-level transaction type */
typedef enum
{
    I2C_XFER_TX = 0,          /* write only */
    I2C_XFER_RX = 1,          /* read only */
    I2C_XFER_TXRX = 2,        /* write then repeated-start read (reg reads) */
} i2c_xfer_kind_t;

/* Driver state machine states (EV IRQ advances these) */
typedef enum
{
    I2C_ST_IDLE = 0,

    /* Start / address */
    I2C_ST_START,             /* START generated, waiting SB */
    I2C_ST_ADDR,              /* address byte written, waiting ADDR */

    I2C_ST_TX,                /* sending tx bytes via TXE/BTF */

    /* Repeated start (for TXRX) */
    I2C_ST_RESTART,           /* generating repeated start, waiting SB */

    /* RX phase */
    I2C_ST_RX,                /* receiving bytes (len >= 3 streaming) */
    I2C_ST_RX_LAST3,          /* last 3-byte sequence setup */
    I2C_ST_RX_LAST2,          /* last 2 bytes sequence (BTF-based) */
    I2C_ST_RX_LAST1,          /* last 1 byte sequence */

    /* Finish */
    I2C_ST_STOP,              /* STOP requested, completing */
    I2C_ST_DONE,              /* completed successfully */
    I2C_ST_ERROR,             /* error occurred (AF/ARLO/etc or SW timeout) */
} i2c_state_t;

/* Completion status */
typedef enum
{
    I2C_STATUS_OK = 0,
    I2C_STATUS_ERR = 1,
} i2c_status_t;

/* Options flags */
typedef enum
{
    I2C_OPT_NONE      = 0,
    I2C_OPT_SEND_STOP = (1u << 0),  /* TX/RX: end with STOP */
} i2c_opts_t;

/* Completion callback signature */
typedef void (*i2c_done_cb_t)(i2c_status_t status, uint32_t err_flags, void *user);

/* Transaction descriptor (user fills this, driver consumes it) */
typedef struct
{
    uint8_t addr7;                  /* 7-bit address */

    i2c_xfer_kind_t kind;

    /* Optional TX phase (for TX or TXRX) */
    const uint8_t *tx;
    size_t tx_len;

    /* Optional RX phase (for RX or TXRX) */
    uint8_t *rx;
    size_t rx_len;

    i2c_opts_t opts;

    /* Completion */
    i2c_done_cb_t done;
    void *user;
} i2c_xfer_t;

/* Internal runtime bookkeeping for the active transfer */
typedef struct
{
    volatile i2c_state_t st;
    volatile uint32_t err_flags;

    /* current transaction (pointers/lengths) */
    i2c_xfer_t xfer;

    /* progress counters */
    size_t tx_idx;
    size_t rx_idx;

    /* cached for ISR convenience */
    uint8_t addr_byte;              /* (addr7<<1 | R/W) currently being sent */
    bool phase_is_read;             /* current phase direction */

    /* for last-byte handling */
    size_t rx_remaining;

} i2c1_ctx_t;

/* ===================== Async API ===================== */

/* Initialize driver context + NVIC + enable I2C interrupts (EV/ER) */
void i2c1_init_irq(uint32_t pclk1_hz, uint32_t i2c_hz);

/* Submit an async transfer. */
bool i2c1_submit(const i2c_xfer_t *xfer);

/* Abort current transfer. */
void i2c1_abort(uint32_t reason_flags);

/* Query busy state */
bool i2c1_busy(void);

