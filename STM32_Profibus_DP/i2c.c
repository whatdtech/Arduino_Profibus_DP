/*
 * i2c.c — Bare-metal I2C1 driver for STM32F411CEU6
 *
 * Pins: PB6 = SCL, PB7 = SDA  (AF4)
 * PCLK1 = 48 MHz (SYSCLK=96 MHz, APB1 prescaler=/2)
 * Fast mode: 400 kHz
 */

#include "i2c.h"

/* --------------------------------------------------------------------------
 * Register base addresses (bare-metal, no CMSIS dependency)
 * -------------------------------------------------------------------------- */
#define PERIPH_BASE         0x40000000U
#define APB1PERIPH_BASE     (PERIPH_BASE + 0x00000000U)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000U)

/* RCC */
#define RCC_BASE_ADDR       (AHB1PERIPH_BASE + 0x3800U)
#define RCC_AHB1ENR_REG     (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x30U))
#define RCC_APB1ENR_REG     (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x40U))

/* GPIOB — already partly configured in main.c's GPIO_Init() but PB6/7 untouched */
#define GPIOB_BASE_ADDR     (AHB1PERIPH_BASE + 0x0400U)
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} I2C_GPIO_TypeDef;

#define GPIOB_I2C  ((I2C_GPIO_TypeDef *)GPIOB_BASE_ADDR)

/* I2C1 */
#define I2C1_BASE_ADDR      (APB1PERIPH_BASE + 0x5400U)
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;

#define I2C1_REG    ((I2C_TypeDef *)I2C1_BASE_ADDR)

/* I2C CR1 bits */
#define I2C_CR1_PE          (1U << 0)
#define I2C_CR1_START       (1U << 8)
#define I2C_CR1_STOP        (1U << 9)
#define I2C_CR1_ACK        (1U << 10)
#define I2C_CR1_SWRST       (1U << 15)

/* I2C SR1 bits */
#define I2C_SR1_SB          (1U << 0)   /* Start bit generated */
#define I2C_SR1_ADDR        (1U << 1)   /* Address sent/matched */
#define I2C_SR1_BTF         (1U << 2)   /* Byte transfer finished */
#define I2C_SR1_TXE         (1U << 7)   /* Data register empty (Tx) */
#define I2C_SR1_RXNE        (1U << 6)   /* Data register not empty (Rx) */
#define I2C_SR1_BERR        (1U << 8)
#define I2C_SR1_ARLO        (1U << 9)
#define I2C_SR1_AF          (1U << 10)  /* Acknowledge failure */
#define I2C_SR1_OVR         (1U << 11)

/* I2C SR2 bits */
#define I2C_SR2_BUSY        (1U << 1)

/* I2C CCR bits */
#define I2C_CCR_FS          (1U << 15)  /* Fast mode */

/* RCC clock enable bits */
#define RCC_AHB1ENR_GPIOBEN (1U << 1)
#define RCC_APB1ENR_I2C1EN  (1U << 21)

/* --------------------------------------------------------------------------
 * Timeout (iterations, not µs — prevents infinite hang on bus error)
 * -------------------------------------------------------------------------- */
#define I2C_TIMEOUT_CYCLES  200000U

/* --------------------------------------------------------------------------
 * Helper: clear any error flags
 * -------------------------------------------------------------------------- */
static inline void i2c_clear_errors(void)
{
    I2C1_REG->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);
}

/* --------------------------------------------------------------------------
 * I2C1_Init
 * -------------------------------------------------------------------------- */
void I2C1_Init(void)
{
    /* --- Enable clocks --- */
    RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR_REG |= RCC_APB1ENR_I2C1EN;

    /* --- Configure PB6 (SCL) and PB7 (SDA) --- */
    /* Alternate function mode */
    GPIOB_I2C->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB_I2C->MODER |=  (2U << (6 * 2)) | (2U << (7 * 2));

    /* Open drain */
    GPIOB_I2C->OTYPER |= (1U << 6) | (1U << 7);

    /* High speed */
    GPIOB_I2C->OSPEEDR |= (3U << (6 * 2)) | (3U << (7 * 2));

    /* Pull-up (external pull-ups are preferred, but this helps)*/
    GPIOB_I2C->PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB_I2C->PUPDR |=  (1U << (6 * 2)) | (1U << (7 * 2));

    /* AF4 = I2C1 for both PB6 and PB7 (AFR[0] covers pins 0..7) */
    GPIOB_I2C->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOB_I2C->AFR[0] |=  (4U  << (6 * 4)) | (4U  << (7 * 4));

    /* --- Reset then configure I2C1 --- */
    I2C1_REG->CR1 |=  I2C_CR1_SWRST;
    I2C1_REG->CR1 &= ~I2C_CR1_SWRST;

    /*
     * PCLK1 = 48 MHz  →  CR2.FREQ = 48
     *
     * Fast mode 400 kHz, DUTY=0:
     *   T_high = CCR × Tpclk1
     *   T_low  = 2 × CCR × Tpclk1
     *   f_SCL  = PCLK1 / (3 × CCR)   →  CCR = 48 000 000 / (3×400 000) = 40
     *
     * TRISE (fast mode, max 300 ns):
     *   TRISE = floor(300 ns × PCLK1) + 1 = floor(14.4) + 1 = 15
     */
    I2C1_REG->CR1   = 0;               /* Disable while configuring */
    I2C1_REG->CR2   = 48U;             /* FREQ = 48 MHz */
    I2C1_REG->CCR   = I2C_CCR_FS | 40U;
    I2C1_REG->TRISE = 15U;

    I2C1_REG->CR1 = I2C_CR1_PE;        /* Enable peripheral */
}

/* --------------------------------------------------------------------------
 * Low-level: send START + slave address (write direction)
 * Returns false on timeout or NACK.
 * -------------------------------------------------------------------------- */
static bool i2c_start(uint8_t dev_addr)
{
    uint32_t t;

    /* Wait for bus free */
    t = I2C_TIMEOUT_CYCLES;
    while (I2C1_REG->SR2 & I2C_SR2_BUSY) {
        if (--t == 0) return false;
    }

    i2c_clear_errors();

    /* Generate START */
    I2C1_REG->CR1 |= I2C_CR1_START;

    /* Wait for SB (start bit detected) */
    t = I2C_TIMEOUT_CYCLES;
    while (!(I2C1_REG->SR1 & I2C_SR1_SB)) {
        if (--t == 0) return false;
    }

    /* Send device address (write) — clears SB */
    I2C1_REG->DR = dev_addr & 0xFEU; /* bit0=0 → write */

    /* Wait for ADDR flag (address ACK received) */
    t = I2C_TIMEOUT_CYCLES;
    while (!(I2C1_REG->SR1 & I2C_SR1_ADDR)) {
        if (I2C1_REG->SR1 & I2C_SR1_AF) {   /* NACK */
            I2C1_REG->SR1 &= ~I2C_SR1_AF;
            I2C1_REG->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (--t == 0) return false;
    }

    /* Clear ADDR (required: read SR1 then SR2) */
    (void)I2C1_REG->SR1;
    (void)I2C1_REG->SR2;

    return true;
}

/* --------------------------------------------------------------------------
 * Low-level: write one byte and wait for TXE
 * -------------------------------------------------------------------------- */
static bool i2c_write_byte(uint8_t byte)
{
    uint32_t t = I2C_TIMEOUT_CYCLES;
    while (!(I2C1_REG->SR1 & I2C_SR1_TXE)) {
        if (I2C1_REG->SR1 & I2C_SR1_AF) {
            I2C1_REG->SR1 &= ~I2C_SR1_AF;
            I2C1_REG->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (--t == 0) return false;
    }
    I2C1_REG->DR = byte;
    return true;
}

/* --------------------------------------------------------------------------
 * Low-level: wait for BTF and generate STOP
 * -------------------------------------------------------------------------- */
static bool i2c_stop(void)
{
    uint32_t t = I2C_TIMEOUT_CYCLES;
    while (!(I2C1_REG->SR1 & I2C_SR1_BTF)) {
        if (--t == 0) {
            I2C1_REG->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }
    I2C1_REG->CR1 |= I2C_CR1_STOP;
    return true;
}

/* --------------------------------------------------------------------------
 * I2C1_WriteData — single I2C transaction: [START][ADDR][data...][STOP]
 * -------------------------------------------------------------------------- */
bool I2C1_WriteData(uint8_t dev_addr, uint8_t *data, uint16_t size)
{
    if (!i2c_start(dev_addr)) return false;

    for (uint16_t i = 0; i < size; i++) {
        if (!i2c_write_byte(data[i])) return false;
    }

    return i2c_stop();
}

/* --------------------------------------------------------------------------
 * I2C1_WriteMem — [START][ADDR][ctrl_byte][data...][STOP]
 * Used for SSD1306:
 *   ctrl_byte = 0x00 → command mode
 *   ctrl_byte = 0x40 → data mode
 * -------------------------------------------------------------------------- */
bool I2C1_WriteMem(uint8_t dev_addr, uint8_t ctrl_byte, uint8_t *data, uint16_t size)
{
    if (!i2c_start(dev_addr)) return false;

    /* Send the SSD1306 control byte first */
    if (!i2c_write_byte(ctrl_byte)) return false;

    for (uint16_t i = 0; i < size; i++) {
        if (!i2c_write_byte(data[i])) return false;
    }

    return i2c_stop();
}
