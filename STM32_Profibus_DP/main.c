/*****************************************************************************
 * STM32F411CEU6 Profibus DP Slave
 * --------------------------------
 * Ported from Arduino (ATmega328P) Profibus DP Slave implementation.
 * Supports baud rates up to 1.5 Mbps using:
 *   - USART1 with DMA RX (circular buffer + IDLE line detection)
 *   - TIM2  for Profibus timing (TSYN / TSDR)
 *   - GPIO  for MAX485 TX enable control
 *
 * Target: STM32F411CEU6 "BlackPill" board (25 MHz HSE, 96 MHz SYSCLK)
 * Toolchain: arm-none-eabi-gcc + CMSIS headers (bare metal, no HAL)
 *
 * Hardware connections:
 *   PA9  -> MAX485 DI  (USART1 TX)
 *   PA10 <- MAX485 RO  (USART1 RX)
 *   PA1  -> MAX485 DE + RE (TX Enable, active high)
 *   PC13 -> Error LED (built-in on BlackPill, active LOW)
 *   PB0  -> User LED
 *   PB1  <- User Input (touch button)
 *   PB10 -> VCC for touch button
 *   PB2  -> GND for touch button
 *
 * Author: Ported from Arduino_Profibus_DP by WhatDTech
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================= */
/*  CMSIS Core & Device Headers (register definitions)                       */
/* ========================================================================= */
#include "i2c.h"
#include "ssd1306.h"
#include <stdio.h> // For snprintf

/*
 * If you have CMSIS installed, replace the block below with:
 *   #include "stm32f411xe.h"
 *
 * For standalone compilation we define the necessary registers inline.
 */
#ifndef __STM32F411xE_H  /* Only define if not already included via CMSIS */

/* Base addresses */
#define PERIPH_BASE           ((uint32_t)0x40000000)
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)

/* RCC */
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define RCC_CR                (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR           (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR              (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR           (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR           (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR           (*(volatile uint32_t *)(RCC_BASE + 0x44))

/* FLASH */
#define FLASH_BASE_ADDR       ((uint32_t)0x40023C00)
#define FLASH_ACR             (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x00))

/* GPIO */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)

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
} GPIO_TypeDef;

#define GPIOA                 ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *)GPIOC_BASE)

/* USART1 */
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART1                ((USART_TypeDef *)USART1_BASE)

/* USART SR bits */
#define USART_SR_PE           (1U << 0)
#define USART_SR_FE           (1U << 1)
#define USART_SR_NE           (1U << 2)
#define USART_SR_ORE          (1U << 3)
#define USART_SR_IDLE         (1U << 4)
#define USART_SR_RXNE         (1U << 5)
#define USART_SR_TC           (1U << 6)
#define USART_SR_TXE          (1U << 7)

/* USART CR1 bits */
#define USART_CR1_RE          (1U << 2)
#define USART_CR1_TE          (1U << 3)
#define USART_CR1_IDLEIE      (1U << 4)
#define USART_CR1_RXNEIE      (1U << 5)
#define USART_CR1_TCIE        (1U << 6)
#define USART_CR1_TXEIE       (1U << 7)
#define USART_CR1_PS          (1U << 9)   /* Parity selection: 0=even, 1=odd */
#define USART_CR1_PCE         (1U << 10)  /* Parity control enable */
#define USART_CR1_M           (1U << 12)  /* Word length: 0=8 data, 1=9 data */
#define USART_CR1_UE          (1U << 13)
#define USART_CR1_OVER8       (1U << 15)

/* USART CR2 bits */
#define USART_CR2_STOP_1      (0U << 12)

/* USART CR3 bits */
#define USART_CR3_DMAR        (1U << 6)

/* TIM2 (32-bit general purpose timer) */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t RESERVED2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR_REG;
    volatile uint32_t OR;
} TIM_TypeDef;

#define TIM2                  ((TIM_TypeDef *)TIM2_BASE)

/* TIM bits */
#define TIM_CR1_CEN           (1U << 0)
#define TIM_CR1_OPM           (1U << 3)  /* One-pulse mode */
#define TIM_DIER_UIE          (1U << 0)
#define TIM_SR_UIF            (1U << 0)
#define TIM_EGR_UG            (1U << 0)

/* DMA2 */
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400)

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

typedef struct {
    volatile uint32_t LISR;
    volatile uint32_t HISR;
    volatile uint32_t LIFCR;
    volatile uint32_t HIFCR;
} DMA_TypeDef;

#define DMA2                  ((DMA_TypeDef *)DMA2_BASE)
/* DMA2 Stream 2 = USART1_RX (Channel 4) */
#define DMA2_Stream2          ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10 + 0x18 * 2))
/* DMA2 Stream 5 = USART1_RX alternate (Channel 4) - used as backup */

/* DMA CR bits */
#define DMA_CR_EN             (1U << 0)
#define DMA_CR_TCIE           (1U << 4)
#define DMA_CR_HTIE           (1U << 3)
#define DMA_CR_TEIE           (1U << 2)
#define DMA_CR_DIR_P2M        (0U << 6)   /* Peripheral to memory */
#define DMA_CR_CIRC           (1U << 8)   /* Circular mode */
#define DMA_CR_MINC           (1U << 10)  /* Memory increment */
#define DMA_CR_CHSEL_4        (4U << 25)  /* Channel 4 selection */

/* NVIC */
#define NVIC_ISER0            (*(volatile uint32_t *)0xE000E100)
#define NVIC_ISER1            (*(volatile uint32_t *)0xE000E104)

/* IRQ numbers */
#define TIM2_IRQn             28
#define USART1_IRQn           37
#define DMA2_Stream2_IRQn     58

/* SysTick */
#define SYSTICK_CTRL          (*(volatile uint32_t *)0xE000E010)
#define SYSTICK_LOAD          (*(volatile uint32_t *)0xE000E014)
#define SYSTICK_VAL           (*(volatile uint32_t *)0xE000E018)

#endif /* __STM32F411xE_H */

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/*
 * HSE crystal frequency (Hz).
 * BlackPill boards typically use 25 MHz. Change to 8000000 if your board
 * uses an 8 MHz crystal.
 */
#define HSE_VALUE             25000000U

/*
 * Target system clock (Hz).
 * 96 MHz gives EXACT 1.5 Mbps baud rate with zero error (USARTDIV = 64).
 * Also gives exact baud for: 500k, 187.5k, 93.75k, 45.45k*, 19.2k*, 9.6k*
 * (* = small error, acceptable per Profibus spec < 0.3%)
 */
#define SYSCLK_FREQ           96000000U

/*
 * Profibus baud rate (bps).
 * Supported: 9600, 19200, 45450, 93750, 187500, 500000, 1500000
 */
#define PROFIBUS_BAUD         1500000U

/*
 * Slave address (1-125). Address 126 = default, 127 = broadcast.
 */
#define SLAVE_ADDRESS         6

/*
 * I/O data sizes (bytes). Must match PLC / GSD configuration.
 */
#define INPUT_DATA_SIZE       16   /* Bytes FROM master TO slave */
#define OUTPUT_DATA_SIZE      16   /* Bytes FROM slave TO master */
#define VENDOR_DATA_SIZE      0
#define EXT_DIAG_DATA_SIZE    0

/* ========================================================================= */
/*  Pin Definitions                                                          */
/* ========================================================================= */

/* TX Enable for MAX485 (active HIGH = transmit, LOW = receive) */
#define TX_EN_PORT            GPIOA
#define TX_EN_PIN             1

/* Error LED (PC13 on BlackPill - active LOW) */
#define LED_ERR_PORT          GPIOC
#define LED_ERR_PIN           13

/* User LED */
#define LED_USR_PORT          GPIOB
#define LED_USR_PIN           0

/* User Input (touch button) */
#define INPUT_PORT            GPIOB
#define INPUT_PIN             1

/* VCC/GND for touch button */
#define VCC_PORT              GPIOB
#define VCC_PIN               10
#define GND_PORT              GPIOB
#define GND_PIN_NUM           2

/* ========================================================================= */
/*  GPIO Macros (using BSRR for atomic single-cycle set/reset)               */
/* ========================================================================= */

#define GPIO_SET(port, pin)   ((port)->BSRR = (1U << (pin)))
#define GPIO_RESET(port, pin) ((port)->BSRR = (1U << ((pin) + 16)))
#define GPIO_READ(port, pin)  (((port)->IDR >> (pin)) & 1U)

#define TX_ENABLE_ON()        GPIO_SET(TX_EN_PORT, TX_EN_PIN)
#define TX_ENABLE_OFF()       GPIO_RESET(TX_EN_PORT, TX_EN_PIN)

/* PC13 LED is active LOW on BlackPill */
#define LED_ERROR_ON()        GPIO_RESET(LED_ERR_PORT, LED_ERR_PIN)
#define LED_ERROR_OFF()       GPIO_SET(LED_ERR_PORT, LED_ERR_PIN)

#define LED_USER_ON()         GPIO_SET(LED_USR_PORT, LED_USR_PIN)
#define LED_USER_OFF()        GPIO_RESET(LED_USR_PORT, LED_USR_PIN)

/* ========================================================================= */
/*  Profibus Constants                                                       */
/* ========================================================================= */

/* Ident Number - must match GSD file */
#define IDENT_HIGH_BYTE       0xC0
#define IDENT_LOW_BYTE        0xDE

/* Addresses */
#define SAP_OFFSET            128
#define BROADCAST_ADD         127
#define DEFAULT_ADD           126

/* Telegram types (Start Delimiters) */
#define SD1                   0x10  /* No data field */
#define SD2                   0x68  /* Variable length data */
#define SD3                   0xA2  /* Fixed 8-byte data */
#define SD4                   0xDC  /* Token */
#define SC                    0xE5  /* Short acknowledgment */
#define ED                    0x16  /* End delimiter */

/* Function Codes - Request */
#define FDL_STATUS            0x09
#define SRD_HIGH              0x0D
#define FCV_                  0x10
#define FCB_                  0x20
#define REQUEST_              0x40

/* Function Codes - Response */
#define FDL_STATUS_OK         0x00
#define DATA_LOW              0x08
#define DIAGNOSE              0x0A

/* Service Access Points */
#define SAP_SET_SLAVE_ADR     55
#define SAP_RD_INP            56
#define SAP_RD_OUTP           57
#define SAP_GLOBAL_CONTROL    58
#define SAP_GET_CFG           59
#define SAP_SLAVE_DIAGNOSIS   60
#define SAP_SET_PRM           61
#define SAP_CHK_CFG           62

/* Global Control */
#define CLEAR_DATA_           0x02
#define UNFREEZE_             0x04
#define FREEZE_               0x08
#define UNSYNC_               0x10
#define SYNC_                 0x20

/* Diagnosis Status Byte 1 */
#define STATION_NOT_EXISTENT_ 0x01
#define STATION_NOT_READY_    0x02
#define CFG_FAULT_            0x04
#define EXT_DIAG_             0x08
#define NOT_SUPPORTED_        0x10
#define INV_SLAVE_RESPONSE_   0x20
#define PRM_FAULT_            0x40
#define MASTER_LOCK           0x80

/* Diagnosis Status Byte 2 */
#define STATUS_2_DEFAULT      0x04
#define PRM_REQ_              0x01
#define STAT_DIAG_            0x02
#define WD_ON_                0x08
#define FREEZE_MODE_          0x10
#define SYNC_MODE_            0x20
#define DEACTIVATED_          0x80

/* Diagnosis Status Byte 3 */
#define DIAG_SIZE_OK          0x00
#define DIAG_SIZE_ERROR       0x80

/* Master address */
#define MASTER_ADD_DEFAULT    0xFF

/* Extended Diagnosis */
#define EXT_DIAG_TYPE_        0xC0
#define EXT_DIAG_GERAET       0x00
#define EXT_DIAG_KENNUNG      0x40
#define EXT_DIAG_KANAL        0x80
#define EXT_DIAG_BYTE_CNT_    0x3F

/* Set Parameters */
#define LOCK_SLAVE_           0x80
#define UNLOCK_SLAVE_         0x40
#define ACTIVATE_SYNC_        0x20
#define ACTIVATE_FREEZE_      0x10
#define ACTIVATE_WATCHDOG_    0x08
#define DPV1_MODE_            0x80
#define FAIL_SAVE_MODE_       0x40
#define PUBLISHER_MODE_       0x20
#define WATCHDOG_TB_1MS       0x04

/* Check Config */
#define CFG_DIRECTION_        0x30
#define CFG_INPUT             0x10
#define CFG_OUTPUT            0x20
#define CFG_INPUT_OUTPUT      0x30
#define CFG_SPECIAL           0x00
#define CFG_KONSISTENZ_       0x80
#define CFG_WIDTH_            0x40
#define CFG_BYTE              0x00
#define CFG_WORD              0x40
#define CFG_BYTE_CNT_         0x0F
#define CFG_SP_DIRECTION_     0xC0
#define CFG_SP_VOID           0x00
#define CFG_SP_INPUT          0x40
#define CFG_SP_OUTPUT         0x80
#define CFG_SP_INPUT_OPTPUT   0xC0
#define CFG_SP_VENDOR_CNT_    0x0F
#define CFG_SP_BYTE_CNT_      0x3F

/* ========================================================================= */
/*  Profibus Timing (in microseconds, computed from baud rate)               */
/* ========================================================================= */

/*
 * 1 TBIT = 1 / BAUD  (in seconds)
 * Timer runs at 1 MHz (1 tick = 1 µs), so:
 *   TBIT_US = 1000000 / BAUD
 *
 * For 1.5 Mbps: TBIT = 0.667 µs
 * TSYN  = 33 * TBIT = 22 µs
 * TSDR  = 11 * TBIT =  7.3 µs (min), 15 * TBIT = 10 µs (max)
 * For sub-microsecond precision at high baud rates, we use ceiling.
 */
#define TBIT_US               (1000000U / PROFIBUS_BAUD)  /* integer µs per bit */

/* Minimum timing values (in timer ticks = µs at 1 MHz timer clock) */
/* We add 1 to compensate for integer truncation at high baud rates */
#if (PROFIBUS_BAUD >= 500000U)
  #define TIMEOUT_SYN         ((33U * 1000000U + PROFIBUS_BAUD - 1U) / PROFIBUS_BAUD)  /* ceil(33 * Tbit) */
  #define TIMEOUT_SDR         ((11U * 1000000U + PROFIBUS_BAUD - 1U) / PROFIBUS_BAUD)  /* ceil(11 * Tbit) */
  #define TIMEOUT_RX          ((15U * 1000000U + PROFIBUS_BAUD - 1U) / PROFIBUS_BAUD)  /* ceil(15 * Tbit) */
  #define TIMEOUT_TX          ((15U * 1000000U + PROFIBUS_BAUD - 1U) / PROFIBUS_BAUD)  /* ceil(15 * Tbit) */
#else
  /* For lower baud rates, standard integer math is fine */
  #define TIMEOUT_SYN         (33U * 1000000U / PROFIBUS_BAUD)
  #define TIMEOUT_SDR         (15U * 1000000U / PROFIBUS_BAUD)
  #define TIMEOUT_RX          (15U * 1000000U / PROFIBUS_BAUD)
  #define TIMEOUT_TX          (15U * 1000000U / PROFIBUS_BAUD)
#endif

/* ========================================================================= */
/*  Profibus State Machine                                                   */
/* ========================================================================= */
#define PROFIBUS_WAIT_SYN     1
#define PROFIBUS_WAIT_DATA    2
#define PROFIBUS_GET_DATA     3
#define PROFIBUS_SEND_DATA    4

/* ========================================================================= */
/*  Buffer sizes                                                             */
/* ========================================================================= */
#define MAX_BUFFER_SIZE       64   /* Increased from 32 for larger frames */
#define DMA_RX_BUF_SIZE       64   /* DMA circular buffer */

/* ========================================================================= */
/*  Global Variables                                                         */
/* ========================================================================= */

/* DMA receive buffer (circular) */
static volatile uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
static uint16_t dma_rx_prev_pos = 0;  /* Previous DMA read position */

/* Profibus frame buffer */
static uint8_t uart_buffer[MAX_BUFFER_SIZE];
static volatile uint16_t uart_byte_cnt = 0;
static volatile uint16_t uart_transmit_cnt = 0;

/* Profibus state */
static volatile uint8_t profibus_status;
static uint8_t diagnose_status;
static uint8_t slave_addr;
static uint8_t master_addr;
static uint8_t group;
static volatile uint8_t new_data;

/* I/O data registers */
#if (OUTPUT_DATA_SIZE > 0)
static volatile uint8_t Profibus_out_register[OUTPUT_DATA_SIZE];
#endif
#if (INPUT_DATA_SIZE > 0)
static uint8_t Profibus_in_register[INPUT_DATA_SIZE];
#endif
#if (VENDOR_DATA_SIZE > 0)
static uint8_t Vendor_Data[VENDOR_DATA_SIZE];
#endif
#if (EXT_DIAG_DATA_SIZE > 0)
static uint8_t Diag_Data[EXT_DIAG_DATA_SIZE];
#endif

static uint8_t Input_Data_size;
static uint8_t Output_Data_size;
static uint8_t Vendor_Data_size;

typedef enum {
    DP_STATE_OFFLINE,
    DP_STATE_WAIT_PRM,
    DP_STATE_WAIT_CFG,
    DP_STATE_DATA_EXCHANGE
} DP_State_t;

static volatile DP_State_t dp_state = DP_STATE_OFFLINE;
static volatile uint32_t last_valid_msg_time = 0;

/* Millisecond tick counter */
static volatile uint32_t systick_ms = 0;

/* ========================================================================= */
/*  Forward Declarations                                                     */
/* ========================================================================= */

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART1_Init(uint32_t baud);
static void DMA2_Stream2_Init(void);
static void TIM2_Init(void);
static void SysTick_Init(void);

static void init_Profibus(void);
static void profibus_RX(void);
static void profibus_send_CMD(uint8_t type, uint8_t function_code,
                              uint8_t sap_offset, uint8_t *pdu,
                              uint8_t length_pdu);
static void profibus_TX(uint8_t *data, uint8_t length);
static uint8_t checksum(uint8_t *data, uint8_t length);
static uint8_t addmatch(uint8_t destination);

static void timer_start(uint32_t timeout_us);
static void timer_stop(void);
static void timer_restart(uint32_t timeout_us);

static uint32_t millis(void);
static void delay_ms(uint32_t ms);

/* ========================================================================= */
/*  System Clock Configuration                                               */
/*  HSE (25 MHz) -> PLL -> 96 MHz SYSCLK                                    */
/*                                                                           */
/*  PLL config for 25 MHz HSE -> 96 MHz:                                     */
/*    PLL_M = 25  (VCO input = 25/25 = 1 MHz)                               */
/*    PLL_N = 192 (VCO output = 1 * 192 = 192 MHz)                          */
/*    PLL_P = 2   (SYSCLK = 192/2 = 96 MHz)                                 */
/*    PLL_Q = 4   (USB = 192/4 = 48 MHz, if needed)                         */
/*                                                                           */
/*  For 8 MHz HSE -> 96 MHz:                                                 */
/*    PLL_M = 8, PLL_N = 192, PLL_P = 2, PLL_Q = 4                          */
/* ========================================================================= */
static void SystemClock_Config(void)
{
    /* ---- Enable HSE and wait for it to stabilize ---- */
    RCC_CR |= (1U << 16);  /* HSEON */
    while (!(RCC_CR & (1U << 17))) {}  /* Wait for HSERDY */

    /* ---- Configure Flash latency for 96 MHz (3 wait states) ---- */
    /* Also enable prefetch buffer and instruction/data caches */
    FLASH_ACR = (3U << 0)   /* LATENCY = 3 WS */
              | (1U << 8)   /* PRFTEN  */
              | (1U << 9)   /* ICEN    */
              | (1U << 10); /* DCEN    */

    /* ---- Configure PLL ---- */
    /* Disable PLL before configuration */
    RCC_CR &= ~(1U << 24);  /* PLLON = 0 */
    while (RCC_CR & (1U << 25)) {}  /* Wait for PLLRDY = 0 */

#if (HSE_VALUE == 25000000U)
    #define PLL_M   25U
#elif (HSE_VALUE == 8000000U)
    #define PLL_M   8U
#else
    #error "Unsupported HSE_VALUE. Use 25000000 or 8000000."
#endif
    #define PLL_N   192U
    #define PLL_P   2U    /* Encoded as (PLL_P/2 - 1) = 0 */
    #define PLL_Q   4U

    RCC_PLLCFGR = (PLL_M << 0)
                | (PLL_N << 6)
                | (((PLL_P / 2U) - 1U) << 16)
                | (1U << 22)       /* PLLSRC = HSE */
                | (PLL_Q << 24);

    /* ---- Enable PLL ---- */
    RCC_CR |= (1U << 24);  /* PLLON */
    while (!(RCC_CR & (1U << 25))) {}  /* Wait for PLLRDY */

    /* ---- Configure bus clocks ---- */
    /*
     * AHB  prescaler = 1 (HCLK  = 96 MHz)
     * APB1 prescaler = 2 (PCLK1 = 48 MHz, timer clk = 96 MHz)
     * APB2 prescaler = 1 (PCLK2 = 96 MHz = USART1 clock)
     */
    RCC_CFGR = (0U << 4)    /* HPRE  = /1 (AHB)  */
             | (4U << 10)   /* PPRE1 = /2 (APB1)  */
             | (0U << 13);  /* PPRE2 = /1 (APB2)  */

    /* ---- Switch system clock to PLL ---- */
    RCC_CFGR = (RCC_CFGR & ~0x03U) | 0x02U;  /* SW = PLL */
    while ((RCC_CFGR & 0x0CU) != 0x08U) {}  /* Wait for SWS = PLL */
}

/* ========================================================================= */
/*  GPIO Initialization                                                      */
/* ========================================================================= */
static void GPIO_Init(void)
{
    /* Enable GPIO clocks: GPIOA, GPIOB, GPIOC */
    RCC_AHB1ENR |= (1U << 0) | (1U << 1) | (1U << 2);

    /* --- PA1: TX Enable output, push-pull, very high speed --- */
    GPIOA->MODER   &= ~(3U << (TX_EN_PIN * 2));
    GPIOA->MODER   |=  (1U << (TX_EN_PIN * 2));   /* General purpose output */
    GPIOA->OTYPER  &= ~(1U << TX_EN_PIN);          /* Push-pull */
    GPIOA->OSPEEDR |=  (3U << (TX_EN_PIN * 2));   /* Very high speed */
    TX_ENABLE_OFF();  /* Start in receive mode */

    /* --- PA9: USART1_TX (AF7) --- */
    GPIOA->MODER   &= ~(3U << (9 * 2));
    GPIOA->MODER   |=  (2U << (9 * 2));   /* Alternate function */
    GPIOA->OSPEEDR |=  (3U << (9 * 2));   /* Very high speed */
    GPIOA->AFR[1]  &= ~(0xFU << ((9 - 8) * 4));
    GPIOA->AFR[1]  |=  (7U << ((9 - 8) * 4));  /* AF7 = USART1 */

    /* --- PA10: USART1_RX (AF7) --- */
    GPIOA->MODER   &= ~(3U << (10 * 2));
    GPIOA->MODER   |=  (2U << (10 * 2));  /* Alternate function */
    GPIOA->PUPDR   &= ~(3U << (10 * 2));
    GPIOA->PUPDR   |=  (1U << (10 * 2));  /* Pull-up (idle high for RS485) */
    GPIOA->AFR[1]  &= ~(0xFU << ((10 - 8) * 4));
    GPIOA->AFR[1]  |=  (7U << ((10 - 8) * 4));  /* AF7 = USART1 */

    /* --- PC13: Error LED output (built-in on BlackPill, active LOW) --- */
    GPIOC->MODER   &= ~(3U << (LED_ERR_PIN * 2));
    GPIOC->MODER   |=  (1U << (LED_ERR_PIN * 2));  /* Output */
    LED_ERROR_OFF();  /* LED off (HIGH) */

    /* --- PB0: User LED output --- */
    GPIOB->MODER   &= ~(3U << (LED_USR_PIN * 2));
    GPIOB->MODER   |=  (1U << (LED_USR_PIN * 2));  /* Output */
    LED_USER_OFF();

    /* --- PB1: User Input (touch button) --- */
    GPIOB->MODER   &= ~(3U << (INPUT_PIN * 2));  /* Input mode (00) */
    GPIOB->PUPDR   &= ~(3U << (INPUT_PIN * 2));
    GPIOB->PUPDR   |=  (2U << (INPUT_PIN * 2));  /* Pull-down */

    /* --- PB10: VCC for touch button (output HIGH) --- */
    GPIOB->MODER   &= ~(3U << (VCC_PIN * 2));
    GPIOB->MODER   |=  (1U << (VCC_PIN * 2));
    GPIO_SET(VCC_PORT, VCC_PIN);

    /* --- PB2: GND for touch button (output LOW) --- */
    GPIOB->MODER   &= ~(3U << (GND_PIN_NUM * 2));
    GPIOB->MODER   |=  (1U << (GND_PIN_NUM * 2));
    GPIO_RESET(GND_PORT, GND_PIN_NUM);
}

/* ========================================================================= */
/*  USART1 Initialization                                                    */
/*  8 data bits, EVEN parity, 1 stop bit (8E1) - Profibus standard          */
/*  DMA-based reception, interrupt-driven transmission                       */
/* ========================================================================= */
static void USART1_Init(uint32_t baud)
{
    /* Enable USART1 clock (APB2) */
    RCC_APB2ENR |= (1U << 4);

    /* Disable USART during configuration */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    /*
     * Baud rate calculation (OVER16 mode):
     *   USARTDIV = f_CK / (16 * baud)  ... but we write the full mantissa+fraction
     *   BRR = f_CK / baud  (for OVER16)
     *
     * APB2 clock = SYSCLK = 96 MHz (prescaler = 1)
     * For 1.5 Mbps: BRR = 96000000 / 1500000 = 64 (exact!)
     * For 500 kbps:  BRR = 96000000 / 500000  = 192 (exact!)
     */
    uint32_t pclk2 = SYSCLK_FREQ;  /* APB2 prescaler = 1 */
    USART1->BRR = pclk2 / baud;

    /*
     * CR1: 9-bit word length (M=1) because 8 data + even parity = 9 bits total
     *      Even parity enable (PCE=1, PS=0)
     *      RX enable, TX enable
     *      IDLE interrupt enable (for DMA frame detection)
     */
    USART1->CR1 = USART_CR1_M       /* 9 bit word (8 data + parity) */
                | USART_CR1_PCE      /* Parity control enable */
                                     /* PS = 0 -> Even parity (default) */
                | USART_CR1_RE       /* Receiver enable */
                | USART_CR1_TE       /* Transmitter enable */
                | USART_CR1_IDLEIE;  /* IDLE line interrupt enable */

    /* CR2: 1 stop bit (default) */
    USART1->CR2 = USART_CR2_STOP_1;

    /* CR3: Enable DMA receiver */
    USART1->CR3 = USART_CR3_DMAR;

    /* Enable USART1 interrupt in NVIC */
    if (USART1_IRQn < 32) {
        NVIC_ISER0 |= (1U << USART1_IRQn);
    } else {
        NVIC_ISER1 |= (1U << (USART1_IRQn - 32));
    }

    /* Enable USART */
    USART1->CR1 |= USART_CR1_UE;

    /* Read SR and DR to clear any pending flags */
    (void)USART1->SR;
    (void)USART1->DR;
}

/* ========================================================================= */
/*  DMA2 Stream 2 Initialization (USART1 RX - Channel 4)                    */
/*  Circular mode, peripheral-to-memory, byte-width                          */
/* ========================================================================= */
static void DMA2_Stream2_Init(void)
{
    /* Enable DMA2 clock */
    RCC_AHB1ENR |= (1U << 22);

    /* Disable stream before configuration */
    DMA2_Stream2->CR &= ~DMA_CR_EN;
    while (DMA2_Stream2->CR & DMA_CR_EN) {}  /* Wait until disabled */

    /* Clear all interrupt flags for Stream 2 */
    DMA2->LIFCR = (0x3FU << 16);  /* Clear all flags for Stream 2 */

    /* Configure DMA stream */
    DMA2_Stream2->CR = DMA_CR_CHSEL_4   /* Channel 4 = USART1_RX */
                     | DMA_CR_MINC       /* Memory address increment */
                     | DMA_CR_CIRC       /* Circular mode */
                     | DMA_CR_DIR_P2M;   /* Peripheral to memory */
                     /* Byte size transfers (default 00) for both PSIZE and MSIZE */

    /* Peripheral address: USART1 Data Register */
    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;

    /* Memory address: circular receive buffer */
    DMA2_Stream2->M0AR = (uint32_t)dma_rx_buf;

    /* Number of data items */
    DMA2_Stream2->NDTR = DMA_RX_BUF_SIZE;

    /* Reset position tracker */
    dma_rx_prev_pos = 0;

    /* Enable DMA stream */
    DMA2_Stream2->CR |= DMA_CR_EN;
}

/* ========================================================================= */
/*  TIM2 Initialization (Profibus Timing)                                    */
/*  32-bit timer, 1 µs resolution, one-pulse mode                           */
/* ========================================================================= */
static void TIM2_Init(void)
{
    /* Enable TIM2 clock (APB1) */
    RCC_APB1ENR |= (1U << 0);

    /* Stop timer */
    TIM2->CR1 = 0;

    /*
     * Timer clock = APB1 timer clock = 96 MHz
     * (APB1 prescaler = 2, so timer clock = 2 * PCLK1 = 2 * 48 = 96 MHz)
     * Prescaler = 95 -> counter clock = 96 MHz / 96 = 1 MHz (1 µs per tick)
     */
    TIM2->PSC = (SYSCLK_FREQ / 1000000U) - 1U;  /* = 95 for 96 MHz */

    /* ARR will be set before each start */
    TIM2->ARR = TIMEOUT_SYN;

    /* One-pulse mode: timer stops after reaching ARR */
    TIM2->CR1 = TIM_CR1_OPM;

    /* Enable update interrupt */
    TIM2->DIER = TIM_DIER_UIE;

    /* Generate update event to load PSC and ARR */
    TIM2->EGR = TIM_EGR_UG;

    /* Clear update flag caused by EGR */
    TIM2->SR = 0;

    /* Enable TIM2 interrupt in NVIC */
    NVIC_ISER0 |= (1U << TIM2_IRQn);
}

/* ========================================================================= */
/*  Timer Helpers                                                            */
/* ========================================================================= */

/* Start timer with given timeout (in µs / timer ticks) */
static void timer_start(uint32_t timeout_us)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;  /* Stop */
    TIM2->CNT = 0;
    TIM2->ARR = timeout_us;
    TIM2->SR  = 0;               /* Clear flags */
    TIM2->CR1 |= TIM_CR1_CEN;   /* Start */
}

/* Stop timer */
static void timer_stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR = 0;
}

/* Restart timer (reset counter, keep running with same or new ARR) */
static void timer_restart(uint32_t timeout_us)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0;
    TIM2->ARR = timeout_us;
    TIM2->SR  = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ========================================================================= */
/*  SysTick Initialization (1 ms tick for millis())                          */
/* ========================================================================= */
static void SysTick_Init(void)
{
    SYSTICK_LOAD = (SYSCLK_FREQ / 1000U) - 1U;  /* 1 ms period */
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = (1U << 0)   /* ENABLE */
                 | (1U << 1)   /* TICKINT */
                 | (1U << 2);  /* CLKSOURCE = processor clock */
}

static uint32_t millis(void)
{
    return systick_ms;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms) {}
}

/* ========================================================================= */
/*  Profibus Initialization                                                  */
/* ========================================================================= */
static void init_Profibus(void)
{
    uint16_t cnt;

    profibus_status = PROFIBUS_WAIT_SYN;
    diagnose_status = 0;
    Input_Data_size = 0;
    Output_Data_size = 0;
    Vendor_Data_size = 0;
    group = 0;
    new_data = 0;

    dp_state = DP_STATE_WAIT_PRM;

    slave_addr = SLAVE_ADDRESS;
    if ((slave_addr == 0) || (slave_addr > 126))
        slave_addr = DEFAULT_ADD;

    /* Clear data buffers */
    #if (OUTPUT_DATA_SIZE > 0)
    for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
        Profibus_out_register[cnt] = 0xFF;
    #endif
    #if (INPUT_DATA_SIZE > 0)
    for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
        Profibus_in_register[cnt] = 0x00;
    #endif
    #if (VENDOR_DATA_SIZE > 0)
    for (cnt = 0; cnt < VENDOR_DATA_SIZE; cnt++)
        Vendor_Data[cnt] = 0x00;
    #endif
    #if (EXT_DIAG_DATA_SIZE > 0)
    for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
        Diag_Data[cnt] = 0x00;
    #endif

    /* Initialize peripherals */
    USART1_Init(PROFIBUS_BAUD);
    DMA2_Stream2_Init();
    TIM2_Init();

    /* Start with TSYN wait */
    timer_start(TIMEOUT_SYN);
}

/* ========================================================================= */
/*  Profibus Checksum                                                        */
/* ========================================================================= */
static uint8_t checksum(uint8_t *data, uint8_t length)
{
    uint8_t csum = 0;
    while (length--)
    {
        csum += data[length];
    }
    return csum;
}

/* ========================================================================= */
/*  Address Match                                                            */
/* ========================================================================= */
static uint8_t addmatch(uint8_t destination)
{
    if ((destination != slave_addr) &&
        (destination != slave_addr + SAP_OFFSET) &&
        (destination != BROADCAST_ADD) &&
        (destination != BROADCAST_ADD + SAP_OFFSET))
    {
        return 0;  /* false */
    }
    return 1;  /* true */
}

/* ========================================================================= */
/*  Profibus RX Processing (protocol handler)                                */
/*  Called after a complete frame has been received.                          */
/*  Logic is identical to the Arduino version.                               */
/* ========================================================================= */
static void profibus_RX(void)
{
    uint8_t cnt;
    uint8_t telegramm_type;
    uint8_t process_data;

    uint8_t destination_add;
    uint8_t source_add;
    uint8_t function_code;
    uint8_t FCS_data;
    uint8_t PDU_size;
    uint8_t DSAP_data;
    uint8_t SSAP_data;

    process_data = 0;
    telegramm_type = uart_buffer[0];

    switch (telegramm_type)
    {
        case SD1:  /* Telegram without data, max. 6 bytes */
            if (uart_byte_cnt != 6) break;
            destination_add = uart_buffer[1];
            source_add      = uart_buffer[2];
            function_code   = uart_buffer[3];
            FCS_data        = uart_buffer[4];
            if (addmatch(destination_add) == 0) break;
            if (checksum(&uart_buffer[1], 3) != FCS_data) break;
            process_data = 1;
            break;

        case SD2:  /* Telegram with variable data length */
            if (uart_byte_cnt != uart_buffer[1] + 6) break;
            PDU_size        = uart_buffer[1];
            destination_add = uart_buffer[4];
            source_add      = uart_buffer[5];
            function_code   = uart_buffer[6];
            FCS_data        = uart_buffer[PDU_size + 4];
            if (addmatch(destination_add) == 0) break;
            if (checksum(&uart_buffer[4], PDU_size) != FCS_data) break;
            process_data = 1;
            break;

        case SD3:  /* Telegram with 5 bytes data, max. 11 bytes */
            if (uart_byte_cnt != 11) break;
            PDU_size        = 8;
            destination_add = uart_buffer[1];
            source_add      = uart_buffer[2];
            function_code   = uart_buffer[3];
            FCS_data        = uart_buffer[9];
            if (addmatch(destination_add) == 0) break;
            if (checksum(&uart_buffer[1], 8) != FCS_data) break;
            process_data = 1;
            break;

        case SD4:  /* Token with 3 Byte Data */
            if (uart_byte_cnt != 3) break;
            destination_add = uart_buffer[1];
            source_add      = uart_buffer[2];
            if (addmatch(destination_add) == 0) break;
            break;

        default:
            break;
    }

    /* Process valid data */
    if (process_data)
    {
        master_addr = source_add;
        last_valid_msg_time = millis(); // Valid message received

        /* Service Access Point detected? */
        if ((destination_add & 0x80) && (source_add & 0x80))
        {
            DSAP_data = uart_buffer[7];
            SSAP_data = uart_buffer[8];

            switch (DSAP_data)
            {
                case SAP_SET_SLAVE_ADR:  /* Set Slave Address (SSAP 62 -> DSAP 55) */
                    slave_addr = uart_buffer[9];
                    profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
                    break;

                case SAP_GLOBAL_CONTROL:  /* Global Control (SSAP 62 -> DSAP 58) */
                    if (uart_buffer[9] & CLEAR_DATA_)
                    {
                        LED_ERROR_ON();   /* PLC not ready */
                    }
                    else
                    {
                        LED_ERROR_OFF();  /* PLC OK */
                    }

                    /* Calculate group */
                    for (cnt = 0; uart_buffer[10] != 0; cnt++)
                        uart_buffer[10] >>= 1;

                    if (cnt == group)
                    {
                        if (uart_buffer[9] & UNFREEZE_)
                        {
                            /* Delete FREEZE state */
                        }
                        else if (uart_buffer[9] & UNSYNC_)
                        {
                            /* Delete SYNC state */
                        }
                        else if (uart_buffer[9] & FREEZE_)
                        {
                            /* Do not read inputs again */
                        }
                        else if (uart_buffer[9] & SYNC_)
                        {
                            /* Set outputs only with SYNC command */
                        }
                    }
                    break;

                case SAP_SLAVE_DIAGNOSIS:  /* Get Diagnostics (SSAP 62 -> DSAP 60) */
                    if (function_code == (REQUEST_ + FCB_ + SRD_HIGH))
                    {
                        uart_buffer[7]  = SSAP_data;
                        uart_buffer[8]  = DSAP_data;
                        uart_buffer[9]  = STATION_NOT_READY_;
                        uart_buffer[10] = STATUS_2_DEFAULT + PRM_REQ_;
                        uart_buffer[11] = DIAG_SIZE_OK;
                        uart_buffer[12] = MASTER_ADD_DEFAULT;
                        uart_buffer[13] = IDENT_HIGH_BYTE;
                        uart_buffer[14] = IDENT_LOW_BYTE;
                        #if (EXT_DIAG_DATA_SIZE > 0)
                        uart_buffer[15] = EXT_DIAG_DATA_SIZE;
                        for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
                            uart_buffer[16 + cnt] = Diag_Data[cnt];
                        #endif
                        profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET,
                                          &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
                    }
                    else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
                             function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
                    {
                        uart_buffer[7]  = SSAP_data;
                        uart_buffer[8]  = DSAP_data;
                        if (diagnose_status)
                            uart_buffer[9] = EXT_DIAG_;
                        else
                            uart_buffer[9] = 0x00;
                        uart_buffer[10] = STATUS_2_DEFAULT;
                        uart_buffer[11] = DIAG_SIZE_OK;
                        uart_buffer[12] = master_addr - SAP_OFFSET;
                        uart_buffer[13] = IDENT_HIGH_BYTE;
                        uart_buffer[14] = IDENT_LOW_BYTE;
                        #if (EXT_DIAG_DATA_SIZE > 0)
                        uart_buffer[15] = EXT_DIAG_DATA_SIZE;
                        for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
                            uart_buffer[16 + cnt] = Diag_Data[cnt];
                        #endif
                        profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET,
                                          &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
                    }
                    break;

                case SAP_SET_PRM:  /* Set Parameters (SSAP 62 -> DSAP 61) */
                    if ((uart_buffer[13] == IDENT_HIGH_BYTE) &&
                        (uart_buffer[14] == IDENT_LOW_BYTE))
                    {
                        Vendor_Data_size = PDU_size - 12;
                        #if (VENDOR_DATA_SIZE > 0)
                        for (cnt = 0; cnt < Vendor_Data_size; cnt++)
                            Vendor_Data[cnt] = uart_buffer[16 + cnt];
                        #endif
                        for (group = 0; uart_buffer[15] != 0; group++)
                            uart_buffer[15] >>= 1;
                        profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
                        dp_state = DP_STATE_WAIT_CFG;
                    }
                    break;

                case SAP_CHK_CFG:  /* Check Config (SSAP 62 -> DSAP 62) */
                    Output_Data_size = 0;
                    Input_Data_size = 0;
                    for (cnt = 0; cnt < uart_buffer[1] - 5; cnt++)
                    {
                        switch (uart_buffer[9 + cnt] & CFG_DIRECTION_)
                        {
                            case CFG_INPUT:
                                Input_Data_size += (uart_buffer[9 + cnt] & CFG_BYTE_CNT_) + 1;
                                if (uart_buffer[9 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                    Input_Data_size += Input_Data_size * 2;
                                break;

                            case CFG_OUTPUT:
                                Output_Data_size += (uart_buffer[9 + cnt] & CFG_BYTE_CNT_) + 1;
                                if (uart_buffer[9 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                    Output_Data_size += Output_Data_size * 2;
                                break;

                            case CFG_INPUT_OUTPUT:
                                Input_Data_size += (uart_buffer[9 + cnt] & CFG_BYTE_CNT_) + 1;
                                Output_Data_size += (uart_buffer[9 + cnt] & CFG_BYTE_CNT_) + 1;
                                if (uart_buffer[9 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                {
                                    Input_Data_size += Input_Data_size * 2;
                                    Output_Data_size += Output_Data_size * 2;
                                }
                                break;

                            case CFG_SPECIAL:
                                if (uart_buffer[9 + cnt] & CFG_SP_VENDOR_CNT_)
                                {
                                    Vendor_Data_size = uart_buffer[9 + cnt] & CFG_SP_VENDOR_CNT_;
                                    uart_buffer[1] -= Vendor_Data_size;
                                }
                                switch (uart_buffer[9 + cnt] & CFG_SP_DIRECTION_)
                                {
                                    case CFG_SP_VOID:
                                        break;
                                    case CFG_SP_INPUT:
                                        Input_Data_size += (uart_buffer[10 + cnt] & CFG_SP_BYTE_CNT_) + 1;
                                        if (uart_buffer[10 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                            Input_Data_size += Input_Data_size * 2;
                                        cnt++;
                                        break;
                                    case CFG_SP_OUTPUT:
                                        Output_Data_size += (uart_buffer[10 + cnt] & CFG_SP_BYTE_CNT_) + 1;
                                        if (uart_buffer[10 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                            Output_Data_size += Output_Data_size * 2;
                                        cnt++;
                                        break;
                                    case CFG_SP_INPUT_OPTPUT:
                                        Output_Data_size += (uart_buffer[10 + cnt] & CFG_SP_BYTE_CNT_) + 1;
                                        if (uart_buffer[10 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                            Output_Data_size += Output_Data_size * 2;
                                        Input_Data_size += (uart_buffer[11 + cnt] & CFG_SP_BYTE_CNT_) + 1;
                                        if (uart_buffer[11 + cnt] & CFG_WIDTH_ & CFG_WORD)
                                            Input_Data_size += Input_Data_size * 2;
                                        cnt += 2;
                                        break;
                                }
                                break;

                            default:
                                Input_Data_size = 0;
                                Output_Data_size = 0;
                                break;
                        }
                    }

                    if (Vendor_Data_size != 0)
                    {
                        /* Evaluate vendor data if needed */
                    }

                    profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
                    dp_state = DP_STATE_DATA_EXCHANGE;
                    break;

                default:
                    break;
            }
        }
        /* Destination: Slave address (data exchange) */
        else if (destination_add == slave_addr)
        {
            /* Status query */
            if (function_code == (REQUEST_ + FDL_STATUS))
            {
                profibus_send_CMD(SD1, FDL_STATUS_OK, 0, &uart_buffer[0], 0);
            }
            /* Data exchange (Send and Request Data) */
            else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
                     function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
            {
                /* Read data from master */
                #if (INPUT_DATA_SIZE > 0)
                for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
                {
                    Profibus_in_register[cnt] = uart_buffer[cnt + 7];
                    new_data = 1;
                }
                #endif

                /* Write data for master into buffer */
                #if (OUTPUT_DATA_SIZE > 0)
                for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
                {
                    uart_buffer[cnt + 7] = Profibus_out_register[cnt];
                }
                #endif

                #if (OUTPUT_DATA_SIZE > 0)
                if (diagnose_status)
                    profibus_send_CMD(SD2, DIAGNOSE, 0, &uart_buffer[7], 0);
                else
                    profibus_send_CMD(SD2, DATA_LOW, 0, &uart_buffer[7], Input_Data_size);
                #else
                if (diagnose_status)
                    profibus_send_CMD(SD1, DIAGNOSE, 0, &uart_buffer[7], 0);
                else
                    profibus_send_CMD(SC, 0, 0, &uart_buffer[7], 0);
                #endif
            }
        }
    }
}

/* ========================================================================= */
/*  Profibus Send Command (build and send telegram)                          */
/* ========================================================================= */
static void profibus_send_CMD(uint8_t type, uint8_t function_code,
                              uint8_t sap_offset, uint8_t *pdu,
                              uint8_t length_pdu)
{
    uint8_t length_data = 0;

    switch (type)
    {
        case SD1:
            uart_buffer[0] = SD1;
            uart_buffer[1] = master_addr;
            uart_buffer[2] = slave_addr + sap_offset;
            uart_buffer[3] = function_code;
            uart_buffer[4] = checksum(&uart_buffer[1], 3);
            uart_buffer[5] = ED;
            length_data = 6;
            break;

        case SD2:
            uart_buffer[0] = SD2;
            uart_buffer[1] = length_pdu + 3;
            uart_buffer[2] = length_pdu + 3;
            uart_buffer[3] = SD2;
            uart_buffer[4] = master_addr;
            uart_buffer[5] = slave_addr + sap_offset;
            uart_buffer[6] = function_code;
            uart_buffer[7 + length_pdu] = checksum(&uart_buffer[4], length_pdu + 3);
            uart_buffer[8 + length_pdu] = ED;
            length_data = length_pdu + 9;
            break;

        case SD3:
            uart_buffer[0] = SD3;
            uart_buffer[1] = master_addr;
            uart_buffer[2] = slave_addr + sap_offset;
            uart_buffer[3] = function_code;
            uart_buffer[9] = checksum(&uart_buffer[4], 8);
            uart_buffer[10] = ED;
            length_data = 11;
            break;

        case SD4:
            uart_buffer[0] = SD4;
            uart_buffer[1] = master_addr;
            uart_buffer[2] = slave_addr + sap_offset;
            length_data = 3;
            break;

        case SC:
            uart_buffer[0] = SC;
            length_data = 1;
            break;

        default:
            break;
    }

    profibus_TX(&uart_buffer[0], length_data);
}

/* ========================================================================= */
/*  Profibus Transmit                                                        */
/*  Switch to TX mode, start sending via interrupt-driven TXE                */
/* ========================================================================= */
static void profibus_TX(uint8_t *data, uint8_t length)
{
    /*
     * Before transmitting, briefly disable USART RX DMA to prevent
     * receiving our own echo on half-duplex RS485 bus
     */

    /* Stop DMA reception temporarily */
    DMA2_Stream2->CR &= ~DMA_CR_EN;

    /* Switch MAX485 to transmit */
    TX_ENABLE_ON();

    /* Set up transmission state */
    profibus_status = PROFIBUS_SEND_DATA;
    uart_byte_cnt = length;
    uart_transmit_cnt = 0;

    /* Start TX timeout timer */
    timer_start(TIMEOUT_TX);

    /* Send first byte and enable TXE interrupt for the rest */
    USART1->DR = uart_buffer[0];
    uart_transmit_cnt = 1;

    /* Enable TXE interrupt to send remaining bytes */
    if (length > 1)
    {
        USART1->CR1 |= USART_CR1_TXEIE;
    }
    else
    {
        /* Only 1 byte: enable TC interrupt to detect completion */
        USART1->CR1 |= USART_CR1_TCIE;
    }
}

/* ========================================================================= */
/*  Finish TX: switch back to receive mode                                   */
/* ========================================================================= */
static void finish_tx(void)
{
    /* Wait for TC (transmission complete) flag */
    /* This ensures the last byte has fully shifted out */

    /* Switch MAX485 back to receive */
    TX_ENABLE_OFF();

    /* Re-enable DMA reception */
    DMA2_Stream2->CR &= ~DMA_CR_EN;
    while (DMA2_Stream2->CR & DMA_CR_EN) {}
    DMA2->LIFCR = (0x3FU << 16);  /* Clear Stream 2 flags */
    DMA2_Stream2->NDTR = DMA_RX_BUF_SIZE;
    dma_rx_prev_pos = 0;
    DMA2_Stream2->CR |= DMA_CR_EN;

    /* Read SR+DR to clear any stale flags */
    (void)USART1->SR;
    (void)USART1->DR;

    /* Restart Profibus state machine with TSYN wait */
    profibus_status = PROFIBUS_WAIT_SYN;
    timer_start(TIMEOUT_SYN);
}

/* ========================================================================= */
/*  Process received DMA data (called from IDLE interrupt)                   */
/*  Extracts bytes from circular DMA buffer into uart_buffer                 */
/* ========================================================================= */
static void process_dma_rx(void)
{
    uint16_t pos;
    uint16_t count;
    uint16_t i;

    /* Current DMA write position */
    pos = DMA_RX_BUF_SIZE - DMA2_Stream2->NDTR;

    if (pos == dma_rx_prev_pos)
        return;  /* No new data */

    /* Calculate number of new bytes (handling circular wrap) */
    if (pos > dma_rx_prev_pos)
    {
        count = pos - dma_rx_prev_pos;
    }
    else
    {
        count = DMA_RX_BUF_SIZE - dma_rx_prev_pos + pos;
    }

    /* Copy to uart_buffer */
    if (profibus_status == PROFIBUS_WAIT_DATA ||
        profibus_status == PROFIBUS_GET_DATA)
    {
        for (i = 0; i < count && uart_byte_cnt < MAX_BUFFER_SIZE; i++)
        {
            uart_buffer[uart_byte_cnt++] =
                dma_rx_buf[(dma_rx_prev_pos + i) % DMA_RX_BUF_SIZE];
        }

        if (profibus_status == PROFIBUS_WAIT_DATA)
        {
            profibus_status = PROFIBUS_GET_DATA;
        }

        /* Restart the inter-byte timeout */
        timer_restart(TIMEOUT_RX);
    }

    dma_rx_prev_pos = pos;
}

/* ========================================================================= */
/*  Interrupt Service Routines                                               */
/* ========================================================================= */

/* SysTick Handler (1 ms tick) */
void SysTick_Handler(void)
{
    systick_ms++;
}

/*
 * TIM2 IRQ Handler - Profibus Timing State Machine
 * This is the direct equivalent of the Arduino TIMER1_COMPA ISR.
 * Timer runs in one-pulse mode, so it stops itself after the interrupt.
 */
void TIM2_IRQHandler(void)
{
    if (!(TIM2->SR & TIM_SR_UIF))
        return;

    TIM2->SR = 0;  /* Clear interrupt flag */

    switch (profibus_status)
    {
        case PROFIBUS_WAIT_SYN:
            /* TSYN expired: now ready to accept data */
            profibus_status = PROFIBUS_WAIT_DATA;
            uart_byte_cnt = 0;
            timer_start(TIMEOUT_SDR);
            break;

        case PROFIBUS_WAIT_DATA:
            /* TSDR expired but no data received - just restart */
            timer_start(TIMEOUT_SDR);
            break;

        case PROFIBUS_GET_DATA:
            /* Inter-byte timeout expired after receiving data -> process frame */
            profibus_status = PROFIBUS_WAIT_SYN;
            /*
             * We've already waited TIMEOUT_RX of bus idle.
             * Subtract that from TSYN for the next cycle.
             */
            {
                uint32_t remaining = 0;
                if (TIMEOUT_SYN > TIMEOUT_RX)
                    remaining = TIMEOUT_SYN - TIMEOUT_RX;
                else
                    remaining = 1;  /* Minimum 1 µs */
                timer_start(remaining);
            }
            profibus_RX();  /* Process the received frame */
            break;

        case PROFIBUS_SEND_DATA:
            /* TX timeout - something went wrong, return to receive */
            finish_tx();
            break;

        default:
            break;
    }
}

/*
 * USART1 IRQ Handler
 * Handles:
 *   - IDLE line detection (DMA frame completion)
 *   - TXE (TX buffer empty - send next byte)
 *   - TC (TX complete - last byte shifted out)
 */
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    /* ---- IDLE line detected ---- */
    if (sr & USART_SR_IDLE)
    {
        /* Clear IDLE flag by reading SR then DR */
        (void)USART1->SR;
        (void)USART1->DR;

        /* Only process if we're in receive mode */
        if (profibus_status == PROFIBUS_WAIT_DATA ||
            profibus_status == PROFIBUS_GET_DATA)
        {
            process_dma_rx();
        }
    }

    /* ---- TXE: transmit data register empty ---- */
    if ((sr & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
    {
        if (uart_transmit_cnt < uart_byte_cnt)
        {
            USART1->DR = uart_buffer[uart_transmit_cnt++];

            /* If this was the last byte, disable TXE and enable TC */
            if (uart_transmit_cnt >= uart_byte_cnt)
            {
                USART1->CR1 &= ~USART_CR1_TXEIE;  /* Disable TXE interrupt */
                USART1->CR1 |= USART_CR1_TCIE;     /* Enable TC interrupt */
            }
        }
        else
        {
            /* Shouldn't get here, but disable TXE just in case */
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }

        /* Reset timer on each byte sent */
        timer_restart(TIMEOUT_TX);
    }

    /* ---- TC: transmission complete (last byte fully shifted out) ---- */
    if ((sr & USART_SR_TC) && (USART1->CR1 & USART_CR1_TCIE))
    {
        USART1->CR1 &= ~USART_CR1_TCIE;  /* Disable TC interrupt */
        USART1->SR &= ~USART_SR_TC;       /* Clear TC flag */
        finish_tx();                        /* Switch back to RX mode */
    }

    /* ---- Error flags: clear and discard ---- */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_PE | USART_SR_NE))
    {
        (void)USART1->SR;
        (void)USART1->DR;
    }
}

/* ========================================================================= */
/*  Main Application                                                         */
/* ========================================================================= */
int main(void)
{
    /* Disable interrupts during init */
    __asm volatile ("cpsid i");

    SystemClock_Config();
    GPIO_Init();
    SysTick_Init();

    init_Profibus();
    TX_ENABLE_OFF();  /* Ensure we start in receive mode */

    /* Enable interrupts — SysTick must be running before delay_ms() works */
    __asm volatile ("cpsie i");

    /* I2C / OLED init AFTER interrupts enabled so SysTick ticks and
     * delay_ms() works correctly. Delay 100 ms for OLED VDD to stabilise
     * before sending the init command sequence. */
    I2C1_Init();
    delay_ms(100);
    SSD1306_Init();

    /* ---- Main loop ---- */
    uint32_t last_input_check    = 0;
    uint32_t last_display_update = 0;
    char oled_buf[32];

    while (1)
    {
        uint32_t current_time = millis();

        // Check for Profibus timeout (e.g. 1500 ms without valid telegram)
        if (dp_state != DP_STATE_OFFLINE && (current_time - last_valid_msg_time > 1500)) {
            dp_state = DP_STATE_OFFLINE;
        }

        // Display update (every 100ms prevents OLED flickering or hogging CPU)
        if (current_time - last_display_update > 100) {
            last_display_update = current_time;

            SSD1306_Fill(0);
            
            SSD1306_GotoXY(0, 0);
            SSD1306_Puts(" Profibus Node", 1);
            
            SSD1306_GotoXY(0, 15);
            snprintf(oled_buf, sizeof(oled_buf), " Addr: %d", slave_addr);
            SSD1306_Puts(oled_buf, 1);

            SSD1306_GotoXY(0, 30);
            SSD1306_Puts(" State:", 1);

            SSD1306_GotoXY(10, 45);
            switch(dp_state) {
                case DP_STATE_OFFLINE:
                    SSD1306_Puts("OFFLINE / WAIT", 1);
                    break;
                case DP_STATE_WAIT_PRM:
                    SSD1306_Puts("WAIT PRM", 1);
                    break;
                case DP_STATE_WAIT_CFG:
                    SSD1306_Puts("WAIT CFG", 1);
                    break;
                case DP_STATE_DATA_EXCHANGE:
                    SSD1306_Puts("DATA EXCHANGE", 1);
                    break;
            }

            SSD1306_UpdateScreen();
        }

        /* Poll touch button input (every ~1 ms) */
        if ((millis() - last_input_check) >= 1)
        {
            if (GPIO_READ(INPUT_PORT, INPUT_PIN) == 0)
            {
                Profibus_out_register[0] = 0x01;
            }
            else
            {
                Profibus_out_register[0] = 0x00;
            }
            last_input_check = millis();
        }

        /* Process data from master */
        if (new_data == 1)
        {
            Profibus_out_register[0] += new_data;
            new_data = 0;
            Profibus_out_register[0] = Profibus_in_register[0];
        }

        /* Update user LED based on master data */
        if ((Profibus_in_register[0] & 0x01) == 0)
            LED_USER_ON();
        else
            LED_USER_OFF();
    }

    return 0;  /* Never reached */
}
