# STM32F411CEU6 Profibus DP Slave

Bare-metal Profibus DP slave implementation for the **STM32F411CEU6** (BlackPill) microcontroller, ported from the Arduino (ATmega328P) version. Supports baud rates up to **1.5 Mbps** for communication with Siemens S7-300/400 PLCs.

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| STM32F411CEU6 | "BlackPill" development board (25 MHz crystal) |
| MAX485 module | RS-485 transceiver (for testing/prototyping) |
| Profibus cable | Standard Profibus DP cable with connectors |
| ST-Link V2 | For programming (or use built-in USB DFU bootloader) |

> **Note**: For industrial-grade reliability, replace MAX485 with a dedicated Profibus transceiver (e.g., MAX3463, SN65HVD1176).

## Wiring Diagram

```
STM32F411 BlackPill          MAX485 Module         Profibus Bus
┌──────────────┐          ┌──────────────┐      ┌──────────────┐
│         PA9  ├────────►│ DI           │      │              │
│         PA10 │◄────────┤ RO           │      │              │
│         PA1  ├────────►│ DE + RE      │      │              │
│              │         │           A  ├──────┤ A (Green)    │
│              │         │           B  ├──────┤ B (Red)      │
│         GND  ├─────────┤ GND     GND  ├──────┤ Shield/GND   │
│         3V3  ├─────────┤ VCC          │      │              │
└──────────────┘          └──────────────┘      └──────────────┘

Touch Button (optional):
  PB10 → VCC (button power)
  PB2  → GND (button ground)
  PB1  ← Signal input

LEDs:
  PC13 → Error LED (built-in, active LOW)
  PB0  → User LED
```

## Configuration

Edit these `#define` values in `main.c`:

```c
#define HSE_VALUE        25000000U   // Crystal: 25 MHz or 8 MHz
#define PROFIBUS_BAUD    1500000U    // 9600 to 1500000
#define SLAVE_ADDRESS    6           // 1-125
#define INPUT_DATA_SIZE  16          // Bytes from master
#define OUTPUT_DATA_SIZE 16          // Bytes to master
```

## Building

### Option 1: Makefile (arm-none-eabi-gcc)

```bash
# Install ARM toolchain first:
# Windows: https://developer.arm.com/downloads/-/gnu-rm
# Linux:   sudo apt install gcc-arm-none-eabi

make
make flash          # Flash via ST-Link
make flash-dfu      # Flash via USB DFU bootloader
```

### Option 2: PlatformIO (recommended for beginners)

```bash
pip install platformio

pio run                                      # Build
pio run --target upload                      # Flash via ST-Link
pio run --target upload -e blackpill_dfu     # Flash via DFU
```

### Option 3: STM32CubeIDE

1. Create new STM32 project for STM32F411CEU6
2. Replace the generated `main.c` with this project's `main.c`
3. Replace the startup file and linker script
4. Build and flash

## Flashing via DFU (No ST-Link needed)

The BlackPill has a built-in USB DFU bootloader:

1. Hold the **BOOT0** button
2. Press and release **RESET**
3. Release **BOOT0**
4. The board appears as a DFU device on USB
5. Run `make flash-dfu` or `pio run --target upload -e blackpill_dfu`

## GSD File

Import `STM32F411_Profibus.gsd` into your STEP 7 / TIA Portal hardware catalog. This file declares the supported baud rates and I/O module configurations.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  STM32F411CEU6  (96 MHz Cortex-M4)                  │
│                                                     │
│  ┌─────────┐   ┌──────────┐   ┌────────────────┐   │
│  │ USART1  │──►│ DMA2 S2  │──►│ Circular RX    │   │
│  │ 1.5Mbps │   │ (auto)   │   │ Buffer (64B)   │   │
│  │ 8E1     │   └──────────┘   └────────┬───────┘   │
│  │         │                           │            │
│  │  IDLE ──┼─── IRQ ──────────────────►│            │
│  │  TXE  ──┼─── IRQ (TX byte-by-byte) │            │
│  │  TC   ──┼─── IRQ (TX complete)      ▼            │
│  └─────────┘              ┌────────────────────┐    │
│                           │  Profibus Protocol │    │
│  ┌─────────┐              │  State Machine     │    │
│  │  TIM2   │── IRQ ──────►│  (TSYN/TSDR)       │    │
│  │ 32-bit  │              └────────────────────┘    │
│  │ 1µs res │                                        │
│  └─────────┘    ┌──────┐  ┌──────┐  ┌──────┐       │
│                 │ PA1  │  │ PC13 │  │ PB1  │       │
│                 │TX_EN │  │ LED  │  │INPUT │       │
│                 └──┬───┘  └──────┘  └──────┘       │
└────────────────────┼────────────────────────────────┘
                     ▼
              ┌──────────────┐
              │   MAX485     │
              │  DE/RE=PA1   │
              └──────┬───────┘
                     │ RS-485
              ┌──────┴───────┐
              │ Profibus Bus │
              └──────────────┘
```

## Key Differences from Arduino Version

| Feature | Arduino (ATmega328P) | STM32F411CEU6 |
|---------|---------------------|---------------|
| Clock | 16 MHz | 96 MHz |
| Max baud | ~500 kbps | **1.5 Mbps** |
| UART RX | Byte-by-byte ISR | **DMA + IDLE detection** |
| UART TX | UDR empty ISR | TXE + TC ISR |
| Timer | Timer1 (16-bit) | **TIM2 (32-bit)** |
| Timer resolution | 0.5 µs (@ /8) | **1 µs** |
| GPIO toggle | Port manipulation | **BSRR (atomic, 1 cycle)** |
| RAM | 2 KB | **128 KB** |
| Flash | 32 KB | **512 KB** |

## Supported Baud Rates

| Baud Rate | USARTDIV | Error | Status |
|-----------|----------|-------|--------|
| 9,600 | 10000 | 0.0% | ✅ Exact |
| 19,200 | 5000 | 0.0% | ✅ Exact |
| 45,450 | 2112 | 0.02% | ✅ OK |
| 93,750 | 1024 | 0.0% | ✅ Exact |
| 187,500 | 512 | 0.0% | ✅ Exact |
| 500,000 | 192 | 0.0% | ✅ Exact |
| **1,500,000** | **64** | **0.0%** | ✅ **Exact** |

## Troubleshooting

- **Slave not visible on Profibus**: Check DE/RE wiring on MAX485, verify slave address matches PLC config
- **Communication errors at 1.5M**: Use short cables (<100m), verify bus termination at both ends
- **LED stays on**: PLC is in STOP mode (CLEAR_DATA received)
- **No response after flash**: Verify 25 MHz crystal on your board, check HSE_VALUE define

## License

Same as the original Arduino_Profibus_DP project.
