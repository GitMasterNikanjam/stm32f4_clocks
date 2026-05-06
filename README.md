# `stm32f4_clocks`  
### Ultra‑lightweight STM32F4 Clock Computation Helpers (CMSIS‑only, no HAL)

`stm32f4_clocks` is a **zero‑overhead**, **header‑only**, **constexpr‑friendly** collection of utilities for computing all major STM32F4 clock frequencies directly from RCC registers.  
This library targets developers who:

- Do **not** use the STM32 HAL  
- Need **true runtime clock values** from the silicon  
- Want **fast**, **inline**, **dependency-free** clock helpers  
- Need reliable access to bus, peripheral, PLL, USB, I2S, ADC, RNG, and timer clocks  

All functions are `constexpr inline`, require **only CMSIS**, and read the actual MCU registers.  
Tested on STM32F407 and compatible across most STM32F4 devices.

---

# Features

- System clock decoding (`SYSCLK`)
- AHB / APB1 / APB2 prescaler decoding  
- Timer clock doubling logic for APB domains  
- Full PLL chain reconstruction:
  - Main PLL (PLLM, PLLN, PLLP, PLLQ)
  - PLLI2S for I2S (N/R)
- Peripheral kernel clocks:
  - SPI1/2/3
  - I2S (SPI2/3)
  - USART1/2/3/6 + UART4/5/7/8
  - I2C1/2/3
  - CAN1/2
- Special domains:
  - ADC common clock
  - SDIO, USB FS, RNG clock (PLL48)
- Single‑call aggregated snapshot of all clocks

---

# Why This Library?

The STM32F4 clock tree is powerful but complex.  
Without HAL, developers must manually decode RCC->CFGR, PLLCFGR, prescalers, and peripheral routing rules.

This header eliminates that burden:

- No HAL
- No external dependencies
- No runtime allocations
- No function call overhead  
- Inlined, constexpr-safe, readable, and robust  
- Accurate across all configurations (HSI, HSE, PLL, prescalers, I2S PLL, etc.)

---

# Installation

Simply drop this file into your CMSIS-based project:

```
stm32f4_clocks.h
```

Include it where needed:

```cpp
#include "stm32f4_clocks.h"
using namespace stm32::clocks;
```

---

# Fundamentals of STM32F4 Clocking  
*A high‑resolution explanation designed for firmware engineers.*

Understanding STM32F4 clocks requires tracking several layers:

## 1. System Clock Source (SYSCLK)
STM32F4 MCU can run the system clock from:

- HSI (Internal 16 MHz RC)
- HSE (External crystal/oscillator)
- PLL (Main system PLL)

`sysclk_hz()` determines the active source using `RCC->CFGR.SWS`.

### If PLL is used:
The final SYSCLK is:

```
VCO_in  = FIN / PLLM  
VCO     = VCO_in * PLLN  
SYSCLK  = VCO / PLLP  
```

Where:
- FIN = HSI or HSE based on PLLSRC  
- PLLM ∈ [2..63]  
- PLLN ∈ [192..432]  
- PLLP ∈ {2,4,6,8}

The library's `decode_pllp()` transparently converts the bit pattern.

---

## 2. AHB Clock (HCLK)

HCLK is derived by dividing SYSCLK:

```
HCLK = SYSCLK / AHB_prescaler
```

The library decodes all valid prescalers (1,2,4,8,...512).

---

## 3. APB Bus Clocks (PCLK1 / PCLK2)

APB1 (low-speed) and APB2 (high-speed) are derived from HCLK:

```
PCLK1 = HCLK / APB1_prescaler
PCLK2 = HCLK / APB2_prescaler
```

Prescalers: 1,2,4,8,16

---

## 4. Timer Clock Doubling
When an APB prescaler is **greater than 1**, timer clocks
(TIM2/3/4/5 on APB1, TIM1/8/9... on APB2) get **multiplied by 2**:

```
TIMxCLK = PCLKx × 2     if APB prescaler > 1  
TIMxCLK = PCLKx         otherwise
```

Handled by:

- `tim_apb1_hz()`
- `tim_apb2_hz()`

---

## 5. PLL48 Domain (USB / SDIO / RNG)
STM32F4 requires an exact 48 MHz clock for several peripherals.

```
PLL48 = VCO / PLLQ
```

Used by USB FS, SDIO, RNG.

Provided via `pll48_hz()`.

---

## 6. I2S Clock via PLLI2S
Some devices include a separate PLL for audio/I2S:

```
PLLI2S_VCO = (FIN / PLLM) * PLLI2SN  
I2SCLK     = PLLI2S_VCO / PLLI2SR
```

Handled seamlessly by:

- `plli2s_vco_hz()`
- `plli2s_i2sclk_hz()`

---

# API Overview

### Core Clock Queries
```cpp
uint32_t sys   = sysclk_hz();
uint32_t hclk  = hclk_hz();
uint32_t p1    = pclk1_hz();
uint32_t p2    = pclk2_hz();
uint32_t t1    = tim_apb1_hz();
uint32_t t2    = tim_apb2_hz();
```

### Peripheral Clock Queries
```cpp
spi_kernel_hz(SPI1);
usart_kernel_hz(USART2);
i2c_kernel_hz(I2C3);
can_kernel_hz(CAN1);
adc_common_hz();
```

### PLL Special Domains
```cpp
pll48_hz();       // USB, SDIO, RNG
plli2s_i2sclk_hz();
```

---

# Clock Snapshot

The most powerful feature: retrieve ALL clocks at once.

```cpp
auto snap = snapshot();
```

Structure:

```cpp
struct ClockSnapshot {
    uint32_t sysclk, hclk, pclk1, pclk2;
    uint32_t tim_apb1, tim_apb2;
    uint32_t pll48;
    uint32_t spi1_kernel, spi2_kernel, spi3_kernel;
    uint32_t usart1_kernel, usart2_kernel, usart3_kernel;
    uint32_t uart4_kernel, uart5_kernel, usart6_kernel;
    uint32_t i2c1_kernel, i2c2_kernel, i2c3_kernel;
    uint32_t can1_kernel, can2_kernel;
    uint32_t adc_common;
    uint32_t sdio, usbfs, rng;
    uint32_t i2sclk;
};
```

Example usage:

```cpp
#include "stm32f4_clocks.h"

void debug_print_clocks() {
    using namespace stm32::clocks;
    auto s = snapshot();
    // Print fields through your favorite debug interface
}
```

---

# Example: Computing an SPI Clock

```cpp
uint32_t fker = spi_kernel_hz(SPI1);
uint32_t fsck = spi_sck_hz(SPI1);
// fsck respects CR1->BR[5:3] prescaler bits
```

---

# Supported Devices

Designed for:

- STM32F405
- STM32F407
- STM32F415
- STM32F417
- STM32F427
- STM32F429
- STM32F437
- STM32F439

Any STM32F4 device with compatible RCC layout will work.

---

# License

MIT  
Copyright (c) Mohammad

---

# Contributing

Contributions, corrections, and device‑port extensions are welcome.

---