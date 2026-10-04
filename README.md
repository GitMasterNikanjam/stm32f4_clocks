# stm32f4_clocks

`stm32f4_clocks` is a header-only C++ library for reading clock frequencies from STM32F4 RCC and peripheral registers. It uses CMSIS device definitions and does not depend on the STM32 HAL.

The helpers calculate frequencies at runtime from the active clock configuration. They are declared `static inline`; they are not compile-time (`constexpr`) calculations.

## What it reports

- SYSCLK, HCLK, PCLK1, and PCLK2
- APB1 and APB2 timer clocks, including the STM32 timer clock multiplier when the APB prescaler is greater than 1
- Main PLL VCO and PLL48 domain (used by USB FS, SDIO, and RNG where present)
- PLLI2S VCO and I2S clock on devices whose CMSIS headers expose the PLLI2S fields
- SPI kernel clocks and SPI SCK after the SPI baud-rate prescaler
- USART/UART, I2C, and CAN clocks
- ADC common clock
- SDIO, USB FS, and RNG clock helpers
- A `ClockSnapshot` containing the major clocks above

Peripheral helpers return the clock feeding the peripheral. They do not calculate protocol bit rates such as UART baud rate, I2C SCL, or CAN bitrate. `spi_sck_hz()` is the exception: it applies the SPI `CR1.BR` divider to the SPI kernel clock.

## Requirements

- An STM32F4 CMSIS device header that provides `stm32f4xx.h`
- `HSI_VALUE` and `HSE_VALUE` configured to match the MCU board and external oscillator
- The selected MCU device macro defined by the project before including its CMSIS header
- C++ (the API is in the `stm32::clocks` namespace)

The header is at `Libraries/stm32f4_clocks/src/stm32f4_clocks.h`. Add that directory to the compiler's include paths and include the `.h` file. This library only observes the clock tree; configure and enable oscillators, PLLs, and peripheral clocks in your startup code or HAL/CMSIS setup.

## Quick start

```cpp
#include "stm32f4_clocks.h"

using namespace stm32::clocks;

void inspect_clocks()
{
    const uint32_t system_hz = sysclk_hz();
    const uint32_t bus_hz = pclk1_hz();
    const uint32_t spi1_hz = spi_kernel_hz(SPI1);
    const uint32_t spi1_sck_hz = spi_sck_hz(SPI1);

    (void)system_hz;
    (void)bus_hz;
    (void)spi1_hz;
    (void)spi1_sck_hz;
}
```

For a read of all supported clock fields:

```cpp
const stm32::clocks::ClockSnapshot clocks = stm32::clocks::snapshot();
// Example fields: clocks.sysclk, clocks.hclk, clocks.pclk1, clocks.spi1_kernel
```

Frequencies are returned in hertz as `uint32_t`. A result of `0` can indicate an invalid or unavailable source, a zero divider, or a peripheral not exposed by the selected device header. Use the named wrappers when you prefer not to pass a peripheral instance:

```cpp
const uint32_t spi1_hz = stm32::clocks::spi1_kernel_hz();
const uint32_t uart2_hz = stm32::clocks::usart2_kernel_hz();
const uint32_t i2c1_hz = stm32::clocks::i2c1_kernel_hz();
```

## API reference

### Core and timer clocks

| Function | Meaning |
| --- | --- |
| `sysclk_hz()` | Active system clock source: HSI, HSE, or main PLL |
| `hclk_hz()` | SYSCLK divided by the AHB prescaler |
| `pclk1_hz()`, `pclk2_hz()` | HCLK divided by the APB1/APB2 prescaler |
| `tim_apb1_hz()`, `tim_apb2_hz()` | Timer clocks for the APB domains, with the x2 rule when the APB prescaler is not 1 |

### PLL and I2S clocks

| Function | Meaning |
| --- | --- |
| `pll_vco_hz()` | Main PLL VCO frequency |
| `pll48_hz()` | Main PLL VCO divided by PLLQ |
| `plli2s_vco_hz()` | PLLI2S VCO, when supported by the device header |
| `plli2s_i2sclk_hz()` | PLLI2S VCO divided by PLLI2SR; returns 0 when unavailable |

### Peripheral clocks

| Function | Meaning |
| --- | --- |
| `spi_kernel_hz(SPIx)`, `spi1_kernel_hz()` etc. | SPI peripheral kernel clock |
| `spi_sck_hz(SPIx)` | SPI kernel clock divided by the configured `CR1.BR` prescaler |
| `i2s_kernel_hz(SPIx)` | I2S clock for SPI2/SPI3 when the device supports it |
| `usart_kernel_hz(instance)`, named USART/UART wrappers | PCLK2 for USART1/6; PCLK1 for the supported PCLK1 UART/USART instances |
| `i2c_kernel_hz(instance)`, named I2C wrappers | I2C peripheral clock (PCLK1 on STM32F4) |
| `can_kernel_hz(instance)` | CAN peripheral clock (PCLK1 on STM32F4) |
| `adc_common_hz()` | PCLK2 divided by the ADC common prescaler |
| `sdio_clk_hz()`, `usb_fs_clk_hz()`, `rng_clk_hz()` | PLL48 frequency when the corresponding peripheral is available |
| `snapshot()` | Aggregate `ClockSnapshot` of core, timer, PLL, and supported peripheral clocks |

## How the calculations work

The active SYSCLK source is read from `RCC->CFGR.SWS`. For PLL SYSCLK, the header reads PLL source, `PLLM`, `PLLN`, and `PLLP` from `RCC->PLLCFGR`. The bus clocks then follow the configured AHB/APB prescalers. PLL48 is calculated as `VCO / PLLQ`; PLLI2S I2S clock is calculated as its VCO divided by `PLLI2SR`.

`HSI_VALUE` and `HSE_VALUE` come from the device configuration headers rather than being measured from the silicon. Their accuracy is therefore required for accurate calculated frequencies. These functions read live registers, so call them after the clock tree has been configured and stabilized.

## Device support and limitations

The header targets STM32F4 CMSIS register layouts and conditionally exposes peripherals based on definitions in the selected device header. The repository's example is for STM32F407. Other STM32F4 parts may have different peripheral sets and PLL limits; check the reference manual and datasheet for the exact part.

This is a clock-frequency reporting helper, not a clock configurator or validator. It does not check that PLL parameters are legal for the device, that a requested peripheral is enabled, or that the derived frequency is within that peripheral's operating limits. Device-specific PLLI2S and peripheral availability also depend on the CMSIS header. Invalid/unavailable queries generally return `0`.

## Example

See [`examples/ex1.cpp`](examples/ex1.cpp) for a `snapshot()`-based report of bus, PLL, and available peripheral clocks. It also reports SPI SCK after the configured baud-rate divider. The example uses `printf`; retarget its output to your debug console and call `print_all_clocks()` after clock setup.

## License

MIT. See the source header for the library's copyright notice.
