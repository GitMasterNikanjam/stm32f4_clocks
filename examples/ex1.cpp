// Example: report the active STM32F4 clock tree over a retargeted printf.
// Call print_all_clocks() after the system clock and peripheral setup is complete.
#include "stm32f4_clocks.h"
#include <cstdio>

namespace {

void print_clock(const char* name, uint32_t frequency_hz)
{
    const uint32_t whole_mhz = frequency_hz / 1000000u;
    const uint32_t fraction_khz = (frequency_hz % 1000000u) / 1000u;

    std::printf("%-10s %10lu Hz  (%lu.%03lu MHz)\r\n",
                name,
                static_cast<unsigned long>(frequency_hz),
                static_cast<unsigned long>(whole_mhz),
                static_cast<unsigned long>(fraction_khz));
}

} // namespace

void print_all_clocks()
{
    using namespace stm32::clocks;

    // Take one snapshot so related fields are collected together.
    const ClockSnapshot clocks = snapshot();

    std::printf("\r\nSTM32F4 clock report\r\n");
    std::printf("--------------------\r\n");

    print_clock("SYSCLK", clocks.sysclk);
    print_clock("HCLK", clocks.hclk);
    print_clock("PCLK1", clocks.pclk1);
    print_clock("PCLK2", clocks.pclk2);
    print_clock("TIM APB1", clocks.tim_apb1);
    print_clock("TIM APB2", clocks.tim_apb2);

    std::printf("\r\nPLL domains\r\n");
    print_clock("Main VCO", pll_vco_hz());
    print_clock("PLL48", clocks.pll48);
    print_clock("PLLI2S", clocks.i2sclk);

    std::printf("\r\nPeripheral clocks\r\n");
#if defined(SPI1_BASE)
    print_clock("SPI1 kernel", clocks.spi1_kernel);
    print_clock("SPI1 SCK", spi_sck_hz(SPI1));
#endif
#if defined(SPI2_BASE)
    print_clock("SPI2 kernel", clocks.spi2_kernel);
    print_clock("SPI2 SCK", spi_sck_hz(SPI2));
#endif
#if defined(SPI3_BASE)
    print_clock("SPI3 kernel", clocks.spi3_kernel);
    print_clock("SPI3 SCK", spi_sck_hz(SPI3));
#endif
#if defined(USART1_BASE)
    print_clock("USART1", clocks.usart1_kernel);
#endif
#if defined(USART2_BASE)
    print_clock("USART2", clocks.usart2_kernel);
#endif
#if defined(USART3_BASE)
    print_clock("USART3", clocks.usart3_kernel);
#endif
#if defined(UART4_BASE)
    print_clock("UART4", clocks.uart4_kernel);
#endif
#if defined(UART5_BASE)
    print_clock("UART5", clocks.uart5_kernel);
#endif
#if defined(USART6_BASE)
    print_clock("USART6", clocks.usart6_kernel);
#endif
#if defined(I2C1_BASE)
    print_clock("I2C1", clocks.i2c1_kernel);
#endif
#if defined(I2C2_BASE)
    print_clock("I2C2", clocks.i2c2_kernel);
#endif
#if defined(I2C3_BASE)
    print_clock("I2C3", clocks.i2c3_kernel);
#endif
#if defined(CAN1_BASE)
    print_clock("CAN1", clocks.can1_kernel);
#endif
#if defined(CAN2_BASE)
    print_clock("CAN2", clocks.can2_kernel);
#endif
    print_clock("ADC common", clocks.adc_common);
    print_clock("SDIO", clocks.sdio);
    print_clock("USB FS", clocks.usbfs);
    print_clock("RNG", clocks.rng);
}
