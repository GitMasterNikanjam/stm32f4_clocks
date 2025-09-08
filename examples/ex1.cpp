#include "stm32f4_clocks.hpp"
#include <cstdio>

void print_all_clocks()
{
    using namespace stm32::clocks;
    const auto c = snapshot();

    printf("SYSCLK   : %lu\r\n", (unsigned long)c.sysclk);
    printf("HCLK     : %lu\r\n", (unsigned long)c.hclk);
    printf("PCLK1    : %lu\r\n", (unsigned long)c.pclk1);
    printf("PCLK2    : %lu\r\n", (unsigned long)c.pclk2);
    printf("TIM APB1 : %lu\r\n", (unsigned long)c.tim_apb1);
    printf("TIM APB2 : %lu\r\n", (unsigned long)c.tim_apb2);
    printf("PLL48    : %lu\r\n", (unsigned long)c.pll48);
#if defined(SPI1_BASE)
    printf("SPI1 kern: %lu  SCK: %lu\r\n",
           (unsigned long)c.spi1_kernel, (unsigned long)spi_sck_hz(SPI1));
#endif
#if defined(SPI2_BASE)
    printf("SPI2 kern: %lu  SCK: %lu\r\n",
           (unsigned long)c.spi2_kernel, (unsigned long)spi_sck_hz(SPI2));
#endif
#if defined(SPI3_BASE)
    printf("SPI3 kern: %lu  SCK: %lu\r\n",
           (unsigned long)c.spi3_kernel, (unsigned long)spi_sck_hz(SPI3));
#endif
#if defined(USART1_BASE)
    printf("USART1   : %lu\r\n", (unsigned long)c.usart1_kernel);
#endif
#if defined(USART2_BASE)
    printf("USART2   : %lu\r\n", (unsigned long)c.usart2_kernel);
#endif
#if defined(USART3_BASE)
    printf("USART3   : %lu\r\n", (unsigned long)c.usart3_kernel);
#endif
#if defined(UART4_BASE)
    printf("UART4    : %lu\r\n", (unsigned long)c.uart4_kernel);
#endif
#if defined(UART5_BASE)
    printf("UART5    : %lu\r\n", (unsigned long)c.uart5_kernel);
#endif
#if defined(USART6_BASE)
    printf("USART6   : %lu\r\n", (unsigned long)c.usart6_kernel);
#endif
#if defined(I2C1_BASE)
    printf("I2C1     : %lu\r\n", (unsigned long)c.i2c1_kernel);
#endif
#if defined(I2C2_BASE)
    printf("I2C2     : %lu\r\n", (unsigned long)c.i2c2_kernel);
#endif
#if defined(I2C3_BASE)
    printf("I2C3     : %lu\r\n", (unsigned long)c.i2c3_kernel);
#endif
#if defined(CAN1_BASE)
    printf("CAN1     : %lu\r\n", (unsigned long)c.can1_kernel);
#endif
#if defined(CAN2_BASE)
    printf("CAN2     : %lu\r\n", (unsigned long)c.can2_kernel);
#endif
    printf("ADC clk  : %lu\r\n", (unsigned long)c.adc_common);
    printf("SDIO clk : %lu\r\n", (unsigned long)c.sdio);
    printf("USBFS clk: %lu\r\n", (unsigned long)c.usbfs);
    printf("RNG clk  : %lu\r\n", (unsigned long)c.rng);
    printf("I2S clk  : %lu\r\n", (unsigned long)c.i2sclk);
}
