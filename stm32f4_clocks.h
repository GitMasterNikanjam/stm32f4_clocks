/**
 * @file    stm32f4_clocks.hpp
 * @brief   STM32F407 clock computation helpers (no HAL; CMSIS-only).
 * @author  You
 * @version 1.1
 * @license MIT
 *
 * @details
 *  Core:   SYSCLK, HCLK, PCLK1, PCLK2
 *  Timers: APB1/APB2 timer clocks (+2 rule), tim_kernel_hz(TIMx*)
 *  SPI:    kernel clock, SCK output freq (from BR)
 *  NEW:    PLL48CLK (USB/SDIO/RNG), USART/I2C/CAN kernels, ADC common clock,
 *          SDIO/USB FS/RNG clocks, PLLI2S & I2S kernel clock
 */

#ifndef STM32F4_CLOCKS_HPP
#define STM32F4_CLOCKS_HPP

#include <cstdint>
#include "stm32f4xx.h"  // CMSIS: RCC, ADC, SPI, I2C, USART, TIM, CAN, SDIO, etc.

namespace stm32::clocks {

//======================== helpers: decode prescalers ========================

static inline uint32_t decode_ahb_prescaler(uint32_t hpre_bits /*[7:4]->LSB*/)
{
    if ((hpre_bits & 0b1000U) == 0) return 1U;  // 0xxx => /1
    switch (hpre_bits & 0xF) {
        case 0b1000: return 2U;
        case 0b1001: return 4U;
        case 0b1010: return 8U;
        case 0b1011: return 16U;
        case 0b1100: return 64U;
        case 0b1101: return 128U;
        case 0b1110: return 256U;
        case 0b1111: return 512U;
        default:     return 1U;
    }
}

static inline uint32_t decode_apb_prescaler(uint32_t ppre_bits /*[2:0]->LSB*/)
{
    if ((ppre_bits & 0b100U) == 0) return 1U;  // 0xx => /1
    switch (ppre_bits & 0x7) {
        case 0b100: return 2U;
        case 0b101: return 4U;
        case 0b110: return 8U;
        case 0b111: return 16U;
        default:     return 1U;
    }
}

static inline uint32_t decode_pllp(uint32_t pllp_bits /*[17:16]->LSB*/)
{
    // Encoding: 00->2, 01->4, 10->6, 11->8
    return 2U * (pllp_bits + 1U);
}

//======================== core PLL products =================================

/** @return SYSCLK in Hz (HSI, HSE, or PLL_P). */
static inline uint32_t sysclk_hz()
{
    const uint32_t cfgr = RCC->CFGR;
    const uint32_t sws  = (cfgr >> 2) & 0x3U;

    if (sws == 0b00) return static_cast<uint32_t>(HSI_VALUE);
    if (sws == 0b01) return static_cast<uint32_t>(HSE_VALUE);
    if (sws == 0b10) {
        const uint32_t pllcfgr = RCC->PLLCFGR;
        const uint32_t pllsrc   = (pllcfgr >> 22) & 0x1U; // 0=HSI,1=HSE
        const uint32_t pllm     = (pllcfgr & 0x3FU);      // [5:0]
        const uint32_t plln     = (pllcfgr >> 6) & 0x1FFU;// [14:6]
        const uint32_t pllpbits = (pllcfgr >> 16) & 0x3U; // [17:16]
        const uint32_t pllp     = decode_pllp(pllpbits);

        const uint32_t fin = pllsrc ? static_cast<uint32_t>(HSE_VALUE)
                                    : static_cast<uint32_t>(HSI_VALUE);
        if (pllm == 0U || pllp == 0U) return 0U;

        const uint32_t vco_in = fin / pllm;
        const uint32_t vco    = vco_in * plln;
        return vco / pllp;
    }
    return 0U; // reserved
}

/** @return VCO (main PLL) frequency in Hz, based on PLLM/PLLN. */
static inline uint32_t pll_vco_hz()
{
    const uint32_t pllcfgr = RCC->PLLCFGR;
    const uint32_t pllsrc   = (pllcfgr >> 22) & 0x1U;
    const uint32_t pllm     = (pllcfgr & 0x3FU);
    const uint32_t plln     = (pllcfgr >> 6) & 0x1FFU;
    if (pllm == 0U) return 0U;
    const uint32_t fin = pllsrc ? static_cast<uint32_t>(HSE_VALUE)
                                : static_cast<uint32_t>(HSI_VALUE);
    return (fin / pllm) * plln;
}

/** @return PLL48CLK in Hz = VCO / PLLQ (USB FS, SDIO, RNG). */
static inline uint32_t pll48_hz()
{
    const uint32_t pllcfgr = RCC->PLLCFGR;
    const uint32_t pllq     = (pllcfgr >> 24) & 0xFU; // PLLQ[27:24]
    const uint32_t vco      = pll_vco_hz();
    if (pllq == 0U) return 0U;
    return vco / pllq;
}

/** @return PLLI2S VCO in Hz = (PLLSRC input / PLLM) * PLLI2SN. */
static inline uint32_t plli2s_vco_hz()
{
#if defined(RCC->PLLI2SCFGR)
    const uint32_t pllcfgr = RCC->PLLCFGR;
    const uint32_t pllsrc   = (pllcfgr >> 22) & 0x1U;
    const uint32_t pllm     = (pllcfgr & 0x3FU);

    const uint32_t plli2s   = RCC->PLLI2SCFGR;
    const uint32_t n        = (plli2s >> 6) & 0x1FFU; // PLLI2SN[14:6]

    if (pllm == 0U) return 0U;
    const uint32_t fin = pllsrc ? static_cast<uint32_t>(HSE_VALUE)
                                : static_cast<uint32_t>(HSI_VALUE);
    return (fin / pllm) * n;
#else
    return 0U;
#endif
}

/** @return PLLI2S output for I2S in Hz = PLLI2S_VCO / PLLI2SR. */
static inline uint32_t plli2s_i2sclk_hz()
{
#if defined(RCC->PLLI2SCFGR)
    const uint32_t r = (RCC->PLLI2SCFGR >> 28) & 0x7U; // PLLI2SR[30:28]
    const uint32_t vco = plli2s_vco_hz();
    if (r == 0U) return 0U;
    return vco / r;
#else
    return 0U;
#endif
}

//======================== core bus clocks ===================================

/** @return HCLK in Hz (core/AHB). */
static inline uint32_t hclk_hz()
{
    const uint32_t hpre_bits = (RCC->CFGR >> 4) & 0xFU; // HPRE[7:4]
    const uint32_t div = decode_ahb_prescaler(hpre_bits);
    const uint32_t sys = sysclk_hz();
    return (div ? (sys / div) : 0U);
}

/** @return PCLK1 in Hz (APB1). */
static inline uint32_t pclk1_hz()
{
    const uint32_t ppre1_bits = (RCC->CFGR >> 10) & 0x7U; // PPRE1[12:10]
    const uint32_t div = decode_apb_prescaler(ppre1_bits);
    const uint32_t hclk = hclk_hz();
    return (div ? (hclk / div) : 0U);
}

/** @return PCLK2 in Hz (APB2). */
static inline uint32_t pclk2_hz()
{
    const uint32_t ppre2_bits = (RCC->CFGR >> 13) & 0x7U; // PPRE2[15:13]
    const uint32_t div = decode_apb_prescaler(ppre2_bits);
    const uint32_t hclk = hclk_hz();
    return (div ? (hclk / div) : 0U);
}

//======================== timer clocks ======================================

/** @return APB1 timer clock (TIM2/3/4/5/6/7/12/13/14). */
static inline uint32_t tim_apb1_hz()
{
    const uint32_t ppre1_bits = (RCC->CFGR >> 10) & 0x7U;
    const uint32_t div = decode_apb_prescaler(ppre1_bits);
    const uint32_t pclk1 = pclk1_hz();
    return (div == 1U) ? pclk1 : (pclk1 * 2U);
}

/** @return APB2 timer clock (TIM1/8/9/10/11). */
static inline uint32_t tim_apb2_hz()
{
    const uint32_t ppre2_bits = (RCC->CFGR >> 13) & 0x7U;
    const uint32_t div = decode_apb_prescaler(ppre2_bits);
    const uint32_t pclk2 = pclk2_hz();
    return (div == 1U) ? pclk2 : (pclk2 * 2U);
}

/** @return Timer kernel clock for a given TIMx*. */
static inline uint32_t tim_kernel_hz(TIM_TypeDef* tim)
{
    if (tim == nullptr) return 0U;
#if defined(TIM1_BASE)
    if (tim == TIM1)  return tim_apb2_hz();
#endif
#if defined(TIM8_BASE)
    if (tim == TIM8)  return tim_apb2_hz();
#endif
#if defined(TIM9_BASE)
    if (tim == TIM9)  return tim_apb2_hz();
#endif
#if defined(TIM10_BASE)
    if (tim == TIM10) return tim_apb2_hz();
#endif
#if defined(TIM11_BASE)
    if (tim == TIM11) return tim_apb2_hz();
#endif
#if defined(TIM2_BASE)
    if (tim == TIM2)  return tim_apb1_hz();
#endif
#if defined(TIM3_BASE)
    if (tim == TIM3)  return tim_apb1_hz();
#endif
#if defined(TIM4_BASE)
    if (tim == TIM4)  return tim_apb1_hz();
#endif
#if defined(TIM5_BASE)
    if (tim == TIM5)  return tim_apb1_hz();
#endif
#if defined(TIM6_BASE)
    if (tim == TIM6)  return tim_apb1_hz();
#endif
#if defined(TIM7_BASE)
    if (tim == TIM7)  return tim_apb1_hz();
#endif
#if defined(TIM12_BASE)
    if (tim == TIM12) return tim_apb1_hz();
#endif
#if defined(TIM13_BASE)
    if (tim == TIM13) return tim_apb1_hz();
#endif
#if defined(TIM14_BASE)
    if (tim == TIM14) return tim_apb1_hz();
#endif
    return 0U;
}

//======================== SPI / I2S =========================================

/** @return SPI kernel clock in Hz: SPI1->PCLK2, SPI2/SPI3->PCLK1. */
static inline uint32_t spi_kernel_hz(SPI_TypeDef* spi)
{
    if (spi == nullptr) return 0U;
#if defined(SPI1_BASE)
    if (spi == SPI1) return pclk2_hz();
#endif
#if defined(SPI2_BASE)
    if (spi == SPI2) return pclk1_hz();
#endif
#if defined(SPI3_BASE)
    if (spi == SPI3) return pclk1_hz();
#endif
    return 0U;
}

/** @return SPI SCK output frequency = kernel / 2^(BR+1). */
static inline uint32_t spi_sck_hz(SPI_TypeDef* spi)
{
    if (spi == nullptr) return 0U;
    const uint32_t f_in = spi_kernel_hz(spi);
    const uint32_t br   = (spi->CR1 >> 3) & 0x7U;     // BR[5:3]
    const uint32_t pres = 1U << (br + 1U);            // 2^(BR+1)
    return (pres ? (f_in / pres) : 0U);
}

/** @return I2S kernel clock for SPI2/3 in I2S mode (from PLLI2S). */
static inline uint32_t i2s_kernel_hz(SPI_TypeDef* spi)
{
#if defined(SPI_I2SCFGR_I2SMOD)
    if (spi == nullptr) return 0U;
    // Only SPI2/SPI3 support I2S on F407
#if defined(SPI2_BASE)
    if (spi == SPI2) return plli2s_i2sclk_hz();
#endif
#if defined(SPI3_BASE)
    if (spi == SPI3) return plli2s_i2sclk_hz();
#endif
#endif
    return 0U;
}

//======================== USART kernels =====================================

/** @return USART kernel clock in Hz (per IP location). */
static inline uint32_t usart_kernel_hz(USART_TypeDef* u)
{
    if (u == nullptr) return 0U;
    // APB2: USART1, USART6
#if defined(USART1_BASE)
    if (u == USART1) return pclk2_hz();
#endif
#if defined(USART6_BASE)
    if (u == USART6) return pclk2_hz();
#endif
    // APB1: USART2, USART3, UART4, UART5, USART7, USART8 (F407 variants)
#if defined(USART2_BASE)
    if (u == USART2) return pclk1_hz();
#endif
#if defined(USART3_BASE)
    if (u == USART3) return pclk1_hz();
#endif
#if defined(UART4_BASE)
    if (u == UART4)  return pclk1_hz();
#endif
#if defined(UART5_BASE)
    if (u == UART5)  return pclk1_hz();
#endif
#if defined(UART7_BASE)
    if (u == UART7)  return pclk1_hz();
#endif
#if defined(UART8_BASE)
    if (u == UART8)  return pclk1_hz();
#endif
    return 0U;
}

//======================== I2C kernels =======================================

/** @return I2C kernel clock in Hz (I2C1/2/3 on APB1). */
static inline uint32_t i2c_kernel_hz(I2C_TypeDef* i)
{
    if (i == nullptr) return 0U;
#if defined(I2C1_BASE)
    if (i == I2C1) return pclk1_hz();
#endif
#if defined(I2C2_BASE)
    if (i == I2C2) return pclk1_hz();
#endif
#if defined(I2C3_BASE)
    if (i == I2C3) return pclk1_hz();
#endif
    return 0U;
}

//======================== CAN kernels =======================================

/** @return CAN kernel clock in Hz (CAN1/2 on APB1). */
static inline uint32_t can_kernel_hz(CAN_TypeDef* c)
{
    if (c == nullptr) return 0U;
#if defined(CAN1_BASE)
    if (c == CAN1) return pclk1_hz();
#endif
#if defined(CAN2_BASE)
    if (c == CAN2) return pclk1_hz();
#endif
    return 0U;
}

//======================== ADC common clock ==================================

/**
 * @return ADC common clock in Hz.
 * @note   ADCPRE bits in ADC->CCR (17:16): 00:/2, 01:/4, 10:/6, 11:/8 from PCLK2.
 */
static inline uint32_t adc_common_hz()
{
#if defined(ADC) && defined(ADC_CCR_ADCPRE)
    const uint32_t pclk2 = pclk2_hz();
    const uint32_t pre = (ADC->CCR >> 16) & 0x3U; // ADCPRE
    const uint32_t div = (pre == 0) ? 2U : (pre == 1) ? 4U : (pre == 2) ? 6U : 8U;
    return pclk2 / div;
#else
    return 0U;
#endif
}

//======================== SDIO / USB FS / RNG ===============================

/** @return SDIO clock source in Hz (driven by PLL48CLK). */
static inline uint32_t sdio_clk_hz()
{
#if defined(SDIO_BASE)
    return pll48_hz();
#else
    return 0U;
#endif
}

/** @return USB FS clock in Hz (must be 48 MHz nominal). */
static inline uint32_t usb_fs_clk_hz()
{
#if defined(USB_OTG_FS)
    return pll48_hz();
#else
    return 0U;
#endif
}

/** @return RNG clock in Hz (from PLL48CLK). */
static inline uint32_t rng_clk_hz()
{
#if defined(RNG)
    return pll48_hz();
#else
    return 0U;
#endif
}

//======================== snapshot ==========================================

struct ClockSnapshot {
    uint32_t sysclk, hclk, pclk1, pclk2;
    uint32_t tim_apb1, tim_apb2;
    uint32_t pll48;
    uint32_t spi1_kernel, spi2_kernel, spi3_kernel;
    uint32_t usart1_kernel, usart2_kernel, usart3_kernel, uart4_kernel, uart5_kernel, usart6_kernel;
    uint32_t i2c1_kernel, i2c2_kernel, i2c3_kernel;
    uint32_t can1_kernel, can2_kernel;
    uint32_t adc_common;
    uint32_t sdio, usbfs, rng;
    uint32_t i2sclk; // PLLI2S output for SPI2/3 I2S
};

static inline ClockSnapshot snapshot()
{
    ClockSnapshot s{};
    s.sysclk   = sysclk_hz();
    s.hclk     = hclk_hz();
    s.pclk1    = pclk1_hz();
    s.pclk2    = pclk2_hz();
    s.tim_apb1 = tim_apb1_hz();
    s.tim_apb2 = tim_apb2_hz();
    s.pll48    = pll48_hz();
#if defined(SPI1_BASE)
    s.spi1_kernel = spi_kernel_hz(SPI1);
#endif
#if defined(SPI2_BASE)
    s.spi2_kernel = spi_kernel_hz(SPI2);
#endif
#if defined(SPI3_BASE)
    s.spi3_kernel = spi_kernel_hz(SPI3);
#endif
#if defined(USART1_BASE)
    s.usart1_kernel = usart_kernel_hz(USART1);
#endif
#if defined(USART2_BASE)
    s.usart2_kernel = usart_kernel_hz(USART2);
#endif
#if defined(USART3_BASE)
    s.usart3_kernel = usart_kernel_hz(USART3);
#endif
#if defined(UART4_BASE)
    s.uart4_kernel = usart_kernel_hz(UART4);
#endif
#if defined(UART5_BASE)
    s.uart5_kernel = usart_kernel_hz(UART5);
#endif
#if defined(USART6_BASE)
    s.usart6_kernel = usart_kernel_hz(USART6);
#endif
#if defined(I2C1_BASE)
    s.i2c1_kernel = i2c_kernel_hz(I2C1);
#endif
#if defined(I2C2_BASE)
    s.i2c2_kernel = i2c_kernel_hz(I2C2);
#endif
#if defined(I2C3_BASE)
    s.i2c3_kernel = i2c_kernel_hz(I2C3);
#endif
#if defined(CAN1_BASE)
    s.can1_kernel = can_kernel_hz(CAN1);
#endif
#if defined(CAN2_BASE)
    s.can2_kernel = can_kernel_hz(CAN2);
#endif
    s.adc_common = adc_common_hz();
    s.sdio       = sdio_clk_hz();
    s.usbfs      = usb_fs_clk_hz();
    s.rng        = rng_clk_hz();
    s.i2sclk     = plli2s_i2sclk_hz();
    return s;
}

} // namespace stm32::clocks

#endif // STM32F4_CLOCKS_HPP
