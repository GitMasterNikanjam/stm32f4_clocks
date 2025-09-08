/**
 * @file    stm32f4_clocks.h
 * @brief   STM32F407 clock computation helpers (CMSIS-only, no HAL).
 * @version 1.1-fix
 * @license MIT
 */

#ifndef STM32F4_CLOCKS_H
#define STM32F4_CLOCKS_H

#include <stdint.h>
#include "stm32f4xx.h"

/* از نگارش کلاسیک فضای نام برای سازگاری با C++11/14 استفاده می‌کنیم */
namespace stm32 { namespace clocks {

/* ----------------- Helpers ----------------- */
static inline uint32_t decode_ahb_prescaler(uint32_t hpre_bits)
{
    if ((hpre_bits & 0x8u) == 0) return 1u;
    switch (hpre_bits & 0xFu) {
        case 0x8: return 2u;
        case 0x9: return 4u;
        case 0xA: return 8u;
        case 0xB: return 16u;
        case 0xC: return 64u;
        case 0xD: return 128u;
        case 0xE: return 256u;
        case 0xF: return 512u;
        default:  return 1u;
    }
}

static inline uint32_t decode_apb_prescaler(uint32_t ppre_bits)
{
    if ((ppre_bits & 0x4u) == 0) return 1u;
    switch (ppre_bits & 0x7u) {
        case 0x4: return 2u;
        case 0x5: return 4u;
        case 0x6: return 8u;
        case 0x7: return 16u;
        default:  return 1u;
    }
}

static inline uint32_t decode_pllp(uint32_t pllp_bits)
{
    /* 00->2, 01->4, 10->6, 11->8 */
    return 2u * (pllp_bits + 1u);
}

/* ----------------- Core PLL products ----------------- */
static inline uint32_t sysclk_hz(void)
{
    uint32_t sws = (RCC->CFGR >> 2) & 0x3u;
    if (sws == 0u)  return (uint32_t)HSI_VALUE;
    if (sws == 1u)  return (uint32_t)HSE_VALUE;
    if (sws == 2u) {
        uint32_t pllcfgr = RCC->PLLCFGR;
        uint32_t pllsrc   = (pllcfgr >> 22) & 0x1u; /* 0=HSI,1=HSE */
        uint32_t pllm     = (pllcfgr & 0x3Fu);
        uint32_t plln     = (pllcfgr >> 6) & 0x1FFu;
        uint32_t pllpbits = (pllcfgr >> 16) & 0x3u;
        uint32_t pllp     = decode_pllp(pllpbits);
        if (pllm == 0u || pllp == 0u) return 0u;
        uint32_t fin = pllsrc ? (uint32_t)HSE_VALUE : (uint32_t)HSI_VALUE;
        uint32_t vco_in = fin / pllm;
        uint32_t vco    = vco_in * plln;
        return vco / pllp;
    }
    return 0u;
}

static inline uint32_t pll_vco_hz(void)
{
    uint32_t pllcfgr = RCC->PLLCFGR;
    uint32_t pllsrc   = (pllcfgr >> 22) & 0x1u;
    uint32_t pllm     = (pllcfgr & 0x3Fu);
    uint32_t plln     = (pllcfgr >> 6) & 0x1FFu;
    if (pllm == 0u) return 0u;
    uint32_t fin = pllsrc ? (uint32_t)HSE_VALUE : (uint32_t)HSI_VALUE;
    return (fin / pllm) * plln;
}

static inline uint32_t pll48_hz(void) /* USB/SDIO/RNG */
{
    uint32_t pllq = (RCC->PLLCFGR >> 24) & 0xFu;
    uint32_t vco  = pll_vco_hz();
    if (pllq == 0u) return 0u;
    return vco / pllq;
}

/* ---------- PLLI2S (برای I2S) با گارد صحیحِ ماکرو ---------- */
/* از ماکروهای بیت‌فیلد استفاده می‌کنیم، نه RCC->... در defined# */
#if defined(RCC_PLLI2SCFGR_PLLI2SN) && defined(RCC_PLLI2SCFGR_PLLI2SR)
static inline uint32_t plli2s_vco_hz(void)
{
    uint32_t pllm = (RCC->PLLCFGR & 0x3Fu);
    if (pllm == 0u) return 0u;
    uint32_t pllsrc = (RCC->PLLCFGR >> 22) & 0x1u;
    uint32_t fin = pllsrc ? (uint32_t)HSE_VALUE : (uint32_t)HSI_VALUE;
    uint32_t n = (RCC->PLLI2SCFGR >> 6) & 0x1FFu; /* PLLI2SN */
    return (fin / pllm) * n;
}

static inline uint32_t plli2s_i2sclk_hz(void)
{
    uint32_t r = (RCC->PLLI2SCFGR >> 28) & 0x7u; /* PLLI2SR */
    uint32_t v = plli2s_vco_hz();
    if (r == 0u) return 0u;
    return v / r;
}
#else
static inline uint32_t plli2s_i2sclk_hz(void) { return 0u; }
#endif

/* ----------------- Bus clocks ----------------- */
static inline uint32_t hclk_hz(void)
{
    uint32_t div = decode_ahb_prescaler((RCC->CFGR >> 4) & 0xFu);
    uint32_t sys = sysclk_hz();
    return div ? (sys / div) : 0u;
}

static inline uint32_t pclk1_hz(void)
{
    uint32_t div = decode_apb_prescaler((RCC->CFGR >> 10) & 0x7u);
    uint32_t h   = hclk_hz();
    return div ? (h / div) : 0u;
}

static inline uint32_t pclk2_hz(void)
{
    uint32_t div = decode_apb_prescaler((RCC->CFGR >> 13) & 0x7u);
    uint32_t h   = hclk_hz();
    return div ? (h / div) : 0u;
}

/* ----------------- Timers ----------------- */
static inline uint32_t tim_apb1_hz(void)
{
    uint32_t div = decode_apb_prescaler((RCC->CFGR >> 10) & 0x7u);
    uint32_t p   = pclk1_hz();
    return (div == 1u) ? p : (p * 2u);
}

static inline uint32_t tim_apb2_hz(void)
{
    uint32_t div = decode_apb_prescaler((RCC->CFGR >> 13) & 0x7u);
    uint32_t p   = pclk2_hz();
    return (div == 1u) ? p : (p * 2u);
}

/* ----------------- SPI / I2S ----------------- */
static inline uint32_t spi_kernel_hz(SPI_TypeDef* spi)
{
#if defined(SPI1_BASE)
    if (spi == SPI1) return pclk2_hz();
#endif
#if defined(SPI2_BASE)
    if (spi == SPI2) return pclk1_hz();
#endif
#if defined(SPI3_BASE)
    if (spi == SPI3) return pclk1_hz();
#endif
    (void)spi;
    return 0u;
}

static inline uint32_t spi_sck_hz(SPI_TypeDef* spi)
{
    if (!spi) return 0u;
    uint32_t f_in = spi_kernel_hz(spi);
    uint32_t br   = (spi->CR1 >> 3) & 0x7u; /* BR[5:3] */
    uint32_t pres = 1u << (br + 1u);        /* 2^(BR+1) */
    return pres ? (f_in / pres) : 0u;
}

/* I2S kernel (برای SPI2/3) */
static inline uint32_t i2s_kernel_hz(SPI_TypeDef* spi)
{
#if defined(SPI_I2SCFGR_I2SMOD)
# if defined(SPI2_BASE)
    if (spi == SPI2) return plli2s_i2sclk_hz();
# endif
# if defined(SPI3_BASE)
    if (spi == SPI3) return plli2s_i2sclk_hz();
# endif
#endif
    (void)spi;
    return 0u;
}

/* ----------------- USART / I2C / CAN ----------------- */
static inline uint32_t usart_kernel_hz(USART_TypeDef* u)
{
#if defined(USART1_BASE)
    if (u == USART1) return pclk2_hz();
#endif
#if defined(USART6_BASE)
    if (u == USART6) return pclk2_hz();
#endif
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
    (void)u;
    return 0u;
}

static inline uint32_t i2c_kernel_hz(I2C_TypeDef* i)
{
#if defined(I2C1_BASE)
    if (i == I2C1) return pclk1_hz();
#endif
#if defined(I2C2_BASE)
    if (i == I2C2) return pclk1_hz();
#endif
#if defined(I2C3_BASE)
    if (i == I2C3) return pclk1_hz();
#endif
    (void)i;
    return 0u;
}

static inline uint32_t can_kernel_hz(CAN_TypeDef* c)
{
#if defined(CAN1_BASE)
    if (c == CAN1) return pclk1_hz();
#endif
#if defined(CAN2_BASE)
    if (c == CAN2) return pclk1_hz();
#endif
    (void)c;
    return 0u;
}

/* ----------------- ADC / SDIO / USB / RNG ----------------- */
static inline uint32_t adc_common_hz(void)
{
#if defined(ADC) && defined(ADC_CCR_ADCPRE)
    uint32_t p2  = pclk2_hz();
    uint32_t pre = (ADC->CCR >> 16) & 0x3u; /* 00:/2,01:/4,10:/6,11:/8 */
    uint32_t div = (pre == 0u) ? 2u : (pre == 1u) ? 4u : (pre == 2u) ? 6u : 8u;
    return p2 / div;
#else
    return 0u;
#endif
}

static inline uint32_t sdio_clk_hz(void)
{
#if defined(SDIO_BASE)
    return pll48_hz();
#else
    return 0u;
#endif
}

static inline uint32_t usb_fs_clk_hz(void)
{
#if defined(USB_OTG_FS)
    return pll48_hz();
#else
    return 0u;
#endif
}

static inline uint32_t rng_clk_hz(void)
{
#if defined(RNG)
    return pll48_hz();
#else
    return 0u;
#endif
}

/* ----------------- Snapshot ----------------- */
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
    uint32_t i2sclk;
};

static inline ClockSnapshot snapshot(void)
{
    ClockSnapshot s;
    s.sysclk   = sysclk_hz();
    s.hclk     = hclk_hz();
    s.pclk1    = pclk1_hz();
    s.pclk2    = pclk2_hz();
    s.tim_apb1 = tim_apb1_hz();
    s.tim_apb2 = tim_apb2_hz();
    s.pll48    = pll48_hz();
#if defined(SPI1_BASE)
    s.spi1_kernel = spi_kernel_hz(SPI1);
#else
    s.spi1_kernel = 0u;
#endif
#if defined(SPI2_BASE)
    s.spi2_kernel = spi_kernel_hz(SPI2);
#else
    s.spi2_kernel = 0u;
#endif
#if defined(SPI3_BASE)
    s.spi3_kernel = spi_kernel_hz(SPI3);
#else
    s.spi3_kernel = 0u;
#endif
#if defined(USART1_BASE)
    s.usart1_kernel = usart_kernel_hz(USART1);
#else
    s.usart1_kernel = 0u;
#endif
#if defined(USART2_BASE)
    s.usart2_kernel = usart_kernel_hz(USART2);
#else
    s.usart2_kernel = 0u;
#endif
#if defined(USART3_BASE)
    s.usart3_kernel = usart_kernel_hz(USART3);
#else
    s.usart3_kernel = 0u;
#endif
#if defined(UART4_BASE)
    s.uart4_kernel = usart_kernel_hz(UART4);
#else
    s.uart4_kernel = 0u;
#endif
#if defined(UART5_BASE)
    s.uart5_kernel = usart_kernel_hz(UART5);
#else
    s.uart5_kernel = 0u;
#endif
#if defined(USART6_BASE)
    s.usart6_kernel = usart_kernel_hz(USART6);
#else
    s.usart6_kernel = 0u;
#endif
#if defined(I2C1_BASE)
    s.i2c1_kernel = i2c_kernel_hz(I2C1);
#else
    s.i2c1_kernel = 0u;
#endif
#if defined(I2C2_BASE)
    s.i2c2_kernel = i2c_kernel_hz(I2C2);
#else
    s.i2c2_kernel = 0u;
#endif
#if defined(I2C3_BASE)
    s.i2c3_kernel = i2c_kernel_hz(I2C3);
#else
    s.i2c3_kernel = 0u;
#endif
#if defined(CAN1_BASE)
    s.can1_kernel = can_kernel_hz(CAN1);
#else
    s.can1_kernel = 0u;
#endif
#if defined(CAN2_BASE)
    s.can2_kernel = can_kernel_hz(CAN2);
#else
    s.can2_kernel = 0u;
#endif
    s.adc_common = adc_common_hz();
    s.sdio       = sdio_clk_hz();
    s.usbfs      = usb_fs_clk_hz();
    s.rng        = rng_clk_hz();
    s.i2sclk     = plli2s_i2sclk_hz();
    return s;
}

}} /* namespace stm32::clocks */

#endif /* STM32F4_CLOCKS_H */
