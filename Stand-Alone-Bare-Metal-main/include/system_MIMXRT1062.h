/*!
 * @file MIMXRT1062
 * @version 1.2
 * @date 2019-04-29
 * @brief Device specific configuration file for MIMXRT1062 (header file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#ifndef _SYSTEM_MIMXRT1062_H_
#define _SYSTEM_MIMXRT1062_H_                    /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#ifndef DISABLE_WDOG
  #define DISABLE_WDOG                 1
#endif

/* Define clock source values */

#define CPU_XTAL_CLK_HZ                   24000000UL          /* Value of the external crystal or oscillator clock frequency in Hz */

#define CCM_ANALOG_PLL_ARM_CLK_HZ         528000000UL
#define CCM_ANALOG_PLL_SYS_CLK_HZ         528000000UL

#define CCM_AHB_CLK_HZ                    132000000UL
#define CCM_IPG_CLK_HZ                    66000000UL
#define CCM_PERCLK_CLK_HZ                 66000000UL
#define CCM_USDHC1_CLK_HZ                 66000000UL
#define CCM_USDHC2_CLK_HZ                 66000000UL
#define CCM_SEMC_CLK_HZ                   132000000UL
#define CCM_CSI_CLK_HZ                    60000000UL
#define CCM_FLEXSPI_CLK_HZ                66000000UL
#define CCM_LPSPI_CLK_HZ                  66000000UL
#define CCM_TRACE_CLK_HZ                  132000000UL
#define CCM_LPI2C_CLK_HZ                  60000000UL
#define CCM_CAN_CLK_HZ                    40000000UL
#define CCM_UART_CLK_HZ                   24000000UL
#define CCM_LCDIF_CLK_HZ                  // PLL5 AYARLA - PLL5 / 4


/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (void);

/**
 * @brief SystemInit function hook.
 *
 * This weak function allows to call specific initialization code during the
 * SystemInit() execution.This can be used when an application specific code needs
 * to be called as close to the reset entry as possible (for example the Multicore
 * Manager MCMGR_EarlyInit() function call).
 * NOTE: No global r/w variables can be used in this hook function because the
 * initialization of these variables happens after this function.
 */
void SystemInitHook (void);

void PLL_ARM_CLK_Init(void);
void PLL_SYS_CLK_Init(void);
void PLL_USB1_CLK_Init(void);
void PLL_VIDEO_CLK_Init(void);

void USDHC1_CLK_Init(void);
void USDHC2_CLK_Init(void);
void CSI_CLK_Init(void);
void LPSPI_CLK_Init(void);
void TRACE_CLK_Init(void);
void LPI2C_CLK_Init(void);
void CAN_CLK_Init(void);
void UART_CLK_Init(void);
void LCDIF_CLK_Init(void);

#ifdef __cplusplus
}
#endif

#endif  /* _SYSTEM_MIMXRT1062_H_ */
