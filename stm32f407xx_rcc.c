/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (c) 2018 Martin SchrĂ¶der <mkschreder.uk@gmail.com>
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"

#include "qemu/timer.h"
#include <stdio.h>

#include "hw/sysbus.h"
#include "stm32fxxx_clktree.h"
#include "stm32fxxx_rcc.h"
#include "qemu/log.h"
#include "hw/arm/arm.h"

#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32F2407_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define IS_RESET_VALUE(new_value, mask, reset_value) ((new_value & mask) == (mask & reset_value))

#define WARN_UNIMPLEMENTED(new_value, mask, reset_value) \
    if (!IS_RESET_VALUE(new_value, mask, reset_value)) { \
        qemu_log_mask(LOG_UNIMP, "Not implemented: RCC " #mask ". Masked value: 0x%08x\n", (new_value & mask)); \
    }

#define WARN_UNIMPLEMENTED_REG(offset) \
        qemu_log_mask(LOG_UNIMP, "STM32f1xx_rcc: unimplemented register: 0x%x", (int)offset)

#define HSI_FREQ 16000000
#define LSI_FREQ 32000

//RCC clock control register(RCC_CR)
#define RCC_CR_RESET_VALUE 0x00000083 
#define RCC_CR_OFFSET 	   0x00
#define RCC_CR_PLLI2SRDY_BIT    27
#define RCC_CR_PLLI2SON_BIT     26
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_CSSON_BIT        19
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x0000ff00
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0

//RCC clock PLL configuration register (RCC_PLLCFGR)
#define RCC_PLLCFGR_RESET_VALUE  0x24003010
#define RCC_PLLCFGR_OFFSET       0x04
#define RCC_PLLCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLCFGR_PLLQ_START   24
#define RCC_PLLCFGR_PLLSRC_BIT   22
#define RCC_PLLCFGR_PLLP_MASK    0x00030000
#define RCC_PLLCFGR_PLLP_START   16
#define RCC_PLLCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLCFGR_PLLN_START   6
#define RCC_PLLCFGR_PLLM_MASK    0x0000003F
#define RCC_PLLCFGR_PLLM_START   0

//RCC clock configuration register (RCC_CFGR)
#define RCC_CFGR_RESET_VALUE     0x00000000
#define RCC_CFGR_OFFSET          0x08
#define RCC_CFGR_MCO2_START      30
#define RCC_CFGR_MCO2_MASK       0xC0000000
#define RCC_CFGR_MCO2PRE_START   27
#define RCC_CFGR_MCO2PRE_MASK    0x38000000
#define RCC_CFGR_MCO1PRE_START   24
#define RCC_CFGR_MCO1PRE_MASK    0x07000000
#define RCC_CFGR_I2SSRC_BIT      23
#define RCC_CFGR_MCO1_START      21
#define RCC_CFGR_MCO1_MASK       0x00600000
#define RCC_CFGR_RTCPRE_START    16
#define RCC_CFGR_RTCPRE_MASK     0x001F0000
#define RCC_CFGR_PPRE2_START     13
#define RCC_CFGR_PPRE2_MASK      0x0000E000
#define RCC_CFGR_PPRE1_START     10
#define RCC_CFGR_PPRE1_MASK      0x00001C00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000F0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000C
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

//RCC clock interrupt register (RCC_CIR)
#define RCC_CIR_RESET_VALUE      0x00000000
#define RCC_CIR_OFFSET           0x0C
#define RCC_CIR_CSSC_BIT         23
#define RCC_CIR_PLLI2SRDYC_BIT   21
#define RCC_CIR_PLLRDYC_BIT      20
#define RCC_CIR_HSERDYC_BIT      19
#define RCC_CIR_HSIRDYC_BIT      18
#define RCC_CIR_LSERDYC_BIT      17
#define RCC_CIR_LSIRDYC_BIT      16
#define RCC_CIR_PLLI2SRDYIE_BIT  13
#define RCC_CIR_PLLRDYIE_BIT     12
#define RCC_CIR_HSERDYIE_BIT     11
#define RCC_CIR_HSIRDYIE_BIT     10
#define RCC_CIR_LSERDYIE_BIT     9
#define RCC_CIR_LSIRDYIE_BIT     8
#define RCC_CIR_CSSF_BIT         7
#define RCC_CIR_PLLI2SRDYF_BIT  5
#define RCC_CIR_PLLRDYF_BIT     4
#define RCC_CIR_HSERDYF_BIT     3
#define RCC_CIR_HSIRDYF_BIT     2
#define RCC_CIR_LSERDYF_BIT     1
#define RCC_CIR_LSIRDYF_BIT     0

//RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
#define RCC_AHB1RSTR_OFFSET        0x10
#define RCC_AHB1RSTR_RESET_VALUE   0x00000000
#define RCC_AHB1RSTR_OTGHSRST_BIT  29
#define RCC_AHB1RSTR_ETHMACRST_BIT 25
#define RCC_AHB1RSTR_DMA2RST_BIT       22
#define RCC_AHB1RSTR_DMA1RST_BIT       21
#define RCC_AHB1RSTR_CRCRST_BIT        12
#define RCC_AHB1RSTR_GPIOIRST_BIT      8
#define RCC_AHB1RSTR_GPIOHRST_BIT      7  
#define RCC_AHB1RSTR_GPIOGRST_BIT      6
#define RCC_AHB1RSTR_GPIOFRST_BIT      5 
#define RCC_AHB1RSTR_GPIOERST_BIT      4
#define RCC_AHB1RSTR_GPIODRST_BIT      3 
#define RCC_AHB1RSTR_GPIOCRST_BIT      2
#define RCC_AHB1RSTR_GPIOBRST_BIT      1 
#define RCC_AHB1RSTR_GPIOARST_BIT      0

//RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
#define RCC_AHB2RSTR_OFFSET        0x14
#define RCC_AHB2RSTR_RESET_VALUE   0x00000000
#define RCC_AHB2RSTR_OTGFSRST_BIT  7
#define RCC_AHB2RSTR_RNGRST_BIT    6
#define RCC_AHB2RSTR_HSAHRST_BIT   5
#define RCC_AHB2RSTR_CRYPRST       4
#define RCC_AHB2RSTR_DCMIRST       0

//RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
#define RCC_AHB3RSTR_OFFSET      0x18
#define RCC_AHB3RSTR_RESET_VALUE 0x00000000 
#define RCC_AHB3RSTR_FSMCRST_BIT 0

//RCC APB1 peripheral reset register (RCC_APB1RSTR)
#define RCC_APB1RSTR_OFFSET      0x20
#define RCC_APB1RSTR_RESET_VALUE 0x00000000
#define RCC_APB1RSTR_DACRST_BIT  	29
#define RCC_APB1RSTR_PWRRST_BIT  	28
#define RCC_APB1RSTR_CAN2RST_BIT 	26
#define RCC_APB1RSTR_CAN1RST_BIT 	25
#define RCC_APB1RSTR_I2C3RST_BIT 	23
#define RCC_APB1RSTR_I2C2RST_BIT 	22
#define RCC_APB1RSTR_I2C1RST_BIT 	21 
#define RCC_APB1RSTR_UART5RST_BIT   20
#define RCC_APB1RSTR_UART4RST_BIT   19
#define RCC_APB1RSTR_UART3RST_BIT   18
#define RCC_APB1RSTR_UART2RST_BIT   17
#define RCC_APB1RSTR_SPI3RST_BIT 	15
#define RCC_APB1RSTR_SPI2RST_BIT 	14
#define RCC_APB1RSTR_WWDGRST_BIT 	11
#define RCC_APB1RSTR_TIM14RST_BIT  	8
#define RCC_APB1RSTR_TIM13RST_BIT  	7
#define RCC_APB1RSTR_TIM12RST_BIT  	6
#define RCC_APB1RSTR_TIM7RST_BIT  	5
#define RCC_APB1RSTR_TIM6RST_BIT  	4
#define RCC_APB1RSTR_TIM5RST_BIT  	3
#define RCC_APB1RSTR_TIM4RST_BIT  	2
#define RCC_APB1RSTR_TIM3RST_BIT  	1
#define RCC_APB1RSTR_TIM2RST_BIT    0

//RCC APB2 Peripheral reset register (RCC_APB2RSTR)
#define RCC_APB2RSTR_RESET_VALUE     0x00000000
#define RCC_APB2RSTR_OFFSET          0x24
#define RCC_APB2RSTR_TIM11RST_BIT    18
#define RCC_APB2RSTR_TIM10RST_BIT    17
#define RCC_APB2RSTR_TIM9RST_BIT     16
#define RCC_APB2RSTR_SYSCFGRST       14
#define RCC_APB2RSTR_SPI1_BIT        12
#define RCC_APB2RSTR_SDIORST_BIT     11
#define RCC_APB2RSTR_ADCRST_BIT      8
#define RCC_APB2RSTR_USART6RST       5
#define RCC_APB2RSTR_USART1RST       4
#define RCC_APB2RSTR_TIM8RST_BIT     1
#define RCC_APB2RSTR_TIM1RST_BIT     0

//RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
#define RCC_AHB1ENR_RESET_VALUE      0x00100000
#define RCC_AHB1ENR_OFFSET           0x30
#define RCC_AHB1ENR_OTGHSULPIEN_BIT  30
#define RCC_AHB1ENR_OTGHSEN_BIT      29
#define RCC_AHB1ENR_ETHMACPTPEN_BIT  28
#define RCC_AHB1ENR_ETHMACRXEN_BIT   27
#define RCC_AHB1ENR_ETHMACTXEN_BIT   26
#define RCC_AHB1ENR_ETHMACEN_BIT     25
#define RCC_AHB1ENR_DMA2EN_BIT       22
#define RCC_AHB1ENR_DMA1EN_BIT       21
#define RCC_AHB1ENR_CCMDATARAMEN_BIT 20
#define RCC_AHB1ENR_BKPSRAMEN_BIT    18
#define RCC_AHB1ENR_CRCEN_BIT        12
#define RCC_AHB1ENR_GPIOIEN_BIT      8
#define RCC_AHB1ENR_GPIOHEN_BIT      7
#define RCC_AHB1ENR_GPIOGEN_BIT      6
#define RCC_AHB1ENR_GPIOFEN_BIT      5
#define RCC_AHB1ENR_GPIOEEN_BIT      4
#define RCC_AHB1ENR_GPIODEN_BIT      3
#define RCC_AHB1ENR_GPIOCEN_BIT      2
#define RCC_AHB1ENR_GPIOBEN_BIT      1
#define RCC_AHB1ENR_GPIOAEN_BIT      0

//RCC AHB2 peripheral clock enable register
#define RCC_AHB2ENR_OFFSET           0x34
#define RCC_AHB2ENR_RESET_VALUE      0x00000000
#define RCC_AHB2ENR_DCMIEN_BIT       0
#define RCC_AHB2ENR_CRYPEN_BIT       4
#define RCC_AHB2ENR_HASHEN_BIT       5
#define RCC_AHB2ENR_RNGEN_BIT        6
#define RCC_AHB2ENR_OTGFSEN_BIT      7

//RCC AHB3 peripheral clock enable resgister
#define RCC_AHB3ENR_OFFSET           0x38
#define RCC_AHB3ENR_RESET_VALUE      0x00000000
#define RCC_AHB3ENR_FSMCEN_BIT       0

//RCC APB1 peripheral clock enable register (RCC_APB1ENR)
#define RCC_APB1ENR_RESET_VALUE  0x00000000
#define RCC_APB1ENR_OFFSET       0x40
#define RCC_APB1ENR_DACEN_BIT    29
#define RCC_APB1ENR_PWREN_BIT    28
#define RCC_APB1ENR_CAN2EN_BIT   26
#define RCC_APB1ENR_CAN1EN_BIT   25
#define RCC_APB1ENR_I2C3EN_BIT   23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_UART5EN_BIT  20
#define RCC_APB1ENR_UART4EN_BIT  19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM14EN_BIT  8
#define RCC_APB1ENR_TIM13EN_BIT  7
#define RCC_APB1ENR_TIM12EN_BIT  6
#define RCC_APB1ENR_TIM7EN_BIT   5
#define RCC_APB1ENR_TIM6EN_BIT   4
#define RCC_APB1ENR_TIM5EN_BIT   3
#define RCC_APB1ENR_TIM4EN_BIT   2
#define RCC_APB1ENR_TIM3EN_BIT   1
#define RCC_APB1ENR_TIM2EN_BIT   0

//RCC APB2 peripheral clock enable register (RCC_APB2ENR)
#define RCC_APB2ENR_RESET_VALUE  0x00000000
#define RCC_APB2ENR_OFFSET       0x44
#define RCC_APB2ENR_MASK         0x00075F33
#define RCC_APB2ENR_TIM11EN      18
#define RCC_APB2ENR_TIM10EN      17
#define RCC_APB2ENR_TIM9EN       16
#define RCC_APB2ENR_SYSCFGEN_BIT 14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT   9
#define RCC_APB2ENR_ADC1EN_BIT   8
#define RCC_APB2ENR_USART6EN_BIT 5
#define RCC_APB2ENR_USART1EN_BIT 4
#define RCC_APB2ENR_TIM8EN_BIT   1
#define RCC_APB2ENR_TIM1EN_BIT   0

//RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR)
#define RCC_AHB1LPENR_RESET_VALUE   0x7E6791FF
#define RCC_AHB1LPENR_OFFSET        0x50
#define RCC_AHB1LPENR_OTGHSULPILPEN_BIT 30
#define RCC_AHB1LPENR_OTGHSLPEN_BIT     29
#define RCC_AHB1LPENR_ETHMACPTPLPEN_BIT 28
#define RCC_AHB1LPENR_ETHMACRXLPEN_BIT  27
#define RCC_AHB1LPENR_ETHMACTXLPEN_BIT  26
#define RCC_AHB1LPENR_ETHMACLPEN_BIT    25
#define RCC_AHB1LPENR_DMA2LPEN_BIT      22
#define RCC_AHB1LPENR_DMA1LPEN_BIT      21
#define RCC_AHB1LPENR_BKPSRAMLPEN_BIT   18
#define RCC_AHB1LPENR_SRAM2LPEN_BIT     17
#define RCC_AHB1LPENR_SRAM1LPEN_BIT     16
#define RCC_AHB1LPENR_FLITFLPEN_BIT     15
#define RCC_AHB1LPENR_CRCLPEN_BIT       12
#define RCC_AHB1LPENR_GPIOILPEN_BIT     8
#define RCC_AHB1LPENR_GPIOHLPEN_BIT     7
#define RCC_AHB1LPENR_GPIOGLPEN_BIT     6
#define RCC_AHB1LPENR_GPIOFLPEN_BIT     5
#define RCC_AHB1LPENR_GPIOELPEN_BIT     4
#define RCC_AHB1LPENR_GPIODLPEN_BIT     3 
#define RCC_AHB1LPENR_GPIOCLPEN_BIT     2 
#define RCC_AHB1LPENR_GPIOBLPEN_BIT     1 
#define RCC_AHB1LPENR_GPIOALPEN_BIT     0

//RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)
#define RCC_AHB2LPENR_RESET_VALUE      0x000000F1
#define RCC_AHB2LPENR_OFFSET           0x54
#define RCC_AHB2LPENR_OTGFSLPEN_BIT    7
#define RCC_AHB2LPENR_RNGLPEN_BIT      6
#define RCC_AHB2LPENR_HASHLPEN_BIT     5
#define RCC_AHB2LPENR_CRYPLPEN_BIT     4
#define RCC_AHB2LPENR_DCMILPEN_BIT     0 

//RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR)
#define RCC_AHB3LPENR_RESET_VALUE  0x00000001
#define RCC_AHB3LPENR_OFFSET       0x58
#define RCC_AHB3LPENR_FSMCLPEN_BIT 0

//RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)
#define RCC_APB1LPENR_RESET_VALUE   	0x36FEC9FF
#define RCC_APB1LPENR_OFFSET        	0x60
#define RCC_APB1LPENR_DACLPEN_BIT   	29
#define RCC_APB1LPENR_PWRLPEN_BIT   	28
#define RCC_APB1LPENR_CAN2LPEN_BIT  	26
#define RCC_APB1LPENR_CAN1LPEN_BIT  	25
#define RCC_APB1LPENR_I2C3LPEN_BIT  	23 
#define RCC_APB1LPENR_I2C2LPEN_BIT  	22
#define RCC_APB1LPENR_I2C1LPEN_BIT  	21
#define RCC_APB1LPENR_UART5LPEN_BIT 	20 
#define RCC_APB1LPENR_UART4LPEN_BIT 	19 
#define RCC_APB1LPENR_USART3LPEN_BIT	18
#define RCC_APB1LPENR_USART2LPEN_BIT	17
#define RCC_APB1LPENR_SPI3LPEN_BIT 		15
#define RCC_APB1LPENR_SPI2LPEN_BIT 		14 
#define RCC_APB1LPENR_WWDGLPEN_BIT 		11 
#define RCC_APB1LPENR_TIM14LPEN_BIT 	8
#define RCC_APB1LPENR_TIM13LPEN_BIT 	7
#define RCC_APB1LPENR_TIM12LPEN_BIT 	6
#define RCC_APB1LPENR_TIM7LPEN_BIT  	5
#define RCC_APB1LPENR_TIM6LPEN_BIT  	4
#define RCC_APB1LPENR_TIM5LPEN_BIT  	3
#define RCC_APB1LPENR_TIM4LPEN_BIT  	2
#define RCC_APB1LPENR_TIM3LPEN_BIT  	1
#define RCC_APB1LPENR_TIM2LPEN_BIT  	0

//RCC APB2 peripheral clock enable in low power mode register (RCC_APB2LPENR)
#define RCC_APB2LPENR_RESET_VALUE  		0x04075F33
#define RCC_APB2LPENR_OFFSET       		0x64
#define RCC_APB2LPENR_TIM11LPEN_BIT		18
#define RCC_APB2LPENR_TIM10LPEN_BIT		17
#define RCC_APB2LPENR_TIM9LPEN_BIT		16
#define RCC_APB2LPENR_SYSCFGLPEN_BIT    14
#define RCC_APB2LPENR_SPI1LPEN_BIT      12
#define RCC_APB2LPENR_SDIOLPEN_BIT		11
#define RCC_APB2LPENR_ADC3LPEN_BIT		10	
#define RCC_APB2LPENR_ADC2LPEN_BIT		9
#define RCC_APB2LPENR_ADC1LPEN_BIT		8
#define RCC_APB2LPENR_USART6LPEN_BIT    5
#define RCC_APB2LPENR_USART1LPEN_BIT    4
#define RCC_APB2LPENR_TIM8LPEN_BIT      1
#define RCC_APB2LPENR_TIM1LPEN_BIT      0

//RCC Back up domain control register (RCC_BDCR)
#define RCC_BDCR_RESET_VALUE       0x00000000
#define RCC_BDCR_OFFSET            0x70
#define RCC_BDCR_BDRST_BIT         16
#define RCC_BDCR_RTCEN_BIT         15
#define RCC_BDCR_RTCSEL_START      8
#define RCC_BDCR_RTCSEL_MASK       0x00000300
#define RCC_BDCR_LSEBYP_BIT        2
#define RCC_BDCR_LSERDY_BIT        1
#define RCC_BDCR_LSEON_BIT         0

//RCC clock control &status register (RCC_CSR)
#define RCC_CSR_RESET_VALUE        0x0E000000
#define RCC_CSR_OFFSET             0x74
#define RCC_CSR_LPWRRSTF_BIT       31
#define RCC_CSR_WWDGRSTF_BIT       30
#define RCC_CSR_WDGRSTF_BIT       29
#define RCC_CSR_SFTRSTF_BIT        28
#define RCC_CSR_PORRSTF_BIT        27
#define RCC_CSR_PADRSTF_BIT        26
#define RCC_CSR_BORRSTF_BIT        25
#define RCC_CSR_RMVF_BIT           24
#define RCC_CSR_LSIRDY_BIT         1
#define RCC_CSR_LSION_BIT          0

//RCC spread spectrum clock generation register (RCC_SSCGR)
#define RCC_SSCGR_RESET_VALUE      0x00000000
#define RCC_SSCGR_OFFSET           0x80
//this is not completed 

//RCC PLLI2S configuration  register (RCC_PLLI2SCFGR)
#define RCC_PLLI2SCFGR_RESET_VALUE 		0x20003000
#define RCC_PLLI2SCFGR_OFFSET      		0x84
#define RCC_PLLI2SCFGR_PLLR_MASK    	0x70000000
#define RCC_PLLI2SCFGR_PLLR_START   	28
#define RCC_PLLI2SCFGR_PLLI2SNx_MASK    0x00007FC0
#define RCC_PLLI2SCFGR_PLLI2SNx_START   6

#define PLLSRC_HSI_SELECTED 0
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

#define GET_BIT_MASK(position, value) ((value ? 1 : 0) << position)
#define GET_BIT_MASK_ONE(position) (1 << position)
#define GET_BIT_MASK_ZERO(position) (~(1 << position))
#define GET_BIT_VALUE(value, position) \
                ((value & GET_BIT_MASK_ONE(position)) >> position)
#define IS_BIT_SET(value, position) ((value & GET_BIT_MASK_ONE(position)) != 0)
#define IS_BIT_RESET(value, position) ((value & GET_BIT_MASK_ONE(position)) ==0)
#define RESET_BIT(var, position) var &= GET_BIT_MASK_ZERO(position)

/* Can be true, false, 0, or 1 */
#define CHANGE_BIT(var, position, new_value) \
            var = new_value ? \
                    (var | GET_BIT_MASK_ONE(position)) : \ 
                    (var & GET_BIT_MASK_ZERO(position))
#define CHANGE_BITS(var, start, mask, new_value) \
            var = (var & ~mask) | ((new_value << start) & mask)

# define STM32_BAD_REG(offset, size)       \
        hw_error("%s: Bad register 0x%x - size %u\n", __FUNCTION__, (int)offset, size)
# define STM32_WARN_RO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_WARN_WO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Write-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_NOT_IMPL_REG(offset, size)      \
        hw_error("%s: Not implemented yet 0x%x - size %u\n", __FUNCTION__, (int)offset, size)
void stm32_hw_warn(const char *fmt, ...)
    __attribute__ ((__format__ (__printf__, 1, 2)));

#define stm32_unimp(x...) qemu_log_mask(LOG_UNIMP, x)

enum {
    STM32_PERIPH_UNDEFINED = -1,
    STM32_RCC_PERIPH = 0,
    STM32_GPIOA,
    STM32_GPIOB,
    STM32_GPIOC,
    STM32_GPIOD,
    STM32_GPIOE,
    STM32_GPIOF,
    STM32_GPIOG,
    STM32_GPIOH,
    STM32_GPIOI,
    STM32_SYSCFG,
    STM32_AFIO_PERIPH,
    STM32_UART1,
    STM32_UART2,
    STM32_UART3,
    STM32_UART4,
    STM32_UART5,
    STM32_UART6,
    STM32_UART7,
    STM32_UART8,
    STM32_ADC1,
    STM32_ADC2,
    STM32_ADC3,
    STM32_DAC,
    STM32_TIM1,
    STM32_TIM2,
    STM32_TIM3,
    STM32_TIM4,
    STM32_TIM5,
    STM32_TIM6,
    STM32_TIM7,
    STM32_TIM8,
    STM32_TIM9,
    STM32_TIM10,
    STM32_TIM11,
    STM32_TIM12,
    STM32_TIM13,
    STM32_TIM14,
    STM32_BKP,
    STM32_PWR,
    STM32_I2C1,
    STM32_I2C2,
    STM32_I2C3,
    STM32_I2C4,
    STM32_I2S2,
    STM32_I2S3,
    STM32_WWDG,
    STM32_CAN1,
    STM32_CAN2,
    STM32_CAN,
    STM32_USB,
    STM32_SPI1,
    STM32_SPI2,
    STM32_SPI3,
    STM32_EXTI_PERIPH,
    STM32_SDIO,
    STM32_FSMC,
    STM32_RTC,
    STM32_CRC,
    STM32_DMA1,
    STM32_DMA2,
    STM32_DCMI_PERIPH,
    STM32_CRYP_PERIPH,
    STM32_HASH_PERIPH,
    STM32_RNG_PERIPH,
    STM32_QSPI,
    STM32_LPTIM1,

    STM32_PERIPH_COUNT,
};

// This structure contains all the clock related variables and registers defined
struct stm32f407xx_rcc{
	union{
		struct Stm32Rcc inherited;
		struct{
			SysBusDevice busdev;
			
			uint32_t osc_freq;
			uint32_t osc32_freq;
			
			MemoryRegion iomem;
			qemu_irq irq;
		};
		
			
	};
	
	//peripheral clocks
	Clk PERIPHCLK[STM32_PERIPH_COUNT], // MUST be first field after `inherited`, because Stm32Rcc's last field aliases this array
    HSICLK,
    HSECLK,
    LSECLK,
    LSICLK,
    SYSCLK,
    IWDGCLK,
    RTCCLK,
	
	PLLM, /* Applies "M" division and "N" multiplication factors for PLL */
    PLLCLK,
    PLL48CLK,

    PLLI2SM, /* Applies "M" division and "N" multiplication factors for PLLI2S */
    PLLI2SCLK,
	
	HCLK, /* Output from AHB Prescaler */
    PCLK1, /* Output from APB1 Prescaler */
    PCLK2; /* Output from APB2 Prescaler */
	
	// all clock related registers
	/* Register Values */
    uint32_t
    RCC_CIR,
    RCC_APB1ENR,
    RCC_APB2ENR;

    /* Register Field Values */
    uint32_t
    RCC_CFGR_PPRE1,
    RCC_CFGR_PPRE2,
    RCC_CFGR_HPRE,
    RCC_AHB1ENR,
    RCC_AHB2ENR,
    RCC_AHB3ENR,
    RCC_CFGR_SW,
    RCC_PLLCFGR,
    RCC_PLLI2SCFGR;

    uint8_t
    RCC_PLLCFGR_PLLM,
    RCC_PLLCFGR_PLLP,
    RCC_PLLCFGR_PLLSRC;

    uint16_t
    RCC_PLLCFGR_PLLN;

    uint8_t
    RCC_PLLI2SCFGR_PLLR,
    RCC_PLLI2SCFGR_PLLQ;

    uint16_t
    RCC_PLLI2SCFGR_PLLN;
};

/* HELPER FUNCTIONS */
struct Clk {                                                                    
    const char *name;                                                           
                                                                                
    bool enabled;                                                               
                                                                                
    uint32_t input_freq, output_freq, max_output_freq;                          
                                                                                
    uint16_t multiplier, divisor;                                               
                                                                                
    unsigned user_count;                                                        
    qemu_irq user[CLKTREE_MAX_IRQ]; /* Who to notify on change */               
                                                                                
    unsigned output_count;                                                      
    struct Clk *output[CLKTREE_MAX_OUTPUT];                                     
                                                                                
    unsigned input_count;                                                       
    int selected_input;                                                         
    struct Clk *input[CLKTREE_MAX_INPUT];                                       
}; 

/* This function enables the clock if the bit in AHB1ENR is set for the particular GPIO*/
static void stm32_rcc_periph_enable(
									struct stm32f407xx_rcc *s,
									uint32_t new_value,
									bool init,
									int periph,
									uint32_t bit_pos)
									
									
{
	printf ("rcc set 0x%x %s %d %x: %sable\n", new_value, s->PERIPHCLK[periph]->name, periph, bit_pos, (new_value & (1 << bit_pos))? "en":"dis"); 
    clktree_set_enabled(s->PERIPHCLK[periph], (new_value & (1 <<bit_pos)) != 0);
	printf ("%x\n", bit_pos);
} 

/*Register Implemention*/
// function implements the read of important bits that change the clock selection
static uint32_t stm32_rcc_RCC_CR_read(struct stm32f407xx_rcc *s)
{
	//set the clocks
	const bool PLLON = true; //clktree_is_enabled (s->PLLCLK);
	const bool HSEON = true; //clktree_is_enabled (s->HSECLK);
	const bool HSION = true; //clktree_is_enabled (s->HSICLK
	const bool PLLI2SON = true; // clktree_is_enabled (s->PLLI2SCLK);
	
	// if clock is set ready bit is set
	
	return ( 
		GET_BIT_MASK (RCC_CR_PLLRDY_BIT, PLLON) | GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLON) |
		GET_BIT_MASK (RCC_CR_HSERDY_BIT, HSEON) | GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |
		GET_BIT_MASK (RCC_CR_HSIRDY_BIT, HSION) | GET_BIT_MASK(RCC_CR_HSION_BIT, HSION) |
		GET_BIT_MASK (RCC_CR_PLLI2SRDY_BIT, PLLI2SON) | GET_BIT_MASK(RCC_CR_PLLI2SON_BIT, PLLI2SON)
		);                    	
}


/* function implements the setting of clocks if the clocks are not set already
Only one can be selected at a time 
Implemetation is for RCC_CR */

static void stm32_rcc_RCC_CR_write(struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	bool new_PLLON, new_HSEON, new_HSION, new_PLLI2SON;
	
	new_PLLON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
	/*checks if any other clock is set or PLL is already set*/
	if((clktree_is_enabled (s->PLLCLK) &&!new_PLLON) &&
		s->RCC_CFGR_SW == SW_HSE_SELECTED ) {
			printf ( "PLL cannot be disabled while it is selected as the system clock.");
			}
			/* if not sets the PLL clock*/
			clktree_set_enabled(s->PLLCLK, new_PLLON);
			
	new_HSEON=IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
	/*checks if any other clock is set or HSE is already set*/
	if((clktree_is_enabled (s->HSECLK)&& !new_HSEON) &&
	    (s->RCC_CFGR_SW == SW_HSE_SELECTED || s->RCC_CFGR_SW ==SW_PLL_SELECTED)){
		printf("HSE oscillator cannot be disabled while it is driving the system clock");	
		}
		clktree_set_enabled(s->HSECLK, new_HSEON);
		
	new_HSION=IS_BIT_SET(new_value,RCC_CR_HSION_BIT);
	/*checks if any other clock is set or HSI is already set*/
	if ((clktree_is_enabled(s->HSECLK) && !new_HSION) &&
	    (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)){
		printf("HSI oscillator cannot be disabled while it is driving the system clock");
		}
		clktree_set_enabled(s->HSICLK,new_HSION);
		 
		
	new_PLLI2SON = IS_BIT_SET (new_value, RCC_CR_PLLI2SON_BIT);
    clktree_set_enabled(s->PLLI2SCLK, new_PLLI2SON);	
	
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_CSSON_BIT, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSICAL_MASK, RCC_CR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CR_HSITRIM_MASK, RCC_CR_RESET_VALUE);
	
}


//RCC_PLLCFGR register implementation
static uint32_t stm32_rcc_RCC_PLLCFGR_read(struct stm32f407xx_rcc *s){
	return s->RCC_PLLCFGR;
}
	
static void stm32_rcc_RCC_PLLCFGR_write(struct stm32f407xx_rcc *s , uint32_t new_value,bool init)
{
	/*PLLM division factor*/ // uint8 because value ranges from 0-63
	const uint8_t new_PLLM = (new_value & RCC_PLLCFGR_PLLM_MASK) >> RCC_PLLCFGR_PLLM_START;
	if (new_PLLM <= 1){
		hw_error ("PLLM division factor cannot be 0 or 1. Given %u", new_PLLM);
	}
	
	/*PLLN multiplication factor*/ // uint16 because value ranges from 0-511
    const uint16_t new_PLLN = (new_value & RCC_PLLCFGR_PLLN_MASK)>>RCC_PLLCFGR_PLLN_START;
	
		if (new_PLLN <=1 || new_PLLN >=433) {
		hw_error("PLLN multiplication factor must be between 1 to 433(exclusive). Given: %u", new_PLLN);
	}
	
	/*PLLSRC*/
	const uint8_t new_PLLSRC = IS_BIT_SET(new_value, RCC_PLLCFGR_PLLSRC_BIT);
	
	/*PLLP division factor*/
	const uint8_t new_PLLP = 2 + (2 * ((new_value & RCC_PLLCFGR_PLLP_MASK) >> RCC_PLLCFGR_PLLP_START));
	
	//incorrect writes warning
	
	if (init ==false){
		const bool are_disabled = (!clktree_is_enabled(s->PLLCLK));
		if(are_disabled == false) { // PLLM is already running
			const char *warning_fmt = "Can only change %s while PLL is disabled\n";
			if (new_PLLM != s->RCC_PLLCFGR_PLLM) {
				printf(warning_fmt, "PLLM"); // new_value is not same as value set in register
			}
			if (new_PLLN != s->RCC_PLLCFGR_PLLN){
				printf(warning_fmt,"PLLN");
			}
			if (new_PLLSRC != s->RCC_PLLCFGR_PLLSRC){
				printf(warning_fmt, "PLLSRC");
			}
	    }
    }
	
	// save the new clock values
	s->RCC_PLLCFGR =new_value;
	
	/*set the new values*/
	s->RCC_PLLCFGR_PLLM = new_PLLM;
	s->RCC_PLLCFGR_PLLN = new_PLLN;
	clktree_set_scale(s->PLLM, new_PLLN, new_PLLM);
	
	s->RCC_PLLCFGR_PLLSRC =new_PLLSRC;
	clktree_set_selected_input(s->PLLM, new_PLLSRC);
	
	s->RCC_PLLCFGR_PLLP =new_PLLP;
	clktree_set_scale(s->PLLCLK, 1, new_PLLP);
	
	WARN_UNIMPLEMENTED(new_value, RCC_PLLCFGR_PLLQ_MASK, RCC_PLLCFGR_RESET_VALUE);
	
	}



//RCC_CFGR register implementation
static uint32_t stm32_rcc_RCC_CFGR_read(struct stm32f407xx_rcc *s)
{
	return 
	(s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
	(s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
	(s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START)|
	(s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
	(s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}

static void stm32_rcc_RCC_CFGR_write(struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	/*PPRE2
	  scaling is done by 2/4/8/16 for APB2 */
	s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
	if(s->RCC_CFGR_PPRE2 < 0x4) {
		clktree_set_scale (s->PCLK2, 1, 1);
	}
	else{
		clktree_set_scale(s->PCLK2, 1, 2 * (s->RCC_CFGR_PPRE2 - 3));
	}
	
	/* PPRE1 
	   scaling is done by 2/4/8/16 for APB1 */
	s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_START) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
		clktree_set_scale(s->PCLK1, 1, 1);
	}	
	else{
		clktree_set_scale(s->PCLK1, 1, 2 * (s->RCC_CFGR_PPRE1 - 3));
	}
	
	
	/*HPRE
	  scaling is done by 2/4/8/16..../512 for AHB */
	s->RCC_CFGR_HPRE =(new_value & RCC_CFGR_HPRE_START) >> RCC_CFGR_HPRE_START;
	if(s->RCC_CFGR_HPRE < 8) {
		clktree_set_scale (s->HCLK, 1, 1);
	}
     else {
		clktree_set_scale(s->HCLK, 1, 2* (s->RCC_CFGR_HPRE - 7));
     }		
	
	/*sw*/
	
	s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
	switch(s->RCC_CFGR_SW){
	case 0x0:
	case 0x1:
	case 0x2:
			clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
			break;
	default:
           hw_error("Invalid input selected for SYSCLK");
		   break;
		   
	}	   
	WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CFGR_I2SSRC_BIT, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_RTCPRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_SWS_MASK, RCC_CFGR_RESET_VALUE);	   
	
}	

// RCC_CIR yet to be implemented		
static uint32_t stm32_rcc_RCC_CIR_read(struct stm32f407xx_rcc *s)
{
    return s->RCC_CIR;
}

static void stm32_rcc_RCC_CIR_write(struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSC_BIT, RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYF_BIT , RCC_CIR_RESET_VALUE);
}

//RCC_AHB1ENR register implementation

static uint32_t stm32_rcc_RCC_AHB1ENR_read (struct stm32f407xx_rcc *s)
{
	return s->RCC_AHB1ENR;
}

static void stm32_rcc_RCC_AHB1ENR_write (struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	clktree_set_enabled(s->PERIPHCLK[STM32_DMA2], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA2EN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_DMA1], IS_BIT_SET(new_value, RCC_AHB1ENR_DMA1EN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_CRC],  IS_BIT_SET(new_value, RCC_AHB1ENR_CRCEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOI],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOH],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT)); 
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOG],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOF],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_GPIOE],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_GPIOD],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_GPIOC],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_GPIOB],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	clktree_set_enabled(s->PERIPHCLK[STM32_GPIOA],IS_BIT_SET(new_value, RCC_AHB1ENR_GPIOIEN_BIT));
	
	// collect the new value
	s->RCC_AHB1ENR = new_value;
	
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSULPIEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_BKPSRAMEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_CCMDATARAMEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_OTGHSEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACPTPEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACRXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB1ENR_ETHMACTXEN_BIT, RCC_AHB1ENR_RESET_VALUE);
		
}

//RCC_AHB2ENR implementation
static uint32_t stm32_rcc_RCC_AHB2ENR_Read  (struct stm32f407xx_rcc *s)
{
	return s->RCC_AHB2ENR;
}

static void stm32_rcc_RCC_AHB2ENR_write (struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
  clktree_set_enabled(s->PERIPHCLK[STM32_DCMI_PERIPH], IS_BIT_SET (new_value, RCC_AHB2ENR_DCMIEN_BIT));
  clktree_set_enabled(s->PERIPHCLK[STM32_CRYP_PERIPH], IS_BIT_SET (new_value, RCC_AHB2ENR_CRYPEN_BIT));  
  clktree_set_enabled(s->PERIPHCLK[STM32_HASH_PERIPH], IS_BIT_SET (new_value, RCC_AHB2ENR_HASHEN_BIT));
  clktree_set_enabled(s->PERIPHCLK[STM32_RNG_PERIPH], IS_BIT_SET (new_value, RCC_AHB2ENR_RNGEN_BIT));
  
  s->RCC_AHB2ENR =new_value;
  
  WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHB2ENR_OTGFSEN_BIT, RCC_AHB2ENR_RESET_VALUE);

}

// RCC_AHB3ENR implementation
static uint32_t stm32_rcc_RCC_AHB3ENR_read (struct stm32f407xx_rcc *s)
{
	return s->RCC_AHB3ENR;
}

static void stm32_rcc_RCC_AHB3ENR_write (struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	clktree_set_enabled(s->PERIPHCLK[STM32_FSMC], IS_BIT_SET(new_value, RCC_AHB3ENR_FSMCEN_BIT));
	
	s->RCC_AHB3ENR = new_value;
	
}

static void stm32_rcc_RCC_APB2ENR_write(struct stm32f407xx_rcc *s, uint32_t new_value, bool init)

{
	stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG, RCC_APB2ENR_SYSCFGEN_BIT);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_APB2ENR_USART1EN_BIT, RCC_APB2ENR_RESET_VALUE);
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_APB2ENR_USART6EN_BIT, RCC_APB2ENR_RESET_VALUE);


s->RCC_APB2ENR =new_value & RCC_APB2ENR_MASK;	

}

static void stm32_rcc_RCC_APB1ENR_write (struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	stm32_rcc_periph_enable (s, new_value, init, STM32_PWR, RCC_APB1ENR_PWREN_BIT);
	
	s->RCC_APB1ENR = new_value & 0x36fec9ff; //mask calculated for APB1
}


static uint32_t stm32_rcc_RCC_BDCR_read (struct stm32f407xx_rcc *s)

{
	bool lseon = clktree_is_enabled(s->LSECLK);
	
	return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) | 
	GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon) | 0x100; /*bypass/force LSE*/
}


static void stm32_rcc_RCC_BDCR_writeb0(struct stm32f407xx_rcc *s, uint8_t new_value, bool init)
{
	clktree_set_enabled( s->LSECLK, IS_BIT_SET(new_value, RCC_BDCR_LSEON_BIT));
	
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_LSEBYP_BIT, RCC_BDCR_RESET_VALUE);
}

static void stm32_rcc_RCC_BDCR_write( struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	stm32_rcc_RCC_BDCR_writeb0(s, new_value & 0xff, init);
	
	WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_BDRST_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_RTCEN_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RESET_VALUE);
}

/*same as CR register*/
static uint32_t stm32_rcc_RCC_CSR_read(struct stm32f407xx_rcc *s)
{
	bool lseon = clktree_is_enabled(s->LSICLK);
	return  GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, lseon) | 
	GET_BIT_MASK(RCC_CSR_LSION_BIT, lseon);
	
}

static void stm32_rcc_RCC_CSR_write(struct stm32f407xx_rcc *s, uint32_t new_value, bool init)
{
	clktree_set_enabled(s->LSICLK, IS_BIT_SET(new_value, RCC_CSR_LSION_BIT));
}

/* define the cases where for each offset which function to be called*/

static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
	struct stm32f407xx_rcc *s = (struct stm32f407xx_rcc *) opaque;
	
	switch (offset) {
		
		case RCC_CR_OFFSET:
			return stm32_rcc_RCC_CR_read(s);
		case RCC_PLLCFGR_OFFSET:
			return stm32_rcc_RCC_PLLCFGR_read(s);
		case RCC_CFGR_OFFSET:
			return stm32_rcc_RCC_CFGR_read(s);
		case RCC_CIR_OFFSET:
			 return stm32_rcc_RCC_CIR_read(s);
		case RCC_AHB1RSTR_OFFSET:
		case RCC_AHB2RSTR_OFFSET:
		case RCC_AHB3RSTR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		case RCC_APB1RSTR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		case RCC_APB2RSTR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		case RCC_AHB1ENR_OFFSET:
			return stm32_rcc_RCC_AHB1ENR_read(s);
		case RCC_AHB2ENR_OFFSET:
			return stm32_rcc_RCC_AHB2ENR_Read(s);
		case RCC_AHB3ENR_OFFSET:
			return stm32_rcc_RCC_AHB3ENR_read(s);
		case RCC_APB1ENR_OFFSET:
			 return s->RCC_APB1ENR;
		case RCC_APB2ENR_OFFSET:
			 return s->RCC_APB2ENR;
		case RCC_AHB1LPENR_OFFSET:
		case RCC_AHB2LPENR_OFFSET:
		case RCC_AHB3LPENR_OFFSET:
		case RCC_APB1LPENR_OFFSET:
		case RCC_APB2LPENR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		case RCC_BDCR_OFFSET:
			return stm32_rcc_RCC_BDCR_read(s);
		case RCC_CSR_OFFSET:
			return stm32_rcc_RCC_CSR_read(s);
		case RCC_SSCGR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		case RCC_PLLI2SCFGR_OFFSET:
			WARN_UNIMPLEMENTED_REG(offset);
			return 0;
		default:
		    WARN_UNIMPLEMENTED_REG(offset);
			break;
		
	}
	return 0;
}

static void stm32_rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value)
{
    struct stm32f407xx_rcc *s = (struct stm32f407xx_rcc *)opaque;

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_PLLCFGR_OFFSET:
            stm32_rcc_RCC_PLLCFGR_write(s, value, false);
            break;
        case RCC_CFGR_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CIR_OFFSET:
            stm32_rcc_RCC_CIR_write(s, value, false);
            break;
        case RCC_APB1RSTR_OFFSET:
            qemu_log_mask(LOG_UNIMP, "Unimplemented write: RCC_APB1RSTR_OFFSET 0x%x\n", (uint32_t)value);
            break;
        case RCC_APB2RSTR_OFFSET:
            qemu_log_mask(LOG_UNIMP, "Unimplemented write: RCC_APB2RSTR_OFFSET 0x%x\n", (uint32_t)value);
            break;
        case RCC_AHB3RSTR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_AHB1ENR_OFFSET:
            stm32_rcc_RCC_AHB1ENR_write(s, value, false);
            break;
        case RCC_AHB2ENR_OFFSET:
            stm32_rcc_RCC_AHB2ENR_write(s, value, false);
            break;
        case RCC_AHB3ENR_OFFSET:
            stm32_rcc_RCC_AHB3ENR_write(s, value, false);
            break;
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_APB2ENR_OFFSET:
            stm32_rcc_RCC_APB2ENR_write(s, value, false);
            break;
        case RCC_APB1ENR_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_AHB1LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB1LPENR_OFFSET\n");
            break;
        case RCC_AHB2LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB2LPENR_OFFSET\n");
            break;
        case RCC_AHB3LPENR_OFFSET:
            stm32_unimp("Unimplemented: RCC_AHB3LPENR_OFFSET\n");
            break;
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        case RCC_SSCGR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_PLLI2SCFGR_OFFSET:
            stm32_unimp("Unimplemented: RCC_PLLI2SCFGR_OFFSET\n");
            break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            stm32_unimp("Unimplemented: RCC read from register at offset %lu", offset);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size) //revisit
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value);
            break;
        case 1: {
            hwaddr woffset = offset & ~0x3;
            uint32_t boffset = offset & 0x3;
            uint32_t val = stm32_rcc_readw(opaque, woffset);
            val &= ~(0xff << boffset);
            val |= (value << boffset);
            stm32_rcc_writew(opaque, woffset, value);
        } break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_rcc_reset(DeviceState *dev)
{
    struct stm32f407xx_rcc *s = DO_UPCAST(struct stm32f407xx_rcc, busdev, SYS_BUS_DEVICE(dev));

    stm32_rcc_RCC_CR_write(s, RCC_CR_RESET_VALUE, true);
    stm32_rcc_RCC_PLLCFGR_write(s, RCC_PLLCFGR_RESET_VALUE, true);
    stm32_rcc_RCC_CFGR_write(s, RCC_CFGR_RESET_VALUE, true);
    stm32_rcc_RCC_APB2ENR_write(s, RCC_APB2ENR_RESET_VALUE, true);
    stm32_rcc_RCC_APB1ENR_write(s, RCC_APB1ENR_RESET_VALUE, true);
    stm32_rcc_RCC_BDCR_write(s, RCC_BDCR_RESET_VALUE, true);
    stm32_rcc_RCC_CSR_write(s, RCC_CSR_RESET_VALUE, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    struct stm32f407xx_rcc *s = (struct stm32f407xx_rcc *)opaque;

    uint32_t hclk_freq = 0;

    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if (hclk_freq > 0) {
        //ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = NANOSECONDS_PER_SECOND / hclk_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
#endif
}

/* DEVICE INITIALIZATION */

/* Set up the clock tree */

static void stm32_rcc_init_clk(struct stm32f407xx_rcc *s)
{
	 int i;
    qemu_irq *hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }
	
	/* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", HSI_FREQ, false);
    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);
	
	s->IWDGCLK = clktree_create_clk("IWDGCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0,
                                    s->LSICLK, NULL);
    s->RTCCLK = clktree_create_clk("RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ,
                                   CLKTREE_NO_INPUT, s->LSECLK, s->LSICLK, s->HSECLK, NULL);
								   
	s->PLLM = clktree_create_clk("PLLM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK,
                                 s->HSECLK, NULL);	
    s->PLLCLK = clktree_create_clk("PLLCLK", 1, 2, false, 120000000, 0, s->PLLM, NULL);								 
	s->PLL48CLK = clktree_create_clk("PLL48CLK", 1, 1, false, 48000000, 0, s->PLLM, NULL);							   
								   
	//s->PLLI2SM = clktree_create_clk("PLLI2SM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->PLLM, NULL);
    //s->PLLI2SCLK = clktree_create_clk("PLLI2SCLK", 1, 2, false, 120000000, 0, s->PLLI2SM, NULL);

    s->SYSCLK = clktree_create_clk("SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   s->HSICLK, s->HSECLK, s->PLLCLK, NULL);							   
								   
    // HCLK: to AHB bus, core memory and DMA
    s->HCLK = clktree_create_clk("HCLK", 0, 1, true, 168000000, 0, s->SYSCLK, NULL);
    clktree_adduser (s->HCLK, hclk_upd_irq[0]);								   
								   
	    // Clock source for APB1 peripherals:
    s->PCLK1 = clktree_create_clk("PCLK1", 0, 1, true, 30000000, 0, s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    s->PCLK2 = clktree_create_clk("PCLK2", 0, 1, true, 60000000, 0, s->HCLK, NULL);							   
								   
	/* Peripheral clocks */
    s->PERIPHCLK[STM32_GPIOA] =
        clktree_create_clk("GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOB] =
        clktree_create_clk("GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOC] =
        clktree_create_clk("GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOD] =
        clktree_create_clk("GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOE] =
        clktree_create_clk("GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOF] =
        clktree_create_clk("GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOG] =
        clktree_create_clk("GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOH] =
        clktree_create_clk("GPIOH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOI] =
        clktree_create_clk("GPIOI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);							   
 
    s->PERIPHCLK[STM32_SYSCFG] =
        clktree_create_clk("SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);	

}

//------------------------------------------------------------------------------------------
static void stm32f407xx_rcc_realize (DeviceState *dev, Error **errp) {
	struct stm32f407xx_rcc *s =OBJECT_CHECK(struct stm32f407xx_rcc, dev, "stm32f407xx_rcc");
	SysBusDevice *busdev = SYS_BUS_DEVICE(dev);
	memory_region_init_io(&s->iomem, OBJECT(s), &stm32_rcc_ops, s,
						  "rcc", 0x40023BFF - 0x40023800 + 1);
						  
    sysbus_init_mmio(busdev, &s->iomem);
	
	sysbus_init_irq(busdev, &s->irq);
	
	stm32_rcc_init_clk(s);
	
}

static Property stm32_rcc_properties[] = {
	DEFINE_PROP_UINT32("osc_freq", struct stm32f407xx_rcc, osc_freq, 0),
	DEFINE_PROP_UINT32("osc32_freq", struct stm32f407xx_rcc, osc32_freq, 0),
	DEFINE_PROP_END_OF_LIST()
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc =DEVICE_CLASS(klass);
	
	dc->reset = stm32_rcc_reset;
	dc->props = stm32_rcc_properties;
	dc->realize = stm32f407xx_rcc_realize;
}

static TypeInfo stm32_rcc_info = {
	.name = "stm32f407xx_rcc",
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct stm32f407xx_rcc),
	.class_init = stm32_rcc_class_init
};

static void stm32_rcc_register_types(void)
{
	type_register_static(&stm32_rcc_info);
}
type_init(stm32_rcc_register_types)