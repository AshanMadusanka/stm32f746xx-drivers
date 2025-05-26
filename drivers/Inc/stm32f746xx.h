//
// Created by Ashan on 19/05/2025.
//

#ifndef STM32F746XX_H
#define STM32F746XX_H


#include <stdint.h>

#define __vo volatile

/*  Base addresses of FLASH and SRAM */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20010000U
#define SRAM2_BASEADDR 0x2004C000U
#define ROM_BASEADDR   0x1FF00000U

/* Base addresses of AHBx and APBx bus peripherals */

#define PERIPH_BASEADDR 0x40000000U
#define APB1_BASEADDR  (PERIPH_BASEADDR)
#define APB2_BASEADDR  0x40010000U
#define AHB1_BASEADDR  0x40020000U
#define AHB2_BASEADDR  0x50000000U
#define AHB3_BASEADDR  0xA0000000U

/* Base addresses of peripherals which are hanging on AHB1 Bus */

#define GPIOA_BASEADDR (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR (AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR (AHB1_BASEADDR + 0x2800)
#define RCC_BASEADDR   (AHB1_BASEADDR + 0x3800)


/* Base addresses of peripherals which are hanging on APB1 Bus */

#define I2C1_BASEADDR (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1_BASEADDR + 0x5C00)
#define I2C4_BASEADDR (APB1_BASEADDR + 0x6000)

#define SPI2_BASEADDR (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1_BASEADDR + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 Bus */

#define SPI1_BASEADDR (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR (APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR (APB2_BASEADDR + 0x5000)
#define SPI6_BASEADDR (APB2_BASEADDR + 0x5400)

#define USART1_BASEADDR (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR  (APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR (APB2_BASEADDR + 0x3800)


/*********************Peripheral register definition structures**************/

typedef struct {

   __vo uint32_t MODER;    /*!< GPIO port mode register, Address offset: 0x00 */
   __vo uint32_t OTYPER;   /*!< GPIO port output type register, Address offset: 0x04 */
   __vo uint32_t OSPEEDR; /*!< GPIO port output speed register, Address offset: 0x08 */
   __vo uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register, Address offset: 0x0C */
   __vo uint32_t IDR;     /*!< GPIO port input data register, Address offset: 0x10 */
   __vo uint32_t ODR;     /*!< GPIO port output data register, Address offset: 0x14 */
   __vo uint32_t BSRR;    /*!< GPIO port bit set/reset register, Address offset: 0x18 */
   __vo uint32_t LCKR;    /*!< GPIO port configuration lock register, Address offset: 0x1C */
   __vo uint32_t AFR[2];  /*!< GPIO alternate function low/high registers, Address offset: 0x20-0x24 */

}GPIO_RegDef_t;

typedef struct {

   __vo uint32_t CR1;       /*!< I2C Control register 1, Address offset: 0x00 */
   __vo uint32_t CR2;       /*!< I2C Control register 2, Address offset: 0x04 */
   __vo uint32_t OAR1;      /*!< I2C Own address register 1, Address offset: 0x08 */
   __vo uint32_t OAR2;      /*!< I2C Own address register 2, Address offset: 0x0C */
   __vo uint32_t TIMINGR;    /*!< I2C Timing register, Address offset: 0x10 */
   __vo uint32_t TIMEOUTR;   /*!< I2C Timeout register, Address offset: 0x14 */
   __vo uint32_t ISR;       /*!< I2C Interrupt and status register, Address offset: 0x18 */
   __vo uint32_t ICR;       /*!< I2C Interrupt clear register, Address offset: 0x1C */
   __vo uint32_t PECR;      /*!< I2C PEC register, Address offset: 0x20 */
   __vo uint32_t RXDR;      /*!< I2C Receive data register, Address offset: 0x24 */
   __vo uint32_t TXDR;      /*!< I2C Transmit data register, Address offset: 0x28 */

}I2C_RegDef_t;

typedef struct {
    __vo uint32_t CR1;        /*!< SPI Control register 1, Address offset: 0x00 */
    __vo uint32_t CR2;       /*!< SPI Control register 2, Address offset: 0x04 */
    __vo uint32_t SR;        /*!< SPI Status register, Address offset: 0x08 */
    __vo uint32_t DR;        /*!< SPI Data register, Address offset: 0x0C */
    __vo uint32_t CRCPR;     /*!< SPI CRC polynomial register, Address offset: 0x10 */
    __vo uint32_t RXCRCR;    /*!< SPI RX CRC register, Address offset: 0x14 */
    __vo uint32_t TXCRCR;    /*!< SPI TX CRC register, Address offset: 0x18 */
    __vo uint32_t I2SCFGR;   /*!< SPI I2S configuration register, Address offset: 0x1C */
    __vo uint32_t I2SPR;     /*!< SPI I2S prescaler register, Address offset: 0x20 */

}SPI_RegDef_t;

typedef struct {

   __vo uint32_t CR1;       /*!< USART Control register 1, Address offset: 0x00 */
   __vo uint32_t CR2;       /*!< USART Control register 2, Address offset: 0x04 */
   __vo uint32_t CR3;       /*!< USART Control register 3, Address offset: 0x08 */
   __vo uint32_t BRR;       /*!< USART Baud rate register, Address offset: 0x0C */
   __vo uint32_t GTPR;      /*!< USART Guard time and prescaler register, Address offset: 0x10 */
   __vo uint32_t RTOR;      /*!< USART Receiver timeout register, Address offset: 0x14 */
   __vo uint32_t RQR;       /*!< USART Request register, Address offset: 0x18 */
   __vo uint32_t ISR;       /*!< USART Interrupt and status register, Address offset: 0x1C */
   __vo uint32_t ICR;       /*!< USART Interrupt flag clear register, Address offset: 0x20 */
   __vo uint32_t RDR;       /*!< USART Receive data register, Address offset: 0x24 */
   __vo uint32_t TDR;       /*!< USART Transmit data register, Address offset: 0x28 */

}USART_RegDef_t;

typedef struct {
  __vo uint32_t CR;        /*!< RCC clock control register, Address offset: 0x00 */
  __vo uint32_t PLLCFGR;   /*!< RCC PLL configuration register, Address offset: 0x04 */
  __vo uint32_t CFGR;      /*!< RCC clock configuration register, Address offset: 0x08 */
  __vo uint32_t CIR;       /*!< RCC clock interrupt register, Address offset: 0x0C */
  __vo uint32_t AHB1RSTR; /*!< RCC AHB1 peripheral reset register, Address offset: 0x10 */
  __vo uint32_t AHB2RSTR; /*!< RCC AHB2 peripheral reset register, Address offset: 0x14 */
  __vo uint32_t AHB3RSTR; /*!< RCC AHB3 peripheral reset register, Address offset: 0x18 */
 uint32_t RESERVED0; /*!< Reserved, Address offset: 0x1C */
  __vo uint32_t APB1RSTR; /*!< RCC APB1 peripheral reset register, Address offset: 0x20 */
  __vo uint32_t APB2RSTR; /*!< RCC APB2 peripheral reset register, Address offset: 0x24 */
   uint32_t RESERVED1[2]; /*!< Reserved, Address offset: 0x28-0x2C */
  __vo uint32_t AHB1ENR ;  /*!< RCC AHB1 peripheral clock enable register, Address offset: 0x30 */
  __vo uint32_t AHB2ENR;  /*!< RCC AHB2 peripheral clock enable register, Address offset: 0x34 */
  __vo uint32_t AHB3ENR;  /*!< RCC AHB3 peripheral clock enable register, Address offset: 0x38 */
   uint32_t RESERVED2; /*!< Reserved, Address offset: 0x3C */
  __vo uint32_t APB1ENR;  /*!< RCC APB1 peripheral clock enable register, Address offset: 0x40 */
  __vo uint32_t APB2ENR;  /*!< RCC APB2 peripheral clock enable register, Address offset: 0x44 */
   uint32_t RESERVED3[2]; /*!< Reserved, Address offset: 0x48-0x4C */
  __vo uint32_t AHB1LPENR; /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __vo uint32_t AHB2LPENR; /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __vo uint32_t AHB3LPENR; /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
   uint32_t RESERVED4; /*!< Reserved, Address offset: 0x5C */
  __vo uint32_t APB1LPENR; /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __vo uint32_t APB2LPENR; /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
   uint32_t RESERVED5[2]; /*!< Reserved, Address offset: 0x68-0x6C */
  __vo uint32_t BDCR;     /*!< RCC Backup domain control register, Address offset: 0x70 */
  __vo uint32_t CSR;      /*!< RCC clock control & status register, Address offset: 0x74 */
   uint32_t RESERVED6[2]; /*!< Reserved, Address offset: 0x78-0x7C */
  __vo uint32_t SSCGR;    /*!< RCC spread spectrum clock generation register, Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR; /*!< RCC PLLI2S configuration register, Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR; /*!< RCC PLLSAI configuration register, Address offset: 0x88 */
  __vo uint32_t DCKCFGR;  /*!< RCC Dedicated Clocks Configuration Register, Address offset: 0x8C */
  __vo uint32_t DCKCFGR2; /*!< RCC Dedicated Clocks Configuration Register 2, Address offset: 0x90 */

}RCC_RegDef_t;

/*Peripheral definitions (Peripheral base addresses type cast to xxx Regdef_t) */

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

/*Clock Enable Macros for GPIOx Peripherals*/

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1 << 10))

/*Clock Enable Macros for I2Cx Peripherals*/

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
#define I2C4_PCLK_EN() (RCC->APB1ENR |= (1 << 24))

/*Clock Enable Macros for SPIx Peripherals*/

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI4_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN() (RCC->APB2ENR |= (1 << 21))

/*Clock Enable Macros for USARTx Peripherals*/

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*Clock Disable Macros for GPIOx Peripherals*/

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 10))

/*Clock Disable Macros for I2Cx Peripherals*/

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
#define I2C4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 24))

/*Clock Disable Macros for SPIx Peripherals*/

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 21))

/*Clock Disable Macros for USARTx Peripherals*/

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

/*Macros to reset GPIOx Peripherals*/

#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &=~ (1 << 0));} while (0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &=~ (1 << 1));} while (0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &=~ (1 << 2));} while (0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &=~ (1 << 3));} while (0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &=~ (1 << 4));} while (0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &=~ (1 << 5));} while (0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &=~ (1 << 6));} while (0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &=~ (1 << 7));} while (0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &=~ (1 << 8));} while (0)
#define GPIOJ_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &=~ (1 << 9));} while (0)
#define GPIOK_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &=~ (1 << 10));} while (0)


/*Generic Macros*/

#define ENABLE        1
#define DISABLE       0
#define SET           ENABLE
#define RESET         DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET RESET

#include "stm32f746xx_gpio_driver.h"

#endif //STM32F746XX_H
