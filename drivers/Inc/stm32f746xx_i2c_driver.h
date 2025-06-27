//
// Created by ashan on 27/06/2025.
//

#ifndef STM32F746XX_I2C_DRIVER_H
#define STM32F746XX_I2C_DRIVER_H

#include "stm32f746xx.h"
#include <stdint.h>

/**
 * @brief I2C Device Mode
 *
 * @note These macros define the possible device modes for the I2C peripheral.
 */
typedef struct {

    uint32_t I2C_SCLSpeed;        /*!< Specifies the I2C SCL speed. Possible values from @ref I2C_SCL_Speed */
    uint8_t I2C_DeviceAddress;    /*!< Specifies the I2C device address */
    uint8_t I2C_ACKControl;       /*!< Specifies whether to enable or disable ACK. Possible values from @ref I2C_ACK_Control */
    uint16_t I2C_FMDutyCycle;    /*!< Specifies the I2C Fast Mode Duty Cycle. Possible values from @ref I2C_FM_Duty_Cycle */

}I2C_Config_t;

typedef struct {

    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;

}I2C_Handle_t;

#define I2C_SCL_SPEED_SM 100000U /*!< Standard Mode speed */
#define I2C_SCL_SPEED_FM4K 400000U /*!< Fast Mode speed */
#define I2C_SCL_SPEED_FM2K 200000U /*!< Fast Mode speed */

/**
 * @brief I2C ACK Control
 *
 * @note These macros define the possible ACK control configurations for the I2C peripheral.
 */

#define I2C_ACK_ENABLE 1 /*!< Enable ACK */
#define I2C_ACK_DISABLE 0 /*!< Disable ACK */

/**
 * @brief I2C Fast Mode Duty Cycle
 *
 * @note These macros define the possible Fast Mode Duty Cycle configurations for the I2C peripheral.
 */
#define I2C_FM_DUTY_2 0 /*!< Fast Mode Duty Cycle 2 */
#define I2C_FM_DUTY_16_9 1 /*!< Fast Mode Duty Cycle 16/9 */


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif //STM32F746XX_I2C_DRIVER_H
