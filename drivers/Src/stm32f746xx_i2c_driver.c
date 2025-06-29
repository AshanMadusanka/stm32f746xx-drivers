//
// Created by ashan on 27/06/2025.
//

#include "stm32f746xx_i2c_driver.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempreg = 0;

    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; // Shift left to set the address in the correct position
    tempreg |= (1 << 15); // Set the ADDR bit in the OAR1 register
    pI2CHandle->pI2Cx->OAR1 |= tempreg; // Set the device address in the OAR1 register

    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x04 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x02 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0xF << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x13 << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register
    } else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x03 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x01 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0x13 << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x2F << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register

    } else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x03 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x01 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0x09 << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x13 << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register
    }
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    // Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // Wait until the START condition is generated
    // I cant Find SB flag in the I2C_ISR register
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    // Generate a START condition
    pI2Cx->CR1 |= (1 << I2C_CR2_START);
}