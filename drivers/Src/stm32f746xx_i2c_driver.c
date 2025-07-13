//
// Created by ashan on 27/06/2025.
//

#include "stm32f746xx_i2c_driver.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static  void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
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
    // 1. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Wait until START condition is generated (check START bit in CR2)
    while (!(pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_START)));

    // 3. Send address phase
    I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Wait until address is sent (ADDR flag in ISR)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ISR_ADDR));

    // 5. Clear ADDR flag by reading ISR and then CR2 (as per reference manual)
    (void)pI2CHandle->pI2Cx->ISR;
    (void)pI2CHandle->pI2Cx->CR2;

    // 6. Send data bytes
    for (uint32_t i = 0; i < Len; i++) {
        // Wait until TXIS (Transmit interrupt status) flag is set
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ISR_TXE));
        // Write data to TXDR
        pI2CHandle->pI2Cx->TXDR = pTxBuffer[i];
    }

    // 7. Wait until transfer complete (TC flag)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ISR_TC));

    // 8. Generate STOP condition 
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);



}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    // Generate a START condition
    pI2Cx->CR1 |= (1 << I2C_CR2_START);
}

static  void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {

    SlaveAddr = SlaveAddr << 1; // Shift left to set the address in the correct position
    SlaveAddr &= ~(1 << 0); // Clear the LSB to indicate a write operation
    pI2Cx->CR2 |= SlaveAddr; // Set the address in the CR2 register


}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
    // Generate a STOP condition
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
    if (pI2Cx->ISR & FlagName) {
        return 1; // Flag is set
    } else {
        return 0; // Flag is not set
    }
}