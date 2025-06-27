//
// Created by ashan on 27/06/2025.
//

#include "stm32f746xx_i2c_driver.h"

void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempreg = 0;

    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; // Shift left to set the address in the correct position
    tempreg |= (1 << 15); // Set the ADDR bit in the OAR1 register
    pI2CHandle->pI2Cx->OAR1 |= tempreg; // Set the device address in the OAR1 register


}