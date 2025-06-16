//
// Created by ashan on 07/06/2025.
//
#include "stm32f746xx_spi_driver.h"

#include <stdlib.h>


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if(EnorDi == ENABLE) {
        if(pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_EN();
        } else if(pSPIx == SPI4) {
            SPI4_PCLK_EN();
        } else if(pSPIx == SPI5) {
            SPI5_PCLK_EN();
        } else if(pSPIx == SPI6) {
            SPI6_PCLK_EN();
        }
    } else {
        if(pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else if(pSPIx == SPI4) {
            SPI4_PCLK_DI();
        } else if(pSPIx == SPI5) {
            SPI5_PCLK_DI();
        } else if(pSPIx == SPI6) {
            SPI6_PCLK_DI();
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {

    uint32_t tempreg = 0;
    uint32_t tempreg2 = 0;
    // Enable the peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure the SPI device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
    // Configure the SPI bus configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {

        tempreg &= ~ (pSPIHandle->SPIConfig.SPI_BusConfig << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {

        tempreg &= ~(1 << SPI_CR1_BIDIOE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }
    // Configure the SPI clock speed
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // Configure the SPI data frame format
    tempreg2 |=(pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS);

    // Configure the SPI software slave management
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // Configure the SPI clock polarity
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // Set FRXTH if using 8-bit data size
    if (pSPIHandle->SPIConfig.SPI_DS == SPI_DS_8BITS) {
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);
    } else {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
    }

    pSPIHandle->pSPIx->CR1 = tempreg;
    pSPIHandle->pSPIx->CR2 |= tempreg2;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    pSPIx->CR1 = 0; // Reset the control register
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName) {

    if (pSPIx->SR & FlagName) {
        return FLAGSET;
    }
    return FLAGRESET ;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
                // Loop until all data is sent
                while (Len > 0) {

                    // Wait until TXE (Transmit buffer empty) flag is set
                    while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAGRESET);

                    // Check if DS (Data Frame Format) is 16-bit
                    if(((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {

                        // Load 16 bits of data into the data register
                        pSPIx->DR = *(uint16_t*)pTxBuffer;
                        // Decrement length by 2 bytes
                        Len--;
                        Len--;
                        // Increment buffer pointer by 2 bytes
                        (uint16_t*)pTxBuffer++;
                    }
                    else {
                        // Load 8 bits of data into the data register
                        // Note: The data register is 16 bits wide, so we can write 8 bits directly
                      *((__vo uint8_t*)&pSPIx->DR) = *pTxBuffer;

                        // Decrement length by 1 byte
                        Len--;
                        // Increment buffer pointer by 1 byte
                        pTxBuffer++;
                    }
                }

            }

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
    // Implementation for receiving data through SPI
    // Loop until all data is sent
    while (Len > 0) {

        // Wait until RXNE (Transmit buffer empty) flag is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAGRESET);

        // Check if DS is 16-bit
        if(((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {

            // Load pTxBuffer with 16 bits of data from the data register
            *((uint16_t*)pRxBuffer )= pSPIx->DR ;
            // Decrement length by 2 bytes
            Len--;
            Len--;
            // Increment buffer pointer by 2 bytes
            (uint16_t*)pRxBuffer++;
        }
        else {
            // Load pTxBuffer with 8 bits of data from the data register
            *pRxBuffer = *((__vo uint8_t*)&pSPIx->DR);
            // Decrement length by 1 byte
            Len--;
            // Increment buffer pointer by 1 byte
            pRxBuffer++;
        }
    }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    // Implementation for configuring SPI interrupt
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // Implementation for configuring SPI interrupt priority
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
    // Implementation for handling SPI interrupts
    // This function should check the status of the SPI peripheral and handle the interrupt accordingly
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE); // Set the SPE bit to enable the SPI peripheral
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); // Clear the SPE bit to disable the SPI peripheral
    }
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI); // Set the SSI bit to enable internal slave select
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); // Clear the SSI bit to disable internal slave select
    }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE); // Set the SSOE bit to enable software slave management
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE); // Clear the SSOE bit to disable software slave management
    }
}
