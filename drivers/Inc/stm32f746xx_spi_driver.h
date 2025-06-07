//
// Created by ashan on 07/06/2025.
//

#ifndef STM32F746XX_SPI_DRIVER_H
#define STM32F746XX_SPI_DRIVER_H

#include "stm32f746xx.h"

/**
 * @brief Configuration structure for SPIx peripheral
 */
typedef struct {

    uint8_t SPI_DeviceMode;        /*!< Specifies the SPI device mode. Possible values from @ref SPI_Device_Mode */
    uint8_t SPI_BusConfig;         /*!< Specifies the SPI bus configuration. Possible values from @ref SPI_Bus_Config */
    uint8_t SPI_SclkSpeed;         /*!< Specifies the SPI clock speed. Possible values from @ref SPI_Sclk_Speed */
    uint8_t SPI_DFF;               /*!< Specifies the SPI data frame format. Possible values from @ref SPI_Data_Frame_Format */
    uint8_t SPI_CPOL;              /*!< Specifies the SPI clock polarity. Possible values from @ref SPI_Clock_Polarity */
    uint8_t SPI_CPHA;              /*!< Specifies the SPI clock phase. Possible values from @ref SPI_Clock_Phase */
    uint8_t SPI_SSM;               /*!< Specifies the SPI software slave management. Possible values from @ref SPI_Software_Slave_Management */

} SPI_Config_t;

/**
 * @brief Handle structure for SPIx peripheral
 */
typedef struct {

    SPI_RegDef_t *pSPIx;           /*!< Pointer to the SPIx peripheral base address */
    SPI_Config_t SPIConfig;        /*!< Configuration settings for the SPI peripheral */

} SPI_Handle_t;

/**
 * @brief Enables or disables the peripheral clock for the given SPI peripheral
 *
 * @param pSPIx Pointer to the SPI peripheral base address
 * @param EnorDi Enable or disable macro (1 to enable, 0 to disable)
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Initializes the given SPI peripheral with the specified configuration
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief Deinitializes the given SPI peripheral
 *
 * @param pSPIx Pointer to the SPI peripheral base address
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief Sends data through the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 * @param pTxBuffer Pointer to the transmit buffer
 * @param Len Length of the data to be sent
 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data through the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 * @param pRxBuffer Pointer to the receive buffer
 * @param Len Length of the data to be received
 */
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Configures the interrupt for the given IRQ number
 *
 * @param IRQNumber IRQ number to configure
 * @param EnorDi Enable or disable macro (1 to enable, 0 to disable)
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configures the priority of the given IRQ number
 *
 * @param IRQNumber IRQ number to configure
 * @param IRQPriority Priority value to set
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Handles the interrupt for the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif //STM32F746XX_SPI_DRIVER_H