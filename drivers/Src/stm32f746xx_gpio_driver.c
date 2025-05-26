/*
 * stm32f746xx_gpio_driver.c
 *
 *  Created on: May 24, 2025,
 *      Author: Ashan
 */

#include "stm32f746xx_gpio_driver.h"


/*Peripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

  if(EnorDi == ENABLE){

      if (pGPIOx == GPIOA) {

          GPIOA_PCLK_EN();
      }
      else if(pGPIOx == GPIOB) {

          GPIOB_PCLK_EN();
      }
      else if(pGPIOx == GPIOC) {

          GPIOC_PCLK_EN();
      }
      else if(pGPIOx == GPIOD) {

          GPIOD_PCLK_EN();
      }
      else if(pGPIOx == GPIOE) {

          GPIOE_PCLK_EN();
      }
      else if(pGPIOx == GPIOF) {

          GPIOF_PCLK_EN();
      }

      else if(pGPIOx == GPIOG) {

          GPIOG_PCLK_EN();
      }
      else if(pGPIOx == GPIOH) {

          GPIOH_PCLK_EN();
      }
      else if(pGPIOx == GPIOI) {

          GPIOI_PCLK_EN();
      }

      else if(pGPIOx == GPIOJ) {

          GPIOJ_PCLK_EN();
      }
      else if(pGPIOx == GPIOK) {

          GPIOK_PCLK_EN();
      }
  }
  else {

      if (pGPIOx == GPIOA) {

          GPIOA_PCLK_DI();
      }
      else if(pGPIOx == GPIOB) {

          GPIOB_PCLK_DI();
      }
      else if(pGPIOx == GPIOC) {

          GPIOC_PCLK_DI();
      }
      else if(pGPIOx == GPIOD) {

          GPIOD_PCLK_DI();
      }
      else if(pGPIOx == GPIOE) {

          GPIOE_PCLK_DI();
      }
      else if(pGPIOx == GPIOF) {

          GPIOF_PCLK_DI();
      }

      else if(pGPIOx == GPIOG) {

          GPIOG_PCLK_DI();
      }
      else if(pGPIOx == GPIOH) {

          GPIOH_PCLK_DI();
      }
      else if(pGPIOx == GPIOI) {

          GPIOI_PCLK_DI();
      }

      else if(pGPIOx == GPIOJ) {

          GPIOJ_PCLK_DI();
      }
      else if(pGPIOx == GPIOK) {

          GPIOK_PCLK_DI();
      }
  }
}

/*Init and De Init*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

   uint32_t temp = 0;

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG) {

        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else {

    }

    temp = 0;

    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    temp =pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
        if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) < 8) {
            temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            pGPIOHandle->pGPIOx->AFR[0] |= temp;
            temp = 0;
        }


        else {
            temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
            pGPIOHandle->pGPIOx->AFR[1] |= temp;
        }
    }
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

    if (pGPIOx == GPIOA) {

        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB) {

        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC) {

        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD) {

        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE) {

        GPIOE_REG_RESET();
    }
    else if(pGPIOx == GPIOF) {

        GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG) {

        GPIOG_REG_RESET();
    }
    else if(pGPIOx == GPIOH) {

        GPIOH_REG_RESET();
    }
    else if(pGPIOx == GPIOI) {

        GPIOI_REG_RESET();
    }
    else if(pGPIOx == GPIOJ) {

        GPIOJ_REG_RESET();
    }
    else if(pGPIOx == GPIOK) {

        GPIOK_REG_RESET();
    }
}

/*Data Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {


    uint8_t value = 0;
    value = pGPIOx->IDR & (1 << PinNumber);
    return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {


    uint16_t value = 0;
    value = pGPIOx->IDR;
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

    if(Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= 1 << PinNumber;
    }
    else {

        pGPIOx->ODR &=~(1 << PinNumber);
    }
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

  pGPIOx->ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

    pGPIOx->ODR ^= (1 << PinNumber);

}

/*IRQ Configuration and ISR Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi) {


}
void GPIO_IRQHandler(uint8_t PinNumber) {


}