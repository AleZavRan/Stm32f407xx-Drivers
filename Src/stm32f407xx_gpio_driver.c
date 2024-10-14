/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Alejandro Zavala
 */

#include "stm32f407xx_gpio_driver.h"


//Peripheral Clock Setup
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[*pGPIOx]         - base address of the gpio peripheral
 * @param[EnorDi]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClkCtlr(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		} else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}

	}

}

//Init and De-Init
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the given GPIO Port.
 * 						It enables the corresponding clock
 * 						It configures: Mode (input, output, analog, interrupt),
 * 						Output type, Speed, the pull up and pull down settings
 * 						and the alternate functionality
 *
 * @param[*pGPIOHandle]         - base address of the GPIO handle structure
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0; //temp. register
	//Enable the peripheral clock
	GPIO_PeriClkCtlr(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//The non interrupt mode

		//Assigning mode to the corresponding bit position
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting



	} else{ //The interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT){ //Interrupt falling edge trigger
			//1. Configure the FTSR (Falling trigger selection register)

			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear RTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //Set

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT){ //Interrupt rising edge trigger
			//1. Configure the RTSR (Rising trigger selection register)

			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear FTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //Set

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFT){
			//1. Configure the FTSR and RTSR

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //Set
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //Set
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // These bits are written by software to select the
																		// source input for the EXTIx external interrupt.
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);



		//3. Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //Set
	}

	temp = 0;
	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2)); //Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4 * temp2);

	}

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function resets the value of given GPIO peripheral register.
 * 						To Reset a specific peripheral it's necessary to set and after clear the
 * 						required bit position on AHB1RSTR register (AHB1 Peripheral reset register).
 *
 * @param[*pGPIOx]    - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	} else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}

}

//Data read and write
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the given input pin and returns the value as an uint8_t
 *
 * @param[*pGPIOx]         - base address of the gpio peripheral
 * @param[PinNumber]        - Pin Number macros
 *
 * @return value           -The bit value stored on selected bit position of IDR (Input Data Register)
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the given input port and returns a uint16_t value
 *
 *
 * @param[*pGPIOx]         - base address of the gpio peripheral
 *
 * @return value           -The uint16_t value stored on IDR (Input Data Register)
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to the given output pin
 *
 * @param[*pGPIOx]         - base address of the gpio peripheral
 * @param[PinNumber]    - Pin Number macros
 * @param[Value]        - Value to be written on the output pin
 *
 * @return           - none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		//Write 1 to the output data register at the bitfield corresponding to the pinnumber
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//Write 0 to the output data register at the bitfield corresponding to the pinnumber
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes to the given output port
 *
 * @param[*pGPIOx]    - base address of the gpio peripheral
 * @param[Value]      - Value to be written on the output port
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the given output pin
 *
 * @param[*pGPIOx]    - base address of the gpio peripheral
 * @param[PinNumber]  - Pin Number macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ configuration and ISR handling
/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function activates the interrupt for the given IRQ number on
 * 						NVIC Registers
 *
 * @param[IRQNumber]  - base address of the gpio peripheral
 * @param[EnorDi]  	  - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){ //Configures interrupt set-enable register
		if(IRQNumber <= 31){
			//Configure ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64){
			//Configure ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96){
			//Configure ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	} else {//Configures interrupt clear-enable register
		if(IRQNumber <= 31){
			//Configure ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64){
			//Configure ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96){
			//Configure ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}


}
//IRQ Priority Configuration
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	//Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx) ) |= (IRQPriority << shift_amount);

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber){

	//Clear the EXTI PR Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){ //If the bit position of the corresponding PinNumber is set in PR
		//Clear
		EXTI->PR |= (1 << PinNumber);


	}
}
