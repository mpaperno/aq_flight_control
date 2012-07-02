/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#ifndef _board_h
#define _board_h

#define SD_POWER_PORT			GPIOC
#define SD_POWER_PIN			GPIO_Pin_14
#define SD_DETECT_GPIO_PORT		GPIOC
#define SD_DETECT_PIN			GPIO_Pin_13
#define SD_DETECT_PORT_SOURCE		EXTI_PortSourceGPIOC
#define SD_DETECT_PIN_SOURCE		EXTI_PinSource13
#define SD_DETECT_IRQ			EXTI15_10_IRQn
#define SD_DETECT_HANDLER		EXTI15_10_IRQHandler
#define SD_DETECT_EXTI_LINE		EXTI_Line13

enum pwmPorts {
    PWM_1 = 0,
    PWM_2,
    PWM_3,
    PWM_4,
    PWM_5,
    PWM_6,
    PWM_7,
    PWM_8,
    PWM_9,
    PWM_10,
    PWM_11,
    PWM_12,
    PWM_13,
    PWM_14,
    PWM_NUM_PORTS
};

#define PWM_TIMERS  const TIM_TypeDef *pwmTimers[] = { \
    TIM1, \
    TIM1, \
    TIM1, \
    TIM1, \
    TIM4, \
    TIM4, \
    TIM4, \
    TIM4, \
    TIM9, \
    TIM9, \
    TIM2, \
    TIM2, \
    TIM10, \
    TIM11 \
};

#define	PWM_AFS	    const uint8_t pwmAFs[] = { \
    GPIO_AF_TIM1, \
    GPIO_AF_TIM1, \
    GPIO_AF_TIM1, \
    GPIO_AF_TIM1, \
    GPIO_AF_TIM4, \
    GPIO_AF_TIM4, \
    GPIO_AF_TIM4, \
    GPIO_AF_TIM4, \
    GPIO_AF_TIM9, \
    GPIO_AF_TIM9, \
    GPIO_AF_TIM2, \
    GPIO_AF_TIM2, \
    GPIO_AF_TIM10, \
    GPIO_AF_TIM11 \
};

#define PWM_PORTS   const GPIO_TypeDef *pwmPorts[] = { \
    GPIOE, \
    GPIOE, \
    GPIOE, \
    GPIOE, \
    GPIOD, \
    GPIOD, \
    GPIOD, \
    GPIOD, \
    GPIOE, \
    GPIOE, \
    GPIOB, \
    GPIOB, \
    GPIOB, \
    GPIOB \
};

#define PWM_PINS    const uint32_t pwmPins[] = { \
    GPIO_Pin_9, \
    GPIO_Pin_11, \
    GPIO_Pin_13, \
    GPIO_Pin_14, \
    GPIO_Pin_12, \
    GPIO_Pin_13, \
    GPIO_Pin_14, \
    GPIO_Pin_15, \
    GPIO_Pin_5, \
    GPIO_Pin_6, \
    GPIO_Pin_10, \
    GPIO_Pin_11, \
    GPIO_Pin_8, \
    GPIO_Pin_9 \
};

#define PWM_PINSOURCES	const uint16_t pwmPinSources[] = { \
    GPIO_PinSource9, \
    GPIO_PinSource11, \
    GPIO_PinSource13, \
    GPIO_PinSource14, \
    GPIO_PinSource12, \
    GPIO_PinSource13, \
    GPIO_PinSource14, \
    GPIO_PinSource15, \
    GPIO_PinSource5, \
    GPIO_PinSource6, \
    GPIO_PinSource10, \
    GPIO_PinSource11, \
    GPIO_PinSource8, \
    GPIO_PinSource9 \
};

#define PWM_TIMERCHANNELS   const uint8_t pwmTimerChannels[] = { \
    1, \
    2, \
    3, \
    4, \
    1, \
    2, \
    3, \
    4, \
    1, \
    2, \
    3, \
    4, \
    1, \
    1 \
};

#define PWM_BDTRS   const uint8_t pwmBDTRs[] = { \
    1, \
    1, \
    1, \
    1, \
    0, \
    0, \
    0, \
    0, \
    0, \
    0, \
    0, \
    0, \
    0, \
    0 \
};

#define PWM_CLOCKS  const uint32_t pwmClocks[] = { \
    168000000, \
    168000000, \
    168000000, \
    168000000, \
    84000000, \
    84000000, \
    84000000, \
    84000000, \
    168000000, \
    168000000, \
    84000000, \
    84000000, \
    168000000, \
    168000000 \
};

#define VN100_UART		USART2

#define VN100_SYNCOUT_PORT	GPIOD
#define VN100_SYNCOUT_PIN	GPIO_Pin_0
#define VN100_SYNCOUT_EXTI_PORT	EXTI_PortSourceGPIOD
#define VN100_SYNCOUT_EXTI_PIN	EXTI_PinSource0
#define VN100_SYNCOUT_EXTI_LINE	EXTI_Line0
#define VN100_SYNCOUT_EXTI_IRQ	EXTI0_IRQn
#define VN100_SYNCOUT_ISR	EXTI0_IRQHandler

#define VN100_RESET_PORT	GPIOE
#define VN100_RESET_PIN		GPIO_Pin_2
#define VN100_REPRGM_PORT	GPIOD
#define VN100_REPRGM_PIN	GPIO_Pin_1
#define VN100_SYNCIN_PORT	GPIOD
#define VN100_SYNCIN_PIN	GPIO_Pin_10

#define VN100_SPI		SPI2
#define VN100_SPI_DR		((uint32_t)(VN100_SPI) + 0x0c)
#define VN100_SPI_CLOCK		RCC_APB1Periph_SPI2
#define VN100_SPI_AF		GPIO_AF_SPI2
#define VN100_SPI_NSS_PORT	GPIOB
#define VN100_SPI_SCK_PORT	GPIOB
#define VN100_SPI_MISO_PORT	GPIOB
#define VN100_SPI_MOSI_PORT	GPIOB
#define VN100_SPI_NSS_PIN	GPIO_Pin_12
#define VN100_SPI_SCK_PIN	GPIO_Pin_13
#define VN100_SPI_MISO_PIN	GPIO_Pin_14
#define VN100_SPI_MOSI_PIN	GPIO_Pin_15
#define VN100_SPI_NSS_SOURCE	GPIO_PinSource12
#define VN100_SPI_SCK_SOURCE	GPIO_PinSource13
#define VN100_SPI_MISO_SOURCE	GPIO_PinSource14
#define VN100_SPI_MOSI_SOURCE	GPIO_PinSource15

#define VN100_DMA_RX		DMA1_Stream3
#define VN100_DMA_RX_CHANNEL	DMA_Channel_0
#define VN100_DMA_RX_IRQ	DMA1_Stream3_IRQn
#define VN100_DMA_RX_FLAGS	(DMA_IT_TEIF3 | DMA_IT_DMEIF3 | DMA_IT_FEIF3 | DMA_IT_TCIF3 | DMA_IT_HTIF3)
#define VN100_DMX_RX_HANDLER	DMA1_Stream3_IRQHandler

#define VN100_DMA_TX		DMA1_Stream4
#define VN100_DMA_TX_CHANNEL	DMA_Channel_0
#define VN100_DMA_TX_FLAGS	(DMA_IT_TEIF4 | DMA_IT_DMEIF4 | DMA_IT_FEIF4 | DMA_IT_TCIF4 | DMA_IT_HTIF4)


#define SPEKTRUM_UART		UART4

#define FUTABA_UART		UART4

#define SUPERVISOR_READY_PORT	GPIOA
#define SUPERVISOR_READY_PIN	GPIO_Pin_8
#define SUPERVISOR_DEBUG_PORT	GPIOD
#define SUPERVISOR_DEBUG_PIN	GPIO_Pin_11

#define GPS_LED_PORT		GPIOE
#define GPS_LED_PIN		GPIO_Pin_0
#define GPS_USART		USART3

#define ADC_PRESSURE_3V3

#endif