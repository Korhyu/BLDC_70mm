#ifndef AL00_MICRO_HT32F52230_H

#define AL00_MICRO_HT32F52230_H

#include "ht32.h"
#include "ht32_board.h"
#include "ht32_board_config.h"
#include "stdio.h"
#include "stdarg.h"


/******************************************************************************/
/*        	 Librería de nivel 0 para abstracción del hardware			 			    */
/******************************************************************************/
#define CK_ON		1
#define CK_OFF	0


/******************************************************************************/
/*												Funciones Generales																	*/
/******************************************************************************/
void micro_config (void);

void uart_init (void);
/******************************************************************************/
/*						Funciones y macros para el uso del timer del sistema 						*/
/******************************************************************************/
uint32_t Systick_GetTick(void);

#define __hardware_get_tick_timer()	Systick_GetTick()

/******************************************************************************/
/*		Definiciones, macros y funciones para el manejo de GPIO's								*/
/******************************************************************************/

//Definición de constantes para 16 pines de GPIO
#define GPIO0				0
#define GPIO1				1
#define	GPIO2				2
#define GPIO3				3
#define GPIO4				4
#define GPIO5				5
#define GPIO6				6
#define GPIO7				7
#define GPIO8				8
#define GPIO9				9
#define GPIO10			10
#define GPIO11			11
#define GPIO12			12
#define GPIO13			13
#define GPIO14			14
#define GPIO15			15

//Definición de constantes para los puertos A y B
#define GPIOA								HT_GPIOA
#define GPIOB								HT_GPIOB

//Linkeo de interrupciones con funciones para eventos en los GPIO's
int32_t 	hardware_gpio_event_rising_event_link_callback	(void (*func_pointer)(void), uint32_t exti_line);
int32_t 	hardware_gpio_event_falling_event_link_callback	(void (*func_pointer)(void), uint32_t exti_line);

//Funciones para configurar los GPIO según la funcionalidad necesaria
void hardware_gpio_config_output_pp_pins_load_config		(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_hiz_pins_load_config					(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_edge_events_pins_load_config	(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_inputs_pins_load_config				(HT_GPIO_TypeDef *GPIOx,uint32_t pins);

//Macros para comandar salidas de GPIO
#define __hardware_gpio_output_reset(gpio_port,GPIOnum)												GPIO_WriteOutBits(gpio_port,(1<<GPIOnum),RESET)
#define __hardware_gpio_output_set(gpio_port,GPIOnum)													GPIO_WriteOutBits(gpio_port,(1<<GPIOnum),SET)
#define __hardware_gpio_output_toggle(gpio_port,GPIOnum)											GPIO_ReadOutBit(gpio_port,(1<<GPIOnum))==SET?GPIO_WriteOutBits(gpio_port,(1<<GPIOnum),RESET):GPIO_WriteOutBits(gpio_port,(1<<GPIOnum),SET)

//Macro para lectura de entrada GPIO
#define __hardware_gpio_input_read_state(gpio_port,GPIOnum)										GPIO_ReadInBit(gpio_port,(1<<GPIOnum))

//Macros para cambiar la configuración de los pines de salida
#define __hardware_gpio_config_set_as_hiz_pin(gpio_port,GPIOnum)							GPIO_DirectionConfig(gpio_port, (1<<GPIOnum), GPIO_DIR_IN)
#define __hardware_gpio_config_set_as_pushpull_output_pin(gpio_port,GPIOnum)	GPIO_DirectionConfig(gpio_port, (1<<GPIOnum), GPIO_DIR_OUT)

//Macros para el control de eventos en GPIO's
#define __hardware_gpio_event_detection_disable(GPIOnum)											(HT_EXTI->CR 				&= ~(1 << GPIOnum))

#define __hardware_gpio_event_rising_only_detection_enable(GPIOnum)						{(HT_EXTI->EDGEFLGR 								|=	(1 << GPIOnum));\
																																							 (HT_EXTI->EDGESR										|=	(1 << GPIOnum));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			&= ~(7 << 28));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			|=	(3 << 28));\
																																							 (HT_EXTI->CR												|=	(1 << GPIOnum));}


#define __hardware_gpio_event_falling_only_detection_enable(GPIOnum)					{(HT_EXTI->EDGEFLGR 								|=	(1 << GPIOnum));\
																																							 (HT_EXTI->EDGESR										|=	(1 << GPIOnum));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			&= ~(7 << 28));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			|=	(2 << 28));\
																																							 (HT_EXTI->CR												|=	(1 <<GPIOnum));}																																										 

#define __hardware_gpio_event_both_edges_detection_enable(GPIOnum)						{(HT_EXTI->EDGEFLGR 								|=	(1 << GPIOnum));\
																																							 (HT_EXTI->EDGESR										|=	(1 << GPIOnum));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			&= ~(7 << 28));\
																																							 (*((&HT_EXTI->CFGR0)+GPIOnum)			|=	(4 << 28));\
																																							 (HT_EXTI->CR												|=	(1 <<GPIOnum));}
																																							 
/******************************************************************************/
/*				Definiciones, macros y funciones para el manejo de timmers					*/
/******************************************************************************/
#define __HARDWARE_TIM_BFTM0_SEL 0
#define __HARDWARE_TIM_SCTM0_SEL 1
#define __HARDWARE_TIM_SCTM1_SEL 2
#define __HARDWARE_TIM_GPTM0_SEL 3
																																						 

#define HT_TIM_BFTM0	HT_BFTM0
#define HT_TIM_SCTM0	HT_SCTM0
#define HT_TIM_SCTM1	HT_SCTM1	
#define	HT_TIM_GPTM0	HT_GPTM0																																							 
					
																																							 
#define	CONST_time_uS_to_cnt_40MHz	40	//Considero el clock del sistema en 40MHz, 25nS de período -> 1uS equivale a 40 cuentas						
#define CONST_CK_OFFSET_uS					5		//Esto es un paliativo dado que aparentemente hay un error de 5uS constantes en el seteo del tiempo para la irq																																					 

int32_t hardware_tim_link_callback (void (*func_pointer)(void), uint32_t __hardware_tim_sel);

//Como este temporizador funciona con el clock del sistema (40MHz), tomo el valor en uS y lo multiplico por 40:
//time_uS<<5 => time_uS x 32, time_uS<<3 => time_uS x 8 ===> time_uS x 32 + time_uS x 8 = time_uS x 40
#define __harwdare_tim_bftm_init_timer_with_timeout_irq_us(HT_TIM_BFTMx,time_us)	{\
																					BFTM_EnaCmd(HT_TIM_BFTMx, DISABLE);\
																					BFTM_IntConfig(HT_TIM_BFTMx, DISABLE);\
																					BFTM_SetCounter(HT_TIM_BFTMx, 0);\
																					BFTM_SetCompare(HT_TIM_BFTMx, ((time_us<<5)+(time_us<<3)));\
																					BFTM_IntConfig(HT_TIM_BFTMx, ENABLE);\
																				  BFTM_EnaCmd(HT_TIM_BFTMx, ENABLE);\
																					}

#define __harwdare_tim_bftm_abort(HT_TIM_BFTMx)		{BFTM_EnaCmd(HT_TIM_BFTMx, DISABLE);BFTM_IntConfig(HT_TIM_BFTMx,DISABLE);BFTM_SetCounter(HT_TIM_BFTMx,0);}
																																																																														
#define __hardware_tim_bftm_get_count_us(HT_TIM_BFTMx)	(BFTM_GetCounter(HT_TIM_BFTMx))



#define __hardware_tim_sctm_init_timer_with_timeout_irq_us(HT_TIM_SCTMx,time_us)	{\
																					TM_Cmd(HT_TIM_SCTMx,DISABLE);\
																					TM_IntConfig(HT_TIM_SCTMx,TM_INT_UEV,DISABLE);	\
																					TM_SetCounter(HT_TIM_SCTMx,0);\
																					TM_SetCounterReload(HT_TIM_SCTMx,time_us);\
																					TM_IntConfig(HT_TIM_SCTMx,TM_INT_UEV,ENABLE);	\
																				  TM_Cmd(HT_TIM_SCTMx,ENABLE);\
																					}

#define __harwdare_tim_sctm_abort(HT_TIM_SCTMx)		{TM_Cmd(HT_TIM_SCTMx,DISABLE);TM_IntConfig(HT_TIM_SCTMx,TM_INT_UEV,DISABLE);TM_SetCounter(HT_TIM_SCTMx,0);}

#define __hardware_tim_sctm_get_count_us(HT_TIM_SCTMx) (TM_GetCounter(HT_TIM_SCTMx))




//-------------------------------------------
//						UART
//-------------------------------------------
#define COM_NUM                     (1)

#define COM1_CLK(CK)                (CK.Bit.UART0)
#define COM1_PORT                   (HT_UART0)
#define COM1_IRQn                   (UART0_IRQn)
#define COM1_IRQHandler             (UART0_IRQHandler)

#define COM1_TX_GPIO_ID             (GPIO_PA)
#define COM1_TX_AFIO_PIN            (AFIO_PIN_4)
#define COM1_TX_AFIO_MODE           (AFIO_FUN_USART_UART)

#define COM1_RX_GPIO_ID             (GPIO_PA)
#define COM1_RX_AFIO_PIN            (AFIO_PIN_5)
#define COM1_RX_AFIO_MODE           (AFIO_FUN_USART_UART)

void uart_init (void);
uint8_t hardware_uart_read 		(void);
void hardware_uart_transmit(char * fmt,...);

//-------------------------------------------
//				 PWM PRINCIPAL
//-------------------------------------------
int32_t hardware_pwm_set_period_us 									(int32_t period_us);
int32_t hardware_pwm_get_period_us 									(void);
void		hardware_pwm_set_ton_us		 									(int32_t ton_us);
int32_t hardware_pwm_get_ton_us		 									(void);
void		hardware_pwm_set_toff_us										(int32_t toff_us);
int32_t harwdare_pwm_get_toff_us										(void);
void    hardware_pwm_start_with_ton									(void);
void 		hardware_pwm_set_outputs_to_toff						(void);


#define __hardware_pwm1_enable()								TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_0, TM_CHCTL_ENABLE)
#define __hardware_pwm1_disable()								TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_0, TM_CHCTL_DISABLE)
#define __hardware_pwm2_enable()								TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_1, TM_CHCTL_ENABLE)
#define __hardware_pwm2_disable()								TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_1, TM_CHCTL_DISABLE)

#endif /* AL00_MICRO_HT32F52230_H */
