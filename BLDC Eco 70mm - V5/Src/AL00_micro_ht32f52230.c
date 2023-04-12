#include "AL00_micro_ht32f52230.h"


/********		SYSTICK - Definiciones, variables y prototipos (locales)		********/
static uint32_t TickCount=0;

void SYSTICK_Configuration(void);
void CKCU_Configuration(void);

#if (ENABLE_CKOUT == 1)
void CKOUTConfig(void);
#endif

/********		GPIOS - Definiciones, variables y prototipos (locales)		********/
#define	GPIOA_SEL						0
#define	GPIOB_SEL						1
#define	GPIOC_SEL						2
#define GPIOD_SEL 					3
#define GPIOx_SEL_COUNT 		4

static HT_GPIO_TypeDef* GPIO_PORTs[GPIOx_SEL_COUNT]={GPIOA,GPIOB};

void GPIO_Configuration(void);

//Conjunto de vectores en los que se carga que pin del GPIO tiene una u otra configuración.
static uint32_t gpio_config_output_pp_pins[GPIOx_SEL_COUNT];
static uint32_t gpio_config_hiz_pins[GPIOx_SEL_COUNT];
static uint32_t gpio_config_edge_events_pins[GPIOx_SEL_COUNT];
static uint32_t gpio_config_inputs_pins[GPIOx_SEL_COUNT];

//Funciones utilizadas para cargar en los vectores previos las configuraciones correspondientes.
void hardware_gpio_config_output_pp_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_hiz_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_edge_events_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins);
void hardware_gpio_config_inputs_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins);

//Funciones utilizadas para cargar en el microcontrolador la configuración de los GPIOS
static void micro_gpios_outputs_pp_config(void);
static void micro_gpios_inputs_pullup_config(void);
static void micro_gpios_hiz_config(void);
static void micro_gpios_edge_events_config(void);

/********		PWM - Definiciones, variables y prototipos (locales)		********/
#define PWM_INIT_FREQUENCY_Hz							5000	
#define	PWM_INIT_PERIOD_uS								200
#define PWM_INIT_DUTY_PERCENTAGE					70
#define CONST_100PERCENT									100
#define PWM_PRESCALER_40MHz_to_1uS_cnt		40
#define	PWM_MAX_PERIOD_uS									32000
#define	PWM_MIN_PERIOD_uS									10
#define	PWM_OUTPUT_MODE										TM_OM_PWM1

//Función para cargar en el microcontrolador la configuración de PWM
static void micro_pwm_config(void);

/********		TIMERS - Definiciones, variables y prototipos (locales)		********/
//Función para cargar en el microcontrolador la configuración de los timers
static void micro_timers_config(void);

/********		ADC - Definiciones, variables y prototipos (locales)		********/
//Función para cargar en el microcontrolador la configuración del ADC
static void micro_adc_config(void);

/********		INTERRUPCIONES - Definiciones, variables y prototipos (locales)		********/
#define EXTI_LINE_COUNT 	16

void NVIC_Configuration(void);

//Función vacía para inicializar los punteros a funcion y evitar el hard fault
void empty_func_to_avoid_hard_fault(void){}

//Inicialización de los punteros a funciones que se van a llamar ante un IRQ de los GPIO's, tanto en flanco positivo como negativo
//Se cargan con empty_func_to_avoid_hard_fault()
void (*func_ptr_callback_rising_exti [EXTI_LINE_COUNT])(void) = {empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault};

void (*func_ptr_callback_falling_exti[EXTI_LINE_COUNT])(void) = {empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault,
																 empty_func_to_avoid_hard_fault,empty_func_to_avoid_hard_fault};

//Inicialización de los punteros a funciones que se llamarán en caso de IRQ de los timers
void (*func_ptr_callback_bftm0)(void) = empty_func_to_avoid_hard_fault;
void (*func_ptr_callback_sctm0)(void) = empty_func_to_avoid_hard_fault;
void (*func_ptr_callback_sctm1)(void) = empty_func_to_avoid_hard_fault;																 
void (*func_ptr_callback_gptm0)(void) = empty_func_to_avoid_hard_fault;															 
	
/********************************************************************************************/																 
/*																FUNCIONES PROGRAMADAS																			*/																 
/********************************************************************************************/
																 
/******************************************************
 * @brief   This function returns SysTick TickCount.	*
 * @retval  TickCount																	*
 ******************************************************/																 
uint32_t Systick_GetTick(void)
{
	return  TickCount;
}

/******************************************************
 * @brief   This function handles SysTick Handler.		*
 * @retval  None																			*
 ******************************************************/
void SysTick_Handler(void)
{
	TickCount++;
}

/*************************	FUNCIONES GPIO	*************************************/

/********************************************************************************/
/*			Funciones de carga de vectores con la configuración de los GPIO's				*/
/********************************************************************************/
//Carga de GPIO's como salidas push pull
void hardware_gpio_config_output_pp_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins)
{
	uint32_t i=0;
	
	for(i=0;i<GPIOx_SEL_COUNT;i++)
	{
		if(GPIOx == GPIO_PORTs[i])
		{
			gpio_config_output_pp_pins[i] = pins;
			i = GPIOx_SEL_COUNT;
		}
	}
}

//Carga de GPIO's como salidas en alta impedancia
void hardware_gpio_config_hiz_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins)
{
	uint32_t i=0;
	for(i=0;i<GPIOx_SEL_COUNT;i++)
	{
		if(GPIOx == GPIO_PORTs[i])
		{
			gpio_config_hiz_pins[i] = pins;
			i = GPIOx_SEL_COUNT;
		}
	}	
	
}	

//Carga de GPIO's como entradas de eventos generados por flancos
void hardware_gpio_config_edge_events_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins)
{
	uint32_t i=0;
	for(i=0;i<GPIOx_SEL_COUNT;i++)
	{
		if(GPIOx == GPIO_PORTs[i])
		{
			gpio_config_edge_events_pins[i] = pins;
			i = GPIOx_SEL_COUNT;
		}
	}
}

//Carga de GPIO's como entradas
void hardware_gpio_config_inputs_pins_load_config(HT_GPIO_TypeDef *GPIOx,uint32_t pins)
{
	uint32_t i=0;
	for(i=0;i<GPIOx_SEL_COUNT;i++)
	{
		if(GPIOx == GPIO_PORTs[i])
		{
			gpio_config_inputs_pins[i] = pins;
			i = GPIOx_SEL_COUNT;
		}
	}
}





/********************************************************************************/
/*						Función que activa el clock del puerto indicado										*/
/********************************************************************************/
static void _HT32F_DVB_ClockConfig(u32 GpioId)
{
  CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
  u8 RegCK[GPIO_PORT_NUM] = {0};
  RegCK[GpioId] = 1;

  CKCUClock.Bit.PA         = RegCK[0];
  CKCUClock.Bit.PB         = RegCK[1];
  #if defined(LIBCFG_GPIOC)
  CKCUClock.Bit.PC         = RegCK[2];
  #endif
  #if defined(LIBCFG_GPIOD)
  CKCUClock.Bit.PD         = RegCK[3];
  #endif
  #if defined(LIBCFG_GPIOE)
  CKCUClock.Bit.PE         = RegCK[4];
  #endif
  #if defined(LIBCFG_GPIOF)
  CKCUClock.Bit.PF         = RegCK[5];
  #endif
  CKCUClock.Bit.AFIO       = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);
}



/********************************************************************************/
/*				Funciones que cargan la configuración de los GPIO's en el uC					*/
/********************************************************************************/
//Carga de GPIO's como salidas push pull
static void micro_gpios_outputs_pp_config(void)
{
	uint32_t gpio_port_sel_index=0,gpio_pin_index;
	
	do{ //Recorro los puertos 
	
		_HT32F_DVB_ClockConfig(gpio_port_sel_index);/* GPIO Ports Clock Enable */
		
		if(gpio_config_output_pp_pins[gpio_port_sel_index] != 0) //Si hay pines selccionados en el GPIOx en cuestion, lo configuro, sino paso al siguiente GPIOx
		{							
				gpio_pin_index=1;
			
				do{ //Recorro los pines
					
							if(gpio_config_output_pp_pins[gpio_port_sel_index] & gpio_pin_index)
							{
								AFIO_GPxConfig				 (gpio_port_sel_index, gpio_pin_index , AFIO_FUN_GPIO);
								
								GPIO_PullResistorConfig(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_PR_DISABLE);
								GPIO_DriveConfig			 (GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DV_8MA);		
								GPIO_DirectionConfig	 (GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DIR_OUT);
							}
							gpio_pin_index<<=1;
							
				}while(gpio_pin_index<(1<<16));
		
		}
		gpio_port_sel_index++;
	}while(gpio_port_sel_index<GPIOx_SEL_COUNT);
}


//Carga de GPIO's como entradas
static void micro_gpios_inputs_pullup_config(void)
{
	uint32_t gpio_port_sel_index=0,gpio_pin_index;

	do{ //Recorro los puertos 
		
		if(gpio_config_inputs_pins[gpio_port_sel_index] != 0) //Si hay pines selccionados en el GPIOx en cuestion, lo configuro, sino paso al siguiente GPIOx
		{							
				gpio_pin_index=1;
			
				do{ //Recorro los pines
					
							if(gpio_config_inputs_pins[gpio_port_sel_index] & gpio_pin_index)
							{
									AFIO_GPxConfig					(gpio_port_sel_index, gpio_pin_index, AFIO_FUN_GPIO);
								
									GPIO_DirectionConfig		(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DIR_IN);
									GPIO_PullResistorConfig	(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_PR_UP);
									GPIO_InputConfig				(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, ENABLE);
							}
							gpio_pin_index<<=1;
							
				}while(gpio_pin_index<(1<<16));
		}
		gpio_port_sel_index++;
	}while(gpio_port_sel_index<GPIOx_SEL_COUNT);
}


//Carga de GPIO's como salidas en alta impedancia
static void micro_gpios_hiz_config(void)
{
	uint32_t gpio_port_sel_index=0,gpio_pin_index;

	do{ //Recorro los puertos 
		
		if(gpio_config_hiz_pins[gpio_port_sel_index] != 0) //Si hay pines selccionados en el GPIOx en cuestion, lo configuro, sino paso al siguiente GPIOx
		{							
				gpio_pin_index=1;
			
				do{ //Recorro los pines
					
							if(gpio_config_hiz_pins[gpio_port_sel_index] & gpio_pin_index)
							{
									AFIO_GPxConfig					(gpio_port_sel_index, gpio_pin_index, AFIO_FUN_GPIO);
								
									GPIO_DirectionConfig		(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DIR_IN);
									GPIO_PullResistorConfig	(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_PR_DISABLE);
									GPIO_DriveConfig				(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DV_8MA);
							}
							gpio_pin_index<<=1;
							
				}while(gpio_pin_index<(1<<16));
		}
		gpio_port_sel_index++;
	}while(gpio_port_sel_index<GPIOx_SEL_COUNT);	
}


//Carga de GPIO's como entradas de eventos generados por flancos
static void micro_gpios_edge_events_config(void)
{
	uint32_t gpio_port_sel_index=0,gpio_pin_index=1,exti_line_index=0;
	uint32_t gpio_event_pins=0;
	
	EXTI_InitTypeDef EXTI_InitStruct = {0};
  CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
			
  /* Enable the EXTI Clock                                                                                */
  CKCUClock.Bit.EXTI       = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);
	
	
	do{ //Recorro los puertos 	
		if(gpio_config_edge_events_pins[gpio_port_sel_index] != 0) //Si hay pines selccionados en el GPIOx en cuestion, lo configuro, sino paso al siguiente GPIOx
		{							
				gpio_pin_index=1;
				exti_line_index=0;
			
			
				do{ //Recorro los pines
					
							if(gpio_config_edge_events_pins[gpio_port_sel_index] & gpio_pin_index)
							{
									gpio_event_pins |= gpio_pin_index;
								
								
									//Lo siguiente es para linkear que puerto corresponde a cada pin. OJO: si hay multiples numeros de pin repetido va a quedar
									//el ultimo puerto con el pin puesto (iniciando por GPIOA yendo hacia GPIOD)
									//GPIO_port_of_edge_event[exti_line_index]=GPIO_PORTs[gpio_port_sel_index];
								
									AFIO_GPxConfig					(gpio_port_sel_index, gpio_pin_index, AFIO_FUN_GPIO);
														
									GPIO_DirectionConfig		(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_DIR_IN);
									GPIO_PullResistorConfig	(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, GPIO_PR_DISABLE);
									GPIO_InputConfig				(GPIO_PORTs[gpio_port_sel_index], gpio_pin_index, ENABLE);
								
									
									AFIO_EXTISourceConfig((AFIO_EXTI_CH_Enum)exti_line_index, (AFIO_ESS_Enum)gpio_port_sel_index);// Esto es para asigar el puerto a la linea de exti	
																																																								// Ya que, por ejemplo, hay que definir de todos los pines "0"
																																																								// cual es el puerto que va a estar enlazado a la linea EXTI
																																																								// la EXTI0 puede estar linkeada a PA0, PB0, PC0, PD0 o PE0
								
								  EXTI_InitStruct.EXTI_Channel = exti_line_index;
									EXTI_InitStruct.EXTI_Debounce = EXTI_DEBOUNCE_DISABLE;
									EXTI_InitStruct.EXTI_DebounceCnt = 0;
									EXTI_InitStruct.EXTI_IntType = EXTI_POSITIVE_EDGE;
									EXTI_Init(&EXTI_InitStruct);	
							}
							
							gpio_pin_index<<=1;
							exti_line_index+=1;	
				}while(gpio_pin_index<(1<<16));
		}
		
		gpio_port_sel_index++;
	}while(gpio_port_sel_index<GPIOx_SEL_COUNT);
	
	
	//Lo siguiente es Habilitar las IRQ en funcion de los pines (independientmenete del puerto) que se hayan seleccionado
	gpio_pin_index=1;
	exti_line_index=0;
	
	do{
		if(gpio_pin_index&gpio_event_pins)
		{			
			switch(gpio_pin_index)
			{
				case (1<<0):	
				case (1<<1):	NVIC_EnableIRQ(EXTI0_1_IRQn);
											gpio_pin_index = (1<<1);
											break;
				case (1<<2):
				case (1<<3):	NVIC_EnableIRQ(EXTI2_3_IRQn);
											gpio_pin_index = (1<<3);
											break;
				case (1<<4):
				case (1<<5):
				case (1<<6):
				case (1<<7):
				case (1<<8):
				case (1<<9):
				case (1<<10):
				case (1<<11):
				case (1<<12):
				case (1<<13):
				case (1<<14):
				case (1<<15):	NVIC_EnableIRQ(EXTI4_15_IRQn);
											gpio_pin_index = (1<<15);
											break;				
			}
		}
		gpio_pin_index<<=1;			
		exti_line_index+=1;
	}while(gpio_pin_index<=(1<<15));
}

/******************************************************
  * @brief  Configure the GPIO ports.									*
  * @retval None																			*
  *****************************************************/
void GPIO_Configuration(void)
{
#if (RETARGET_PORT == RETARGET_USART0)
  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_2 | AFIO_PIN_3, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_USART1)
  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_UART0)
  AFIO_GPxConfig(GPIO_PC, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_UART1)
  AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1 | AFIO_PIN_3, AFIO_FUN_USART_UART);
#endif
}
									 
/*************************	FUNCIONES PWM		*************************************/	

static void micro_pwm_config(void)
{
	//Estructuras para inicializar el timer y la salida, con sus respectivos parámetros
	TM_TimeBaseInitTypeDef 	 TM_TimeBaseInitStructure;
  TM_OutputInitTypeDef 		 TM_OutputInitStructure;
	
	//Configuro los pines deseados con la función alternativa correspondiente a los GPTM
  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_14, AFIO_FUN_MCTM_GPTM);	//CH0 PA14 (LIN1)
	AFIO_GPxConfig(GPIO_PB, AFIO_PIN_0 , AFIO_FUN_MCTM_GPTM);	//CH1 PB0 (LIN2)
	 
	//Configuro la base de tiempo y el modo de conteo
	TM_TimeBaseInitStructure.CounterReload = PWM_INIT_PERIOD_uS-1; //Período
  TM_TimeBaseInitStructure.Prescaler = PWM_PRESCALER_40MHz_to_1uS_cnt;
  TM_TimeBaseInitStructure.RepetitionCounter = 0;
  TM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
  TM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
  TM_TimeBaseInit(HT_TIM_GPTM0, &TM_TimeBaseInitStructure);
	
	//Configuro las salidas asociadas al temporizador que serán las salidas de PWM
  TM_OutputInitStructure.Channel = TM_CH_0;
  TM_OutputInitStructure.OutputMode = PWM_OUTPUT_MODE; //PWM1: arranca en Ton - PWM2: arranca en Toff
  TM_OutputInitStructure.Control = TM_CHCTL_ENABLE;	 //TM_CHCTL_DISABLE
  TM_OutputInitStructure.ControlN = TM_CHCTL_DISABLE;//TM_CHCTL_ENABLE;
  TM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
  TM_OutputInitStructure.PolarityN = TM_CHP_INVERTED;
	TM_OutputInitStructure.IdleState = MCTM_OIS_LOW; 	//Para el micro más simple 52230 o 52220 esto no sirve
  TM_OutputInitStructure.IdleStateN = MCTM_OIS_HIGH;// "			"				"				"
  TM_OutputInitStructure.Compare = ((PWM_INIT_PERIOD_uS-1) * (CONST_100PERCENT - PWM_INIT_DUTY_PERCENTAGE)) / CONST_100PERCENT;
  TM_OutputInit(HT_TIM_GPTM0, &TM_OutputInitStructure);
	
	
	TM_OutputInitStructure.Channel = TM_CH_1;
	TM_OutputInitStructure.Compare = ((PWM_INIT_PERIOD_uS-1) * (CONST_100PERCENT - PWM_INIT_DUTY_PERCENTAGE)) / CONST_100PERCENT;
	TM_OutputInit(HT_TIM_GPTM0, &TM_OutputInitStructure);
	
	TM_Cmd(HT_TIM_GPTM0, ENABLE);

	TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_0, TM_CHCTL_ENABLE);
	TM_ChannelConfig(HT_TIM_GPTM0, TM_CH_1, TM_CHCTL_ENABLE);

}

/*void hardware_pwm_end_toff_link_callback (void (*func_pointer)(void))
{
	func_ptr_callback_pwm_end_toff=func_pointer;
}

void hardware_pwm_break_function_link_callback(void (*func_pointer)(void))
{
	func_ptr_callback_pwm_break=func_pointer;
}*/

/*****	Cargo el registro con el período del PWM, en uS	*****/
int32_t hardware_pwm_set_period_us 	(int32_t period_us)
{
	if(period_us<PWM_MAX_PERIOD_uS && period_us>PWM_MIN_PERIOD_uS)
	{
		TM_SetCounterReload(HT_TIM_GPTM0,period_us-1);							//Cambio el periodo
	}
	else 
		return -1;
	return 0;
}

/*****	Retorno el valor actual del período en uS	*****/
int32_t hardware_pwm_get_period_us (void)
{
	return HT_TIM_GPTM0->CRR+1;
}	

/*****	Configuro el tiempo de encendido en uS	*****/
void hardware_pwm_set_ton_us	(int32_t ton_us)
{
		if(PWM_OUTPUT_MODE==TM_OM_PWM1)	//En Modo PWM1, se arranca en ton, entonces el umbral es el valor cargado
		{
			TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_0, ton_us+1);
			TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_1, ton_us+1);
		}
		else	// En Modo PWM2, se arranca en toff, entonces el umbral es el período menos el valor cargado
		{
			TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_0,HT_TIM_GPTM0->CRR-ton_us+1);
			TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_1,HT_TIM_GPTM0->CRR-ton_us+1);
		}
}

/*****	Retorno el valor actual del Ton en uS	*****/
int32_t hardware_pwm_get_ton_us		(void)
{
	if(PWM_OUTPUT_MODE==TM_OM_PWM1)
	{
		return(TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0)+1);
	}
	else
	{
		return(HT_TIM_GPTM0->CRR-TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0)+1);
	}
}

/*****	Configuro el tiempo de apagado en uS	*****/
void hardware_pwm_set_toff_us(int32_t toff_us)
{
		TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_0,((PWM_OUTPUT_MODE==TM_OM_PWM1) ? (HT_TIM_GPTM0->CRR)-toff_us : toff_us));
		TM_SetCaptureCompare(HT_TIM_GPTM0,TM_CH_1,((PWM_OUTPUT_MODE==TM_OM_PWM1) ? (HT_TIM_GPTM0->CRR)-toff_us : toff_us));
}

/*****	Retorno el valor actual de Toff en uS	*****/
int32_t hardware_pwm_get_toff_us(void)
{
	return ((PWM_OUTPUT_MODE==TM_OM_PWM1) ? (HT_TIM_GPTM0->CRR)-TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0) : TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0));
}
 
void    hardware_pwm_start_with_ton									(void)
{
	(PWM_OUTPUT_MODE==TM_OM_PWM1) ? TM_SetCounter(HT_TIM_GPTM0,0):TM_SetCounter(HT_TIM_GPTM0,TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0));
}

void 		hardware_pwm_set_outputs_to_toff						(void)
{
	(PWM_OUTPUT_MODE==TM_OM_PWM1) ? TM_SetCounter(HT_TIM_GPTM0,TM_GetCaptureCompare(HT_TIM_GPTM0,TM_CH_0)+1):TM_SetCounter(HT_TIM_GPTM0,0);
}


/*************************	FUNCIONES ADC		*************************************/

static void micro_adc_config(void)
{

  /* ADCLK frequency is set to CK_AHB/64                                                                    */
  CKCU_SetADCPrescaler(CKCU_ADCPRE_DIV64);	
  /* Config AFIO mode as ADC function  	*/
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_3, AFIO_FUN_ADC);
  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_6, AFIO_FUN_ADC);	
  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_7, AFIO_FUN_ADC);
	
	/* Continuous Mode, Length 1, SubLength 1                                                                 */
  ADC_RegularGroupConfig(HT_ADC, CONTINUOUS_MODE, 3, 3);	
	  /* ADC Channel n, Rank 0, Sampling clock is (1.5 + 0) ADCLK
     Conversion time = (sampling clock + 12.5) / ADCLK = 12.4 uS */
	ADC_SamplingTimeConfig(HT_ADC,10);
	
	ADC_RegularChannelConfig(HT_ADC, ADC_CH_3, 0);
	ADC_RegularChannelConfig(HT_ADC, ADC_CH_6, 1);
  ADC_RegularChannelConfig(HT_ADC, ADC_CH_7, 2);
	
	  /* Use Software Trigger as ADC trigger source                                                             */
  ADC_RegularTrigConfig(HT_ADC, ADC_TRIG_SOFTWARE);
	
	ADC_Cmd(HT_ADC, ENABLE);

  /* Software trigger to start continuous mode                                                              */
  ADC_SoftwareStartConvCmd(HT_ADC, ENABLE);
	
	
}


/*************************	FUNCIONES UART	*************************************/
/*
uint8_t hardware_uart_read (void)
{
	uint8_t	retval;
	
	
	if(RETARGET_USART_PORT->SR & (1<<5))	//check RXDY flag
	{
		retval = RETARGET_USART_PORT->DR;
		RETARGET_USART_PORT->SR|=(1<<5);		//clear RXDY flag
	}
	else
		retval = 0;
	
	return retval;
}
*/
void hardware_uart_transmit(char * fmt,...)
{
	//https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
	//https://stackoverflow.com/questions/21540778/pass-varargs-to-printf
	//http://software-dl.ti.com/ccs/esd/documents/sdto_cgt_tips_for_using_printf.html

	//Esto es para habilitar impresion de Floats
	//http://www.openstm32.org/forumthread954

	va_list ap; 				// points to each unnamed arg in turn
	va_start(ap, fmt); 	// make ap point to 1st unnamed arg
	vprintf(fmt,ap);
	fflush(stdout);

	va_end(ap); 				// clean up when done
}



void uart_init (void)
{
	USART_InitTypeDef USART_InitStructure;

  /* Enable peripheral clock of AFIO, USART0                                                                */
 /* CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
  CKCUClock.Bit.AFIO   = 1;
  COM1_CLK(CKCUClock)  = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);*/

  /* Config AFIO mode as USART0_Rx and USART0_Tx function.                                                  */
  AFIO_GPxConfig(COM1_TX_GPIO_ID, COM1_TX_AFIO_PIN, AFIO_FUN_USART_UART);
  AFIO_GPxConfig(COM1_RX_GPIO_ID, COM1_RX_AFIO_PIN, AFIO_FUN_USART_UART);

  /* USART0 configuration ----------------------------------------------------------------------------------*/
  /* USART0 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - None parity bit
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WORDLENGTH_8B;
  USART_InitStructure.USART_StopBits = USART_STOPBITS_1;
  USART_InitStructure.USART_Parity = USART_PARITY_NO;
  USART_InitStructure.USART_Mode = USART_MODE_NORMAL;

  USART_Init(COM1_PORT, &USART_InitStructure);
  USART_TxCmd(COM1_PORT, ENABLE);
  USART_RxCmd(COM1_PORT, ENABLE);

}
uint8_t hardware_uart_read (void)
{
	uint8_t	retval;
	
	
	if(RETARGET_USART_PORT->SR & (1<<5))	//check RXDY flag
	{
		retval = RETARGET_USART_PORT->DR;
		RETARGET_USART_PORT->SR|=(1<<5);		//clear RXDY flag
	}
	else
		retval = 0;
	
	return retval;
}


/*************************	FUNCIONES TIMERS	*************************************/

static void micro_timers_config (void)
{
	TM_TimeBaseInitTypeDef TM_Struct_init;
	
	//Inicializo el BFTM. Este temporizador es ascendente y puede funcionar como repetitivo o como "one shot"
	//Este temporizador lo puedo usar como avance. Activarlo cuando llega el flanco del hall y cuando llega al 
	//valor requerido dispara la irq y se detiene.
	BFTM_ClearFlag(HT_BFTM0);
	BFTM_SetCounter(HT_BFTM0, 0);
	BFTM_OneShotModeCmd(HT_BFTM0,ENABLE);
	BFTM_IntConfig(HT_BFTM0, ENABLE);
	NVIC_EnableIRQ(BFTM0_IRQn);

	//Inicializo el SCTM0.
	TM_Struct_init.CounterMode = TM_CNT_MODE_UP;
	TM_Struct_init.CounterReload = 0xFFFF;
	TM_Struct_init.Prescaler = CONST_time_uS_to_cnt_40MHz; //Cargo el prescaler para generar una cuenta cada 1uS
	TM_Struct_init.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
	
	TM_TimeBaseInit(HT_SCTM0,&TM_Struct_init);
	
	TM_ClearIntPendingBit(HT_SCTM0,TM_INT_CH0CC);
	TM_ClearFlag(HT_SCTM0,TM_INT_UEV);
	NVIC_EnableIRQ(SCTM0_IRQn);
	TM_SetCounter(HT_SCTM0,0);
	TM_IntConfig(HT_SCTM0,TM_INT_UEV,ENABLE);	

	//Inicializo el SCTM1.
	TM_TimeBaseInit(HT_SCTM1,&TM_Struct_init);
	
	TM_ClearIntPendingBit(HT_SCTM1,TM_INT_CH0CC);
	TM_ClearFlag(HT_SCTM1,TM_INT_UEV);
	NVIC_EnableIRQ(SCTM1_IRQn);
	TM_SetCounter(HT_SCTM1,0);
	TM_IntConfig(HT_SCTM1,TM_INT_UEV,ENABLE);	
}

/*************************	FUNCIONES IRQ		*************************************/

//Linkea la función a la que se llamará ante un evento en el timer indicado.
int32_t hardware_tim_link_callback (void (*func_pointer)(void), uint32_t __hardware_tim_sel)
{
	switch(__hardware_tim_sel)
	{
		case __HARDWARE_TIM_BFTM0_SEL: func_ptr_callback_bftm0 = func_pointer;
															break;
		case __HARDWARE_TIM_SCTM0_SEL: func_ptr_callback_sctm0 = func_pointer;
															break;		
		case __HARDWARE_TIM_SCTM1_SEL: func_ptr_callback_sctm1 = func_pointer;
															break;
		case __HARDWARE_TIM_GPTM0_SEL: func_ptr_callback_gptm0 = func_pointer;
															break;
		default:	return -1;
	}
	
	return 0;
}

//Linkea la funcion que se desea llamar cuando suceda un evento EXTI (tipo rising)
//Argumentos: - puntero de la funcion que se desea llamar cuando haya un evento rising en la linea indicada por el otro argumento
//            - Linea a la cual se desea linkear el callback (de 0 a 15)
int32_t hardware_gpio_event_rising_event_link_callback(void (*func_pointer)(void),uint32_t exti_line)
{ 
	if(exti_line<=15)
		func_ptr_callback_rising_exti[exti_line] = func_pointer;
	else
		return -1;
	
	return 0;
}
//Linkea la funcion que se desea llamar cuando suceda un evento EXTI (tipo falling)
//Argumentos: - puntero de la funcion que se desea llamar cuando haya un evento rising en la linea indicada por el otro argumento
//            - Linea a la cual se desea linkear el callback (de 0 a 15)
int32_t hardware_gpio_event_falling_event_link_callback(void (*func_pointer)(void),uint32_t exti_line)
{
	if(exti_line<=15)
		func_ptr_callback_falling_exti[exti_line] = func_pointer;
	else
		return -1;
	
	return 0;
}

#define EXTI_CALLBACKS_MACRO(EXTI_CHANNEL)	{if(EXTI_GetEdgeStatus(EXTI_CHANNEL,EXTI_EDGE_POSITIVE)){\
																							(*func_ptr_callback_rising_exti[EXTI_CHANNEL])();EXTI_ClearEdgeFlag(EXTI_CHANNEL);}\
																						 if(EXTI_GetEdgeStatus(EXTI_CHANNEL,EXTI_EDGE_NEGATIVE)){\
																							(*func_ptr_callback_falling_exti[EXTI_CHANNEL])();EXTI_ClearEdgeFlag(EXTI_CHANNEL);}}
										
void EXTI0_1_IRQHandler(void)
{
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_0);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_1);
}

void EXTI2_3_IRQHandler(void)
{
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_2);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_3);
}

void EXTI4_15_IRQHandler(void)
{
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_4);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_5);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_6);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_7);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_8);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_9);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_10);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_11);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_12);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_13);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_14);
	EXTI_CALLBACKS_MACRO(EXTI_CHANNEL_15);
}

void SCTM0_IRQHandler (void)
{
	TM_ClearFlag(HT_SCTM0,TM_INT_UEV);
	(*func_ptr_callback_sctm0)();	
}

void SCTM1_IRQHandler (void)
{
	TM_ClearFlag(HT_SCTM1,TM_INT_UEV);
	(*func_ptr_callback_sctm1)();	
}

void BFTM0_IRQHandler(void)
{
  BFTM_ClearFlag(HT_BFTM0);
	 (*func_ptr_callback_bftm0)();
}

/******************************************************
  * @brief  Configure the NVIC vector table.					*
  * @retval None																			*
  *****************************************************/
void NVIC_Configuration(void)
{
  NVIC_SetVectorTable(NVIC_VECTTABLE_FLASH, 0x0);     /* Set the Vector Table base location at 0x00000000   */
}

/*************************	FUNCIONES SYSTICK	*************************************/

/******************************************************
  * @brief  Configure the NVIC Systick								*
  * @retval None																			*
  *****************************************************/
void SYSTICK_Configuration(void)
{
	 /* SYSTICK configuration */
  SYSTICK_ClockSourceConfig(SYSTICK_SRC_STCLK);       // Default : CK_SYS/8
  SYSTICK_SetReloadValue(SystemCoreClock / 8 / 1000); // (CK_SYS/8/1000) = 1ms on chip
  SYSTICK_IntConfig(ENABLE);                          // Enable SYSTICK Interrupt
	SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);
  SYSTICK_CounterCmd(SYSTICK_COUNTER_ENABLE);
}

	


/******************************************************
  * @brief  Configure the system clocks. 							*
  * @retval None																			*
	* Internal 8MHz RC oscillator HSI - PLL to 40MHz		*
  *****************************************************/
void CKCU_Configuration(void)
{
/*
//<e0> Enable Peripheral Clock
//  <h> Communication
//    <q5> EBI
//    <q11> I2C0   <q12> I2C1
//    <q23> I2S
//    <q21> SCI0 <q22> SCI1
//    <q13> SPI0   <q14> SPI1
//    <q17> UART0  <q18> UART1
//    <q15> USART0 <q16> USART1
//    <q3>  USB
//  </h>
//  <h> IO
//    <q7> GPIO Port A <q8>  GPIO Port B <q9>  GPIO Port C <q10>  GPIO Port D
//    <q19> AFIO
//    <q20> EXTI
//  </h>
//  <h> System
//    <q32> ADC
//    <q4>  CKREF
//    <q6>  CRC
//    <q31> CMP
//    <q2>  PDMA
//    <q26> PWRCU
//  </h>
//  <h> Timer
//    <q29> BFTM0 <q30> BFTM1
//    <q33> SCTM0 <q34> SCTM1 <q35> SCTM2 <q36> SCTM3
//    <q27> GPTM0 <q28> GPTM1
//    <q24> MCTM0
//    <q26> RTC   <q25> WDT
//  </h>
//</e>
*/

  CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
  CKCUClock.Bit.PDMA       = CK_OFF;
  CKCUClock.Bit.USBD       = CK_OFF;
  CKCUClock.Bit.CKREF      = CK_OFF;
  CKCUClock.Bit.EBI        = CK_OFF;
  CKCUClock.Bit.CRC        = CK_OFF;
  CKCUClock.Bit.PA         = CK_ON;
  CKCUClock.Bit.PB         = CK_ON;
  CKCUClock.Bit.PC         = CK_OFF;
  CKCUClock.Bit.PD         = CK_OFF;
  CKCUClock.Bit.I2C0       = CK_OFF;
  CKCUClock.Bit.I2C1       = CK_OFF;
  CKCUClock.Bit.SPI0       = CK_OFF;
  CKCUClock.Bit.SPI1       = CK_OFF;
  CKCUClock.Bit.USART0     = CK_ON;
  CKCUClock.Bit.USART1     = CK_ON;
  CKCUClock.Bit.UART0      = CK_ON;
  CKCUClock.Bit.UART1      = CK_ON;
  CKCUClock.Bit.AFIO       = CK_ON;
  CKCUClock.Bit.EXTI       = CK_ON;
  CKCUClock.Bit.SCI0       = CK_OFF;
  CKCUClock.Bit.SCI1       = CK_OFF;
  CKCUClock.Bit.I2S        = CK_OFF;
  CKCUClock.Bit.MCTM0      = CK_OFF;
  CKCUClock.Bit.WDT        = CK_OFF;
  CKCUClock.Bit.BKP        = CK_OFF;
  CKCUClock.Bit.GPTM0      = CK_ON;
  CKCUClock.Bit.GPTM1      = CK_OFF;
  CKCUClock.Bit.BFTM0      = CK_ON;
  CKCUClock.Bit.BFTM1      = CK_OFF;
  CKCUClock.Bit.CMP        = CK_OFF;
  CKCUClock.Bit.ADC        = CK_ON;
  CKCUClock.Bit.SCTM0      = CK_ON;
  CKCUClock.Bit.SCTM1      = CK_ON;
  CKCUClock.Bit.SCTM2      = CK_OFF;
  CKCUClock.Bit.SCTM3      = CK_OFF;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);


#if (ENABLE_CKOUT == 1)
  CKOUTConfig();
#endif
}

#if (ENABLE_CKOUT == 1)
/*********************************************************************************************************//**
  * @brief  Configure the debug output clock.
  * @retval None
  ***********************************************************************************************************/
void CKOUTConfig(void)
{
  CKCU_CKOUTInitTypeDef CKOUTInit;

  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_9, AFIO_MODE_15);
  CKOUTInit.CKOUTSRC = CKCU_CKOUTSRC_HCLK_DIV16;
  CKCU_CKOUTConfig(&CKOUTInit);
}
#endif


#if (HT32_LIB_DEBUG == 1)
/*********************************************************************************************************//**
  * @brief  Report both the error name of the source file and the source line number.
  * @param  filename: pointer to the source file name.
  * @param  uline: error line source number.
  * @retval None
  ***********************************************************************************************************/
void assert_error(u8* filename, u32 uline)
{
  /*
     This function is called by IP library that the invalid parameters has been passed to the library API.
     Debug message can be added here.
     Example: printf("Parameter Error: file %s on line %d\r\n", filename, uline);
  */

  while (1)
  {
  }
}
#endif



/*******	FUNCIÓN PÚBLICA PARA LA CONFIGURACIÓN INTEGRAL DE LOS PERIFÉRICOS	*******/
void micro_config (void)
{
	//RECORDAR DE MODIFICAR EL ARCHIVO "system_ht32f5xxxxx_01.c" PARA CONFIGURAR EL CLOCK CON EL OSC INTERNO (HSI)
	
  NVIC_Configuration();               /* NVIC configuration                                                 */
  CKCU_Configuration();               /* System Related configuration                                       */
  GPIO_Configuration();               /* GPIO Related configuration                                         */
  RETARGET_Configuration();           /* Retarget Related configuration                                     */
	SYSTICK_Configuration();
	
	micro_gpios_outputs_pp_config();	
	micro_gpios_inputs_pullup_config();
	micro_gpios_hiz_config();
	micro_gpios_edge_events_config();
		
	micro_pwm_config();
	
	micro_timers_config();

	micro_adc_config();	

}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
