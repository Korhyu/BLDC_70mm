#ifndef AL01_BOARD_MOTOR_DEV_REV2019_01

#define AL01_BOARD_MOTOR_DEV_REV2019_01

#include "AL00_micro_ht32f52230.h"

void board_hardware_configuration (void);

/******************************************************************************************/
/*	Definiciones, macros y prototipos para el manejo de timers en el programa principal		*/
/******************************************************************************************/
#define BOARD_TIM_WATCHDOG_SEL_CALLBACK/**********************************/__HARDWARE_TIM_BFTM0_SEL
#define BOARD_TIM_ADV_COMM_SEL_CALLBACK/**********************************/__HARDWARE_TIM_SCTM0_SEL
#define BOARD_TIM_MEASURE_SEL_CALLBACK/***********************************/__HARDWARE_TIM_SCTM1_SEL

#define BOARD_WATCHDOG_TIMER/*********************************************/HT_BFTM0
#define BOARD_ADVANCE_COMM_TIMER/*****************************************/HT_SCTM0
#define	BOARD_MEASURE_TIMER/**********************************************/HT_SCTM1
																																		 																																						 
#define board_timer_link_callback(func_ptr,__hardware_tim_sel)/***********/hardware_tim_link_callback(func_ptr,__hardware_tim_sel)

//Manejo del Watchdog Timer
#define board_watchdog_timer_init_with_timeout_irq_us(time_1us_cnt)/******/__harwdare_tim_bftm_init_timer_with_timeout_irq_us(BOARD_WATCHDOG_TIMER,time_1us_cnt)
#define board_watchdog_timer_abort()/*************************************/__harwdare_tim_bftm_abort(BOARD_WATCHDOG_TIMER)			
#define board_watchdog_timer_get_count_us()/******************************/__hardware_tim_bftm_get_count_us(BOARD_WATCHDOG_TIMER)

//Manejo del Advance Commutation Timer
#define board_advance_comm_timer_init_with_timeout_irq_us(time_1us_cnt)/**/__hardware_tim_sctm_init_timer_with_timeout_irq_us(BOARD_ADVANCE_COMM_TIMER,time_1us_cnt)
#define board_advance_comm_timer_abort()/*********************************/__harwdare_tim_sctm_abort(BOARD_ADVANCE_COMM_TIMER)		
#define board_advance_comm_timer_get_count_us()/**************************/__hardware_tim_sctm_get_count_us(BOARD_ADVANCE_COMM_TIMER)

//Manejo del Measure Timer
#define board_measure_timer_init_with_timeout_irq_us(time_1us_cnt)/*******/__hardware_tim_sctm_init_timer_with_timeout_irq_us(BOARD_MEASURE_TIMER,time_1us_cnt)
#define board_measure_timer_abort()/**************************************/__harwdare_tim_sctm_abort(BOARD_MEASURE_TIMER)		
#define board_measure_timer_get_count_us()/*******************************/__hardware_tim_sctm_get_count_us(BOARD_MEASURE_TIMER)

//Manejo del Scheduler	
int32_t board_scheduler_load_timer(int32_t time_ms);
int32_t board_scheduler_is_time_expired(int32_t timer);

/****************************************************************************************/
/*	Definiciones, macros y prototipos para el manejo del ADC en el programa principal		*/
/****************************************************************************************/
#define BOARD_ADC_CHANNEL_NTC_POWER_SEL	0
#define BOARD_ADC_CHANNEL_SHUNT_SEL 		1			
#define BOARD_ADC_CHANNEL_VBUS_SEL			2			

int32_t board_adc_get_measure (uint32_t board_adc_channel_select, uint32_t * measure_result);
int32_t board_get_mosfets_temperature_C(void);
int32_t board_get_bus_voltage(void);
int32_t	board_get_shunt_current(void);
/******************************************************************************************/
/*	Definiciones, macros y prototipos para el manejo de GPIO's en el programa principal		*/
/******************************************************************************************/
#define	LIN1_pin								GPIO14
#define	LIN1_pin_mask						GPIO_PIN_14
#define	LIN1_port								GPIOA
#define	HIN1_pin								GPIO15
#define	HIN1_pin_mask						GPIO_PIN_15
#define	HIN1_port								GPIOA

#define	LIN2_pin								GPIO0
#define	LIN2_pin_mask						GPIO_PIN_0
#define	LIN2_port								GPIOB
#define	HIN2_pin								GPIO1
#define	HIN2_pin_mask						GPIO_PIN_1
#define	HIN2_port								GPIOB

#define	HALL_SENSOR_pin					GPIO7
#define	HALL_SENSOR_pin_mask		GPIO_PIN_7
#define	HALL_SENSOR_port				GPIOB

#define	OVER_CURRENT_pin				GPIO8
#define	OVER_CURRENT_pin_mask		GPIO_PIN_8
#define	OVER_CURRENT_port				GPIOB

#define	BOARD_LED_pin					  GPIO4
#define	BOARD_LED_pin_mask			GPIO_PIN_4
#define BOARD_LED_port					GPIOB

//Cambio de configuración de pines
#define board_hardware_gpio_config_output_pp_pins_load_config(gpio_port,gpio_pin)/****/hardware_gpio_config_output_pp_pins_load_config(gpio_port,gpio_pin)
#define board_hardware_gpio_config_edge_events_pins_load_config(gpio_port,gpio_pin)/**/hardware_gpio_config_edge_events_pins_load_config(gpio_port,gpio_pin)
#define board_hardware_gpio_config_hiz_pins_load_config(gpio_port,gpio_pin)/**********/hardware_gpio_config_hiz_pins_load_config(gpio_port,gpio_pin)
#define board_hardware_gpio_config_inputs_pins_load_config(gpio_port,gpio_pin)/*******/hardware_gpio_config_inputs_pins_load_config(gpio_port,gpio_pin)

//Linkeo de funciones en eventos de flanco ascendente o descendente
#define	board_gpio_event_rising_link_callback(func_ptr,exti_line)/********************/hardware_gpio_event_rising_event_link_callback(func_ptr,exti_line)
#define	board_gpio_event_falling_link_callback(func_ptr,exti_line)/*******************/hardware_gpio_event_falling_event_link_callback(func_ptr,exti_line)

//Control de eventos en los gpio's
#define	board_gpio_event_rising_only_detection_enable(gpio_pin)/**********************/__hardware_gpio_event_rising_only_detection_enable(gpio_pin)
#define	board_gpio_event_falling_only_detection_enable(gpio_pin)/*********************/__hardware_gpio_event_falling_only_detection_enable(gpio_pin)
#define	board_gpio_event_both_edges_detection_enable(gpio_pin)/***********************/__hardware_gpio_event_both_edges_detection_enable(gpio_pin)
#define board_gpio_event_detection_disable(gpio_pin)/*********************************/__hardware_gpio_event_detection_disable(gpio_pin)

//Control de salidas
#define board_gpio_output_reset(gpio_port,GPIOnum)/***********************************/__hardware_gpio_output_reset(gpio_port,GPIOnum)
#define board_gpio_output_set(gpio_port,GPIOnum)/*************************************/__hardware_gpio_output_set(gpio_port,GPIOnum)
#define board_gpio_output_toggle(gpio_port,GPIOnum)/**********************************/__hardware_gpio_output_toggle(gpio_port,GPIOnum)

#define board_gpio_hiz_set_as_hiz(gpio_port,gpio_pin)/********************************/__hardware_gpio_config_set_as_hiz_pin(gpio_port,gpio_pin)
#define board_gpio_hiz_set_as_pushpull_output_pin(gpio_port,gpio_pin)/****************/__hardware_gpio_config_set_as_pushpull_output_pin(gpio_port,gpio_pin)
		
//Lectura de entradas
#define board_gpio_input_read_state(gpio_port,gpio_pin)/******************************/__hardware_gpio_input_read_state(gpio_port,gpio_pin)

//Comando de salidas especiales
/*Desmagnetizacion a traves de los transistores altos*/
#define	board_hin1_enable()/****************/__hardware_gpio_output_set(HIN1_port,HIN1_pin)
#define	board_hin1_disable()/***************/__hardware_gpio_output_reset(HIN1_port,HIN1_pin)
#define	board_hin2_enable()/****************/__hardware_gpio_output_set(HIN2_port,HIN2_pin)
#define	board_hin2_disable()/***************/__hardware_gpio_output_reset(HIN2_port,HIN2_pin)
#define	board_lin1_enable()/***************/__hardware_pwm1_enable()
#define	board_lin1_disable()/**************/__hardware_pwm1_disable()
#define	board_lin2_enable()/***************/__hardware_pwm2_enable()
#define	board_lin2_disable()/**************/__hardware_pwm2_disable()

/******************************************************************************************/
/*	Definiciones, macros y prototipos para el manejo del PWM en el programa principal			*/
/******************************************************************************************/
#define	board_pwm_set_period_us(period_uS)/********/hardware_pwm_set_period_us(period_uS)
#define	board_pwm_get_period_us()/*****************/hardware_pwm_get_period_us()
#define	board_pwm_set_ton_us(ton_uS)/**************/hardware_pwm_set_ton_us(ton_uS)
#define	board_pwm_get_ton_us()/********************/hardware_pwm_get_ton_us()
#define	board_pwm_set_toff_us(toff_uS)/************/hardware_pwm_set_toff_us(toff_uS)
#define	board_pwm_get_toff_us()/*******************/harwdare_pwm_get_toff_us()
#define	board_pwm_start_with_ton()/****************/hardware_pwm_start_with_ton()
#define	board_pwm_set_outputs_to_toff()/***********/hardware_pwm_set_outputs_to_toff()

/******************************************************************************************/
/*	Definiciones, macros y prototipos para el manejo de la UART en el programa principal	*/
/******************************************************************************************/
#define board_uart_read()/*****************************/hardware_uart_read()




#endif /* AL01_BOARD_MOTOR_DEV_REV2019_01 */
