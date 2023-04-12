/*********************************************************************************************************//**
 * @file    GPIO/InputOutput/main.c
 * @version $Rev:: 3148         $
 * @date    $Date:: 2018-10-18 #$
 * @brief   Main program.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "AL01_board_motor_dev_rev2019_01.h"


/* Global functions ----------------------------------------------------------------------------------------*/
/* Settings ------------------------------------------------------------------------------------------------*/
/* Private types -------------------------------------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------------------------------------*/
/* Global variables ----------------------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------------------------------------*/
/* Global functions ----------------------------------------------------------------------------------------*/


/*********************************************************************************************************//**
  * @brief  Main program.
  * @retval None
  ***********************************************************************************************************/
//#include "main.h"

#include "AL01_board_motor_dev_rev2019_01.h"
#include "AL03_motor_1phase_driver.h"

#define	TERMINAL_REFRESH_TEMP_mS/*********/1000
#define	LED_BLINK_TIME_mS/****************/1000
#define TEMP_CHK_PERIOD_mS/***************/997
#define	VBUS_CHK_PERIOD_mS/***************/7
#define	VBUS_FILTER_DEPTH/****************/4
#define	VBUS_MAX_VALUE/*******************/328 //Con 230Vac de entrada y Cin=27uF, leo un m�ximo de 328V
#define	VBUS_TH_n/************************/100
#define	VBUS_TH_p/************************/200
#define VBUS_TH_low_speed/****************/150		//En la medici�n, cuando rectifico media onda, el Vbus llega a un minimo de 60,8V
#define VBUS_TH_high_speed/***************/190	//En la medici�n, cuando no rectifico, el Vbus llega a un m�nimo de 205,5V
#define VBUS_TH_shut_down/****************/45		//Dado que el m�nimo medido es de 60,8V estimo que si leo un minimo de 45V entonces se est� apagando el equipo
#define	VBUS_HIGH/************************/1
#define	VBUS_LOW/*************************/0
#define VBUS_MED/*************************/2
#define VBUS_MAX_SAMPLES/*****************/20

#define LOW_VOLTAGE_START_DEADTIME_mS/****/1000
#define LOW_VOLTAGE_RESTART_DEADTIME_mS/**/200
#define	DISCHARGE_TIME_mS/****************/350

int32_t restart_timer=0;
int32_t discharge_timer=0;

int32_t	aux_index=0;
int32_t	aux_index2=0;
int32_t	vbus_array[VBUS_FILTER_DEPTH]={0,0,0,0};
int32_t vbus_min=VBUS_MAX_VALUE;
int32_t	vbus_filt=0;
char		vbus_flag=VBUS_MED;
char		vbus_prev_flag=VBUS_MED;

extern struct motor_1phase_drive motor;

int32_t temp_chk_timer=0;
int32_t	vbus_chk_timer=0;
int32_t LEDoffTime,LEDonTime;

void check_VBus_task(void);
void check_Temp_task(void);

int main(void)
{
	//Inicializo todo lo necesario para comandar el motor monofase. Se configura todo el hardware de la placa
	//y se activan las interrupciones necesarias.
	motor_1phase_init();
	
	//uart_init();
	
	vbus_chk_timer=board_scheduler_load_timer(VBUS_CHK_PERIOD_mS);
	temp_chk_timer=board_scheduler_load_timer(TEMP_CHK_PERIOD_mS);
	
	board_gpio_output_reset(BOARD_LED_port,BOARD_LED_pin);
	
	//Chequeo que la temperatura de los transistores se encuentre dentro de lo requerido
	if (board_get_mosfets_temperature_C() <= MOTOR_TEMP_2_ALLOW_START)
	{
		//Configuro los estados iniciales de la m�quina de
		//estados para iniciar el arranque del motor
		motor_1phase_start_motor();
	}
	
	while (1)
	{
		motor_1phase_state_machine();
		
		//Tarea peri�dica de medici�n y filtrado del voltaje de VBus
		check_VBus_task();
	
		//Ante las condiciones necesarias, se ordena el arranque del motor
		if(board_scheduler_is_time_expired(restart_timer)&&(isMotorStopped())&&(vbus_flag==VBUS_HIGH))
		{
			motor_1phase_start_motor();
		}
		
		//Tarea peri�dica de chequeo de temperatura de los transistores
		check_Temp_task();
	}
}

/****************************************************************/
/*				Tarea de chequeo de voltaje del bus de cont�nua				*/
/****************************************************************/
void check_VBus_task(void)
{
	static int32_t sample_cnt=0;
	int32_t sample;
		if(board_scheduler_is_time_expired(vbus_chk_timer))
		{
			sample=board_get_bus_voltage();
			if(sample<vbus_min)
				vbus_min=sample;
			sample_cnt++;

			//Cuando chequeo el minimo de "VBUS_MAX_SAMPLES" muestras, 
			//analizo ese valor para determinar el estado del VBUS y ejecuto la acci�n necesaria
			if(sample_cnt==VBUS_MAX_SAMPLES)
			{
				//Primero chequeo cual es el nivel y modifico el flag
				if(vbus_min<VBUS_TH_shut_down)
					vbus_flag=VBUS_LOW;
				else
				{
					if(vbus_min<=VBUS_TH_low_speed)
						vbus_flag=VBUS_MED;
				}
				
				if(vbus_min>=VBUS_TH_high_speed)
					vbus_flag=VBUS_HIGH;
				
				sample_cnt=0;
				vbus_min=VBUS_MAX_VALUE;
			
		
				//En caso de detectar un cambio en el bus de alimentaci�n, ejecuto la tarea que corresponda al cambio
				if(vbus_flag==VBUS_HIGH)
				{
					if(vbus_prev_flag==VBUS_LOW)
					{
						restart_timer=board_scheduler_load_timer(LOW_VOLTAGE_START_DEADTIME_mS);
					}
					if((vbus_prev_flag==VBUS_MED)||(vbus_prev_flag==VBUS_HIGH))
					{
						set_motor_high_speed();
						//set_motor_low_speed();
					}
				}
				else if(vbus_flag==VBUS_LOW)
				{
					if((vbus_prev_flag==VBUS_HIGH)||(vbus_prev_flag==VBUS_MED))
					{
						//motor_1phase_stop_motor(MOTOR_STOP_METHOD_BREAK,MOTOR_STOPPED);
						discharge_timer=board_scheduler_load_timer(DISCHARGE_TIME_mS);
					}
					else
					{
						if(board_scheduler_is_time_expired(discharge_timer))
							motor_1phase_stop_motor(MOTOR_STOP_METHOD_FREEWHEEL,MOTOR_STOPPED);
						//mientras el bus esta bajo, mantengo reiniciando el timer para que no intente arrancar
						restart_timer=board_scheduler_load_timer(LOW_VOLTAGE_RESTART_DEADTIME_mS);
						//board_gpio_output_reset(BOARD_LED_port,BOARD_LED_pin);
					}
				}
				else if(vbus_flag==VBUS_MED)
				{
					//si me mantengo en VBUS_MED, no hago nada
					if(vbus_prev_flag!=VBUS_MED)
					{
						//si estoy en VBUS_MED y vengo de otro estado, configuro la velocidad baja
						//set_motor_low_speed();
						set_motor_high_speed();
						if(vbus_prev_flag==VBUS_LOW)
						{
							//Si el estado previo era VBUS_LOW establezco el temporizador para el rearranque
							restart_timer=board_scheduler_load_timer(LOW_VOLTAGE_START_DEADTIME_mS);
							//board_gpio_output_set(BOARD_LED_port,BOARD_LED_pin);
						}
					}
				}
				vbus_prev_flag=vbus_flag;
			}
			//Recargo temporizador peri�dico
			vbus_chk_timer=board_scheduler_load_timer(VBUS_CHK_PERIOD_mS);
		}
}

/************************************************************/
/*		Tarea de chequeo de temperatura de MOSFETs			*/
/************************************************************/
void check_Temp_task(void)
{
		if(board_scheduler_is_time_expired(temp_chk_timer))
		{
			//Chequeo de forma peri�dica que los transistores no excedan la temperatura recomendada
						
			if((board_get_mosfets_temperature_C()>=MOTOR_TEMP_2_FORCE_STOP))//||(board_get_shunt_current()>MOTOR_CURRENT_TO_FORCE_STOP))
			{
				motor_1phase_stop_motor(MOTOR_STOP_METHOD_FREEWHEEL, MOTOR_FAIL);
				LEDoffTime=board_scheduler_load_timer(LED_BLINK_TIME_mS);
				LEDonTime=board_scheduler_load_timer(LED_BLINK_TIME_mS+LED_BLINK_TIME_mS);
				
				while(1)
				{
					if(board_scheduler_is_time_expired(LEDoffTime))
					{
						board_gpio_output_set(BOARD_LED_port,BOARD_LED_pin);
					}
					if(board_scheduler_is_time_expired(LEDonTime))
					{
						board_gpio_output_reset(BOARD_LED_port,BOARD_LED_pin);
						LEDoffTime=board_scheduler_load_timer(LED_BLINK_TIME_mS);
						LEDonTime=board_scheduler_load_timer(LED_BLINK_TIME_mS+LED_BLINK_TIME_mS);
					}			
				}
			}
			//Reseteo el temporizador de esta tarea.
			temp_chk_timer=board_scheduler_load_timer(TEMP_CHK_PERIOD_mS);
		}
}
