/************************************
 * AL02_inverter_1phase.c						*
 *																	*
 *  Created on: 10/09/19						*
 *      Author: Hernán Couto				*
 ************************************/
#include	"AL02_inverter_1phase.h"
#include	"AL01_board_motor_dev_rev2019_01.h"

static int32_t inverter_actual_comm_seq=INVERTER_COMM_FREEWHEEL;

static void current_sensor_i_peak_callback (void);

/************************************************************************************
 *	Función con las configuraciones iniciales necesarias para utilizar el inverter	*
 ************************************************************************************/
int32_t inverter_1phase_init_config(void)
{
	//Configuro primero todo el hardware de la placa
	board_hardware_configuration();

	//Configuro el PWM con los parámetros deseados para el inverter
	inverter_1phase_pwm_set_period_us(INVERTER_PWM_PERIOD_uS);
	inverter_1phase_pwm_set_toff_us(INVERTER_PWM_TOFF_uS);	
	
	//Cada vez que la corriente atraviese un umbral predeterminado se va a llamar
	//a "current_sensor_i_peak_callback", esta función deberá accionar ante una
	//suba importante de la corriente, cortando las salidas.
	board_gpio_event_falling_link_callback(current_sensor_i_peak_callback,OVER_CURRENT_pin);
	board_gpio_event_falling_only_detection_enable(OVER_CURRENT_pin);
	
	//Me aseguro que el estado inicial del inverter sea de giro libre
	inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL, INVERTER_STATE_OVERWRITE);

	return 0;
}

/************************************************************************************
*	Función para configurar la secuencia en la que se desea que esté el inverter	*
*																					*
*	En el caso del motor monofásico, sólo hay dos pasos, la corriente circula en un	*
*	sentido o en el otro, a través de un puente H.									*
*	El resto de las secuencias son de frenado o de giro libre						*
*************************************************************************************/
int32_t inverter_1phase_comm_set_seq (int32_t inverter_comm_seq,int32_t inverter_state_overwrite)
{
	int32_t aux;

	if(inverter_state_overwrite == INVERTER_STATE_NOT_OVERWRITE)
	{
		aux = inverter_actual_comm_seq;		//Guardo el valor del estado
	}

	switch(inverter_comm_seq)
	{
		case INVERTER_COMM_SEQ1://Esta secuencia habilita la circulación de corriente a través de
								//los transistores HIN2 y LIN1
								inverter_actual_comm_seq = INVERTER_COMM_SEQ1;

								inverter_1phase_hin2_disable();
								inverter_1phase_lin2_enable();

								inverter_1phase_lin1_disable();
								inverter_1phase_hin1_enable();
								
								/*inverter_1phase_lin2_disable();
								inverter_1phase_hin2_enable();
		
								inverter_1phase_hin1_disable();
								inverter_1phase_lin1_enable();*/
								//Se configura que el pwm arranque encendido
								inverter_1phase_pwm_start_with_ton();

								break;

		case INVERTER_COMM_SEQ1_DISCHARGE://Esta secuencia se usa para descargar la inductancia
										  //luego de la secuencia 1, dejando habilitado sólo el
										  //transistor HIN2
								inverter_actual_comm_seq = INVERTER_COMM_SEQ1_DISCHARGE;

								inverter_1phase_lin1_disable();
								inverter_1phase_hin2_disable();
								inverter_1phase_lin2_disable();
								inverter_1phase_hin1_enable();			

								break;

		case INVERTER_COMM_SEQ2://Esta secuencia habilita la circulación de corriente a través de
								//los transistores HIN1 y LIN2
								inverter_actual_comm_seq = INVERTER_COMM_SEQ2;
								
								inverter_1phase_hin1_disable();
								inverter_1phase_lin1_enable();

								inverter_1phase_lin2_disable();
								inverter_1phase_hin2_enable();
								/*inverter_1phase_lin1_disable();
								inverter_1phase_hin1_enable();

								inverter_1phase_hin2_disable();
								inverter_1phase_lin2_enable();*/

								//Se configura que el pwm arranque encendido
								inverter_1phase_pwm_start_with_ton();
		
								break;

		case INVERTER_COMM_SEQ2_DISCHARGE://Esta secuencia se usa para descargar la inductancia
			  	  	  	  	  	  	  	  //luego de la secuencia 2, dejando habilitado sólo el
			  	  	  	  	  	  	  	  //transistor LIN2
								inverter_actual_comm_seq = INVERTER_COMM_SEQ2_DISCHARGE;

								inverter_1phase_lin2_disable();
								inverter_1phase_hin1_disable();
								inverter_1phase_lin1_disable();
								inverter_1phase_hin2_enable();

								break;

		case INVERTER_COMM_FREEWHEEL://Esta secuencia deja apagados todos los transistores dejando
									//al motor en giro libre
								inverter_actual_comm_seq = INVERTER_COMM_FREEWHEEL;

								inverter_1phase_hin1_disable();
								inverter_1phase_hin2_disable();
								inverter_1phase_lin1_disable();
								inverter_1phase_lin2_disable();

								break;

		case INVERTER_COMM_BREAK_LOW://Esta secuencia frena el motor encendiendo los dos transistores
									 //de la parte baja del puente H, o sea conectando los dos terminales
									 //del motor a GND
								inverter_actual_comm_seq = INVERTER_COMM_BREAK_LOW;
								
								inverter_1phase_hin1_disable();
								inverter_1phase_hin2_disable();
								
								inverter_1phase_lin1_enable();
								inverter_1phase_lin2_enable();
	
								break;

		case INVERTER_COMM_BREAK_HIGH://Esta secuencia frena el motor encendiendo los dos transistores
			 	 	 	 	 	 	  //de la parte alta del puente H, o sea conectando los dos terminales
			 	 	 	 	 	 	  //del motor a VBUS
								inverter_actual_comm_seq = INVERTER_COMM_BREAK_HIGH;
								
								inverter_1phase_lin1_disable();
								inverter_1phase_lin2_disable();

								inverter_1phase_hin1_enable();
								inverter_1phase_hin2_enable();
		
								break;

		default:
							return -1;
	}

	//En caso que se haya requerido no sobre-escribir el estado actual, se vuelve a
	//dejar registrado el estado en el que se encontraba al inicio de la función.
	if(inverter_state_overwrite == INVERTER_STATE_NOT_OVERWRITE)
	{
		inverter_actual_comm_seq = aux;
	}

	return 0;
}

/********************************************************************************
 *	Esta función hace el cambio de secuencia funcional (SEQ1 o SEQ2) de una 	*
 *	a la otra. Son sólo dos dado que el motor es monofase.						*
 ********************************************************************************/
int32_t inverter_1phase_comm_next_seq (void)
{
	if(inverter_actual_comm_seq==INVERTER_COMM_SEQ1)
	{
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
		
		inverter_actual_comm_seq = INVERTER_COMM_SEQ2;
	}
	else
	{
		if(inverter_actual_comm_seq==INVERTER_COMM_SEQ2)
		{
			inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
			inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
			inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
			inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
			
			inverter_actual_comm_seq = INVERTER_COMM_SEQ1;
		}
	}

	inverter_1phase_comm_set_seq(inverter_actual_comm_seq,INVERTER_STATE_OVERWRITE);

	return 0;
}

/************************************************************
 *	Función que retorna la secuencia actual establecida		*
 ************************************************************/
int32_t	inverter_1phase_get_actual_comm_seq (void)
{
	return inverter_actual_comm_seq;
}

/************************************************************************
 * Función a la que se llamará cada vez que la corriente en el shunt 	*
 * atraviese el umbral que tiene definido (por hardware).				*
 ************************************************************************/
static void current_sensor_i_peak_callback (void)
{
	inverter_1phase_pwm_set_outputs_to_toff();
}

