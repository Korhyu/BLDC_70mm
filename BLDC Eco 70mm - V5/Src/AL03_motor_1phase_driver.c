/************************************
 * AL03_motor_1phase_driver.c		*
 *									*
 *  Created on: 10/09/19			*
 *      Author: Hernán Couto		*
 ************************************/

#include "AL03_motor_1phase_driver.h"

							/*	CONSTANTES	*/
/*Tiempo de carga de bootstrap (ojo que tambien es frenado electrico del motor)*/
#define MOTOR_CHARGE_BOOTSTRAP_TIME_mS						100

/*Cantidad de secuencias que se va a excitar al motor sensando ZCD en modo SAMPLE AT END TOFF*/
#define START_FIRST_STEPS_FROM_STAND_COUNT 			 	100

/*Configuracion de PWM que se va a usar desde la primer conmutacion */
#define PWM_START_PERIOD_uS 											INVERTER_PWM_PERIOD_uS
#define	PWM_START_TOFF_uS													INVERTER_PWM_TOFF_uS

#define PWM_PERIOD_BOOTSTRAP_CARGE_uS					1000
#define	PWM_TOFF_BOOTSTRAP_CHARGE_uS					200

#define PWM_PERIOD_1st_SETPS_uS								2000
#define	PWM_TOFF_1st_SETPS_uS									200

#define PWM_PERIOD_RUNNING_uS									250
#define	PWM_TOFF_RUNNING_uS										150

/*Estos parametros dominan la rampa de aceleracion*/
#define PWM_TOFF_UPDATE_TIME_mS								50		//Cada cuanto tiempo actualizo el valor del Toff
#define PWM_TOFF_MAX_SPEED_uS				 					40		//Toff correspondiente a la velocidad máxima del motor - Valor original 20
#define PWM_TOFF_MIN_SPEED_uS									100		//Toff correspondiente a la velocidad mínima del motor
#define PWM_TOFF_INC_DEC_uS	 				 					10 		//Valor de incremento/decremento de Toff
#define PWM_TOFF_SET_POINT_uS									PWM_TOFF_MIN_uS		//Set point de Toff

#define ADVANCE_FRACTION(time)								FRACTION_20_32(time)

#define TIME_TO_GET_RUNNING_TIMEOUT_mS				7400

#define	FIRST_STEPS_DEADTIME_SP_mS	40

#define SOFT_START_UPDATE_TIME_mS		15
/*Este tiempo sirve para determinar cuanto blanking aplicar al primer paso que se da*/
#define START_FIRST_STEP_DURATION_INIT_us				  7000

#define MAX_TIMEOUT_WATCHDOG_us 						  30000

//Estados del sensor hall
#define MOTOR_S_HALL_HIGH	1
#define	MOTOR_S_HALL_LOW	0

#define MOTOR_ROTOR_NUMBER_OF_POLES	4
#define CONSTANT_MHZ_TO_HZ			1000000
#define CONSTANT_HZ_TO_RPM			60
#define FREEWHEEL_COMM_USED			1

#define RISING_EVENT	1
#define	FALLING_EVENT	0

#define	ADVANCE_PERCENTAGE		30
#define DELAY_PERCENTAGE			(100-ADVANCE_PERCENTAGE)
#define	CONSTANT_100PERCENT		100

#define	MOTOR_S_HALL_BLOCK_TIME_PERCENT	60

#define MEAS_TIM_OVERFLOW						1
#define MEAS_TIM_RESET							0
#define MEASURE_TIMER_MAX_COUNT_uS 	(30000)

#define STOP_RUNNING_FLAG_STOP_MOTOR						0
#define STOP_RUNNING_FLAG_DONT_STOP_MOTOR				1

										/*	MACROS	*/
//Macros creadas para habilitar un temporizador que salte una interrupción al finalizar el conteo.
//El objetivo es desfasar la conmutación de los cruces por cero para hacer más eficiente el control.
//Estas macros se utilizarán dentro del callback del cruce por cero. En esa interrupción se obtendran las
//rpm actuales del motor y en base a ese dato se podrá calcular en cuanto tiempo debe saltar este timer
//para efectuar la conmutación.
#define advance_commutation_timer_callback_link(func_ptr)/***/board_timer_link_callback(func_ptr,BOARD_TIM_ADV_COMM_SEL_CALLBACK)
#define advance_commutation_timer_set_within_us(time_us)/****/board_advance_comm_timer_init_with_timeout_irq_us(time_us)
#define advance_commutation_timer_get_count_us()/************/board_advance_comm_timer_get_count_us()
#define advance_commutation_timer_abort()/*******************/board_advance_comm_timer_abort()

//Macros creadas para habilitar un temporizador que sirva como watchdog del motor. En cada cruce por cero
//se debería re cargar el timeout para que, si no sucede un evento de cruce por cero en cierto tiempo, salte
//esta interrupción y detenga el motor mediante un aborto del giro.
#define motor_watchdog_timer_callback_link(func_ptr)/**/board_timer_link_callback(func_ptr,BOARD_TIM_WATCHDOG_SEL_CALLBACK)
#define motor_watchdog_timer_set_timeout_us(time_us)/**/board_watchdog_timer_init_with_timeout_irq_us(time_us)
#define motor_watchdog_timer_get_count_us()/***********/board_watchdog_timer_get_count_us()
#define motor_watchdog_timer_abort()/******************/board_watchdog_timer_abort()

//Macros creadas para habilitar un temporizador que sirva para tomar mediciones en el motor
#define motor_measure_timer_callback_link(func_ptr)/***/board_timer_link_callback(func_ptr,BOARD_TIM_MEASURE_SEL_CALLBACK)
#define motor_measure_timer_set_timeout_us(time_us)/***/board_measure_timer_init_with_timeout_irq_us(time_us)
#define motor_measure_timer_get_count_us()/************/board_measure_timer_get_count_us()
#define motor_measure_timer_abort()/*******************/board_measure_timer_abort()

//Macros para asociar las interrupciones generadas por flancos del sensor hall, con las funciones correspondientes.
//También se definen las macros que habilitan y deshabilitan las interrupciones.
#define motor_sensor_hall_link_rising_callback(func_ptr)/****/board_gpio_event_rising_link_callback(func_ptr,HALL_SENSOR_pin)
#define motor_sensor_hall_link_falling_callback(func_ptr)/***/board_gpio_event_falling_link_callback(func_ptr,HALL_SENSOR_pin)
#define motor_sensor_hall_irq_enable()/**********************/board_gpio_event_both_edges_detection_enable(HALL_SENSOR_pin)
#define motor_sensor_hall_irq_disable()/*********************/board_gpio_event_detection_disable(HALL_SENSOR_pin)

						/* 	FUNCIONES	*/
void 	motor_watchdog_timer_callback					(void);
void 	advance_commutation_timer_callback		(void);
void 	hall_sensor_rising_event_callback 		(void);
void 	hall_sensor_falling_event_callback 		(void);
void 	hall_sensor_event											(void);
void 	hall_sensor_event_first_steps_action	(void);
void 	hall_sensor_event_advance_comm_action	(void);
void 	hall_sensor_event_freewheel_meas			(void);
void 	motor_measure_timer_callback 					(void);

						/*	VARIABLES	*/
struct motor_1phase_drive{
	int32_t state;

	int32_t running_flag;
	int32_t stop_running_flag;

	int32_t start_first_steps_counter;
	int32_t speed_control_method;

	int32_t	first_steps_deadtime_timer;
	int32_t bootstrap_charge_timer;
	int32_t start_running_timmer;
	int32_t soft_start_timmer;
	int32_t pwm_toff;
	int32_t pwm_SP;
	
	int32_t hall_event;
	int32_t	hall_status;

	int32_t time_from_zcd_to_zcd;
	int32_t time_from_zcd_to_zcd_avg;

	int32_t electrical_period_us;
	int32_t electrical_period_us_avg;

	int32_t frequency_hz;
	int32_t rpm;

	int32_t time_2_comm_since_hall_event_us;

	int32_t measure_timmer_overflow;
};

int32_t i;
static volatile struct motor_1phase_drive motor;
extern char vbus_flag;

/************************************************************************************************************/
/*										FUNCIONES PROGRAMADAS												*/
/************************************************************************************************************/
/************************************************************/
/*	Función para configurar el tiempo Ton del pwm en uS		*/
/************************************************************/
void motor_1phase_set_pwm_ton_us (int32_t pwm_ton_uS)
{
	inverter_1phase_pwm_set_ton_us (pwm_ton_uS);
}
/************************************************************/
/*	Función para configurar el tiempo Toff del pwm en uS	*/
/************************************************************/
void motor_1phase_set_pwm_toff_us (int32_t pwm_toff_uS)
{
	inverter_1phase_pwm_set_toff_us (pwm_toff_uS);
}
/************************************************************/
/*	Función para configurar el paríodo del pwm en uS		*/
/************************************************************/
void motor_1phase_set_pwm_period_us (int32_t pwm_period_uS)
{
	inverter_1phase_pwm_set_period_us (pwm_period_uS);
}

/*******************************************************************************
 *	Esta funcion retorna el valor de la maquina de estados del motor:
 *	MOTOR_STATE_STOPPED
 *	MOTOR_STATE_STARTING
 *	MOTOR_STATE_RUNNING
 *	MOTOR_STATE_FAIL
********************************************************************************/
int32_t motor_1phase_get_state (void)
{
	return motor.state;
}

/*******************************************************************************
 *	Se llama una sola vez en el inicio del sistema, linkea callbacks
 *	y llama a configuraciones de hardware de mas bajo nivel (inverter trifasico)
 *	Tambien inicializa la maquina de estados del motor en el estado inicial
********************************************************************************/
int32_t motor_1phase_init(void)
{
	inverter_1phase_init_config();
	motor.pwm_toff=PWM_START_TOFF_uS;
	//Configuro las interrupciones del sensor Hall
	motor_sensor_hall_link_rising_callback(hall_sensor_rising_event_callback);
	motor_sensor_hall_link_falling_callback(hall_sensor_falling_event_callback);
	motor_sensor_hall_irq_disable();

	//Asigno la función a la cual se llama cuando salta la interrupción del watchdog timer.
	motor_watchdog_timer_callback_link(motor_watchdog_timer_callback);
	motor_watchdog_timer_abort();

	//Asigno la función a la cual se llama cuando salta la interrupción del temporizador para la conmutación adelantada
	advance_commutation_timer_callback_link(advance_commutation_timer_callback);
	advance_commutation_timer_abort();

	//Asigno la función a la cual se llama cuando salta la interrupción del temporizador para las mediciones
	motor_measure_timer_callback_link(motor_measure_timer_callback);

	//Asigno el estado inicial del motor
	motor.state = MOTOR_STOPPED;
	motor.running_flag = STOPPED;
	//inverter_1phase_comm_set_seq(INVERTER_COMM_FREWHEEL,INVERTER_STATE_OVERWRITE);
	
	//Inicializo la medición de velocidad de rotación
	motor_meas_rotation_speed_init();

	//Configuro el control de velocidad
	//motor.speed_control_method = CONTROL_BY_SPEED;
	motor.speed_control_method = CONTROL_BY_VOLTAGE;

	
	return 0;
}

/****************************************************************************/
/*			Función para arrancar el motor									*/
/****************************************************************************/
int32_t motor_1phase_start_motor(void)
{
	if(motor.state != MOTOR_START_INIT && motor.running_flag != RUNNING)
	{
		motor.state = MOTOR_START_INIT;
		motor.running_flag = STARTING;
		motor.stop_running_flag = STOP_RUNNING_FLAG_DONT_STOP_MOTOR;
	}
	else
		return -1;

	return 0;
}

/****************************************************************************************/
/*											Función para detener el motor																		*/
/****************************************************************************************/
int32_t  motor_1phase_stop_motor (int32_t motor_stop_method,int32_t motor_state_exit_value)
{
	//Detengo los temporizadores que generan interrupciones
	advance_commutation_timer_abort();
	motor_sensor_hall_irq_disable();
	motor_watchdog_timer_abort();

	//Levanto el flag de detención del motor
	motor.stop_running_flag = STOP_RUNNING_FLAG_STOP_MOTOR;
	//Bajo el flag de motor girando
	motor.running_flag = STOPPED;

	//Chequeo que el método de detención se encuentre en las opciones válidas
	if(motor_stop_method==MOTOR_STOP_METHOD_BREAK || motor_stop_method==MOTOR_STOP_METHOD_FREEWHEEL)
	{
		//Dependiendo del método requerido, establezco la secuencia necesaria
		if(motor_stop_method==MOTOR_STOP_METHOD_FREEWHEEL)
		{
			inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL, INVERTER_STATE_OVERWRITE);
		}
		else
		{
			inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_LOW, INVERTER_STATE_OVERWRITE);
		}

		//Dado que el motor puede detenerse por pedido o por falla,
		//dejo indicado cual fué la situación.
		if	(motor_state_exit_value == MOTOR_STOPPED)
		{
			motor.state = MOTOR_STOPPED;
			return 0;
		}
		else if (motor_state_exit_value == MOTOR_FAIL)
		{
			motor.state = MOTOR_FAIL;
			return 0;
		}
		else
		{
			motor.state = MOTOR_FAIL;
			return -1;
		}
	}
	else
	{
		return -1;
	}

}

/********************************************************************************/
/*		Máquina de estados que lleva a cabo el funcionamiendo del motor			*/
/********************************************************************************/
void motor_1phase_state_machine(void)
{
	switch (motor.state)
	{
		case MOTOR_STOPPED:
				break;

		case MOTOR_FAIL:
				break;

		case MOTOR_START_INIT:

				motor.bootstrap_charge_timer = 0;
				motor.state = MOTOR_START_BREAK_AND_CHARGE_BOOTSTRAP;
				break;

		case MOTOR_START_BREAK_AND_CHARGE_BOOTSTRAP:

				if(motor.bootstrap_charge_timer == 0)
				{
					//Se carga el timer con el tiempo que se mantendrá cargando los capacitores de bootstrap
					motor.bootstrap_charge_timer = board_scheduler_load_timer(MOTOR_CHARGE_BOOTSTRAP_TIME_mS);
					inverter_1phase_pwm_set_period_us(PWM_PERIOD_BOOTSTRAP_CARGE_uS);
					inverter_1phase_pwm_set_toff_us(PWM_TOFF_BOOTSTRAP_CHARGE_uS);
					//Para cargar los capacitores de bootstrap, fuerzo un frenado con los transistores bajos
					//del puente H. De esta forma el punto medio se conecta a GND cargando los capacitores.
					inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_LOW, INVERTER_STATE_OVERWRITE);
				}
				else if(board_scheduler_is_time_expired(motor.bootstrap_charge_timer))
				{
					//Cuando se cumple el tiempo de carga pedido, se indica cuál es el próximo estado
					//y se configuran lo necesario para el cambio.
					motor.state = MOTOR_START_FIRST_STEPS_FROM_STAND;
					//Cargo cuantos pasos se darán en la etapa inicial
					motor.start_first_steps_counter=START_FIRST_STEPS_FROM_STAND_COUNT;
					//Habilito la interrupción del sensor hall para comenzar las conmutaciones
					motor_sensor_hall_irq_enable();

					//Inicializo el timer que chequea que el arranque suceda en el tiempo esperado
					motor.start_running_timmer = board_scheduler_load_timer(TIME_TO_GET_RUNNING_TIMEOUT_mS);
					//Inicializo un temporizador para chequear que el rotor haya girado luego de un tiempo.
					motor.first_steps_deadtime_timer = board_scheduler_load_timer(FIRST_STEPS_DEADTIME_SP_mS);
					//Almaceno cuál es la posición actual del rotor, dada por el estado del sensor hall.
					motor.hall_status=motor_hall_sensor_state();
					
					//Configuro el PWM para los primeros pasos
					inverter_1phase_pwm_set_period_us(PWM_PERIOD_1st_SETPS_uS);
					inverter_1phase_pwm_set_toff_us(PWM_TOFF_1st_SETPS_uS);
					//Inicializo el temporizador utilizado para realizar las mediciones de velocidad del motor
					motor_measure_timer_set_timeout_us(MEASURE_TIMER_MAX_COUNT_uS);
					//Fuerzo una secuencia cualquiera para que el motor comience a moverse
					if(motor.hall_status==MOTOR_S_HALL_HIGH)
						inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1, INVERTER_STATE_OVERWRITE);
					else
						inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2, INVERTER_STATE_OVERWRITE);
					
					//Inicializo un temporizador que se deberá resetear constantemente. Si no se
					//resetea, se considera que algo a fallado y por lo tanto se fuerza la detención
					//del motor.
					motor_watchdog_timer_set_timeout_us(START_FIRST_STEP_DURATION_INIT_us<<2);
				}
				break;

		case MOTOR_START_FIRST_STEPS_FROM_STAND:
				if(board_scheduler_is_time_expired(motor.start_running_timmer))
				{
					motor_1phase_stop_motor(MOTOR_STOP_METHOD_FREEWHEEL,MOTOR_FAIL);
				}
				else
				{
					if(board_scheduler_is_time_expired(motor.first_steps_deadtime_timer))
					{
						//De forma periódica durante los primeros pasos chequeo que el
						//rotor no se quede estático en una posición.
						if (motor_hall_sensor_state()==motor.hall_status)
						{
							//Si detecto que se mantuvo la posición, fuerzo un cambio de secuencia.
							inverter_1phase_comm_next_seq();
						}
						//Reseteo el temporizador.
						motor.first_steps_deadtime_timer=board_scheduler_load_timer(FIRST_STEPS_DEADTIME_SP_mS);
						//board_gpio_output_toggle(BOARD_LED_port,BOARD_LED_pin);
					}
				}
				break;

		case MOTOR_RUNNING_ADVANCE_COMMUTATION:
				if(board_scheduler_is_time_expired(motor.soft_start_timmer))
				{
					if(motor.pwm_toff>motor.pwm_SP)
					{
						motor.pwm_toff--;
						inverter_1phase_pwm_set_toff_us(motor.pwm_toff);
					}
					else if(motor.pwm_toff<motor.pwm_SP)
					{
						motor.pwm_toff++;
						inverter_1phase_pwm_set_toff_us(motor.pwm_toff);
					}
					motor.soft_start_timmer=board_scheduler_load_timer(SOFT_START_UPDATE_TIME_mS);
				}
				break;

		default:	while(1);//DEV_ERROR
	}
}

/****************************************************************
 * 	Cuando el sensor hall cambia de estado bajo a estado alto,	*
 * 	se genera una interrupción que llama a esta función. Se 	*
 * 	registra el evento y se llama a la función que maneja las	*
 * 	acciones en base al evento registrado.						*
 ****************************************************************/
void hall_sensor_rising_event_callback (void)
{
	motor.hall_event = RISING_EVENT;
	hall_sensor_event();
}
/****************************************************************
 * 	Cuando el sensor hall cambia de estado alto a estado bajo,	*
 * 	se genera una interrupción que llama a esta función. Se 	*
 * 	registra el evento y se llama a la función que maneja las	*
 * 	acciones en base al evento registrado.						*
 ****************************************************************/
void hall_sensor_falling_event_callback (void)
{
	motor.hall_event = FALLING_EVENT;
	motor_meas_rotation_speed();
	hall_sensor_event();
}
/********************************************************************************************************
 * 	Cada vez que se presente una conmutación en la señal del sensor hall, se producirá una interrupción	*
 * 	que va a ralizar un llamado a las funciones previas y ellas a esta función.							*
 * 	Durante el arranque del motor, debe cambiar la secuencia que se aplica a la salida directamente.	*
 * 	En funcionamiento con avance, activa un temporizador que debe disparar una interrupción antes del	*
 * 	próximo flanco del sensor hall y esta interrupción realizará el cambio de secuencia.				*
 * ******************************************************************************************************/
void hall_sensor_event(void)
{
		//Las funciones que realizan los cambio de secuencia sólo se llaman si no se ha
		//levantado el flag que requiere la detención del motor.
		if(motor.stop_running_flag != STOP_RUNNING_FLAG_STOP_MOTOR)
		{
			if(motor.state == MOTOR_START_FIRST_STEPS_FROM_STAND)
			{
				hall_sensor_event_first_steps_action();
			}
			else if(motor.state==MOTOR_RUNNING_ADVANCE_COMMUTATION)
			{
				hall_sensor_event_advance_comm_action();
			}
		}
	motor_watchdog_timer_set_timeout_us(MAX_TIMEOUT_WATCHDOG_us);
}
/*******************************************************************************
*	Esta función es la encargada de forzar la secuencia necesaria durante
*	los primeros pasos del rotor.
*	La función es ejecutada en tiempo de interrupción. Se llama a la misma
*	cuando sucede un cambio de estado en el sensor hall.
********************************************************************************/
void hall_sensor_event_first_steps_action (void)
{
	//Durante los primeros pasos, ya comienzo a tomar la medición de la velocidad del rotor.
		//motor_meas_rotation_speed();
	//Dado que hubo un evento del sensor hall, se interpreta que el rotor se ha movido y por lo
	//tanto reseteo el temporizador que va a chequear esta situación en el MAIN de forma periódica.
	motor.first_steps_deadtime_timer=board_scheduler_load_timer(FIRST_STEPS_DEADTIME_SP_mS);

	if(motor.start_first_steps_counter>1)
	{
		//Durante la cantidad de pasos indicada en la inicialización, chequeo el
		//estado del sensor hall y en base a configuro la secuencia que deberá
		//seguir para realizar el siguiente paso.
		//Esta condición es la que marca el sentido de giro del rotor.
		motor.start_first_steps_counter--;
		if(motor_hall_sensor_state()==MOTOR_S_HALL_HIGH)
		{
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba activando ambos transistores
				//inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_HIGH,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba dejando todo abierto
				//inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL,INVERTER_STATE_OVERWRITE);
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1, INVERTER_STATE_OVERWRITE);
		}
		else
		{
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba activando ambos transistores
				//inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_HIGH,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba dejando todo abierto
				//inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL,INVERTER_STATE_OVERWRITE);
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2, INVERTER_STATE_OVERWRITE);
		}
	}
	else
	{
		//Cuando se complete la cantidad de pasos requerida para el arranque, paso al siguiente estado.
		motor.state = MOTOR_RUNNING_ADVANCE_COMMUTATION;
		//Dado que en el estado con avance cambia la secuencia forzada en el evento del hall (en relación
		//a los primeros pasos), la secuencia inicial la fuerzo a giro libre.
		inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL, INVERTER_STATE_OVERWRITE);
		//Configuro el PWM para el estado de giro estable
		motor.pwm_SP=PWM_TOFF_MIN_SPEED_uS;
		motor.pwm_toff=PWM_TOFF_RUNNING_uS;
		inverter_1phase_pwm_set_period_us(PWM_PERIOD_RUNNING_uS);
		inverter_1phase_pwm_set_toff_us(motor.pwm_toff);
		motor.soft_start_timmer=board_scheduler_load_timer(SOFT_START_UPDATE_TIME_mS);
		
		//Levanto el flag que indica que el motor está girando.
		motor.running_flag = RUNNING;
	}

	//Por cada paso registro el estado en el que se encuentra el sensor hall.
	motor.hall_status=motor_hall_sensor_state();
	
}

/****************************************************************************
*	Esta función es la encargada de configurar dentro de cuánto tiempo debe	*
*	saltar la interrupción del temporizador que forzará la secuencia 		*
*	correspondiente, de forma anticipada al próximo evento del sensor hall.	*
*****************************************************************************/
void hall_sensor_event_advance_comm_action(void)
{
	//Tomo la medición de velocidad de rotación
	//motor_meas_rotation_speed();
	
	//Calculo en cuánto tiempo se debe hacer la conmutación en base al porcenaje de avance requerido
	//motor.time_2_comm_since_hall_event_us = (motor.time_from_zcd_to_zcd_avg*DELAY_PERCENTAGE)/CONSTANT_100PERCENT;
	//Para reducir el tiempo de computo, cambio la expresion de arriba por la de abajo. El calculo esta hecho para un 30% de adelanto (da 29,7%)
	//Haciendo los shifteos y las sumas se pasa de 20uS a 5uS de tiempo de calculo.
	//ADELANTO PARA EL SENSOR DE +-2,7mT DE UMBRAL
	//motor.time_2_comm_since_hall_event_us = (motor.time_from_zcd_to_zcd_avg>>1)-(motor.time_from_zcd_to_zcd_avg>>5);
	//ADELANTO PROGRAMADO PARA EL SENSOR DE +-0,7mT DE UMBRAL
	//motor.time_2_comm_since_hall_event_us = (motor.time_from_zcd_to_zcd_avg>>1)+(motor.time_from_zcd_to_zcd_avg>>4);
	//motor.time_2_comm_since_hall_event_us = FRACTION_17_32(motor.time_from_zcd_to_zcd_avg);										//Valor Hernan 18/32
	motor.time_2_comm_since_hall_event_us = ADVANCE_FRACTION(motor.time_from_zcd_to_zcd_avg);
	//ADELANTO PROGRAMADO PARA EL SENSOR MH188 DE +-0,5mT a +-5,5mT PLACAS FABRICADAS POR FULL
	//motor.time_2_comm_since_hall_event_us = (motor.time_from_zcd_to_zcd_avg>>1)-(motor.time_from_zcd_to_zcd_avg>>3)-(motor.time_from_zcd_to_zcd_avg>>5);
	
	//Configuro la interrupción que realizará la conmutación
	advance_commutation_timer_set_within_us(motor.time_2_comm_since_hall_event_us);
	
}

/****************************************************************************************
 * Esta función se ejecuta cuando salta la interrupción del temporizador configurado	*
 * en la función previa.																*
 * Según si el evento del sensor hall fue ascendente o descendente, se realiza la 		*
 * conmutación a la secuencia que corresponde para mantener el sentido de giro.			*
 ****************************************************************************************/
void advance_commutation_timer_callback(void)
{
	advance_commutation_timer_abort();
	if (motor.hall_event == RISING_EVENT)
	{
		//Forma crota de generar un tiempo de descarga de 20uS antes de cambiar de secuencia (24 iteraciones)
			for(i=1;i<=2;i++)
			{
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1_DISCHARGE,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba activando ambos transistores
				//inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_HIGH,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba dejando todo abierto
				//inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL,INVERTER_STATE_OVERWRITE);
			}
		
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2, INVERTER_STATE_OVERWRITE);
	}
	if (motor.hall_event == FALLING_EVENT)
	{
			for(i=1;i<=2;i++)
			{
				inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ2_DISCHARGE,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba activando ambos transistores
				//inverter_1phase_comm_set_seq(INVERTER_COMM_BREAK_HIGH,INVERTER_STATE_OVERWRITE);
				//Opcion de prueba dejando todo abierto
				//inverter_1phase_comm_set_seq(INVERTER_COMM_FREEWHEEL,INVERTER_STATE_OVERWRITE);
			}
		
		inverter_1phase_comm_set_seq(INVERTER_COMM_SEQ1, INVERTER_STATE_OVERWRITE);
	}
}

/************************************************************************************
 * Inicialización para la correcta medición de la velocidad de rotación del motor.	*
 * Dado que uso promedios, en la inicialización los configuro todos en 0.			*
 ************************************************************************************/
void motor_meas_rotation_speed_init (void)
{
	//Tomo el valor actual en la primer instancia para luego calcular correctamente el período
	motor.measure_timmer_overflow = MEAS_TIM_RESET;

	//Inicializo los promedios
	motor.electrical_period_us_avg = 0;
	motor.time_from_zcd_to_zcd_avg = 0;
	motor.time_from_zcd_to_zcd = 0;

}

/**************************************************************************************
 * Función para tomar la medición de la velocidad de rotación del motor a partir de la
 * medición de ticks del temporizador creado con este fin.
 * Los resultados se almacenan en la variable global en distintas unidades:
 * 	- time from zcd to zcd: tiempo en microsegundos entre un cruce por cero y otro
 * 	- motor electrical period us: período eléctrico del motor en microsegundos (tiempo
 * 	  que demora en dar una vuelta)
 * 	- motor frequency hz: frecuencia a la que gira el motor en Hz.
 * 	- motor rpm: revoluciones por minuto a las que gira el motor.
 * También se calcula un promedio realizado con la muestra anterior y la actual
 **************************************************************************************/
void motor_meas_rotation_speed (void)
{
	//Tomo el valor actual del temporizador, dado que reinicio el mismo en cada vuelta,
	//el valor leído el exactamente el tiempo entre cruces.
	//Dado que se filtra la señal del sensor hall, ésta no tiene la misma pendiente de
	//subida que de bajada, lo cual termina generando un error si se mide todos los
	//flancos del sensor. Se programa para que sólo mida entre flancos descendentes que
	//son los de mayor pendiente y luego se divide por dos el resultado.
	motor.time_from_zcd_to_zcd=motor_measure_timer_get_count_us()>>1;
	//Reinicio el temporizador.
	motor_measure_timer_set_timeout_us(MEASURE_TIMER_MAX_COUNT_uS);

	//Realizo los cálculos correspondientes para tener la velocidad en distintas unidades
	motor.electrical_period_us = MOTOR_ROTOR_NUMBER_OF_POLES*motor.time_from_zcd_to_zcd;
	motor.frequency_hz = CONSTANT_MHZ_TO_HZ/motor.electrical_period_us;
	motor.rpm = motor.frequency_hz*CONSTANT_HZ_TO_RPM;

	//Calculo los valores promedio
	if(motor.time_from_zcd_to_zcd_avg==0)
	{
		//En la primer vuelta, cargo los valores instantáneos medidos
		motor.time_from_zcd_to_zcd_avg = motor.time_from_zcd_to_zcd;
		motor.electrical_period_us_avg = motor.electrical_period_us;
	}
	else
	{
		//Luego de la primer vuelta, promedio los resultados
		motor.time_from_zcd_to_zcd_avg = (motor.time_from_zcd_to_zcd_avg+motor.time_from_zcd_to_zcd)>>1;
		motor.electrical_period_us_avg =(motor.electrical_period_us_avg+motor.electrical_period_us)>>1;

		//TODO Hay que cambiar los filtros estos para aumentar la profundidad y tener mejor medicion
		/*
		Tomar en cuenta que se llama a esta funcion cada flanco negativo del Sensor Hall, esto hay
		que tenerlo en mente cuando se dimensione la actuacion sobre el duty para evitar oscilaciones
		sobre la velocidad.
		*/

	}
}

/*******************************************************************************
*	Solo saltara a esta funcion cuando no reciba más señal de ZCD en un tiempo
*	establecido. Ejecuta el procedimiento para frenar el motor y pone
*	el estado a FAIL
********************************************************************************/
void motor_watchdog_timer_callback(void)
{
	motor_1phase_stop_motor(MOTOR_STOP_METHOD_FREEWHEEL,MOTOR_FAIL);
}

void motor_measure_timer_callback (void)
{
	motor.measure_timmer_overflow = MEAS_TIM_OVERFLOW;

	motor_measure_timer_set_timeout_us(MEASURE_TIMER_MAX_COUNT_uS);
}

int32_t motor_hall_sensor_state (void)
{
	if(board_gpio_input_read_state(HALL_SENSOR_port,HALL_SENSOR_pin))
	{
		return MOTOR_S_HALL_HIGH;
	}
	else
	{
		return MOTOR_S_HALL_LOW;
	}
}

int32_t motor_get_actual_RPM(void)
{
	return motor.rpm;
}

bool isMotorStopped (void)
{
	return ((motor.running_flag==STOPPED)?TRUE:FALSE);
}

void set_motor_low_speed (void)
{
	motor.pwm_SP=PWM_TOFF_MIN_SPEED_uS;
}

void set_motor_high_speed (void)
{
	motor.pwm_SP=PWM_TOFF_MAX_SPEED_uS;
}
