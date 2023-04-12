#ifndef AL03_MOTOR_1PHASE_DRIVER_H_

#define AL03_MOTOR_1PHASE_DRIVER_H_

#include "AL02_inverter_1phase.h"

/****************************************************
 * ESTADOS DE LA MÁQUINA DE ESTADOS DEL MOTOR		*
 ****************************************************/
#define MOTOR_STOPPED/***************************/0
#define MOTOR_FAIL/******************************/1
#define MOTOR_START_INIT/************************/2
#define MOTOR_START_BREAK_AND_CHARGE_BOOTSTRAP/**/3
#define MOTOR_START_ALIGNMENT/*******************/4
#define MOTOR_START_FIRST_STEPS_FROM_STAND/******/5
#define MOTOR_RUNNING_ADVANCE_COMMUTATION/*******/6
#define MOTOR_STATES_COUNT/**********************/7

/* Estados del running_flag	*/
#define STOPPED		0
#define	STARTING	1
#define RUNNING		2

/*	Métodos para detener el motor		*/
#define MOTOR_STOP_METHOD_BREAK			0
#define MOTOR_STOP_METHOD_FREEWHEEL		1

/* Temperaturas de uso del motor		*/
#define MOTOR_TEMP_2_ALLOW_START	45
#define MOTOR_TEMP_2_FORCE_STOP		70

/* Corriente para detencion de emergencia	*/
#define MOTOR_CURRENT_TO_FORCE_STOP		3

/************************************************************************
 * 		Funciones públicas para el comando del motor BLDC monofase		*
 ************************************************************************/
int32_t motor_1phase_init 			(void);
void	motor_1phase_state_machine	(void);

int32_t  motor_1phase_start_motor	(void);
int32_t  motor_1phase_stop_motor	(int32_t motor_stop_method,int32_t motor_state_exit_value);
int32_t  motor_1phase_get_state		(void);

void 	motor_meas_rotation_speed_init 	(void);
void	motor_meas_rotation_speed 		(void);
int32_t motor_get_actual_RPM			(void);

void motor_1phase_set_pwm_period_us (int32_t pwm_period_uS);
void motor_1phase_set_pwm_ton_us 	(int32_t pwm_ton_uS);
void motor_1phase_set_pwm_toff_us 	(int32_t pwm_toff_uS);

int32_t motor_hall_sensor_state (void);

bool isMotorStopped (void);

void set_motor_low_speed (void);
void set_motor_high_speed (void);



/*							MACROS DE ANGULOS							*/
#define	FRACTION_1_2(time)					(time)>>1

#define	FRACTION_10_32(time)				(time>>2)+(time>>4)
#define	FRACTION_11_32(time)				(time>>1)-(time>>3)-(time>>5)
#define	FRACTION_12_32(time)				(time>>1)-(time>>3)
#define	FRACTION_13_32(time)				(time>>1)-(time>>4)-(time>>5)
#define	FRACTION_14_32(time)				(time>>1)-(time>>4)
#define	FRACTION_15_32(time)				(time>>1)-(time>>5)
#define	FRACTION_16_32(time)				(time>>1)
#define	FRACTION_17_32(time)				(time>>1)+(time>>5)
#define	FRACTION_18_32(time)				(time>>1)+(time>>4)
#define	FRACTION_19_32(time)				(time>>1)+(time>>4)+(time>>5)
#define	FRACTION_20_32(time)				(time>>1)+(time>>3)
#define	FRACTION_21_32(time)				(time>>1)+(time>>3)+(time>>5)
#define	FRACTION_22_32(time)				(time>>1)+(time>>3)+(time>>4)
#define	FRACTION_23_32(time)				(time>>1)+(time>>2)-(time>>5)
#define	FRACTION_24_32(time)				(time>>1)+(time>>2)
#define	FRACTION_25_32(time)				(time>>1)+(time>>2)+(time>>5)

#define	FRACTION_31_64(time)				(time>>1)-(time>>6)
#define	FRACTION_32_64(time)				(time>>1)
#define	FRACTION_33_64(time)				(time>>1)+(time>>6)


#endif /* AL03_MOTOR_1PHASE_DRIVER_H_ */

