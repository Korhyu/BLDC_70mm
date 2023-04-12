#ifndef AL02_INVERTER_1PHASE

#define AL02_INVERTER_1PHASE

#include "AL01_board_motor_dev_rev2019_01.h"

				/*		CONSTANTES		*/
#define INVERTER_COMM_FREEWHEEL				(1<<0)/*Todos los transistores abiertos*/

#define INVERTER_COMM_SEQ1						(1<<1)
#define INVERTER_COMM_SEQ2						(1<<2)

#define INVERTER_COMM_SEQ1_DISCHARGE	(1<<3)
#define INVERTER_COMM_SEQ2_DISCHARGE	(1<<4)

#define INVERTER_COMM_BREAK_LOW				(1<<7)/*Todos los transistores de arriba abiertos y todos los de abajo conduciendo*/
#define	INVERTER_COMM_BREAK_HIGH			(1<<8)/*Todos los transistores de arriba conduciendo y todos los de abajo abiertos*/
#define INVERTER_COMM_COUNT						(1<<9)

#define INVERTER_STATE_OVERWRITE		0
#define INVERTER_STATE_NOT_OVERWRITE	1

#define	INVERTER_PWM_PERIOD_uS	150
#define	INVERTER_PWM_TOFF_uS		120

				/*		MACROS		*/
#define inverter_1phase_pwm_set_period_us(pwm_period_us)							board_pwm_set_period_us(pwm_period_us)
#define inverter_1phase_pwm_get_period_us() 							  					board_pwm_get_period_us()
#define inverter_1phase_pwm_set_ton_us(pwm_ton_us)										board_pwm_set_ton_us(pwm_ton_us)
#define inverter_1phase_pwm_get_ton_us()								  						board_pwm_get_ton_us()
#define inverter_1phase_pwm_set_toff_us(pwm_toff_us)									board_pwm_set_toff_us(pwm_toff_us)
#define inverter_1phase_pwm_get_toff_us()								  						board_pwm_get_toff_us()
#define	inverter_1phase_pwm_start_with_ton()													board_pwm_start_with_ton()
#define	inverter_1phase_pwm_set_outputs_to_toff()											board_pwm_set_outputs_to_toff()

#define	inverter_1phase_hin1_enable()																	board_hin1_enable()
#define	inverter_1phase_hin1_disable()																board_hin1_disable()
#define	inverter_1phase_hin2_enable()																	board_hin2_enable()
#define	inverter_1phase_hin2_disable()																board_hin2_disable()
//Para el caso del driver DGD2103 los LIN tienen lógica inversa
#define	inverter_1phase_lin1_enable()																	board_lin1_enable()
#define	inverter_1phase_lin1_disable()																board_lin1_disable()
#define	inverter_1phase_lin2_enable()																	board_lin2_enable()
#define	inverter_1phase_lin2_disable()																board_lin2_disable()

				/*		FUNCIONES		*/
int32_t inverter_1phase_comm_set_seq		(int32_t inverter_comm_seq,int32_t inverter_state_overwrite);
int32_t	inverter_1phase_get_actual_comm_seq (void);
int32_t inverter_1phase_comm_next_seq 		(void);
int32_t inverter_1phase_init_config			(void);

#endif /* AL02_INVERTER_1PHASE */



