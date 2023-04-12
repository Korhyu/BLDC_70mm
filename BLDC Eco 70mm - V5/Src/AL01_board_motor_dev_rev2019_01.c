#include "AL01_board_motor_dev_rev2019_01.h"


/******************************************************************************************/
/*				Funci�n principal para la configuraci�n de los perif�ricos de la placa					*/
/******************************************************************************************/
void board_hardware_configuration (void)
{
	//Configuraciones de GPIO
	
	//GPIOAx como Salida Push Pull
	board_hardware_gpio_config_output_pp_pins_load_config  	(GPIOA,HIN1_pin_mask);
	//GPIOAx Como "Open Drain" / HiZ
	board_hardware_gpio_config_hiz_pins_load_config		   		(GPIOA,0);
	//GPIOAx Como deteccion(irq) en flancos
	board_hardware_gpio_config_edge_events_pins_load_config	(GPIOA,0);
	//GPIOAx Como entradas logicas
	board_hardware_gpio_config_inputs_pins_load_config	   	(GPIOA,0);

//Idem para GPIOBx
	board_hardware_gpio_config_output_pp_pins_load_config		(GPIOB,BOARD_LED_pin_mask|HIN2_pin_mask);
	board_hardware_gpio_config_hiz_pins_load_config					(GPIOB,0);
	board_hardware_gpio_config_edge_events_pins_load_config	(GPIOB,HALL_SENSOR_pin_mask|OVER_CURRENT_pin_mask);
	board_hardware_gpio_config_inputs_pins_load_config			(GPIOB,0);
	
	micro_config();		
}



//-------------------------------------------
//				SCHEDULER/TEMPORIZADOR
//-------------------------------------------
//Calcula el tiempo futuro en el que debe ejecutarse una tarea 
//(calcula a partir de que Numero de tick debe ejecutar la tarea)
int32_t board_scheduler_load_timer			(int32_t time_ms)
{
		int32_t actual_tick;
		actual_tick=__hardware_get_tick_timer();
		actual_tick+=time_ms;
		return actual_tick;
}


//Revisa si debe ejecutarse o no una tarea, en funcion del Numero de tick actual 
//y el numero de ticks pasado por parametro
int32_t board_scheduler_is_time_expired	(int32_t timer)
{
		int32_t actual_tick;
		actual_tick=__hardware_get_tick_timer();
		actual_tick-=timer;
	
		if(actual_tick>=0)
			return 1;
		else
			return 0;
}

//-------------------------------------------
//				      	ADC 
//-------------------------------------------
int32_t board_adc_get_measure (uint32_t board_adc_channel_select, uint32_t * measure_result)
{
	switch(board_adc_channel_select)
	{
		case BOARD_ADC_CHANNEL_NTC_POWER_SEL: 	 *measure_result = ADC_GetConversionData(HT_ADC, BOARD_ADC_CHANNEL_NTC_POWER_SEL);
												  break;
		case BOARD_ADC_CHANNEL_SHUNT_SEL: 		 *measure_result = ADC_GetConversionData(HT_ADC, BOARD_ADC_CHANNEL_SHUNT_SEL);
			 									  break;
		case BOARD_ADC_CHANNEL_VBUS_SEL: 		 *measure_result = ADC_GetConversionData(HT_ADC, BOARD_ADC_CHANNEL_VBUS_SEL);
												  break;
		default : return -1;
	}
	return 0;
}

int32_t board_get_mosfets_temperature_C(void)
{
	uint32_t val;
	static int32_t adc_avg=0;
	int32_t temp;
	int32_t voltage;

	board_adc_get_measure(BOARD_ADC_CHANNEL_NTC_POWER_SEL,&val);

	if(adc_avg==0)
		adc_avg = val;
	else
		adc_avg = (val+adc_avg)>>1;

	//Convierto el valor del ADC en tension del divisor
	// 3.3V - 4096
	voltage = adc_avg ;

	/*
	if(adc_avg<=3700)
		return ((37683-(adc_avg*9))>>8);
	else
		return ((63808-(adc_avg*17))>>7);
	*/

	//Aproximo mediante 3 rectas de 150~100 de 100~20 y de 20~0
	/*
	Los valores de las constantes se estimaron buscar el menor error
	entre la curva real y las aproximaciones.
	Mirar documento excel "Temperatura.xls" en el mismo directorio que el programa
	*/
	if (adc_avg<=1100)
	{
		//Recta de 150 a 100
		temp = (((2482-voltage)*(93))*13)>>14;
	}
	else if (voltage<=3520)
	{
		//Recta de 100 a 20
		temp = (((4244-voltage)*(41))*13)>>14;
	}
	else if (adc_avg<=3830)
	{
		//Recta de 20 a 0
		temp = (((3847-voltage)*(83))*13)>>14;
	}
	else
	{
		//Resto de las temperaturas
		temp = 0;
	}
	return temp;
}

int32_t board_get_bus_voltage(void)
{
	uint32_t val;
	
	board_adc_get_measure(BOARD_ADC_CHANNEL_VBUS_SEL,&val);

	return ((val*401)>>12);
}

int32_t	board_get_shunt_current(void)
{
	uint32_t val;
	
	board_adc_get_measure(BOARD_ADC_CHANNEL_SHUNT_SEL,&val);

	return ((val*3300)>>12);
}

