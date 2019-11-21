#ifndef __ITB_H
#define __ITB_H

#include "gpio.h"
#include "crc16.h"
#include "math.h"
#include <stdlib.h>

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

typedef unsigned short uint16_t;

// настройки прибора
#define DEV_ID (0x01)
#define CHAN_NUM (8)

#define DEAFAULT_DEAD_TIME_MS 100
#define MIN_DEAD_TIME_MS 10
#define MAX_DEAD_TIME_MS 1000

#define DEAFAULT_MEAS_TIME_MS 1000
#define MIN_MEAS_TIME_MS 500
#define MAX_MEAS_TIME_MS 10000

#define TRM1075_ADDR0 0x48
#define TRM1075_ADDR1 0x49
#define TRM1075_ADDR2 0x4A
#define TRM1075_ADDR3 0x4B
// свойства микроконтроллера
#define ADC_MAX (4095)
// раскрашивание переменных
#define MEAS_ZERO 1
#define MEAS_SIGNAL 0

typedef struct
{
	GPIO_TypeDef* bank;
	uint16_t position;
} type_GPIO_setting;

typedef struct
{
	int16_t	val, max, min;
	int32_t summ;
	int16_t num;
}type_ADC_VALUE;

typedef struct
{
	type_GPIO_setting gpio_ku[2];
	type_GPIO_setting gpio_zero;
	uint8_t ku_state, zero_state; //переменные в которых хранятся состояния GPIO !ВАЖНО, при изменении состояни переменных необходимо синхронизовать состояния GPIO
	uint8_t adc_num;
	uint8_t spi_term_addr;	
	uint8_t ku_meas;  //значение ku при котором было получено значение тока
	type_ADC_VALUE signal, zero, meas; //meas = signal - zero
	int16_t temp;
}type_CHANNEL_DATA;

typedef struct
{
	uint8_t mode; //в порядке приоритета: 1 - циклический режим, 2 - одиночный запуск, 4 - чтение осциллограммы
	uint8_t debug; // 1 - постоянный КУ
	uint32_t dead_time_ms;
	uint32_t measure_full_duration_ms;
	uint32_t measure_time_ms;
	uint8_t ku[CHAN_NUM];
	uint32_t zero_dead_time_ref_point, zero_cycle_ref_point;
	uint32_t signal_dead_time_ref_point, signal_cycle_ref_point;
} type_MEASURE_CONTROL;

typedef struct
{
	uint16_t temp;
} type_TRM;

typedef struct
{
	type_CHANNEL_DATA channel[CHAN_NUM];
	uint16_t adc_data[32];
	uint32_t global_time_s;
	type_MEASURE_CONTROL control;
} type_ITB_DEVICE;

//*** ITB ***//
void itb_device_init(type_ITB_DEVICE* itb_ptr);
//*** GPIO ***//
void _set_channel_ku_gpio_settings(type_CHANNEL_DATA* chan_ptr, uint8_t channel, GPIO_TypeDef* GPIOx_ku0, uint16_t position_ku0, GPIO_TypeDef* GPIOx_ku1, uint16_t position_ku1, GPIO_TypeDef* GPIOx_zero, uint16_t position_zero);
void _channel_init(type_CHANNEL_DATA* chan_ptr);
void set_ch_gpio_settings(type_CHANNEL_DATA* ch_ptr, uint8_t ch, uint8_t ku, uint8_t zero_state);
//*** MEASURE ***//
void Current_Calc_Step_10ms(type_ITB_DEVICE* itb_ptr);
void _ku_change_checker(type_ITB_DEVICE* itb_ptr);
void _cycle_measure_struct_init(type_ITB_DEVICE* itb_ptr);
void _calc_measure_value(type_ITB_DEVICE* itb_ptr);
void set_measure_parameters(type_ITB_DEVICE* itb_ptr, uint32_t meas_full_time, uint32_t dead_time_ms);
//*** ADC **//
void _adc_setup(type_ITB_DEVICE* itb_ptr);
void _adc_value_restart(type_ADC_VALUE* adc);
void _adc_data_get(type_ITB_DEVICE *signal_ptr);
//*** протокол для передачи через VCP ***//
uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com);
void get_current_measure_data(type_ITB_DEVICE* itb_ptr, uint8_t* data, uint8_t* len);
void get_current_measure_parameters(type_ITB_DEVICE* itb_ptr, uint8_t* data, uint8_t* len);
//*** функции общего назначения
uint32_t get_uint32_val_from_bound(uint32_t val, uint32_t min, uint32_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница
#endif
