#include "itb.h"

//*** ITB ***//
void itb_device_init(type_ITB_DEVICE* itb_ptr)
{
	_channel_init(itb_ptr->channel);
	_adc_setup(itb_ptr);
	memset(itb_ptr->adc_data, 0x00, 64);
	itb_ptr->control.mode = 0x00;
	// dead_time
	itb_ptr->control.dead_time_ms = DEAFAULT_DEAD_TIME_MS;
	// measure time
	itb_ptr->control.measure_full_duration_ms = DEAFAULT_MEAS_TIME_MS;
	//
	itb_ptr->global_time_s = 0;
	
}

// *** термодатчик TMR1075MR  ***//
void init_trm(type_TRM* trm_ptr)
{
	
}

void read_trm(type_TRM* trm_ptr)
{
	
}


///*** GPIO ***//
void _set_channel_ku_gpio_settings(type_CHANNEL_DATA* chan_ptr, uint8_t channel, GPIO_TypeDef* GPIOx_ku0, uint16_t position_ku0, GPIO_TypeDef* GPIOx_ku1, uint16_t position_ku1, GPIO_TypeDef* GPIOx_zero, uint16_t position_zero)
{
	chan_ptr[channel].gpio_ku[0].bank = GPIOx_ku0; chan_ptr[channel].gpio_ku[0].position = 1 << position_ku0;
	chan_ptr[channel].gpio_ku[1].bank = GPIOx_ku1; chan_ptr[channel].gpio_ku[1].position = 1 << position_ku1;
	chan_ptr[channel].gpio_zero.bank = GPIOx_zero; chan_ptr[channel].gpio_zero.position = 1 << position_zero;
}

void _channel_init(type_CHANNEL_DATA* chan_ptr)
{
	//настройка пинов для управления каналами
	_set_channel_ku_gpio_settings(chan_ptr, 0, GPIOD, 8, GPIOD, 9, GPIOE, 4);
	_set_channel_ku_gpio_settings(chan_ptr, 1, GPIOD, 10, GPIOD, 11, GPIOE, 5);
	_set_channel_ku_gpio_settings(chan_ptr, 2, GPIOD, 12, GPIOD, 13, GPIOE, 6);
	_set_channel_ku_gpio_settings(chan_ptr, 3, GPIOD, 14, GPIOD, 15, GPIOE, 7);
	_set_channel_ku_gpio_settings(chan_ptr, 4, GPIOE, 8, GPIOE, 9, GPIOB, 12);
	_set_channel_ku_gpio_settings(chan_ptr, 5, GPIOE, 10, GPIOE, 11, GPIOB, 13);
	_set_channel_ku_gpio_settings(chan_ptr, 6, GPIOE, 12, GPIOE, 13, GPIOB, 14);
	_set_channel_ku_gpio_settings(chan_ptr, 7, GPIOE, 14, GPIOE, 15, GPIOB, 15);
}

void set_ch_gpio_settings(type_CHANNEL_DATA* ch_ptr, uint8_t ch, uint8_t ku, uint8_t zero_state)
{
	
	HAL_GPIO_WritePin(ch_ptr[ch].gpio_ku[0].bank, ch_ptr[ch].gpio_ku[0].position, (GPIO_PinState)((ku >> 0) & 0x01));
	HAL_GPIO_WritePin(ch_ptr[ch].gpio_ku[1].bank, ch_ptr[ch].gpio_ku[1].position, (GPIO_PinState)((ku >> 1) & 0x01));
	HAL_GPIO_WritePin(ch_ptr[ch].gpio_zero.bank, ch_ptr[ch].gpio_zero.position, (GPIO_PinState)(zero_state & 0x01));
}

void synch_all_chs_gpio_settings(type_ITB_DEVICE* itb_ptr, uint8_t zero_state)
{
	if (itb_ptr->control.mode & 0x80){ //debug mode
		//
	}
	else{
		for(int i=0; i<CHAN_NUM; i++){
			itb_ptr->channel[i].zero_state = zero_state;
			set_ch_gpio_settings(itb_ptr->channel, i, itb_ptr->channel[i].ku_state, zero_state);
		}
	}
}

//*** MEASURE ***//

/**
  * @brief  функция для подсчета тока в различных режимах: одиночном или цикличном
  *		@note необходимо запускать один раз в 10 мс
  * @param  *itb_ptr: указатель на структуру управления ИТБ
  */
void Current_Calc_Step_10ms(type_ITB_DEVICE* itb_ptr)
{
	if (itb_ptr->control.mode & 0x01){ // циклический режимах
		if (itb_ptr->control.measure_time_ms == 0){
			_ku_change_checker(itb_ptr);
			_cycle_measure_struct_init(itb_ptr);			
			// что бы выйти из инициализации
			itb_ptr->control.measure_time_ms = 1;
		}
		else{
			
			itb_ptr->control.measure_time_ms += 10; //не гарантировано точные часы подсчета времени цикла
			// измерение нуля
			if ((itb_ptr->control.measure_time_ms > 0) && (itb_ptr->control.measure_time_ms < itb_ptr->control.zero_cycle_ref_point)){ //определяем мертвое время для измерения нуля
				synch_all_chs_gpio_settings(itb_ptr, MEAS_ZERO);
				if ((itb_ptr->control.measure_time_ms >=  itb_ptr->control.zero_dead_time_ref_point)){ //измерение нуля
					_adc_data_get(itb_ptr);
				}
			}
			// измерение сигнала
			else if ((itb_ptr->control.measure_time_ms >= itb_ptr->control.zero_cycle_ref_point) && (itb_ptr->control.measure_time_ms < itb_ptr->control.signal_cycle_ref_point)){ //определяем мертвое время для измерения нуля
				synch_all_chs_gpio_settings(itb_ptr, MEAS_SIGNAL);
				if ((itb_ptr->control.measure_time_ms >=  itb_ptr->control.signal_dead_time_ref_point)){ //измерение нуля
					_adc_data_get(itb_ptr);
				}
			}
			
			// финишируем
			else if (itb_ptr->control.measure_time_ms >= itb_ptr->control.signal_cycle_ref_point){
				itb_ptr->control.measure_time_ms = 0;
				_calc_measure_value(itb_ptr);
			}
		}
	}
}

/**
  * @brief  функция для определения необходимости переключения КУ для измерения тока
  * @param  *itb_ptr: указатель на структуру управления ИТБ
  */
void _ku_change_checker(type_ITB_DEVICE* itb_ptr)
{
	type_CHANNEL_DATA* ch;
	ch = itb_ptr->channel;
    /* определение корректного ku_state */
	for(int i=0; i<CHAN_NUM; i++){
		switch (ch[i].ku_state)
		{
			case 3:
				if ((abs(ch[i].signal.max) > 0.95*ADC_MAX) || (abs(ch[i].zero.max) > 0.95*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].meas.val) > 0.8*ADC_MAX)) ch[i].ku_state = 2;
				break;
			case 2:
				if ((abs(ch[i].signal.max) > 0.95*ADC_MAX) || (abs(ch[i].zero.max) > 0.95*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].meas.val) > 0.8*ADC_MAX)) ch[i].ku_state = 1;
				else if(abs(ch[i].meas.val) < 0.03*ADC_MAX) ch[i].ku_state = 3;
				break;
			case 1:
				if ((abs(ch[i].signal.max) > 0.95*ADC_MAX) || (abs(ch[i].zero.max) > 0.95*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].signal.min) < 0.05*ADC_MAX) || (abs(ch[i].meas.val) > 0.8*ADC_MAX)) ch[i].ku_state = 0;
				else if(abs(ch[i].meas.val) < 0.03*ADC_MAX) ch[i].ku_state = 2;
				break;
			case 0:
				if(abs(ch[i].meas.val) < 0.03*ADC_MAX) ch[i].ku_state = 1;
				break;
			default:
				ch[i].ku_state = 0;
				break;
		}
	}
}

void _cycle_measure_struct_init(type_ITB_DEVICE* itb_ptr)
{
	// приведение временных характиристик
	itb_ptr->control.measure_time_ms = 0;
	itb_ptr->control.zero_dead_time_ref_point = itb_ptr->control.dead_time_ms;
	itb_ptr->control.zero_cycle_ref_point = (itb_ptr->control.measure_full_duration_ms/2) - 5;
	itb_ptr->control.signal_dead_time_ref_point = itb_ptr->control.dead_time_ms + (itb_ptr->control.measure_full_duration_ms/2) - 10;
	itb_ptr->control.signal_cycle_ref_point = itb_ptr->control.measure_full_duration_ms - 10; //оставляем запас по вермени в 20мс для того, что бы успеть обработать данные
	// обнуление переменных подсчета данных типа type_ADC_VALUE
	type_CHANNEL_DATA* ch;
	ch = itb_ptr->channel;
	for (int i=0; i<CHAN_NUM; i++){
		_adc_value_restart(&ch[i].signal);
		_adc_value_restart(&ch[i].zero);
		_adc_value_restart(&ch[i].meas);
	}
}

void _calc_measure_value(type_ITB_DEVICE* itb_ptr)
{
	type_ADC_VALUE* meas;
	type_ADC_VALUE* zero;
	type_ADC_VALUE* signal;
	for(int i=0; i<CHAN_NUM; i++){
		itb_ptr->channel[i].ku_meas = itb_ptr->channel[i].ku_state;
		meas = &itb_ptr->channel[i].meas;
		zero = &itb_ptr->channel[i].zero;
		signal = &itb_ptr->channel[i].signal;
		//
		zero->val = zero->summ / zero->num;
		signal->val = signal->summ / signal->num;
		meas->val = signal->val - zero->val;
	}
}

void set_measure_parameters(type_ITB_DEVICE* itb_ptr, uint32_t meas_full_time, uint32_t dead_time_ms)
{
	itb_ptr->control.measure_full_duration_ms = get_uint32_val_from_bound(meas_full_time, MIN_MEAS_TIME_MS, MAX_MEAS_TIME_MS);
	if (dead_time_ms >= itb_ptr->control.measure_full_duration_ms){
		itb_ptr->control.dead_time_ms = DEAFAULT_DEAD_TIME_MS;
	}
	else{
		itb_ptr->control.dead_time_ms = get_uint32_val_from_bound(dead_time_ms, MIN_DEAD_TIME_MS, MAX_DEAD_TIME_MS);
	}
}
//*** ADC **//
/**
  * @brief  функция сопоставления номера канала и номера входа АЦП
  * @param  *itb_ptr: указатель на структуру управления ИТБ
  */
void _adc_setup(type_ITB_DEVICE* itb_ptr)
{
	itb_ptr->channel[0].adc_num = 0;
	itb_ptr->channel[1].adc_num = 1;
	itb_ptr->channel[2].adc_num = 2;
	itb_ptr->channel[3].adc_num = 3;
	itb_ptr->channel[4].adc_num = 4;
	itb_ptr->channel[5].adc_num = 5;
	itb_ptr->channel[6].adc_num = 6;
	itb_ptr->channel[7].adc_num = 7;
}

void _adc_value_restart(type_ADC_VALUE* adc)
{
	adc->max = (int16_t)(1 << 15); //ставим максимум в минимальное  значение
	adc->min = 0x7FFF;  //ставим минимум в максимальное  значение
	adc->summ = 0;  
	adc->num = 0;  
}

void _adc_data_get(type_ITB_DEVICE *itb_ptr)
{
	type_ADC_VALUE* adc;
	uint16_t val;
	for(int i=0; i<CHAN_NUM; i++){
		if (itb_ptr->channel[i].zero_state == MEAS_ZERO){
			adc = &itb_ptr->channel[i].zero;
		}
		else if (itb_ptr->channel[i].zero_state == MEAS_SIGNAL){
			adc = &itb_ptr->channel[i].signal;
		}
		val = itb_ptr->adc_data[itb_ptr->channel[i].adc_num];
		adc->summ += val;
		adc->num += 1;
		adc->max = max(adc->max, val);
		adc->min = min(adc->min, val);
	}
}

//*** протокол для передачи через VCP ***//
uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com)
{	
	uint16_t crc=0;
	uint8_t i=0;
	ans_com[0] = req_id & 0xFF;
	ans_com[1] = self_id & 0xFF;
	ans_com[2] = *seq_num & 0xFF;
	ans_com[3] = 0x00 & 0xFF;
	ans_com[4] = type & 0xFF;
	ans_com[5] = leng & 0xFF;
	for(i=0; i < leng; i++)
	{
		ans_com[i+6] = com_data[i];
	}
	crc = crc16_ccitt(ans_com, leng+6); // like modbus
	ans_com[leng+6] = (uint8_t)((crc>>8) & 0xFF);
	ans_com[leng+7] = (uint8_t)((crc>>0) & 0xFF);
	*seq_num+=1;
	return leng+8;
}

void get_current_measure_data(type_ITB_DEVICE* itb_ptr, uint8_t* data, uint8_t* len) // todo: пока только для 4-х каналов
{
	for(int i=0; i<4; i++){
		data[0+8*i] = itb_ptr->channel[i].ku_meas;
		data[1+8*i] = itb_ptr->channel[i].temp >> 8;
		data[2+8*i] = itb_ptr->channel[i].meas.val >> 8;
		data[3+8*i] = itb_ptr->channel[i].meas.val;
		data[4+8*i] = itb_ptr->channel[i].signal.val >> 8;
		data[5+8*i] = itb_ptr->channel[i].signal.val;
		data[6+8*i] = itb_ptr->channel[i].zero.val >> 8;
		data[7+8*i] = itb_ptr->channel[i].zero.val;
	}
	*len = 4*8;
}

void get_current_measure_parameters(type_ITB_DEVICE* itb_ptr, uint8_t* data, uint8_t* len)
{
	*((uint32_t*)&data[0]) = __REV(itb_ptr->control.measure_full_duration_ms);
	*((uint32_t*)&data[4]) = __REV(itb_ptr->control.dead_time_ms);
	*len = 2*4;
}

//*** функции общего назначения
uint32_t get_uint32_val_from_bound(uint32_t val, uint32_t min, uint32_t max) //если число внутри границ - используется оно, если нет, то ближайшая граница
{
	if (val > max) return max;
	else if (val < min) return min;
	return val;
}
