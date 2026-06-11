/**
* Create by Miguel Ángel López on 20/07/19
* and modify by xaxexa
* Refactoring & component making:
* Соловей с паяльником 15.03.2024
**/
#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"

namespace esphome{
namespace tclac{


ClimateTraits tclacClimate::traits() {
	auto traits = climate::ClimateTraits();
	traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
	
	// Ответственно заявляю, что это все я взял у christoph5180
	if (this->supported_modes_.empty()) {
		traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
		traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);
	} else {
		for (auto mode : this->supported_modes_)
			traits.add_supported_mode(mode);
	}
	if (this->supported_presets_.empty()) {
		traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);
	} else {
		for (auto preset : this->supported_presets_)
			traits.add_supported_preset(preset);
	}
	if (this->supported_fan_modes_.empty()) {
		traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
	} else {
		for (auto fan_mode : this->supported_fan_modes_)
			traits.add_supported_fan_mode(fan_mode);
	}
	if (this->supported_swing_modes_.empty()) {
		traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
	} else {
		for (auto swing_mode : this->supported_swing_modes_)
			traits.add_supported_swing_mode(swing_mode);
	}

	return traits;
}


void tclacClimate::setup() {

#ifdef CONF_RX_LED
	this->rx_led_pin_->setup();
	this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
	this->tx_led_pin_->setup();
	this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop()  {
	while (esphome::uart::UARTDevice::available() > 0) {
		uint8_t byte = esphome::uart::UARTDevice::read();
		// ESP_LOGD("TCL", "UART RX byte %02X pos=%zu exp=%zu", byte, this->rx_buffer_pos_, this->rx_expected_size_);

		if (this->rx_buffer_pos_ == 0) {
			if (byte != 0xBB) {
				ESP_LOGD("TCL", "Wrong packet header byte=%02X", byte);
				continue;
			}
			this->dataRX[this->rx_buffer_pos_++] = byte;
			continue;
		}

		this->dataRX[this->rx_buffer_pos_++] = byte;

		if (this->rx_buffer_pos_ == 5) {
			this->rx_expected_size_ = (size_t)this->dataRX[4] + 6;
			ESP_LOGD("TCL", "Header parsed length=%02X expected_size=%zu", this->dataRX[4], this->rx_expected_size_);
			if (this->rx_expected_size_ > sizeof(this->dataRX) || this->rx_expected_size_ < 6) {
				ESP_LOGD("TCL", "Invalid packet size %zu", this->rx_expected_size_);
				this->rx_buffer_pos_ = 0;
				this->rx_expected_size_ = 0;
				continue;
			}
		}

		if (this->rx_expected_size_ > 0 && this->rx_buffer_pos_ == this->rx_expected_size_) {
			ESP_LOGD("TCL_RAW", "B18=%02X B29=%02X B30=%02X B34=%02X B35=%02X B36=%02X B37=%02X B38=%02X B39=%02X B40=%02X B44=%02X",
				dataRX[18], dataRX[29], dataRX[30],
				dataRX[34], dataRX[35], dataRX[36],
				dataRX[37], dataRX[38], dataRX[39],
				dataRX[40], dataRX[44]);

			size_t packet_size = this->rx_expected_size_;
			uint8_t check = getChecksum(dataRX, packet_size);
			if (check != dataRX[packet_size - 1]) {
				ESP_LOGD("TCL", "Invalid checksum %02X expected %02X", check, dataRX[packet_size - 1]);
				this->dataShow(0,0);
				this->rx_buffer_pos_ = 0;
				this->rx_expected_size_ = 0;
				continue;
			}

			ESP_LOGD("TCL", "Packet complete size=%zu checksum OK", packet_size);
			this->dataShow(0,0);
			this->readData();
			this->rx_buffer_pos_ = 0;
			this->rx_expected_size_ = 0;
		}

		if (this->rx_buffer_pos_ >= sizeof(this->dataRX)) {
			ESP_LOGD("TCL", "RX overflow reset pos=%zu", this->rx_buffer_pos_);
			this->rx_buffer_pos_ = 0;
			this->rx_expected_size_ = 0;
		}
	}
}

void tclacClimate::update() {
	tclacClimate::dataShow(1,1);
	this->esphome::uart::UARTDevice::write_array(poll, sizeof(poll));
	//auto raw = tclacClimate::getHex(poll, sizeof(poll));
	ESP_LOGD("TCL", "chek status sended");
	tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {
	
	current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
	target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

	//ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

	if (dataRX[MODE_POS] & ( 1 << 4)) {
		// Если кондиционер включен, то разбираем данные для отображения
		ESP_LOGD("TCL", "AC is on");
		uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
		uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
		uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

		switch (modeswitch) {
			case MODE_AUTO:
				this->mode = climate::CLIMATE_MODE_AUTO;
				break;
			case MODE_COOL:
				this->mode = climate::CLIMATE_MODE_COOL;
				break;
			case MODE_DRY:
				this->mode = climate::CLIMATE_MODE_DRY;
				break;
			case MODE_FAN_ONLY:
				this->mode = climate::CLIMATE_MODE_FAN_ONLY;
				break;
			case MODE_HEAT:
				this->mode = climate::CLIMATE_MODE_HEAT;
				break;
			default:
				this->mode = climate::CLIMATE_MODE_AUTO;
		}

		if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
			fan_mode = climate::CLIMATE_FAN_QUIET;
		} else if (dataRX[MODE_POS] & FAN_DIFFUSE){
			fan_mode = climate::CLIMATE_FAN_DIFFUSE;
		} else {
			switch (fanspeedswitch) {
				case FAN_AUTO:
					fan_mode = climate::CLIMATE_FAN_AUTO;
					break;
				case FAN_LOW:
					fan_mode = climate::CLIMATE_FAN_LOW;
					break;
				case FAN_MIDDLE:
					fan_mode = climate::CLIMATE_FAN_MIDDLE;
					break;
				case FAN_MEDIUM:
					fan_mode = climate::CLIMATE_FAN_MEDIUM;
					break;
				case FAN_HIGH:
					fan_mode = climate::CLIMATE_FAN_HIGH;
					break;
				case FAN_FOCUS:
					fan_mode = climate::CLIMATE_FAN_FOCUS;
					break;
				default:
					fan_mode = climate::CLIMATE_FAN_AUTO;
			}
		}

		switch (swingmodeswitch) {
			case SWING_OFF: 
				swing_mode = climate::CLIMATE_SWING_OFF;
				break;
			case SWING_HORIZONTAL:
				swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
				break;
			case SWING_VERTICAL:
				swing_mode = climate::CLIMATE_SWING_VERTICAL;
				break;
			case SWING_BOTH:
				swing_mode = climate::CLIMATE_SWING_BOTH;
				break;
		}
		
		// Обработка данных о пресете
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
		if (dataRX[7] & (1 << 6)){
			preset = ClimatePreset::CLIMATE_PRESET_ECO;
		} else if (dataRX[9] & (1 << 2)){
			preset = ClimatePreset::CLIMATE_PRESET_COMFORT;
		} else if (dataRX[19] & (1 << 0)){
			preset = ClimatePreset::CLIMATE_PRESET_SLEEP;
		}
		
	} else {
		ESP_LOGD("TCL", "AC is OFF");
		// Если кондиционер выключен, то все режимы показываются, как выключенные
		this->mode = climate::CLIMATE_MODE_OFF;
		//fan_mode = climate::CLIMATE_FAN_OFF;
		this->swing_mode = climate::CLIMATE_SWING_OFF;
		this->preset = ClimatePreset::CLIMATE_PRESET_NONE;
	}
	// Публикуем данные
	this->publish_state();
	// เพิ่มตรงนี้ (improved power estimation)
	ESP_LOGI("TCL_POWER",
		"B38=%d B39=%d target=%.1f current=%.1f",
		dataRX[38],
		dataRX[39],
		target_temperature,
		current_temperature
	);

	float hz = dataRX[38];
	float estimated_watt = 0.0f;

	// AC OFF = 0W
	if (this->mode == climate::CLIMATE_MODE_OFF || hz == 0.0f) {
		estimated_watt = 0.0f;
	} else {
		// clamp frequency
		if (hz < 15.0f) hz = 15.0f;
		if (hz > 100.0f) hz = 100.0f;

		// inverter power curve
		estimated_watt =
			120.0f + ((hz - 15.0f) * (1350.0f - 120.0f) / (100.0f - 15.0f));
	}

	ESP_LOGI("TCL_POWER",
		"B38=%d B39=%d mode=%d estimated=%.0fW",
		dataRX[38],
		dataRX[39],
		(int)this->mode,
		estimated_watt
	);

	if (this->power_sensor_ != nullptr) {
		this->power_sensor_->publish_state(estimated_watt);
	}
	allow_take_control = true;
}

// Climate control
void tclacClimate::control(const climate::ClimateCall &call) {
	
	ESP_LOGD("TCL", "Call from UI");
	
	// А это и ниже я подрезал у Vi3jo.
	
	if (call.get_mode().has_value()) this->mode = *call.get_mode();
    if (call.get_target_temperature().has_value()) this->target_temperature = *call.get_target_temperature();
    if (call.get_fan_mode().has_value()) this->fan_mode = *call.get_fan_mode();
	if (call.get_swing_mode().has_value()) this->swing_mode = *call.get_swing_mode();
	if (call.get_preset().has_value()) this->preset = *call.get_preset();
	
	this->publish_state();
	this->takeControl();
	this->allow_take_control = true;
}
	
	
void tclacClimate::takeControl() {
	
	dataTX[7]  = 0b00000000;
	dataTX[8]  = 0b00000000;
	dataTX[9]  = 0b00000000;
	dataTX[10] = 0b00000000;
	dataTX[11] = 0b00000000;
	dataTX[19] = 0b00000000;
	dataTX[32] = 0b00000000;
	dataTX[33] = 0b00000000;
	
	uint8_t target_temperature_set = 31-(int)target_temperature;
	
	// Включаем или отключаем пищалку в зависимости от переключателя в настройках
	if (beeper_status_){
		ESP_LOGD("TCL", "Beep mode ON");
		dataTX[7] += 0b00100000;
	} else {
		ESP_LOGD("TCL", "Beep mode OFF");
		dataTX[7] += 0b00000000;
	}
	
	// Включаем или отключаем дисплей на кондиционере в зависимости от переключателя в настройках
	// Включаем дисплей только если кондиционер в одном из рабочих режимов
	
	// ВНИМАНИЕ! При выключении дисплея кондиционер сам принудительно переходит в автоматический режим!
	
	if ((display_status_) && (mode != climate::CLIMATE_MODE_OFF)){
		ESP_LOGD("TCL", "Dispaly turn ON");
		dataTX[7] += 0b01000000;
	} else {
		ESP_LOGD("TCL", "Dispaly turn OFF");
		dataTX[7] += 0b00000000;
	}
		
	// Настраиваем режим работы кондиционера
	switch (this->mode) {
		case climate::CLIMATE_MODE_OFF:
			dataTX[7] += 0b00000000;
			dataTX[8] += 0b00000000;
			break;
		case climate::CLIMATE_MODE_AUTO:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00001000;
			break;
		case climate::CLIMATE_MODE_COOL:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000011;	
			break;
		case climate::CLIMATE_MODE_DRY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000010;	
			break;
		case climate::CLIMATE_MODE_FAN_ONLY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000111;	
			break;
		case climate::CLIMATE_MODE_HEAT:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000001;	
			break;
	}

	// Настраиваем режим вентилятора
	if (this->fan_mode.has_value()) {
		switch(*this->fan_mode) {
			case climate::CLIMATE_FAN_AUTO:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000000;
				break;
			case climate::CLIMATE_FAN_QUIET:
				dataTX[8]	+= 0b10000000;
				dataTX[10]	+= 0b00000000;
				break;
			case climate::CLIMATE_FAN_LOW:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000001;
				break;
			case climate::CLIMATE_FAN_MIDDLE:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000110;
				break;
			case climate::CLIMATE_FAN_MEDIUM:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000011;
				break;
			case climate::CLIMATE_FAN_HIGH:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000111;
				break;
			case climate::CLIMATE_FAN_FOCUS:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000101;
				break;
			case climate::CLIMATE_FAN_DIFFUSE:
				dataTX[8]	+= 0b01000000;
				dataTX[10]	+= 0b00000000;
				break;
		}
	}
	
	// Устанавливаем режим качания заслонок
	switch(this->swing_mode) {
		case climate::CLIMATE_SWING_OFF:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_VERTICAL:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_HORIZONTAL:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00001000;
			break;
		case climate::CLIMATE_SWING_BOTH:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00001000;  
			break;
	}
	
	// Устанавливаем предустановки кондиционера
	if (this->preset.has_value()) {
		switch(*this->preset) {
			case ClimatePreset::CLIMATE_PRESET_NONE:
				break;
			case ClimatePreset::CLIMATE_PRESET_ECO:
				dataTX[7]	+= 0b10000000;
				break;
			case ClimatePreset::CLIMATE_PRESET_SLEEP:
				dataTX[19]	+= 0b00000001;
				break;
			case ClimatePreset::CLIMATE_PRESET_COMFORT:
				dataTX[8]	+= 0b00010000;
				break;
		}
	}

        //Режим заслонок
		//	Вертикальная заслонка
		//		Качание вертикальной заслонки [10 байт, маска 00111000]:
		//			000 - Качание отключено, заслонка в последней позиции или в фиксации
		//			111 - Качание включено в выбранном режиме
		//		Режим качания вертикальной заслонки (режим фиксации заслонки роли не играет, если качание включено) [32 байт, маска 00011000]:
		//			01 - качание сверху вниз, ПО УМОЛЧАНИЮ
		//			10 - качание в верхней половине
		//			11 - качание в нижней половине
		//		Режим фиксации заслонки (режим качания заслонки роли не играет, если качание выключено) [32 байт, маска 00000111]:
		//			000 - нет фиксации, ПО УМОЛЧАНИЮ
		//			001 - фиксация вверху
		//			010 - фиксация между верхом и серединой
		//			011 - фиксация в середине
		//			100 - фиксация между серединой и низом
		//			101 - фиксация внизу
		//	Горизонтальные заслонки
		//		Качание горизонтальных заслонок [11 байт, маска 00001000]:
		//			0 - Качание отключено, заслонки в последней позиции или в фиксации
		//			1 - Качание включено в выбранном режиме
		//		Режим качания горизонтальных заслонок (режим фиксации заслонок роли не играет, если качание включено) [33 байт, маска 00111000]:
		//			001 - качание слева направо, ПО УМОЛЧАНИЮ
		//			010 - качание слева
		//			011 - качание по середине
		//			100 - качание справа
		//		Режим фиксации горизонтальных заслонок (режим качания заслонок роли не играет, если качание выключено) [33 байт, маска 00000111]:
		//			000 - нет фиксации, ПО УМОЛЧАНИЮ
		//			001 - фиксация слева
		//			010 - фиксация между левой стороной и серединой
		//			011 - фиксация в середине
		//			100 - фиксация между серединой и правой стороной
		//			101 - фиксация справа
		
		
	// Устанавливаем режим для качания вертикальной заслонки
	switch(vertical_swing_direction_) {
		case VerticalSwingDirection::UP_DOWN:
			dataTX[32]	+= 0b00001000;
			ESP_LOGD("TCL", "Vertical swing: up-down");
			break;
		case VerticalSwingDirection::UPSIDE:
			dataTX[32]	+= 0b00010000;
			ESP_LOGD("TCL", "Vertical swing: upper");
			break;
		case VerticalSwingDirection::DOWNSIDE:
			dataTX[32]	+= 0b00011000;
			ESP_LOGD("TCL", "Vertical swing: downer");
			break;
	}
	// Устанавливаем режим для качания горизонтальных заслонок
	switch(horizontal_swing_direction_) {
		case HorizontalSwingDirection::LEFT_RIGHT:
			dataTX[33]	+= 0b00001000;
			ESP_LOGD("TCL", "Horizontal swing: left-right");
			break;
		case HorizontalSwingDirection::LEFTSIDE:
			dataTX[33]	+= 0b00010000;
			ESP_LOGD("TCL", "Horizontal swing: lefter");
			break;
		case HorizontalSwingDirection::CENTER:
			dataTX[33]	+= 0b00011000;
			ESP_LOGD("TCL", "Horizontal swing: center");
			break;
		case HorizontalSwingDirection::RIGHTSIDE:
			dataTX[33]	+= 0b00100000;
			ESP_LOGD("TCL", "Horizontal swing: righter");
			break;
	}
	// Устанавливаем положение фиксации вертикальной заслонки
	switch(vertical_direction_) {
		case AirflowVerticalDirection::LAST:
			dataTX[32]	+= 0b00000000;
			ESP_LOGD("TCL", "Vertical fix: last position");
			break;
		case AirflowVerticalDirection::MAX_UP:
			dataTX[32]	+= 0b00000001;
			ESP_LOGD("TCL", "Vertical fix: up");
			break;
		case AirflowVerticalDirection::UP:
			dataTX[32]	+= 0b00000010;
			ESP_LOGD("TCL", "Vertical fix: upper");
			break;
		case AirflowVerticalDirection::CENTER:
			dataTX[32]	+= 0b00000011;
			ESP_LOGD("TCL", "Vertical fix: center");
			break;
		case AirflowVerticalDirection::DOWN:
			dataTX[32]	+= 0b00000100;
			ESP_LOGD("TCL", "Vertical fix: downer");
			break;
		case AirflowVerticalDirection::MAX_DOWN:
			dataTX[32]	+= 0b00000101;
			ESP_LOGD("TCL", "Vertical fix: down");
			break;
	}
	// Устанавливаем положение фиксации горизонтальных заслонок
	switch(horizontal_direction_) {
		case AirflowHorizontalDirection::LAST:
			dataTX[33]	+= 0b00000000;
			ESP_LOGD("TCL", "Horizontal fix: last position");
			break;
		case AirflowHorizontalDirection::MAX_LEFT:
			dataTX[33]	+= 0b00000001;
			ESP_LOGD("TCL", "Horizontal fix: left");
			break;
		case AirflowHorizontalDirection::LEFT:
			dataTX[33]	+= 0b00000010;
			ESP_LOGD("TCL", "Horizontal fix: lefter");
			break;
		case AirflowHorizontalDirection::CENTER:
			dataTX[33]	+= 0b00000011;
			ESP_LOGD("TCL", "Horizontal fix: center");
			break;
		case AirflowHorizontalDirection::RIGHT:
			dataTX[33]	+= 0b00000100;
			ESP_LOGD("TCL", "Horizontal fix: righter");
			break;
		case AirflowHorizontalDirection::MAX_RIGHT:
			dataTX[33]	+= 0b00000101;
			ESP_LOGD("TCL", "Horizontal fix: right");
			break;
	}

	// Установка температуры
	dataTX[9] = target_temperature_set;
		
	// Собираем массив байт для отправки в кондиционер
	dataTX[0] = 0xBB;	//стартовый байт заголовка
	dataTX[1] = 0x00;	//стартовый байт заголовка
	dataTX[2] = 0x01;	//стартовый байт заголовка
	dataTX[3] = 0x03;	//0x03 - управление, 0x04 - опрос
	dataTX[4] = 0x20;	//0x20 - управление, 0x19 - опрос
	dataTX[5] = 0x03;	//??
	dataTX[6] = 0x01;	//??
	//dataTX[7] = 0x64;	//eco,display,beep,ontimerenable, offtimerenable,power,0,0
	//dataTX[8] = 0x08;	//mute,0,turbo,health, mode(4) mode 01 heat, 02 dry, 03 cool, 07 fan, 08 auto, health(+16), 41=turbo-heat 43=turbo-cool (turbo = 0x40+ 0x01..0x08)
	//dataTX[9] = 0x0f;	//0 -31 ;    15 - 16 0,0,0,0, temp(4) settemp 31 - x
	//dataTX[10] = 0x00;	//0,timerindicator,swingv(3),fan(3) fan+swing modes //0=auto 1=low 2=med 3=high
	//dataTX[11] = 0x00;	//0,offtimer(6),0
	dataTX[12] = 0x00;	//fahrenheit,ontimer(6),0 cf 80=f 0=c
	dataTX[13] = 0x01;	//??
	dataTX[14] = 0x00;	//0,0,halfdegree,0,0,0,0,0
	dataTX[15] = 0x00;	//??
	dataTX[16] = 0x00;	//??
	dataTX[17] = 0x00;	//??
	dataTX[18] = 0x00;	//??
	//dataTX[19] = 0x00;	//sleep on = 1 off=0
	dataTX[20] = 0x00;	//??
	dataTX[21] = 0x00;	//??
	dataTX[22] = 0x00;	//??
	dataTX[23] = 0x00;	//??
	dataTX[24] = 0x00;	//??
	dataTX[25] = 0x00;	//??
	dataTX[26] = 0x00;	//??
	dataTX[27] = 0x00;	//??
	dataTX[28] = 0x00;	//??
	dataTX[30] = 0x00;	//??
	dataTX[31] = 0x00;	//??
	//dataTX[32] = 0x00;	//0,0,0,режим вертикального качания(2),режим вертикальной фиксации(3)
	//dataTX[33] = 0x00;	//0,0,режим горизонтального качания(3),режим горизонтальной фиксации(3)
	dataTX[34] = 0x00;	//??
	dataTX[35] = 0x00;	//??
	dataTX[36] = 0x00;	//??
	dataTX[37] = 0xFF;	//Контрольная сумма
	dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

	tclacClimate::sendData(dataTX, sizeof(dataTX));
	allow_take_control = false;
	is_call_control = false;
}

// Отправка данных в кондиционер
void tclacClimate::sendData(uint8_t * message, uint8_t size) {
	tclacClimate::dataShow(1,1);
	//Serial.write(message, size);
	this->esphome::uart::UARTDevice::write_array(message, size);
	//auto raw = getHex(message, size);
	ESP_LOGD("TCL", "Message to TCL sended...");
	tclacClimate::dataShow(1,0);
}

// Преобразование байта в читабельный формат
std::string tclacClimate::getHex(uint8_t *message, uint8_t size) {
        std::string raw;
        char buf[8];
        for (int i = 0; i < size; i++) {
                snprintf(buf, sizeof(buf), "\n%02X", message[i]);
                raw += buf;
        }
        return raw;
}

// Вычисление контрольной суммы
uint8_t tclacClimate::getChecksum(const uint8_t * message, size_t size) {
	uint8_t position = size - 1;
	uint8_t crc = 0;
	for (int i = 0; i < position; i++)
		crc ^= message[i];
	return crc;
}

// Мигаем светодиодами
void tclacClimate::dataShow(bool flow, bool shine) {
	if (module_display_status_){
		if (flow == 0){
			if (shine == 1){
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(false);
#endif
			}
		}
		if (flow == 1) {
			if (shine == 1){
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(false);
#endif
			}
		}
	}
}

// Действия с данными из конфига

// Получение состояния пищалки
void tclacClimate::set_beeper_state(bool state) {
	this->beeper_status_ = state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение состояния дисплея кондиционера
void tclacClimate::set_display_state(bool disp_state) {
	this->display_status_ = disp_state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение состояния режима принудительного применения настроек
void tclacClimate::set_force_mode_state(bool f_state) {
	this->force_mode_status_ = f_state;
}
// Получение пина светодиода приема данных
#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
	this->rx_led_pin_ = rx_led_pin;
}
#endif
// Получение пина светодиода передачи данных
#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
	this->tx_led_pin_ = tx_led_pin;
}
#endif
// Получение состояния светодиодов связи модуля
void tclacClimate::set_module_display_state(bool d_state) {
	this->module_display_status_ = d_state;
}

void tclacClimate::set_power_sensor(sensor::Sensor *power_sensor) {
	this->power_sensor_ = power_sensor;
}

// Получение режима фиксации вертикальной заслонки
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection v_airflow) {
	this->vertical_direction_ = v_airflow;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение режима фиксации горизонтальных заслонок
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection h_airflow) {
	this->horizontal_direction_ = h_airflow;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение режима качания вертикальной заслонки
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection vs_direction) {
	this->vertical_swing_direction_ = vs_direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение доступных режимов работы кондиционера
void tclacClimate::set_supported_modes(climate::ClimateModeMask modes) {
	this->supported_modes_ = modes;
	ESP_LOGD("TCL", "Set up Modes");
}
// Получение режима качания горизонтальных заслонок
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection hs_direction) {
	horizontal_swing_direction_ = hs_direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Получение доступных скоростей вентилятора
void tclacClimate::set_supported_fan_modes(climate::ClimateFanModeMask fan_modes){
	this->supported_fan_modes_ = fan_modes;
}
// Получение доступных режимов качания заслонок
void tclacClimate::set_supported_swing_modes(climate::ClimateSwingModeMask swing_modes) {
	this->supported_swing_modes_ = swing_modes;
}
// Получение доступных предустановок
void tclacClimate::set_supported_presets(climate::ClimatePresetMask presets) {
  this->supported_presets_ = presets;
}


}
}
