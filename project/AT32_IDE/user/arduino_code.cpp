/*
 * arduino_code.cpp
 *
 *  Created on: 2024 Dec 13
 *      Author: vlad
 */

#include "at32f435_437.h"
#include "SimpleFOC.h"

HallSensor sensor = HallSensor(pinToArduino(GPIOB, GPIO_PINS_7), pinToArduino(GPIOB, GPIO_PINS_8), pinToArduino(GPIOB, GPIO_PINS_9), 66);
BLDCDriver6PWM driver = BLDCDriver6PWM(0, 0, 0, 0, 0, 0, 0);
InlineCurrentSense current_sense = InlineCurrentSense(-2.5, 1, 0, 2);
BLDCMotor motor = BLDCMotor(66);

void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
extern "C" void interruptcallback() {sensor.handleAll();}

void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
	_UNUSED(pwm_frequency);
	_UNUSED(dead_zone);
	_UNUSED(pinA_h);
	_UNUSED(pinA_l);
	_UNUSED(pinB_h);
	_UNUSED(pinB_l);
	_UNUSED(pinC_h);
	_UNUSED(pinC_l);
	return (void*)0;
}

void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(phase_state);
  _UNUSED(params);
  if (((uint8_t) *phase_state) == 0) {
	  tmr_output_enable(TMR1, FALSE);
	  tmr_counter_enable(TMR1, FALSE);
	  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
	  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
	  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
    return;
  }
  // CH1 - V, CH2 - W, CH3 - U
  // A - W, B - V, C - U
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, (uint16_t) (dc_b*1000));
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, (uint16_t) (dc_a*1000));
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, (uint16_t) (dc_c*1000));
  if (TMR1->ctrl1_bit.tmren == 0) {
	  tmr_output_enable(TMR1, TRUE);
	  tmr_counter_enable(TMR1, TRUE);
  }
  /*Serial.print(dc_a);
  Serial.print(" ");
  Serial.print(dc_b);
  Serial.print(" ");
  Serial.print(dc_c);
  Serial.print(" ");
  Serial.print(((uint8_t) *phase_state));
  Serial.print(" ");
  Serial.println("Write duty cycle");*/
}

void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  //if( _isset(pinA) ) pinMode(pinA, INPUT_ANALOG);
  //if( _isset(pinB) ) pinMode(pinB, INPUT_ANALOG);
  //if( _isset(pinC) ) pinMode(pinC, INPUT_ANALOG);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (3.3f)/(4096.0f)
  };

  return params;
}

uint16_t ADC1_Buf[8] = {0};
uint16_t ADC2_Buf[2] = {0};
#define FAZEA_INDEX 0
#define FAZEC_INDEX 1

float _readADCVoltageInline(const int pinA, const void* cs_params){
  //Serial.println("Analog Read");
  //uint32_t raw_adc = analogRead(pinA);
  if (pinA == 1) {
    return ADC2_Buf[FAZEA_INDEX] * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
  } else if (pinA == 2) {
    return ADC2_Buf[FAZEC_INDEX] * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
  } else {
    return 0;
  }
}

void setup() {
	dac_1_data_set(DAC1_12BIT_RIGHT, 0);
	sensor.init();
	driver.init();
	current_sense.linkDriver(&driver);
    current_sense.init();
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&current_sense);
    motor.controller = MotionControlType::velocity;
    motor.init();
    motor.initFOC();
    motor.enable();
}

void loop() {
	motor.loopFOC();
}
