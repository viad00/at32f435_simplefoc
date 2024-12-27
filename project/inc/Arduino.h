/*
 * Arduino.h mimic for artery
 *
 *  Created on: 2024 Dec 13
 *      Author: vlad
 */

#ifndef USER_INC_ARDUINO_H_
#define USER_INC_ARDUINO_H_

#include "at32f435_437_wk_config.h"
#include "wk_system.h"
#include "ArduinoAPI.h"

void noInterrupts();
void interrupts();
pin_size_t digitalPinToInterrupt(pin_size_t pin);

#ifdef __cplusplus
extern "C" {
#endif

void delay_us(uint32_t delay);
void arduinoToPin(pin_size_t arduino_pin, gpio_type **port, uint16_t *pins);
pin_size_t pinToArduino(gpio_type *port, uint16_t pins);
void print_usb(char *c, uint16_t len);
uint16_t get_string_len(char *s);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
void digitalWrite(pin_size_t pinNumber, bool status);
void digitalWrite(unsigned char pinNumber, bool status);
#endif

#endif /* USER_INC_ARDUINO_H_ */
