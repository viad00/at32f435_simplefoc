/*
 * Arduino.cpp
 *
 *  Created on: 2024 Dec 13
 *      Author: vlad
 */

#include "Arduino.h"

void noInterrupts() {

}

void interrupts() {

}

void pinMode(pin_size_t pinNumber, PinMode pinMode) {
	// Fuck it, CubeMX better
	return;
}

pin_size_t pinToArduino(gpio_type *port, uint16_t pins) {
	uint8_t port_num = 0;
	if (port == GPIOA) {
	    port_num = 0;
	} else if (port == GPIOB) {
	    port_num = 1;
	} else if (port == GPIOC) {
	    port_num = 2;
	} else if (port == GPIOD) {
	    port_num = 3;
	} else if (port == GPIOE) {
	    port_num = 4;
	} else if (port == GPIOF) {
	    port_num = 5;
	} else if (port == GPIOG) {
	    port_num = 6;
	} else if (port == GPIOH) {
	    port_num = 7;
	} else {
	    port_num = 8;
	}
    uint8_t pin_num = 0;
	for (uint8_t i = 0; i < 16; i++) {
		pins = pins >> 1;
		if (!pins) {
			pin_num = i;
			break;
		}
	}
	pin_size_t arduino_pin = port_num * 16 + pin_num;
	return arduino_pin;
}

void arduinoToPin(pin_size_t arduino_pin, gpio_type **port, uint16_t *pins) {
	if (arduino_pin > 127) return;
	uint8_t port_num = arduino_pin / 16;
	uint8_t pins_num = arduino_pin % 16;
	*pins = 1 << pins_num;
	switch (port_num) {
	case 0:
		*port = GPIOA;
		break;
	case 1:
		*port = GPIOB;
		break;
	case 2:
		*port = GPIOC;
		break;
	case 3:
		*port = GPIOD;
		break;
	case 4:
		*port = GPIOE;
		break;
	case 5:
		*port = GPIOF;
		break;
	case 6:
		*port = GPIOG;
		break;
	case 7:
		*port = GPIOH;
		break;
	}
}

void digitalWrite(unsigned char pinNumber, bool status) {
	return digitalWrite((pin_size_t) pinNumber, (PinStatus) status);
}

void digitalWrite(pin_size_t pinNumber, PinStatus status) {
	gpio_type *port;
	uint16_t pins;
	arduinoToPin(pinNumber, &port, &pins);
	gpio_bits_write(port, pins, (confirm_state) status);
}

PinStatus digitalRead(pin_size_t pinNumber) {
	gpio_type *port;
	uint16_t pins;
	arduinoToPin(pinNumber, &port, &pins);
	return (PinStatus) gpio_input_data_bit_read(port, pins);
	//return gpio_output_data_bit_read(port, pins);
}

unsigned long millis(void) {
	return (unsigned long) tmr_counter_value_get(TMR2)/1000;
}

unsigned long micros(void) {
	return (unsigned long) tmr_counter_value_get(TMR2);
}

void delay(unsigned long delay) {
	wk_delay_ms(delay);
}

void delayMicroseconds(unsigned int us) {
	//delay_us((uint32_t)us);
	unsigned long old_us = micros();
	while(true) {
		if((micros()-old_us) > us) break;
	}

}

uint16_t get_string_len(char *s) {
	uint16_t length = 0;

	while (s[length] != '\0') {
	    length++;
	}
	return length;
}
