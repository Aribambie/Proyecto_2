#include "Ojito_R.h"  // Incluye el encabezado que contiene las declaraciones de las funciones y constantes necesarias

// Función para inicializar el PWM en el canal A del Timer 1
void initPWM1FastA(uint8_t inverted, uint16_t precaler) {
	// Configuración del pin correspondiente al canal A como salida
	DDRB |= (1<<DDB1);
	
	// Configuración del modo de operación del Timer 1 y del modo de salida PWM del canal A
	TCCR1A = 0;
	if (inverted) {
		TCCR1A |= (1<<COM1A1)|(1<<COM1A0);
		} else {
		TCCR1A |= (1<<COM1A1);
	}
	TCCR1A |= (1<<WGM10);  // Configuración del modo PWM
	TCCR1B |= (1<<WGM12);
	
	// Configuración del preescalador del Timer 1
	if (precaler == 1024) {
		TCCR1B |= (1<<CS12)|(1<<CS10);
	}
}

// Función para inicializar el PWM en el canal B del Timer 1
void initPWM1FastB(uint8_t inverted, uint16_t precaler) {
	// Configuración del pin correspondiente al canal B como salida
	DDRB |= (1<<DDB2);
	
	// Configuración del modo de salida PWM del canal B
	if (inverted) {
		TCCR1A |= (1<<COM1B1)|(1<<COM1B0);
		} else {
		TCCR1A |= (1<<COM1B1);
	}
	TCCR1A |= (1<<WGM10);  // Configuración del modo PWM
	TCCR1B |= (1<<WGM12);
	
	// Configuración del preescalador del Timer 1
	TCCR1B |= (1<<CS12)|(1<<CS10);
}

// Función para actualizar el ciclo de trabajo del canal A del Timer 1
void updateDCA1(uint8_t duty) {
	OCR1A = duty;  // Se establece el valor del registro de comparación del canal A
}

// Función para actualizar el ciclo de trabajo del canal B del Timer 1
void updateDCB1(uint8_t duty) {
	OCR1B = duty;  // Se establece el valor del registro de comparación del canal B
}
