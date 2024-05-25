#ifndef Ojito_R
#define Ojito_R

#include <avr/io.h>   // Incluye la biblioteca AVR Input/Output
#include <stdint.h>   // Incluye la biblioteca de tipos de datos estándar de C

// Definición de constantes para configurar la polaridad de la señal PWM
#define invertido 1
#define no_invertido 0

// Declaración de funciones para inicializar y actualizar el PWM del canal A del timer 1
void initPWM1FastA(uint8_t inverted, uint16_t precaler);
void updateDCA1(uint8_t duty);

// Declaración de funciones para inicializar y actualizar el PWM del canal B del timer 1
void initPWM1FastB(uint8_t inverted, uint16_t precaler);
void updateDCB1(uint8_t duty);

#endif
