#ifndef Ojito_L
#define Ojito_L

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida AVR
#include <stdint.h>    // Incluye la biblioteca de tipos de datos est�ndar enteros

#define invertido 1       // Definici�n de la constante para configuraci�n invertida
#define no_invertido 0   // Definici�n de la constante para configuraci�n no invertida

// Declaraci�n de las funciones para el control del PWM en el canal A del Timer 0
void initPWM0FastA(uint8_t inverted, uint16_t precaler);
void updateDCA(uint8_t duty);

// Declaraci�n de las funciones para el control del PWM en el canal B del Timer 0
void initPWM0FastB(uint8_t inverted, uint16_t precaler);
void updateDCB(uint8_t duty);

#endif  // Fin de la directiva de inclusi�n condicional
