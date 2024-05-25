#ifndef Ojito_L
#define Ojito_L

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida AVR
#include <stdint.h>    // Incluye la biblioteca de tipos de datos estándar enteros

#define invertido 1       // Definición de la constante para configuración invertida
#define no_invertido 0   // Definición de la constante para configuración no invertida

// Declaración de las funciones para el control del PWM en el canal A del Timer 0
void initPWM0FastA(uint8_t inverted, uint16_t precaler);
void updateDCA(uint8_t duty);

// Declaración de las funciones para el control del PWM en el canal B del Timer 0
void initPWM0FastB(uint8_t inverted, uint16_t precaler);
void updateDCB(uint8_t duty);

#endif  // Fin de la directiva de inclusión condicional
