//*****************************************************************************
//Universidad del Valle de Guatemala
//Programaci�n de Microcontroladores
//Archivo:Funtime_Foxy_eyes
//Hardware:ATMEGA328P
//Autor:Adriana Marcela Gonzalez
//Carnet:22438
//*****************************************************************************
//Encabezado 
//*****************************************************************************

#define F_CPU 16000000ul //Se define la frecuencia del microcontrolador 

#include <avr/io.h> //Se importa la librer�a general 
#include <avr/interrupt.h> //Se importa la librer�a de interrupciones
#include <util/delay.h> //Se importa la librer�a para delays
#include <avr/eeprom.h> //Se importa la librer�a de eeprom 
#include "Ojito_L/Ojito_L.h" /*Se importa la librer�a de PWM para el ojo 
izquierdo (PWM0/Timer0)*/
#include "Ojito_R/Ojito_R.h" /*Se importa la librer�a de PWM para el ojo
derecho (PWM1/Timer1)*/

//*****************************************************************************
//Definici�n de funciones
//*****************************************************************************

void initUART9600(void); //Funci�n de la comunicaci�n UART
void initADC(void); //Funci�n para m�dulo ADC 
void writeTextUART(char* texto); /*Definici�n de funci�n para la escritura en
UART haciendo uso de punteros para el texto que escribe*/

//*****************************************************************************
//Definici�n de variables globales
//*****************************************************************************

volatile char bufferRX; //Buffer para comunicaci�n UART

uint16_t dutyCycle1; //Ciclo de trabajo para servo motor 1
uint16_t dutyCycle2; //Ciclo de trabajo para servo motor 2
uint16_t dutyCycle3; //Ciclo de trabajo para servo motor 3
uint16_t dutyCycle4; //Ciclo de trabajo para servo motor 4

uint16_t* Yellow_address = 0x00; /*Direcci�n de eeprom para modo 1 
(bot�n amarillo)*/
uint16_t* Red_address = 0x01; /*Direcci�n de eeprom para modo 2
(bot�n rojo)*/
uint16_t* Blue_address = 0x64; /*Direcci�n de eeprom para modo 3
(bot�n blue)*/
uint16_t* White_address = 0x0D; /*Direcci�n de eeprom para modo 4
(bot�n blanco)*/

volatile uint8_t buttonPress3 = 0;  // Estado del bot�n PD7
volatile uint32_t buttonTime3 = 0;  // Tiempo de presi�n del bot�n
volatile uint8_t buttonPress1 = 0;  // Estado del bot�n PD3
volatile uint32_t buttonTime1 = 0;  // Tiempo de presi�n del bot�n
volatile uint8_t buttonPress2 = 0;  // Estado del bot�n PD4
volatile uint32_t buttonTime2 = 0;  // Tiempo de presi�n del bot�n
volatile uint8_t buttonPress4 = 0;  // Estado del bot�n PB0
volatile uint32_t buttonTime4 = 0;  // Tiempo de presi�n del bot�n

//Variables para almacenar los datos del UART
uint16_t newdutyCycle1 = 0; 
uint16_t newdutyCycle2 = 0;
uint16_t newdutyCycle3 = 0;
uint16_t newdutyCycle4 = 0;

volatile uint8_t buffer_ADC;
volatile uint8_t buffer_RX2;


//*****************************************************************************
//Funciones de comunicaci�n UART 
//*****************************************************************************

void initUART9600(void){
    // Configuraci�n de RX y TX
    DDRD &= ~(1<<DDD0); // RX como entrada
    DDRD |= (1<<DDD1); // TX como salida
    
    // Configuraci�n del modo fast
    UCSR0A = 0;
    UCSR0A |= (1<<U2X0); // Modo de doble velocidad
    
    // Configuraci�n del registro B
    UCSR0B = 0;
	// Habilitar RX, TX e interrupci�n de RX
    UCSR0B |= (1<<RXCIE0)|(1 << RXEN0)|(1 << TXEN0); 
    
    /*Configuraci�n del registro C: frame - 8 bits de datos, no paridad,
	 1 bit de stop*/
    UCSR0C = 0;
    UCSR0C |= (1 << UCSZ01)|(1 << UCSZ00);
    
    // Baudrate = 9600
    UBRR0 = 207; // F�rmula para 9600 baudios con F_CPU = 16MHz
}

void transUART (unsigned char valorT) {
	// Esperar a que el buffer de transmisi�n est� vac�o
	while (!(UCSR0A & (1 << UDRE0))); 
	UDR0 = valorT; // Transmitir car�cter
}

unsigned char recivUART(void) {
	return bufferRX;
	/*while (!(UCSR0A & (1 << RXC0))); // Esperar a que haya datos disponibles
	return UDR0; */// Devolver el dato recibido
}

void writeTextUART(char* texto){
	uint8_t i;
	for(i=0; texto[i]!='\n'; i++){ // Iterar hasta encontrar el car�cter de nueva l�nea
		while(!(UCSR0A & (1<<UDRE0))); // Esperar a que el buffer de transmisi�n est� vac�o
		UDR0 = texto[i]; // Transmitir car�cter por car�cter
	}
}


//*****************************************************************************
//Funciones de m�dulo ADC
//*****************************************************************************

void initADC(void){
	//Inicializa el registro ADMUX a 0
	ADMUX = 0;

	//Selecciona la referencia de voltaje AVcc con un capacitor en AREF
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);

	//Ajuste a la izquierda del resultado del ADC para facilitar la lectura
	ADMUX |= (1 << ADLAR);

	//Inicializa el registro ADCSRA a 0
	ADCSRA = 0;
	
	//Habilita el ADC
	ADCSRA |= (1 << ADEN);
	
	//Establece la frecuencia de prescaler del ADC a 128 
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	//Deshabilita las entradas digitales en los pines ADC5 y ADC4 
	DIDR0 |= (1 << ADC5D);
	DIDR0 |= (1 << ADC4D);
}

uint16_t valorADC(uint8_t canal) {
	//Selecciona el canal del ADC limpiando los cuatro bits menos significativos
	ADMUX &= 0xF0;
	ADMUX |= canal;
	
	//Inicia una conversi�n de ADC
	ADCSRA |= (1 << ADSC);
	
	//Espera a que la conversi�n de ADC termine (el bit ADSC se pondr� a 0)
	while (ADCSRA & (1 << ADSC));
	
	//Retorna los 8 bits m�s significativos del resultado del ADC
	return ADCH;
}


//*****************************************************************************
//Configuraci�n de puertos 
//*****************************************************************************

void setup (void){
    //BOTONES
    DDRD &= ~(1 << DDD2); //Declaraci�n como entrada (Negro - modo manual)
    PORTD |= (1 << DDD2); //Activar resistencias internas para pull_up
	
    DDRD &= ~(1 << DDD3); //Declaraci�n como entrada (Amarillo)
    PORTD |= (1 << DDD3); //Activar resistencias internas para pull_up
    
    DDRD &= ~(1 << DDD4); //Declaraci�n como entrada (Rojo)
    PORTD |= (1 << DDD4); //Activar resistencias internas para pull_up
    
    DDRD &= ~(1 << DDD7); //Declaraci�n como entrada (Azul)
    PORTD |= (1 << DDD7); //Activar resistencias internas para pull_up
    
    DDRB &= ~(1 << DDB0); //Declaraci�n como entrada (Blanco)
    PORTB |= (1 << DDB0); //Activar resistencias internas para pull_up 
	
    
    //LEDS
	//Declaraci�n de puertos como salida para leds indicadores
    DDRB |= (1 << DDB4); //Led amarillo
    DDRC |= (1 << DDC0); //Led rojo
    DDRC |= (1 << DDC1); //Led azul
    DDRC |= (1 << DDC2); //Led blanco 
}

//*****************************************************************************
//Main Code
//*****************************************************************************

int main(void)
{
	// Inicializaci�n de los PWMs en los canales A y B del Timer 0 y 1 con un preescalador de 1024
	initPWM0FastA(0,1024);
	initPWM0FastB(0,1024);
	initPWM1FastA(0,1024);
	initPWM1FastB(0,1024);
	
	// Configuraci�n inicial del sistema
	setup();
	// Inicializaci�n de la comunicaci�n UART a 9600 baudios
	initUART9600();
	// Inicializaci�n del convertidor anal�gico-digital (ADC)
	initADC();
	// Inicializaci�n del Timer 2
	initTimer2();
	// Habilitar las interrupciones globales
	sei();
	
	// Variables para almacenar los valores de los ciclos de trabajo obtenidos del ADC
	uint16_t dutyCycle1 = valorADC(6);
	uint16_t dutyCycle2 = valorADC(7);
	uint16_t dutyCycle3 = valorADC(5);
	uint16_t dutyCycle4 = valorADC(4);
	
	// Apagar los leds
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	writeTextUART("Elija una posici�n del 1 al 4\n");
	
	// Loop principal del programa
	while (1)
	{
		
		
		if (UDR0 == 49) {
			Yellow_read();
			} else if (UDR0 == 50) {
			Red_read();
			} else if (UDR0 == 51) {
			Blue_read();
			} else if (UDR0 == 52) {
			White_read();
		}
		
		// Comprobaci�n de si el bot�n 1 est� presionado
		if (!(PIND & (1 << DDD2))){
			Manual_Mode();
		}
		
		// Comprobaci�n de si el bot�n 2 est� presionado
		if (!(PIND & (1 << DDD3))) {
			if (!buttonPress1) {
				buttonPress1 = 1;
				buttonTime1 = 0;
			}
			while (!(PIND & (1 << DDD3))) {
				_delay_ms(10);
			}
			buttonPress1 = 0;
			if (buttonTime1 >= 3000) {
				//Si el bot�n estuvo presionado durante al menos 5 segundos
				Yellow_write();
				} else {
				//Si el bot�n estuvo presionado por menos de 5 segundos
				Yellow_read();
			}
		}
		
		// Comprobaci�n de si el bot�n 3 est� presionado
		if (!(PIND & (1 << DDD4))) {
			if (!buttonPress2) {
				buttonPress2 = 1;
				buttonTime2 = 0;
			}
			while (!(PIND & (1 << DDD4))) {
				_delay_ms(10);
			}
			buttonPress2 = 0;
			if (buttonTime2 >= 3000) {
				// Si el bot�n estuvo presionado durante al menos 5 segundos
				Red_write();
				} else {
				// Si el bot�n estuvo presionado por menos de 5 segundos
				Red_read();
			}
		}
		
		// Comprobaci�n de si el bot�n 4 est� presionado
		if (!(PIND & (1 << DDD7))) {
			if (!buttonPress3) {
				buttonPress3 = 1;
				buttonTime3 = 0;
			}
			while (!(PIND & (1 << DDD7))) {
				_delay_ms(10);
			}
			buttonPress3 = 0;
			if (buttonTime3 >= 3000) {
				// Si el bot�n estuvo presionado durante al menos 5 segundos
				Blue_write();
				} else {
				// Si el bot�n estuvo presionado por menos de 5 segundos
				Blue_read();
			}
		}
		
		// Comprobaci�n de si el bot�n 5 est� presionado
		if (!(PINB & (1 << DDB0))) {
			if (!buttonPress4) {
				buttonPress4 = 1;
				buttonTime4 = 0;
			}
			while (!(PINB & (1 << DDB0))) {
				_delay_ms(10);
			}
			buttonPress4 = 0;
			if (buttonTime4 >= 3000) {
				// Si el bot�n estuvo presionado durante al menos 5 segundos
				White_write();
				} else {
				// Si el bot�n estuvo presionado por menos de 5 segundos
				White_read();
			}
		}
		
		 /* if ( bufferRX == '1') { // Si se selecciona la opci�n 1
			  
			  moveServosToPosition();
			  
		  }*/
		
		// Imprimir el buffer de datos
		//printBuffer();
	}
	
	return 0;
}


//*****************************************************************************
//Modo manual 
//*****************************************************************************

void Manual_Mode(void){
		PORTB |= (1 << DDB4);
		PORTC &= ~(1 << DDC0);
		PORTC &= ~(1 << DDC1);
		PORTC &= ~(1 << DDC2);
		_delay_ms(200);
		PORTC |= (1 << DDC2);
		PORTC &= ~(1 << DDC0);
		PORTC &= ~(1 << DDC1);
		PORTB &= ~(1 << DDB4);
		_delay_ms(200);
		PORTC |= (1 << DDC1);
		PORTC &= ~(1 << DDC0);
		PORTC &= ~(1 << DDC2);
		PORTB &= ~(1 << DDB4);
		_delay_ms(200);
		PORTC |= (1 << DDC0);
		PORTC &= ~(1 << DDC2);
		PORTC &= ~(1 << DDC1);
		PORTB &= ~(1 << DDB4);
		_delay_ms(200);
		PORTC &= ~(1 << DDC0);
		PORTC &= ~(1 << DDC2);
		PORTC &= ~(1 << DDC1);
		PORTB &= ~(1 << DDB4);
		_delay_ms(200);
		PORTC |= (1 << DDC0);
		PORTC |= (1 << DDC2);
		PORTC |= (1 << DDC1);
		PORTB |= (1 << DDB4);
		_delay_ms(200);
		
		Servos_move();  // Funci�n que muestra el movimiento de los servos 
}

//*****************************************************************************
//Funciones de escritura de Eeprom
//*****************************************************************************

// Funci�n para realizar una acci�n y escribir datos en la direcci�n de memoria correspondiente a la acci�n Yellow
void Yellow_write (void){
	// Configura los pines correspondientes para la acci�n Yellow
	PORTC |= (1 << DDC0);
	PORTC |= (1 << DDC2);
	PORTC |= (1 << DDC1);
	PORTB &= ~(1 << DDB4);
	
	// Realiza movimientos con servos
	Servos_move();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t PrimerGesto[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)PrimerGesto, (void*)Yellow_address, sizeof(PrimerGesto));
}

// Funci�n para realizar una acci�n y escribir datos en la direcci�n de memoria correspondiente a la acci�n Red
void Red_write (void){
	// Configura los pines correspondientes para la acci�n Red
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC2);
	PORTC |= (1 << DDC1);
	PORTB |= (1 << DDB4);
	
	// Realiza movimientos con servos
	Servos_move();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t SegundoGesto[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)SegundoGesto, (void*)Red_address, sizeof(SegundoGesto));
}

// Funci�n para realizar una acci�n y escribir datos en la direcci�n de memoria correspondiente a la acci�n Blue
void Blue_write (void){
	// Configura los pines correspondientes para la acci�n Blue
	PORTC |= (1 << DDC0);
	PORTC |= (1 << DDC2);
	PORTC &= ~(1 << DDC1);
	PORTB |= (1 << DDB4);
	
	// Realiza movimientos con servos
	Servos_move();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t TercerGesto[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)TercerGesto, (void*)Blue_address, sizeof(TercerGesto));
}

// Funci�n para realizar una acci�n y escribir datos en la direcci�n de memoria correspondiente a la acci�n White
void White_write (void){
	// Configura los pines correspondientes para la acci�n White
	PORTC &= ~(1 << DDC0);
	PORTC |= (1 << DDC2);
	PORTC |= (1 << DDC1);
	PORTB |= (1 << DDB4);
	
	// Realiza movimientos con servos
	Servos_move();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t CuartoGesto[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)CuartoGesto, (void*)White_address, sizeof(CuartoGesto));
}

//*****************************************************************************
//Funciones de lectura de Eeprom
//*****************************************************************************

// Funci�n para leer datos de la memoria EEPROM correspondientes a la acci�n Yellow y aplicar los valores le�dos a los servos
void Yellow_read (){

	PORTB |= (1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores le�dos de la memoria EEPROM
	uint16_t read_Yellow_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Yellow_eeprom, (const void*)Yellow_address, sizeof(read_Yellow_eeprom));
	
	// Asigna los valores le�dos a las variables dutyCycle
	dutyCycle1 = read_Yellow_eeprom[0];
	dutyCycle2 = read_Yellow_eeprom[1];
	dutyCycle3 = read_Yellow_eeprom[2];
	dutyCycle4 = read_Yellow_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores le�dos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Funci�n para leer datos de la memoria EEPROM correspondientes a la acci�n Red y aplicar los valores le�dos a los servos
void Red_read (){
	
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC |= (1 << DDC2);
	
	// Arreglo para almacenar los valores le�dos de la memoria EEPROM
	uint16_t read_Red_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Red_eeprom, (const void*)Red_address, sizeof(read_Red_eeprom));
	
	// Asigna los valores le�dos a las variables dutyCycle
	dutyCycle1 = read_Red_eeprom[0];
	dutyCycle2 = read_Red_eeprom[1];
	dutyCycle3 = read_Red_eeprom[2];
	dutyCycle4 = read_Red_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores le�dos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Funci�n para leer datos de la memoria EEPROM correspondientes a la acci�n Blue y aplicar los valores le�dos a los servos
void Blue_read (){
	// Configura los pines correspondientes para la acci�n Blue
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC |= (1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores le�dos de la memoria EEPROM
	uint16_t read_Blue_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Blue_eeprom, (const void*)Blue_address, sizeof(read_Blue_eeprom));
	
	// Asigna los valores le�dos a las variables dutyCycle
	dutyCycle1 = read_Blue_eeprom[0];
	dutyCycle2 = read_Blue_eeprom[1];
	dutyCycle3 = read_Blue_eeprom[2];
	dutyCycle4 = read_Blue_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores le�dos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Funci�n para leer datos de la memoria EEPROM correspondientes a la acci�n White y aplicar los valores le�dos a los servos
void White_read (){
	// Configura los pines correspondientes para la acci�n White
	PORTB &= ~(1 << DDB4);
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores le�dos de la memoria EEPROM
	uint16_t read_White_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_White_eeprom, (const void*)White_address, sizeof(read_White_eeprom));
	
	// Asigna los valores le�dos a las variables dutyCycle
	dutyCycle1 = read_White_eeprom[0];
	dutyCycle2 = read_White_eeprom[1];
	dutyCycle3 = read_White_eeprom[2];
	dutyCycle4 = read_White_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores le�dos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

//*****************************************************************************
//Funci�n de movimiento de servos 
//*****************************************************************************

void Servos_move(void){
	// Bucle infinito para controlar el movimiento de los servos
	while (1) {
		// Verificar si se ha detectado una se�al en alguno de los pines espec�ficos
		if (!(PIND & (1 << DDD3)) || !(PIND & (1 << DDD4)) || !(PIND & (1 << DDD7)) || !(PINB & (1 << DDB0))) {
			// Si se ha detectado una se�al, salir del bucle
			break;
		}
		
		// Actualizar el ciclo de trabajo del servo del ojo izquierdo basado en el valor del ADC en el pin 6
		dutyCycle1 = valorADC(6);
		_delay_ms(10);
		updateDCA(dutyCycle1/6);

		// Actualizar el ciclo de trabajo del servo del ojo derecho basado en el valor del ADC en el pin 7
		dutyCycle2 = valorADC(7);
		_delay_ms(10);
		updateDCB(dutyCycle2/6);

		// Actualizar el ciclo de trabajo del segundo servo del ojo izquierdo basado en el valor del ADC en el pin 5
		dutyCycle3 = valorADC(5);
		_delay_ms(10);
		updateDCA1(dutyCycle3/6);

		// Actualizar el ciclo de trabajo del segundo servo del ojo derecho basado en el valor del ADC en el pin 4
		dutyCycle4 = valorADC(4);
		_delay_ms(10);
		updateDCB1(dutyCycle4/6);

		// Retraso antes de la pr�xima iteraci�n del bucle
		_delay_ms(25);
	}
}


//*****************************************************************************
//Interrupciones 
//*****************************************************************************

// Rutina de interrupci�n para manejar la recepci�n de datos por UART
ISR(USART_RX_vect){
	
	bufferRX = UDR0;
	buffer_RX2 = UDR0;
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = bufferRX;
	UDR0 = buffer_RX2;


       /* char receivedChar = UDR0;

        // Procesar el comando recibido
        buffer[bufferIndex++] = receivedChar;

        // Si hemos recibido suficiente informaci�n para un comando
        if (bufferIndex >= 4) {
	        bufferIndex = 0;
	        
	        // Leer el comando
	        command = buffer[0];

	        switch (command) {
		        case '0':
		        // Leer el siguiente caracter para determinar el modo
		        mode = buffer[1];
		        if (mode == '1') {
			        // UART mode
			        } else if (mode == '0') {
			        // EEPROM mode
			        } else if (mode == '1') {
			        // Manual mode
		        }
		        break;
		        case '1':
		        if (buffer[1] == '2') {
			        newdutyCycle1 = readTwoBytes(buffer[2], buffer[3]);
			        updateDCA(newdutyCycle1 / 6);
			        _delay_ms(10);
			        } else if (buffer[1] == '4') {
			        newdutyCycle2 = readTwoBytes(buffer[2], buffer[3]);
			        updateDCB(newdutyCycle2 / 6);
			        _delay_ms(10);
			        } else if (buffer[1] == '6') {
			        newdutyCycle3 = readTwoBytes(buffer[2], buffer[3]);
			        updateDCA1(newdutyCycle3 / 6);
			        _delay_ms(10);
			        } else if (buffer[1] == '8') {
			        newdutyCycle4 = readTwoBytes(buffer[2], buffer[3]);
			        updateDCB1(newdutyCycle4 / 6);
			        _delay_ms(10);
		        }
		        break;
		        default:
		        break;
	        }
        }*/
}


// Rutina de servicio de interrupci�n (ISR) para el temporizador TIMER2 en modo de comparaci�n A
ISR(TIMER2_COMPA_vect) {
	// Incrementar el contador de tiempo del bot�n 4 si est� presionado
	if (buttonPress4) {
		buttonTime4++;
	}
	// Incrementar el contador de tiempo del bot�n 1 si est� presionado
	else if (buttonPress1) {
		buttonTime1++;
	}
	// Incrementar el contador de tiempo del bot�n 2 si est� presionado
	else if (buttonPress2) {
		buttonTime2++;
	}
	// Incrementar el contador de tiempo del bot�n 3 si est� presionado
	else if (buttonPress3) {
		buttonTime3++;
	}
}



//*****************************************************************************
//Timer 2
//*****************************************************************************
void initTimer2(void) {
	// Configurar Timer2 para que se desborde cada 1 ms
	TCCR2A = (1 << WGM21); // Modo CTC
	OCR2A = 249; // Para que se desborde cada 1 ms con un prescaler de 64
	TIMSK2 = (1 << OCIE2A); // Habilitar interrupci�n de comparaci�n
	TCCR2B = (1 << CS22); // Prescaler 64
}



//*****************************************************************************
//Funciones ADAFRUIT :(
//*****************************************************************************

/*void readTwoBytes(char highByte, char lowByte) {
	return (highByte << 8) + lowByte;
}*/




