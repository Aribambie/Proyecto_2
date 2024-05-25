//*****************************************************************************
//Universidad del Valle de Guatemala
//Programación de Microcontroladores
//Archivo:Funtime_Foxy_eyes
//Hardware:ATMEGA328P
//Autor:Adriana Marcela Gonzalez
//Carnet:22438
//*****************************************************************************
//Encabezado 
//*****************************************************************************

#define F_CPU 16000000ul //Se define la frecuencia del microcontrolador 

#include <avr/io.h> //Se importa la librería general 
#include <avr/interrupt.h> //Se importa la librería de interrupciones
#include <util/delay.h> //Se importa la librería para delays
#include <avr/eeprom.h> //Se importa la librería de eeprom 
#include "Ojito_L/Ojito_L.h" /*Se importa la librería de PWM para el ojo 
izquierdo (PWM0/Timer0)*/
#include "Ojito_R/Ojito_R.h" /*Se importa la librería de PWM para el ojo
derecho (PWM1/Timer1)*/

//*****************************************************************************
//Definición de funciones
//*****************************************************************************

void initUART9600(void); //Función de la comunicación UART
void initADC(void); //Función para módulo ADC 
void writeTextUART(char* texto); /*Definición de función para la escritura en
UART haciendo uso de punteros para el texto que escribe*/

//*****************************************************************************
//Definición de variables globales
//*****************************************************************************

volatile char bufferRX; //Buffer para comunicación UART

uint16_t dutyCycle1; //Ciclo de trabajo para servo motor 1
uint16_t dutyCycle2; //Ciclo de trabajo para servo motor 2
uint16_t dutyCycle3; //Ciclo de trabajo para servo motor 3
uint16_t dutyCycle4; //Ciclo de trabajo para servo motor 4

uint16_t* Yellow_address = 0x00; /*Dirección de eeprom para modo 1 
(botón amarillo)*/
uint16_t* Red_address = 0x01; /*Dirección de eeprom para modo 2
(botón rojo)*/
uint16_t* Blue_address = 0x64; /*Dirección de eeprom para modo 3
(botón blue)*/
uint16_t* White_address = 0x0D; /*Dirección de eeprom para modo 4
(botón blanco)*/

volatile uint8_t buttonPress3 = 0;  // Estado del botón PD7
volatile uint32_t buttonTime3 = 0;  // Tiempo de presión del botón
volatile uint8_t buttonPress1 = 0;  // Estado del botón PD3
volatile uint32_t buttonTime1 = 0;  // Tiempo de presión del botón
volatile uint8_t buttonPress2 = 0;  // Estado del botón PD4
volatile uint32_t buttonTime2 = 0;  // Tiempo de presión del botón
volatile uint8_t buttonPress4 = 0;  // Estado del botón PB0
volatile uint32_t buttonTime4 = 0;  // Tiempo de presión del botón

//Variables para almacenar los datos del UART
uint16_t newdutyCycle1 = 0; 
uint16_t newdutyCycle2 = 0;
uint16_t newdutyCycle3 = 0;
uint16_t newdutyCycle4 = 0;

volatile uint8_t buffer_ADC;
volatile uint8_t buffer_RX2;


//*****************************************************************************
//Funciones de comunicación UART 
//*****************************************************************************

void initUART9600(void){
    // Configuración de RX y TX
    DDRD &= ~(1<<DDD0); // RX como entrada
    DDRD |= (1<<DDD1); // TX como salida
    
    // Configuración del modo fast
    UCSR0A = 0;
    UCSR0A |= (1<<U2X0); // Modo de doble velocidad
    
    // Configuración del registro B
    UCSR0B = 0;
	// Habilitar RX, TX e interrupción de RX
    UCSR0B |= (1<<RXCIE0)|(1 << RXEN0)|(1 << TXEN0); 
    
    /*Configuración del registro C: frame - 8 bits de datos, no paridad,
	 1 bit de stop*/
    UCSR0C = 0;
    UCSR0C |= (1 << UCSZ01)|(1 << UCSZ00);
    
    // Baudrate = 9600
    UBRR0 = 207; // Fórmula para 9600 baudios con F_CPU = 16MHz
}

void transUART (unsigned char valorT) {
	// Esperar a que el buffer de transmisión esté vacío
	while (!(UCSR0A & (1 << UDRE0))); 
	UDR0 = valorT; // Transmitir carácter
}

unsigned char recivUART(void) {
	return bufferRX;
	/*while (!(UCSR0A & (1 << RXC0))); // Esperar a que haya datos disponibles
	return UDR0; */// Devolver el dato recibido
}

void writeTextUART(char* texto){
	uint8_t i;
	for(i=0; texto[i]!='\n'; i++){ // Iterar hasta encontrar el carácter de nueva línea
		while(!(UCSR0A & (1<<UDRE0))); // Esperar a que el buffer de transmisión esté vacío
		UDR0 = texto[i]; // Transmitir carácter por carácter
	}
}


//*****************************************************************************
//Funciones de módulo ADC
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
	
	//Inicia una conversión de ADC
	ADCSRA |= (1 << ADSC);
	
	//Espera a que la conversión de ADC termine (el bit ADSC se pondrá a 0)
	while (ADCSRA & (1 << ADSC));
	
	//Retorna los 8 bits más significativos del resultado del ADC
	return ADCH;
}


//*****************************************************************************
//Configuración de puertos 
//*****************************************************************************

void setup (void){
    //BOTONES
    DDRD &= ~(1 << DDD2); //Declaración como entrada (Negro - modo manual)
    PORTD |= (1 << DDD2); //Activar resistencias internas para pull_up
	
    DDRD &= ~(1 << DDD3); //Declaración como entrada (Amarillo)
    PORTD |= (1 << DDD3); //Activar resistencias internas para pull_up
    
    DDRD &= ~(1 << DDD4); //Declaración como entrada (Rojo)
    PORTD |= (1 << DDD4); //Activar resistencias internas para pull_up
    
    DDRD &= ~(1 << DDD7); //Declaración como entrada (Azul)
    PORTD |= (1 << DDD7); //Activar resistencias internas para pull_up
    
    DDRB &= ~(1 << DDB0); //Declaración como entrada (Blanco)
    PORTB |= (1 << DDB0); //Activar resistencias internas para pull_up 
	
    
    //LEDS
	//Declaración de puertos como salida para leds indicadores
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
	// Inicialización de los PWMs en los canales A y B del Timer 0 y 1 con un preescalador de 1024
	initPWM0FastA(0,1024);
	initPWM0FastB(0,1024);
	initPWM1FastA(0,1024);
	initPWM1FastB(0,1024);
	
	// Configuración inicial del sistema
	setup();
	// Inicialización de la comunicación UART a 9600 baudios
	initUART9600();
	// Inicialización del convertidor analógico-digital (ADC)
	initADC();
	// Inicialización del Timer 2
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
	
	writeTextUART("Elija una posición del 1 al 4\n");
	
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
		
		// Comprobación de si el botón 1 está presionado
		if (!(PIND & (1 << DDD2))){
			Manual_Mode();
		}
		
		// Comprobación de si el botón 2 está presionado
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
				//Si el botón estuvo presionado durante al menos 5 segundos
				Yellow_write();
				} else {
				//Si el botón estuvo presionado por menos de 5 segundos
				Yellow_read();
			}
		}
		
		// Comprobación de si el botón 3 está presionado
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
				// Si el botón estuvo presionado durante al menos 5 segundos
				Red_write();
				} else {
				// Si el botón estuvo presionado por menos de 5 segundos
				Red_read();
			}
		}
		
		// Comprobación de si el botón 4 está presionado
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
				// Si el botón estuvo presionado durante al menos 5 segundos
				Blue_write();
				} else {
				// Si el botón estuvo presionado por menos de 5 segundos
				Blue_read();
			}
		}
		
		// Comprobación de si el botón 5 está presionado
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
				// Si el botón estuvo presionado durante al menos 5 segundos
				White_write();
				} else {
				// Si el botón estuvo presionado por menos de 5 segundos
				White_read();
			}
		}
		
		 /* if ( bufferRX == '1') { // Si se selecciona la opción 1
			  
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
		
		Servos_move();  // Función que muestra el movimiento de los servos 
}

//*****************************************************************************
//Funciones de escritura de Eeprom
//*****************************************************************************

// Función para realizar una acción y escribir datos en la dirección de memoria correspondiente a la acción Yellow
void Yellow_write (void){
	// Configura los pines correspondientes para la acción Yellow
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

// Función para realizar una acción y escribir datos en la dirección de memoria correspondiente a la acción Red
void Red_write (void){
	// Configura los pines correspondientes para la acción Red
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

// Función para realizar una acción y escribir datos en la dirección de memoria correspondiente a la acción Blue
void Blue_write (void){
	// Configura los pines correspondientes para la acción Blue
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

// Función para realizar una acción y escribir datos en la dirección de memoria correspondiente a la acción White
void White_write (void){
	// Configura los pines correspondientes para la acción White
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

// Función para leer datos de la memoria EEPROM correspondientes a la acción Yellow y aplicar los valores leídos a los servos
void Yellow_read (){

	PORTB |= (1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t read_Yellow_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Yellow_eeprom, (const void*)Yellow_address, sizeof(read_Yellow_eeprom));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = read_Yellow_eeprom[0];
	dutyCycle2 = read_Yellow_eeprom[1];
	dutyCycle3 = read_Yellow_eeprom[2];
	dutyCycle4 = read_Yellow_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Función para leer datos de la memoria EEPROM correspondientes a la acción Red y aplicar los valores leídos a los servos
void Red_read (){
	
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC |= (1 << DDC2);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t read_Red_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Red_eeprom, (const void*)Red_address, sizeof(read_Red_eeprom));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = read_Red_eeprom[0];
	dutyCycle2 = read_Red_eeprom[1];
	dutyCycle3 = read_Red_eeprom[2];
	dutyCycle4 = read_Red_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Función para leer datos de la memoria EEPROM correspondientes a la acción Blue y aplicar los valores leídos a los servos
void Blue_read (){
	// Configura los pines correspondientes para la acción Blue
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC |= (1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t read_Blue_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_Blue_eeprom, (const void*)Blue_address, sizeof(read_Blue_eeprom));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = read_Blue_eeprom[0];
	dutyCycle2 = read_Blue_eeprom[1];
	dutyCycle3 = read_Blue_eeprom[2];
	dutyCycle4 = read_Blue_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

// Función para leer datos de la memoria EEPROM correspondientes a la acción White y aplicar los valores leídos a los servos
void White_read (){
	// Configura los pines correspondientes para la acción White
	PORTB &= ~(1 << DDB4);
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t read_White_eeprom[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)read_White_eeprom, (const void*)White_address, sizeof(read_White_eeprom));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = read_White_eeprom[0];
	dutyCycle2 = read_White_eeprom[1];
	dutyCycle3 = read_White_eeprom[2];
	dutyCycle4 = read_White_eeprom[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
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
//Función de movimiento de servos 
//*****************************************************************************

void Servos_move(void){
	// Bucle infinito para controlar el movimiento de los servos
	while (1) {
		// Verificar si se ha detectado una señal en alguno de los pines específicos
		if (!(PIND & (1 << DDD3)) || !(PIND & (1 << DDD4)) || !(PIND & (1 << DDD7)) || !(PINB & (1 << DDB0))) {
			// Si se ha detectado una señal, salir del bucle
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

		// Retraso antes de la próxima iteración del bucle
		_delay_ms(25);
	}
}


//*****************************************************************************
//Interrupciones 
//*****************************************************************************

// Rutina de interrupción para manejar la recepción de datos por UART
ISR(USART_RX_vect){
	
	bufferRX = UDR0;
	buffer_RX2 = UDR0;
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = bufferRX;
	UDR0 = buffer_RX2;


       /* char receivedChar = UDR0;

        // Procesar el comando recibido
        buffer[bufferIndex++] = receivedChar;

        // Si hemos recibido suficiente información para un comando
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


// Rutina de servicio de interrupción (ISR) para el temporizador TIMER2 en modo de comparación A
ISR(TIMER2_COMPA_vect) {
	// Incrementar el contador de tiempo del botón 4 si está presionado
	if (buttonPress4) {
		buttonTime4++;
	}
	// Incrementar el contador de tiempo del botón 1 si está presionado
	else if (buttonPress1) {
		buttonTime1++;
	}
	// Incrementar el contador de tiempo del botón 2 si está presionado
	else if (buttonPress2) {
		buttonTime2++;
	}
	// Incrementar el contador de tiempo del botón 3 si está presionado
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
	TIMSK2 = (1 << OCIE2A); // Habilitar interrupción de comparación
	TCCR2B = (1 << CS22); // Prescaler 64
}



//*****************************************************************************
//Funciones ADAFRUIT :(
//*****************************************************************************

/*void readTwoBytes(char highByte, char lowByte) {
	return (highByte << 8) + lowByte;
}*/




