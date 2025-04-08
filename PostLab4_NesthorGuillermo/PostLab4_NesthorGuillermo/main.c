/*
 * Laboratorio4C_NesthorGuillermo.c
 *
 * Created: 1/04/2025 14:26:37
 * Author : NesthorGuillermo
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t counter_10ms;
uint8_t adc_value = 0;
uint8_t display_digit = 0;
uint8_t contador = 0;  


// Tabla de conversión para display de 7 segmentos
const uint8_t segment_map[] = {
	0b00100000, 0b01110111, 0b00011010, 0b00010001, 0b01000111, 0b10000011, 0b10000000, 0b00110111, 0b00000000, 0b00000101, 0b00000100, 0b11000000, 0b10101000, 0b01010000, 0b10001000,0b10001100
};

/****************************************/

// Function prototypes
void setup();
void initADC();
void update_displays();
void funcion();
void actualizar_leds();
void alarma();

/****************************************/

// Main Function
int main(void)
{
	setup();
	while (1)
	{
		funcion();
		alarma();
	}
}

/****************************************/
// NON-Interrupt subroutines
void setup()
{
	cli();
		// Configuración de puertos
		DDRB = 0xFF;
		DDRC |= (1 << PC2) | (1 << PC3); //salidas
		
		UCSR0B = 0;
		
		PORTB = 0x00;
		PORTC &= ~((1 << PC2) | (1 << PC3));
		
		// Configuración de botones
		DDRC &= ~((1 << PC4) | (1 << PC5));
		PORTC |= (1 << PC4) | (1 << PC5);
		
	// Configuración de puertos
	DDRD = 0xFF;    // PORTD para segmentos de 7 segmentos
	DDRC |= (1 << PC0) | (1 << PC1);  // PC0 y PC1 como salidas para control de displays
	
	// PC6 como entrada para el potenciómetro (ADC6)
	DDRC &= ~(1 << PC6);
	PORTC &= ~(1 << PC6);  // Sin pull-up
	
	// Configuración del timer para multiplexación de displays
	TCCR0A = 0x00;
	TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler de 1024
	TCNT0 = 100;  // Cargar contador
	TIMSK0 = (1 << TOIE0);  // Habilitar interrupción de desbordamiento
	
	counter_10ms = 0;
	
	contador = 0;
	initADC();
	
	sei();
}

/****************************************/
// Interrupt routines
ISR(TIMER0_OVF_vect)
{
	TCNT0 = 100;  // Reiniciar contador
	counter_10ms++;
	
	if (counter_10ms >= 1)  // Multiplexar displays cada ~1ms
	{
		counter_10ms = 0;
		update_displays();
	}
}

ISR(ADC_vect)
{
	adc_value = ADCH;  // Leer solo los 8 bits más significativos (por ADLAR)
	ADCSRA |= (1 << ADSC);  // Iniciar nueva conversión
}

/****************************************/

void initADC()
{
	ADMUX = 0;
	// Referencia AVcc, ajuste a izquierda (ADLAR), canal ADC6 (MUX2:1 = 01, MUX0 = 0)
	ADMUX |= (1 << REFS0) | (1 << ADLAR) | (1 << MUX2) | (1 << MUX1);
	ADCSRA = 0;
	// Habilitar ADC, interrupción, prescaler 8 (125kHz @ 1MHz)
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN) | (1 << ADIE);
	ADCSRA |= (1 << ADSC);  // Iniciar primera conversión
}

void update_displays()
{
	// Obtener mitad del valor ADC
	uint8_t high_nibble = (adc_value >> 4) & 0x0F;
	uint8_t low_nibble = adc_value & 0x0F;
	
	// Apagar ambos displays primero
	PORTC &= ~((1 << PC0) | (1 << PC1));
	
	if(display_digit == 0) {
		// Mostrar dígito más significativo (nibble alto)
		//ver si pd1 esta encendido (y no apagarlo por accidente)
		PORTD = segment_map[high_nibble];
		PORTC |= (1 << PC1);  // Habilitar display 1 (PC0)
		display_digit = 1;
		} else {
		// Mostrar dígito menos significativo (nibble bajo)
		PORTD = segment_map[low_nibble];
		PORTC |= (1 << PC0);  // Habilitar display 2 (PC1)
		display_digit = 0;
	}
}

// Function to update all LEDs based on contador
void actualizar_leds()
{
	
	PORTB = contador & 0x3F;
	
	// Actualizar PC2 y PC3 con los bits 6 y 7
	if (contador & (1 << 6))
	PORTC |= (1 << PC2);
	else
	PORTC &= ~(1 << PC2);
	
	if (contador & (1 << 7))
	PORTC |= (1 << PC3);
	else
	PORTC &= ~(1 << PC3);
}

/****************************************/

void funcion()
{
	// Botón PC4 presionado
	if (!(PINC & (1 << PC4)))
	{
		contador++;  // Incrementar contador de 8 bits
		actualizar_leds();
		while (!(PINC & (1 << PC4)));
	}

	// Botón PC5 presionado
	if (!(PINC & (1 << PC5)))
	{
		contador--;  // Decrementar contador de 8 bits
		actualizar_leds();
		while (!(PINC & (1 << PC5)));
	}
}

void alarma()
{
	if (adc_value >= contador)
	{
		PORTD |= (1 << PD1);
	}
	else
	{
		PORTD &= ~(1 << PD1);
	}
}