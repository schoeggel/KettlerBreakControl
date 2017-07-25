/*
 * GccApplication1.cpp
 *
 * Created: 24.06.2017 16:49:19
 * Author : joel
 * 
 * Steuert den Stellmotor für die Bremse beim Crosstrainer Kettler Mondeo. (Original Elektronik defekt)
 * Input : + und - Tasten, Abgriff vom Poti der Stellmechanik
 * Output: MotorLinks und Motor Rechts, ev. Indikator-LED über Reset-Pin ?
 *
 * Die Poti-Spannung wird mit nur 8bit ADC auf 0..255 gemappt.
 * davon kann aber nur ein Teilbereich ausgenutzt werden, 
 * Einschränkungen ergeben sich durch PotiLimit[Upper/Lower]
 * Der Stellwert bewegt sich im identischen bereich.
 * Der Stellwert kann durch die Tasten verändert werden
 * Es findet ein Ständiger Abgleich statt zwischen ist-soll
 * entsprechend wird der Motor angesteuert.
 * 
 * Einige der vars sind volatile, damit der compiler sie nicht wegoptimiert. 
 * Sonst sind sie im Debugmode (Simulator) nicht mehr sichtbar. 
 * der DC motor kommt an einen L793  IC ==> Ansteuerung , pins etc TODO !!
 */ 

#include <avr/io.h>

// Pin definition
#define PinPlus		PB0				// Eingang
#define PinMinus	PB1				// Eingang
#define PinVin		PB2				// Eingang ADC
#define PinMotP		PB4				// Ausgang	
#define PinMotN		PB3				// Ausgang
#define PinLED		PB5				// optionaler Ausgang

#define vRef				5.0		// [V]
#define PotiVoltageUpper	4.9		// [V]
#define PotiVoltageLower	0.1		// [V]
#define PotiLimitUpper		((PotiVoltageUpper/vRef)*255)		// nach ADC obere Limite für Poti
#define PotiLimitLower		((PotiVoltageLower/vRef)*255)		// nach ADC untere Limite für Poti	
#define debounceCounter		60		// Konstante zum Taster entprellen	std = 50 oder für sw-debug std = 1
#define keyGain				1		// Ein Tastendruck ändert den Sollwert um (+/-)*keyGain, max. 127


// functions
void init();
void initADC();
void dbgPulse(int pulses);
int8_t readKeys();
int main();



int main(void)
{

	volatile int8_t delta = 0;
	volatile uint8_t sollwert = 128;
	volatile uint8_t istwert = 127;
	volatile uint8_t toleranz = 5;

	init();
	initADC();

    while (1) 
    {
		delta = keyGain * readKeys();  // TODO: Overflow und co !
		sollwert += delta;

		// stellwert Limitieren
		if (sollwert < PotiLimitLower) {
			sollwert = PotiLimitLower;
			} else if (sollwert > PotiLimitUpper) {
			sollwert = PotiLimitUpper;
		}


		// Ist-Wert einlesen, (ADC Left Adjust Result, deshalb nur ein 8bit Register lesen)
		istwert = ADCH;


		// Motor steuer Signal
		if ((sollwert - istwert) > toleranz) {
			// motor + signal geben
			// mehr bremsen
			PORTB |= (1 << PinMotP);	//PinMotP im PORTB setzen
	        PORTB &= ~(1 << PinMotN);	//PinMotN im PORTB löschen

		} else if  ((istwert - sollwert) > toleranz) {
			// motor - signal geben
			// weniger bremsen
			PORTB |= (1 << PinMotN);	//PinMotN im PORTB setzen
			PORTB &= ~(1 << PinMotP);	//PinMotP im PORTB löschen

		} else {
			// motor soll nicht laufen
			PORTB &= ~(1 << PinMotN);	//PinMotN im PORTB löschen
			PORTB &= ~(1 << PinMotP);	//PinMotP im PORTB löschen
		}



	}
}


int8_t readKeys() {
/* Liefert +1 , 0 , oder -1 zurück, je nach Tastendruck 
 * debounceCycles: so oft muss die funktion durchlaufen werden, bevor ein 
 * Tastendruck gültig sein kann. 
 * 
 * Keystates: 
 * 0: nichts
 * 1: + gedrückt, abwarten
 * 2: - gedrückt, abwarten
 *
 */
	static uint16_t keyCounter = 0;
	static uint8_t keyState = 0;
	volatile static bool up  = 0;
	volatile static bool down = 0;

	// Eingänge lesen
	up	=  PINB & (1 << PinPlus);
	down = PINB & (1 << PinMinus);
	++keyCounter;

	// Beide Tasten gleichzeitig --> ungültig.
	if (up & down) {
		keyState = 0;
		return(0);
	}

	// Es wurde zuvor noch nichts gedrückt
	if (keyState == 0) {
		if (up) {
			keyState= 1;
			keyCounter = 0;
			return(0);
		} else if (down) {
			keyState= 2;
			keyCounter = 0;
			return(0);
		}
	}

	// up gedrückt, am warten
	if ((keyState == 1) & (keyCounter >= debounceCounter)) {
		if (up) {
			keyState = 0;
			keyCounter = 0;
			dbgPulse(4);
			return(1);
		} else {
			keyState = 0;
			keyCounter = 0;
			return(0);
		}
	}

		
	// down gedrückt, am warten
	if ((keyState == 2) & (keyCounter >= debounceCounter)) {
		if (down) {
			keyState = 0;
			keyCounter = 0;
			dbgPulse(7);
			return(-1);
			} else {
			keyState = 0;
			keyCounter = 0;
			return(0);
		}
	}
	return(0);
}

void dbgPulse(int pulses){
	// macht pulse am motP ausgang 
	for (int i = 0; i< pulses; i++){
	PORTB |= (1 << PinMotP);	//PinMotP im PORTB setzen
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	PORTB &= ~(1 << PinMotP);	//PinMotP im PORTB löschen
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");	
	}

}


void init(){
	// Als Eingang konfigurieren (Eingang: clear bit)
	DDRB &= ~(1 << PinPlus);
	DDRB &= ~(1 << PinMinus);
	DDRB &= ~(1 << PinVin);

	// Für die Taster die internen PullUp Resistors aktivieren
	// richtig: set bit:
	PORTB |= (1 << PinPlus);
	PORTB |= (1 << PinMinus);


	
	//Als Ausgang konfigurieren: (Ausgang: set bit)
	DDRB |= (1 << PinMotP);
	DDRB |= (1 << PinMotN);
	DDRB |= (1 << PinLED);
}


void initADC()
{
  /* this function initialises the ADC 

        ADC Prescaler Notes:
	--------------------

	   ADC Prescaler needs to be set so that the ADC input frequency is between 50 - 200kHz.
  
           For more information, see table 17.5 "ADC Prescaler Selections" in 
           chapter 17.13.2 "ADCSRA – ADC Control and Status Register A"
          (pages 140 and 141 on the complete ATtiny25/45/85 datasheet, Rev. 2586M–AVR–07/10)

           Valid prescaler values for various clock speeds
	
	     Clock   Available prescaler values
           ---------------------------------------
             1 MHz   8 (125kHz), 16 (62.5kHz)
             4 MHz   32 (125kHz), 64 (62.5kHz)
             8 MHz   64 (125kHz), 128 (62.5kHz)
            16 MHz   128 (125kHz)

           Below example set prescaler to 128 for mcu running at 8MHz
           (check the datasheet for the proper bit values to set the prescaler)
  */

  // 8-bit resolution
  // set ADLAR to 1 to enable the Left-shift result (only bits ADC9..ADC2 are available)
  // then, only reading ADCH is sufficient for 8-bit results (256 values)

  ADMUX =
            (1 << ADLAR) |		// left shift result
            (0 << REFS1) |		// Sets ref. voltage to VCC, bit 1 (tiny45 != mega8)
            (0 << REFS0) |		// Sets ref. voltage to VCC, bit 0 (tiny45 != mega8)
            (0 << MUX3)  |		// use ADC2 for input (PinVin=PB2), MUX bit 3
            (0 << MUX2)  |		// use ADC2 for input (PinVin=PB2), MUX bit 2
            (0 << MUX1)  |		// use ADC2 for input (PinVin=PB2), MUX bit 1
            (1 << MUX0);		// use ADC2 for input (PinVin=PB2), MUX bit 0

  ADCSRA = 
            (1 << ADEN)  |		// Enable ADC 
            (1 << ADSC)  |		// start conversion
			(1 << ADATE) |		// ADC auto trigger enable (trigger select in ASCSRB)
			(1 << ADPS2) |		// set prescaler to 64, bit 2 
            (1 << ADPS1) |		// set prescaler to 64, bit 1 
            (0 << ADPS0);		// set prescaler to 64, bit 0  
					
	ADCSRB =					// Set trigger source = Free Running mode (000)
			 (0 << ADTS2)  |	// ADC Auto Trigger Source Bit 2
			 (0 << ADTS1)  |	// ADC Auto Trigger Source Bit 1
			 (0 << ADTS0);		// ADC Auto Trigger Source Bit 0
}


