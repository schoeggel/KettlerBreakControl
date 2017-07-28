/*
 * MotorCtrl.cpp
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
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Pin definition
#define PinPlus		PB1				// Eingang
#define PinMinus	PB0				// Eingang
#define PinVin		PB2				// Eingang ADC
#define PinMotP		PB4				// Ausgang	
#define PinMotN		PB3				// Ausgang
#define PinLED		PB5				// optionaler Ausgang

#define vRef				5.0			// [V]
#define PotiVoltageUpper	4.94		// [V]
#define PotiVoltageLower	0.06		// [V]
#define PotiLimitUpper		((PotiVoltageUpper/vRef)*255)		// nach ADC obere Limite für Poti
#define PotiLimitLower		((PotiVoltageLower/vRef)*255)		// nach ADC untere Limite für Poti	
#define debounceCounter		30		// Konstante zum Taster entprellen	std = 60 oder für sw-debug std = 1 (0..255)
#define debounceSpecial		200		// Taster entprellen, wenn beide Tasten gefrückt werden.	
#define keyGain				1		// Ein Tastendruck ändert den Sollwert um (+/-)*keyGain, max. 127

// global var
uint8_t modus = 0;				// modus 0 = standard ; modus 1 = programm
uint16_t sekunden = 0;

// functions
void init();
void initADC();
int8_t readKeys();
int main();
void toggleMode();
void initTimer();
uint8_t program(uint16_t s);

ISR(TIMER0_OVF_vect) {
	// cpu clock 8 mhz (int)
	// 8 mhz/1024/7812 = 1hz
	static uint16_t i = 0;		// wird alle 1024 / cpuf Sekunden inkrementiert.
	i++;
	// TEST:
	if (i >= 700) {			// Es ist eine Test-Sekunde vergangen
	//if (i >= 7812) {			// Es ist eine Sekunde vergangen
	
		i = 0;
		sekunden++;
		if (sekunden > 200*12) {  // auf 40 Minuten limitieren
			sekunden = 200*12;
			toggleMode();
		}
	}
}


int main(void)
{

	volatile int8_t delta = 0;
	volatile uint8_t userwert = 128;
	volatile uint8_t sollwert = 128;
	volatile uint8_t istwert = 127;
	volatile uint8_t toleranz = 6;
	 

	init();
	initADC();
	initTimer();
	//toggleMode();	// debug: direkt im programm mode starten


    while (1) 
    {
		delta = keyGain * readKeys();  // TODO: Overflow und co !
		userwert += delta;

		// Ist-Wert einlesen, (ADC Left Adjust Result, deshalb nur ein 8bit Register lesen)
		istwert = ADCH;


		// Modus berücksichtigen
		if (modus == 0) {				// Bremse manuell verstellen
			sollwert = userwert;
		} else	{						// Programm läuft
			sollwert = 	program(sekunden) + delta;	
//			userwert = sollwert;	
		}




		// stellwert Limitieren
		if (sollwert < PotiLimitLower) {
			sollwert = PotiLimitLower;
			} else if (sollwert > PotiLimitUpper) {
			sollwert = PotiLimitUpper;
		}

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
 * 3: + und - gleichzeitig gedrückt, abwarten
 */
	static uint8_t keyCounter = 0;
	static uint8_t keyState = 0;
	volatile static bool up  = 0;
	volatile static bool down = 0;

	// Eingänge lesen
	// acive-low !
	// weshalb kommt hier ein up =1 statt up= 0 :  up	=  ~(PINB & (1 << PinPlus));
	up	=  (~PINB & (1 << PinPlus));
	down = (~PINB & (1 << PinMinus));
	

	if (keyState != 0) {
		++keyCounter;
	}
	

	// Es wurde zuvor noch nichts gedrückt
	if (keyState == 0) {		
		
		if (up & down) {				// Beide Tasten gleichzeitig --> modus wechseln.
			keyState = 3;
			keyCounter = 0;
			return(0);
		} else if (up & ~down) {	 
			keyState= 1;
			keyCounter = 0;
			return(0);
		} else if (down & ~up) {
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
			return(-1);
			} else {
			keyState = 0;
			keyCounter = 0;
			return(0);
		}
	}

	// up und down gedrückt, am warten
	if ((keyState == 3) & (keyCounter >= debounceSpecial)) {
		if (up & down) {
			keyState = 0;
			keyCounter = 0;
			toggleMode();
			return(0);
			} else {
			keyState = 0;
			keyCounter = 0;
			return(0);
		}
	}

	return(0);
}


 void toggleMode(){
/*	wechselt den Modus
	* Modus 0: Standard, manuelles Einstellen der Bremsen über + und -
	* Modus 1: Programm wird durchlaufen ca. 20 Minuten. Während dem Programm kann 
	*          der Schwierigkeitsgrad durch + und - Tasten verändert werden.
	* Modus > 1 : TODO ....
 */

	 if (modus == 0) {
		modus = 1;
		sekunden = 0;
		sei();

	 } else {
		modus = 0;
		cli();
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

void initTimer(){
// konfiguriert und startet den timer, damit der Programmfortschritt bekannt ist.
    
	TCCR0B |= (1<<CS02);		// prescale CPU-Takt / 1024
	TCCR0B |= (1<<CS00);		// prescale CPU-Takt / 1024
    TIMSK  |= (1<<TOIE0);		// TOIE0: Timer/Counter0 Overflow Interrupt Enable
	TIFR   |= (1<<TOV0);		// TIFR – Timer/Counter Interrupt Flag Register: set Timer/Counter0 Overflow Flag
}




uint8_t program(uint16_t s){
// liefert einen stellwert für den zeitpunkt s
// s geht von 0s 200*12s 
// Stellwertbereich 0..255
static const uint8_t prgData[] = { 
	0x03, 0x0A, 0x10, 0x17, 0x1D, 0x1F, 0x20, 0x21,
	0x22, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B,
	0x2D, 0x2E, 0x2F, 0x31, 0x3A, 0x54, 0x58, 0x5D,
	0x61, 0x65, 0x69, 0x6D, 0x71, 0x75, 0x78, 0x7B,
	0x7E, 0x80, 0x82, 0x80, 0x7D, 0x7C, 0x7A, 0x78,
	0x77, 0x76, 0x75, 0x75, 0x74, 0x74, 0x74, 0x74,
	0x74, 0x75, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A,
	0x7D, 0x7E, 0x80, 0x83, 0x84, 0x87, 0x89, 0x8C,
	0x8E, 0x91, 0x94, 0x9A, 0xA0, 0xA9, 0xB3, 0xBF,
	0x8D, 0x5C, 0x58, 0x55, 0x52, 0x4E, 0x4B, 0x47,
	0x44, 0x40, 0x3D, 0x3A, 0x37, 0x33, 0x30, 0x2D,
	0x2A, 0x27, 0x24, 0x21, 0x1E, 0x1C, 0x19, 0x17,
	0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0B, 0x0A, 0x08,
	0x07, 0x06, 0x06, 0x05, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A, 0x0C,
	0x0D, 0x10, 0x12, 0x15, 0x17, 0x1A, 0x1D, 0x21,
	0x23, 0x27, 0x2B, 0x30, 0x34, 0x38, 0x3C, 0x41,
	0x46, 0x4B, 0x50, 0x56, 0x5C, 0x61, 0x67, 0x6C,
	0x72, 0x78, 0x7E, 0x85, 0x8A, 0x90, 0x97, 0x9D,
	0xA3, 0xA9, 0xAF, 0xB5, 0xBB, 0xC1, 0xC7, 0xCD,
	0xD1, 0xD7, 0xDC, 0xE1, 0xE5, 0xEA, 0xEE, 0xF2,
	0xF4, 0xF8, 0xFB, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFE, 0xFD, 0xFA, 0xF8, 0xF3, 0xEF, 0xE9,
	0xE1, 0xD5, 0xC4, 0xAF, 0x99, 0x79, 0x58, 0x40,
	0x31, 0x28, 0x22, 0x1B, 0x15, 0x0F, 0x08, 0x05
};

uint8_t p = s/12;
return(prgData[p]);

} 