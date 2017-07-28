/*
 * MotorCtrl.cpp
 *
 * Created: 24.06.2017 16:49:19
 * Author : joel
 * 
 * Steuert den Stellmotor für die Bremse beim Crosstrainer Kettler Mondeo. (Original Elektronik defekt)
 * Input : + und - Tasten, Abgriff vom Poti der Stellmechanik
 * Output: MotorLinks und Motor Rechts, ev. Indikator-LED über Reset-Pin ? ==> Nein
 *
 * Unsauber: PB0 und PB1 sind vertauscht. diverse andere sind ebenfalls verpolt. 
 * Müsste überarbeitet werden. 
 * 
 * Die Poti-Spannung wird mit nur 8bit ADC auf 0..255 gemappt.
 * davon kann aber nur ein Teilbereich ausgenutzt werden, 
 * Einschränkungen ergeben sich durch PotiLimit[Upper/Lower]
 * Der Stellwert bewegt sich im identischen bereich.
 * Der Stellwert kann durch die Tasten verändert werden
 * Es findet ein Ständiger Abgleich statt zwischen ist-soll
 * entsprechend wird der Motor angesteuert.
 * Test 28.07.2017: "+" Button zeiht die Bremse an. Vpot sinkt!
 * Stellwert = 0 --> Bremse wird stark angezogen
 * Stellwert = 255 --> Bremse gelöst

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
#define PinVin		PB2				// Eingang ADC (v0 = max bremse; vmax = bremse gelöst)
#define PinMotP		PB4				// Ausgang	
#define PinMotN		PB3				// Ausgang
#define PinLED		PB5				// optionaler Ausgang

#define vRef				5.00	// [V]
#define PotiVoltageUpper	4.97	// [V]
#define PotiVoltageLower	0.03	// [V]
#define PotiLimitUpper		((PotiVoltageUpper/vRef)*255)		// nach ADC obere Limite für Poti
#define PotiLimitLower		((PotiVoltageLower/vRef)*255)		// nach ADC untere Limite für Poti	
#define debounceCounter		40		// Konstante zum Taster entprellen	std = 60 oder für sw-debug std = 1 (0..255)
#define debounceSpecial		100		// Taster entprellen, wenn beide Tasten gefrückt werden.	
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
void startTimer();
void stopTimer();
uint8_t program();

ISR(TIMER0_OVF_vect) {
	// cpu läuft auf internen 8Mhz / 8 = 1 MHz. (Fuses CKDIV8 und CKSEL)
	// 1 mhz/256/256/15 = 1hz
	static uint16_t i = 0;		// wird alle 256*256/cpuf Sekunden inkrementiert.
	i++;
	if (i >= 15) {			// Es ist eine Test-Sekunde vergangen	
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
	volatile uint8_t toleranz = 4;
	volatile static int16_t check_hi = 0;
	volatile static int16_t check_lo = 0;

	init();
	initADC();

    while (1) 
    {
		delta = keyGain * readKeys(); 
		// userwert darf nicht davonziehen:
		check_hi = 255 - delta;		// check_hi ist 16bit signed!
		check_lo =   0 - delta;		// check_lo ist 16bit signed!
		if (check_hi < userwert ) {
			userwert = 255;
		} else if (check_lo > userwert ){
			userwert = 0;
		} else {
			userwert += delta;
		}

		// Ist-Wert einlesen, (ADC Left Adjust Result, deshalb nur ein 8bit Register lesen)
		istwert = ADCH;


		// Modus berücksichtigen
		if (modus == 0) {				// Bremse manuell verstellen
			sollwert = userwert;
		} else	{						// Programm läuft
			sollwert = 	program();	
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
 * 
 * Keystates: 
 * 0: nichts
 * 1: + gedrückt, abwarten
 * 2: - gedrückt, abwarten
 * 3: + und - gleichzeitig gedrückt, abwarten
 * 4: + und - als gültig erkannt. erneutes + und - erst nach loslassen möglich.
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
	
	// Nach ModeToggle müssen erst beide Tasten losgelassen werden.
	if (keyState == 4){
		if (~up && ~down && keyCounter >= debounceSpecial) {
			keyState = 0;
			return(0);
		} else {
			return(0);
		}
	}


	

	// Es wurde zuvor noch nichts gedrückt
	if (keyState == 0) {		
		
		if (up && down) {				// Beide Tasten gleichzeitig --> modus wechseln.
			keyState = 3;
			keyCounter = 0;
			return(0);
		} else if (up && ~down) {	 
			keyState= 1;
			keyCounter = 0;
			return(0);
		} else if (down && ~up) {
			keyState= 2;
			keyCounter = 0;
			return(0);
		}
	} 

	// Meist ist es so, dass wenn beabsichtigt wird, beide Buttons zu drücken,
	// diese nicht exakt gleichzeitig gedrückt werden --> keyState != 3. Deshalb
	// wird der hier überschrieben, aber nur wenn nicht zuerst losgelassen 
	// werden muss (keyState 4)
	
	if (keyState !=3 && keyState !=4 && up && down) {
		keyState = 3;
		keyCounter = 0;
		return(0);
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
			keyState = 4;
			toggleMode();
			return(0);
		} else {
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
	cli();
	if (modus == 0) {
		modus = 1;
		sekunden = 0;
		startTimer();
		sei();

	 } else {
		modus = 0;
		stopTimer();
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

void startTimer(){
// konfiguriert und startet den timer, damit der Programmfortschritt bekannt ist.
    
	TCCR0B |= (1 << CS02);		// prescale CPU-Takt / 256
    TIMSK  |= (1 << TOIE0);		// TOIE0: Timer/Counter0 Overflow Interrupt Enable
	TIFR   |= (1 << TOV0);		// TIFR – Timer/Counter Interrupt Flag Register: set Timer/Counter0 Overflow Flag
}

void stopTimer(){
	TIMSK  &= ~(1 << TOIE0);	// TOIE0: Timer/Counter0 Overflow Interrupt disable
	TIFR   &= ~(1 << TOV0);		// TIFR – Timer/Counter Interrupt Flag Register: clear Timer/Counter0 Overflow Flag
	TCCR0B &= ~(1 << CS02);		// timer 0 stoppen
	TCCR0B &= ~(1 << CS01);
	TCCR0B &= ~(1 << CS00);
}




uint8_t program(){
// liefert einen stellwert für den zeitpunkt s
// s geht von 0s 200*12s 
// Stellwertbereich 0..255
static const uint8_t prgData[] = { 
	0xFF, 0x00, 0xFF, 0xE7, 0xE1, 0xDF, 0xDE, 0xDD,
	0xDC, 0xDA, 0xD9, 0xD8, 0xD6, 0xD5, 0xD4, 0xD3,
	0xD1, 0xD0, 0xCF, 0xCD, 0xC4, 0xAA, 0xA6, 0xA1,
	0x9D, 0x99, 0x95, 0x91, 0x8D, 0x89, 0x86, 0x83,
	0x80, 0x7E, 0x7C, 0x7E, 0x81, 0x82, 0x84, 0x86,
	0x87, 0x88, 0x89, 0x8A, 0x8A, 0x8B, 0x8B, 0x8B,
	0x8B, 0x8A, 0x89, 0x89, 0x87, 0x86, 0x85, 0x84,
	0x81, 0x80, 0x7E, 0x7B, 0x7A, 0x77, 0x75, 0x72,
	0x70, 0x6D, 0x6A, 0x64, 0x5E, 0x55, 0x4B, 0x3F,
	0x71, 0xA2, 0xA6, 0xA9, 0xAC, 0xB0, 0xB3, 0xB7,
	0xBA, 0xBE, 0xC1, 0xC4, 0xC7, 0xCB, 0xCE, 0xD1,
	0xD4, 0xD7, 0xDA, 0xDD, 0xE0, 0xE2, 0xE5, 0xE7,
	0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF3, 0xF4, 0xF6,
	0xF7, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFB, 0xFB,
	0xFA, 0xFA, 0xF9, 0xF8, 0xF7, 0xF6, 0xF4, 0xF2,
	0xF1, 0xEE, 0xEC, 0xE9, 0xE7, 0xE4, 0xE1, 0xDD,
	0xDB, 0xD7, 0xD3, 0xCE, 0xCA, 0xC6, 0xC2, 0xBD,
	0xB8, 0xB3, 0xAE, 0xA8, 0xA2, 0x9D, 0x97, 0x92,
	0x8C, 0x86, 0x80, 0x79, 0x74, 0x6E, 0x67, 0x61,
	0x5B, 0x55, 0x4F, 0x49, 0x43, 0x3D, 0x37, 0x31,
	0x2D, 0x27, 0x22, 0x1D, 0x19, 0x14, 0x10, 0x0C,
	0x0A, 0x06, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x04, 0x06, 0x0B, 0x0F, 0x15,
	0x1D, 0x29, 0x3A, 0x4F, 0x65, 0x85, 0xA6, 0xBE,
	0xCD, 0xD6, 0xDC, 0xE3, 0xE9, 0xEF, 0xF6, 0xFA
};

uint8_t p = sekunden/12;
if (p >= 0 && p <=200) {  // p MUSS innerhalb 0..200 liegen.
	return(prgData[p]);
} else {
	return(127);
}

} 