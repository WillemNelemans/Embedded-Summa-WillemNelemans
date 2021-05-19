/* 
 * Summa Embedded Design Library vs 31-3-2021
 * LIB_ED_JV.c
 *
 * Created: 31-3-21
 * Author : john
 */ 

#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include <stdlib.h>
#include "PWM_Toon_uitgangen.h"


#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define PULL_UP 2
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5

// instellingen uitgangen voor PWM0.PWM1, PWM2 en Toon zie bestand "PWM_Toon_uitgangen.h"

// ExtIO MCP23S17 Registers Definition for BANK=0 (default)
#define IODIRA 0x00
#define IODIRB 0x01
#define IOCONA 0x0A
#define GPPUA  0x0C
#define GPPUB  0x0D
#define GPIOA  0x12
#define GPIOB  0x13
#define ExtIO_A0 0
#define ExtIO_A1 1
#define ExtIO_A2 2
#define ExtIO_A3 3
#define ExtIO_A4 4
#define ExtIO_A5 5
#define ExtIO_A6 6
#define ExtIO_A7 7
#define ExtIO_B0 8
#define ExtIO_B1 9
#define ExtIO_B2 10
#define ExtIO_B3 11
#define ExtIO_B4 12
#define ExtIO_B5 13
#define ExtIO_B6 14
#define ExtIO_B7 15
#define ExtIO_A8 16
#define ExtIO_A9 17
#define ExtIO_A10 18
#define ExtIO_A11 19
#define ExtIO_A12 20
#define ExtIO_A13 21
#define ExtIO_A14 22
#define ExtIO_A15 23
#define ExtIO_A16 32
#define ExtIO_A17 33
#define ExtIO_A18 34
#define ExtIO_A19 35
#define ExtIO_A20 36
#define ExtIO_A21 37
#define ExtIO_A22 38
#define ExtIO_A23 39
#define ExtIO_B0 8
#define ExtIO_B1 9
#define ExtIO_B2 10
#define ExtIO_B3 11
#define ExtIO_B4 12
#define ExtIO_B5 13
#define ExtIO_B6 14
#define ExtIO_B7 15
#define ExtIO_B8 24
#define ExtIO_B9 25
#define ExtIO_B10 26
#define ExtIO_B11 27
#define ExtIO_B12 28
#define ExtIO_B13 29
#define ExtIO_B14 30
#define ExtIO_B15 31
#define ExtIO_B16 40
#define ExtIO_B17 41
#define ExtIO_B18 42
#define ExtIO_B19 43
#define ExtIO_B20 44
#define ExtIO_B21 45
#define ExtIO_B22 46
#define ExtIO_B23 47
// stel in output voor cs extio chip
#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_CS   0
// MCP23S17 SPI Slave Device
#define SPI_SLAVE_ID    0x40
#define SPI_SLAVE_WRITE 0x00
#define SPI_SLAVE_READ  0x01

#define MCP23S17_EMULATION 0

//MCP4922 DAC
#define DAC_KanaalA 0			// tbv selectie kanaal A van DAQ MCP4922
#define DAC_KanaalB 1			// tbv selectie kanaal B van DAQ MCP4922
#define MCP4922 4				//12 bits 2 kanaals DAC
#define MCP4912 2				//10 bits 2 kanaals DAC
#define MCP4902 1				//8 bits 2 kanaals DAC
#define MCP4921 4				//12 bits 1 kanaals DAC
#define MCP4911 2				//10 bits 1 kanaals DAC
#define MCP4901 1				//8 bits 1 kanaals DAC
//stel in output pin voor cs van DAC
#define NotCS_DAC 1				// DAQ 4922 CS selected(0) of not selected(1) /use port of port DAC_PORT
#define DAC_DDR DDRB
#define DAC_PORT PORTB
#define NotAofB 15				// DAQ 4922 knaalA (0) of kanaalB(1)
#define Buf 14					// DAQ 4922 ongebufferd (0) of gebufferd(1)
#define NotGA 13				// DAQ 4922 1xGain(0) of 2xGain(1)
#define NotSHDN 12				// DAQ 4922 active(0) of shutdown(1)
#define ShiftByte 8				// Byte shift

extern uint8_t PWM0_Aan;		//Timer0-A
extern uint8_t PWM1_Aan;		//Timer0-B
extern uint8_t PWM2_Aan;		//Timer2-B
extern uint8_t TOON_Aan;		// Timer1
extern uint32_t msTeller;	//telt de milliseconds na opstart max ong 50 dagen dan teller weer 0 Timer2-A
extern uint16_t T10Teller;   //10ste van een ms
extern uint8_t PWM_is_Set;	//setpwm execute once
extern uint8_t AnaloogIn_is_Set;	//AnaloogInSetup execute once
extern uint8_t LCD_is_Set;	//LCDinit en LCDclr execute once
extern uint16_t TempFrequentie;  // tbv functie toon
extern uint8_t DAC_is_Set; // setup only once
extern uint8_t EXT_IO_is_Set; // setup only once
extern int32_t PulsPos; // bepalen waar de stappenmotor is
extern uint8_t DirStepper; //  richting stepper

//mcp23s17
extern uint8_t SPI_SLAVE_ADDR;     // A2=0,A1=0,A0=0
extern uint8_t	ExtIOA0_ddr;
extern uint8_t	ExtIOB0_ddr;
extern uint8_t	ExtIOA1_ddr;
extern uint8_t	ExtIOB1_ddr;
extern uint8_t	ExtIOA2_ddr;
extern uint8_t	ExtIOB2_ddr;
extern uint8_t ExtIOA0_PullUp;
extern uint8_t ExtIOB0_PullUp;
extern uint8_t ExtIOA1_PullUp;
extern uint8_t ExtIOB1_PullUp;
extern uint8_t ExtIOA2_PullUp;
extern uint8_t ExtIOB2_PullUp;
extern uint8_t ExtIOA0_Write;
extern uint8_t ExtIOB0_Write;
extern uint8_t ExtIOA1_Write;
extern uint8_t ExtIOB1_Write;
extern uint8_t ExtIOA2_Write;
extern uint8_t ExtIOB2_Write;
extern uint8_t ExtIOA_Read;
extern uint8_t ExtIOB_Read;

//DE routines:
void INIT_SPI_DAC(void);
// Initialisatie DAC, zonder parameters

unsigned char spi_tranceiver (uint8_t data);
// Deze routine wordt opgeroepen in routine Zend_naar_DAQ. Beter niet zelfstandig gebruiken.
// Deze routine stuurt de controle/datawaarden (byte) naar de DAC via SPI.
// De pre-parameter is de te versturen data, post-parameter heeft geen toepassing in dit programma

void Analog_Write_DAC(uint16_t Analog_Uit_Value, uint8_t Kanaal, uint8_t Type_dac);   //DAC 4922 heeft 2 kanalen (uitgangen)
// Verder zijn er zes types mcp: MCP4921/MCP4922 12bits, MCP4911/MCP4912 10 bits en MCP4901/MCP4902 8 bits
// Deze routine voert de volledige stappen uit om een waarde (tussen 1 en 100) te sturen naar de DAC. Hij gebruikt routine spi_tranceiver.
// De pre_parameters zijn de analoge 8bits waarde , de tweede pre-parameter is op het gewenste kanaal van de DAC te kiezen,
// er zijn twee analoge output kanalen. Deze routine heeft geen post-parameter.

void SPI_Write(unsigned char addr,unsigned char data);
//hulp functie voor EXTIO via SPI

unsigned char SPI_Read(unsigned char addr);
//hulp functie voor EXTIO via SPI

void INIT_SPI_EXT_IO(void);
// Initialisatie ext-IO, zonder parameters

uint8_t Digital_Read_Ext_IO(uint8_t PortPin);
//Uitlezen van een poortpinextio
// Adressering 0 -15 is A0-B7


void Digital_Write_Ext_IO(uint8_t PortPin,uint8_t Value);
//Wegschrijven van een 1 of 0 naar een output pin van extIO
// Adressering 0 -15 is A0-B7


void Set_Pinmode_Ext_IO(uint8_t PortPin,uint8_t IO_Choice);
// Set the Pin of port PB on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
// Adressering 0 -15 is A0-B7

// Deze functie bepaald een delay in msec , Het aantal ms is afhankelijk van de parameter Hoeveelmsdelay. max delay 65000ms
void Delay_ms(uint16_t Hoeveelmsdelay);

// Deze routine stelt de analoge poorten in voor de zes adc ingangen (AD0-AD5)
// Geen parameters nodig
void AnaloogInSetup(void);

//Inlezen analoge ingang en omzetten in een uint8_t variabele
// PreParameter kanaal is de uint8_t variabele waarmee gekozen kan worden welk kanaal (0-5) wordt uitgelezen
uint8_t Analoog_Read_8bit (uint8_t kanaal);

// Deze routine stuurt een tekststring (woord of string van max. 40 karakters) naar de gewenste locatie op het display
// Pre-parameters zijn de tekststring, gewenste xpositie (0 t/m 15) en y-positie(0 of1 tbv regel)
// Deze routine heeft geen post-parameter.
void TekstNaarDisplay(char tekst[40], uint8_t xpos, uint8_t ypos);

// maximale waarde en minimale waarde  moet een geheel getal zijn kleiner dan 999
// Deze waarde zet een analoge waarde op een gewenste plek naar het display.
// De analogewaarde 8-bits (o-255) wordt omgezet naar de gewenst max-woorden
// pre-parameter zijn de 8-bits analogewaarde, de maxwaarden (tbv omzetting naar 0-maxwaarden), gewenste xpositie (0 t/m 15) en y-positie(0 of1 tbv regel)
// Geen post-parameters in deze routine.
void Analoog_naar_LCD(uint8_t AnaWaarde,uint32_t MaxWaarde, uint32_t MinWaarde, uint8_t xpos, uint8_t ypos);

//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PB(uint8_t PortPin);

//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PC(uint8_t PortPin);

//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PD(uint8_t PortPint);

//Wegschrijven van een 1 of 0 naar een output pin van PB
void Digital_Write_PB(uint8_t PortPin,uint8_t Value);

//Wegschrijven van een 1 of 0 naar een output pin van PC
void Digital_Write_PC(uint8_t PortPin,uint8_t Value);

//Wegschrijven van een 1 of 0 naar een output pin van PD
void Digital_Write_PD(uint8_t PortPin,uint8_t Value);

// Set the Pin of port PB on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PB(uint8_t PortPin,uint8_t IO_Choice);

// Set the Pin of port PC on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PC(uint8_t PortPin,uint8_t IO_Choice);

// Set the Pin of port PD on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PD(uint8_t PortPin,uint8_t IO_Choice);

// Een Decimaal (komma of punt)  invoegen bij een tekststring
// bv. aanroep in main:
//	AnaloogString='1295';
//	Decimaal_Invoegen(AnaloogString,1,',');
// Analoogstring wordt "129,5"
void Decimaal_Invoegen(char Tekststring[17], uint8_t Decimaals_na_decimaal_kar, char Decimaal_kar);

// zet een Analogewaarde om in een tekststring van max 16 karakters, geef aan hoeveel karakters moeten worden weergegeven
// geef ook aan wat als aanvulling voor het getal moet worden weergegeven bv "00321""  of "  321" of "321"
// kleinste tekst min één 0
void Analoog_naar_string(char Tekststring[16], uint32_t Getal, uint8_t MinAantalKar, char Aanvulling);

// Functie verschaald een Waarde van tussen de ValueMin en ValueMax naar een outputparameter (Verschaal)
// tussen RangeMin en RangeMax. both max's and Value min = 0 and max =1024
uint32_t Verschaal(uint32_t Value,uint32_t ValueMin,uint32_t ValueMax,uint32_t RangeMin,uint32_t RangeMax);

// instellen en activeren msTeller via timer2. Stelt ook andere timers in voor PWM en toon().
void SetmsTeller();  

// instellen en activeren van de toon functie via timer2. Stelt ook andere timers in voor PWM en msTeller.
void SetToon();


// instellen PWM en TOON via timer0,1 en 2 
void SetPWM();		// instellen PWM via timer0

// Elke 0,5usec wordt TCNT1 1 hoger, afhankelijk van gewenste frequentie wordt bij een bepaalde waarde de uitgang hoog en laag
// De dutycycle == 50%. De ingestelde waarde is 2000000/frequentie. maar  bij f<100Hz 15625/frequentie.
//Maximale frequentie waarbij afwijking binnen de 1% licht is 20kHz
void Toon(uint16_t Frequentie);

//Set the pwm port dutycycle (0-255
void Analog_Write_PWM(uint8_t PWMIndex,uint8_t Value);
