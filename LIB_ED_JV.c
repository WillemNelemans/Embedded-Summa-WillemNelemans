/* 
 * Summa Embedded Design Library vs 1.0
 * LIB_ED_JV.c
 *
 * Created: 20-2-2020 20:21:14
 * Author : john
 */ 

#include "LIB_ED_JV.h"

//variabele t.b.v. Functions

	uint8_t Verschuif_1_plaats=0;
	char Analoogtekst[40] ;
	uint8_t PWM0_Aan=0;		//Timer0-A
	uint8_t PWM1_Aan=0;		//Timer0-B
	uint8_t PWM2_Aan=0;		//Timer2-B
	uint8_t TOON_Aan;		// Timer1
	uint32_t msTeller;	//telt de milliseconds na opstart max ong 50 dagen dan teller weer 0 Timer2-A
	uint16_t T10Teller;   //10ste van een ms
	uint8_t PWM_is_Set;	//setpwm execute once
	uint8_t AnaloogIn_is_Set;	//AnaloogInSetup execute once
	uint8_t LCD_is_Set;	//LCDinit en LCDclr execute once
	uint16_t TempFrequentie;  // tbv functie toon
	uint16_t ExtIO_is_Set;    //ext_IO setup only once

	uint8_t DAC_is_Set; // setup only once
	int32_t PulsPos=0; // bepalen waar de stappenmotor is
	uint8_t DirStepper; //  richting stepper
//mcp23s17
	uint8_t SPI_SLAVE_ADDR = 0x00 ;     // A2=0,A1=0,A0=0
	uint8_t	ExtIOA0_ddr;
	uint8_t	ExtIOB0_ddr;
	uint8_t	ExtIOA1_ddr;
	uint8_t	ExtIOB1_ddr;
	uint8_t	ExtIOA2_ddr;
	uint8_t	ExtIOB2_ddr;
	uint8_t ExtIOA0_PullUp;
	uint8_t ExtIOB0_PullUp;
	uint8_t ExtIOA1_PullUp;
	uint8_t ExtIOB1_PullUp;
	uint8_t ExtIOA2_PullUp;
	uint8_t ExtIOB2_PullUp;
	uint8_t ExtIOA0_Write;
	uint8_t ExtIOB0_Write;
	uint8_t ExtIOA1_Write;
	uint8_t ExtIOB1_Write;
	uint8_t ExtIOA2_Write;
	uint8_t ExtIOB2_Write;
	uint8_t ExtIOA_Read;
	uint8_t ExtIOB_Read;
//DE routines:

void INIT_SPI_DAC(void)
// Initialisatie DAC, zonder parameters
{
	// Initial the AVR ATMega328 SPI Peripheral
	//Set MOSI (PB3),SCK (PB5) and CS (see NOTcs_DAC in .h file) as output, others as input
	DAC_DDR |= (1<<PB3)|(1<<PB5)|(1<<NotCS_DAC);

	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);
	SPSR |= (1<<SPI2X);
	
	DAC_PORT |= (1<<NotCS_DAC);  //Maak CS en LDAC initieel hoog;
	DAC_is_Set=1;	//initialisatie is uitgevoerd
}

unsigned char spi_tranceiver (uint8_t data)
// Deze routine wordt opgeroepen in routine Analog_Write_DAQ. Beter niet zelfstandig gebruiken.
// Deze routine stuurt de controle/datawaarden (byte) naar de DAC via SPI.
// De pre-parameter is de te versturen data, post-parameter heeft geen toepassing in dit programma
{
	SPDR = data;                       //Schrijf de gewenste data naar het zendregister, zenden van byte start
	while(!(SPSR & (1<<SPIF) ));       //Wacht totdat de zenden van de data klaar is
	{;}
	return(SPDR);                      //Ontvang de data-byte van de externe SPI module (bij de dac doen we hier niets mee)
}

void Analog_Write_DAC(uint16_t Analog_Uit_Value, uint8_t Kanaal, uint8_t Type_dac)   //DAC 4922 heeft 2 kanalen (uitgangen)
// Deze routine voert de volledige stappen uit om een analoge waarde te sturen naar de DAC. Hij gebruikt routine spi_tranceiver.
// De pre_parameters zijn de analoge 8bits waarde tussen 0 en 255 , de tweede pre-parameter is op het gewenste kanaal van de DAC te kiezen,
// er zijn twee analoge output kanalen. Deze routine heeft geen post-parameter.
{
	//unsigned char dummy=0;	//tijdelijke variabelen om compiler error te voorkomen
	uint16_t Analog_value  = Analog_Uit_Value * 4; // Analoge uitgang is 12 bit 0-4096
	uint16_t Datawrd=0;						//DAQwoord tbv spi DAQ
	if (DAC_is_Set==0) INIT_SPI_DAC();	//eerste keer initialisatie uitvoeren
	//zend twee bytes voor het activeren van een analoge waarde op de DAQ
	Datawrd |= (Kanaal<<NotAofB)|(1<<NotGA)|(1<<NotSHDN)|((Analog_value * Type_dac ) & 0x0FFF);  //controle bits DAC instellen
	DAC_PORT &= ~(1<<NotCS_DAC);						//CS laag maken
	
	//verstuur MSByte DAQwoord
	spi_tranceiver((Datawrd >> ShiftByte));  //bit 15 t/m 8
	//verstuur LSByte DAQwoord
	spi_tranceiver(Datawrd & 0xFF);		//bit7 t/m 0
	// Analoge waarde op de geselecteerde uitgang van de DAQ
	DAC_PORT |= (1<<NotCS_DAC);						//CS hoog
	Datawrd=0;
}

void SPI_Write(unsigned char addr,unsigned char data)
//hulp functie voor EXTIO via SPI
{
	// Activate the CS pin
	SPI_PORT &= ~(1<<SPI_CS);
	// Start MCP23S17 OpCode transmission
	SPDR = SPI_SLAVE_ID | ((SPI_SLAVE_ADDR << 1) & 0x0E)| SPI_SLAVE_WRITE;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	// Start MCP23S17 Register Address transmission
	SPDR = addr;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	// Start Data transmission
	SPDR = data;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	// CS pin is not active
	SPI_PORT |= (1<<SPI_CS);
}

unsigned char SPI_Read(unsigned char addr)
//hulp functie voor EXTIO via SPI
{
	// Activate the CS pin
	SPI_PORT &= ~(1<<SPI_CS);
	// Start MCP23S17 OpCode transmission
	SPDR = SPI_SLAVE_ID | ((SPI_SLAVE_ADDR << 1) & 0x0E)| SPI_SLAVE_READ;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	// Start MCP23S17 Address transmission
	SPDR = addr;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	// Send Dummy transmission for reading the data
	SPDR = 0x00;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	// CS pin is not active
	SPI_PORT |= (1<<SPI_CS);
	return(SPDR);
}

void INIT_SPI_EXT_IO(void)
// Initialisatie ext-IO, zonder parameters
{
	// Initial the AVR ATMega328 SPI Peripheral
	//Set MOSI (PB3),SCK (PB5) and CS (see spi_cs in .h file) as output, others as input
	SPI_DDR |= (1<<PB3)|(1<<PB5)|(1<<PB2)|(1<<SPI_CS);
	// CS pin is not active
	SPI_PORT |= (1<<SPI_CS);
	
	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(1<<SPR0);
	SPSR |= (0<<SPI2X);

	// Initial the MCP23S17 SPI I/O Expander
	SPI_Write(IOCONA,0x28);   // I/O Control Register: BANK=0, SEQOP=1, HAEN=1 (Enable Addressing)
	while(!(SPSR & (1<<SPIF)));	
	ExtIO_is_Set=1; //initialisatie is uitgevoerd
}

uint8_t Digital_Read_Ext_IO(uint8_t PortPin)
//Uitlezen van een poortpinextio
// Adressering 0 -15 is A0-B7
{	
	if (ExtIO_is_Set==0) INIT_SPI_EXT_IO();	//eerste keer initialisatie uitvoeren
	if ((PortPin>39) && (PortPin<48)) // poort B van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		ExtIOB_Read= SPI_Read(GPIOB);  // input waarde poort B uit chip lezen
		return !((ExtIOB_Read & (1<<(PortPin-40)))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	if ((PortPin>31) && (PortPin<40))  // Poort A van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		ExtIOA_Read= SPI_Read(GPIOA); // input waarde poort A uit chip lezen
		return !((ExtIOA_Read & (1<<(PortPin-32)))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	if ((PortPin>23) && (PortPin<32)) // poort B van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		ExtIOB_Read= SPI_Read(GPIOB);  // input waarde poort B uit chip lezen
		return !((ExtIOB_Read & (1<<(PortPin-24)))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	if ((PortPin>15) && (PortPin<24))  // Poort A van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		ExtIOA_Read= SPI_Read(GPIOA); // input waarde poort A uit chip lezen
		return !((ExtIOA_Read & (1<<(PortPin-16)))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	if ((PortPin>7) && (PortPin<16))  // poort B van extio adres 0
	{
		SPI_SLAVE_ADDR=0;
		ExtIOB_Read= SPI_Read(GPIOB);  // input waarde poort B uit chip lezen
		return !((ExtIOB_Read & (1<<(PortPin-8)))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	if ((PortPin<8))  // poort A van extio adres 0
	{	
		SPI_SLAVE_ADDR=0;
		ExtIOA_Read= SPI_Read(GPIOA); // input waarde poort A uit chip lezen
		return !((ExtIOA_Read & (1<<PortPin))==0);  // ingang is 0=0  en ingang is 1=1 als antwoord van functie
	}
	//DDRB &= ~(1<<PB2);
	return (2);
}

void Digital_Write_Ext_IO(uint8_t PortPin,uint8_t Value)
//Wegschrijven van een 1 of 0 naar een output pin van extIO
//// Adressering 0 -15 is A0-B7
{		
	if (ExtIO_is_Set==0) INIT_SPI_EXT_IO();	//eerste keer initialisatie uitvoeren
	if ((PortPin>39) && (PortPin<48)) // poort B van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		if (Value == 1) ExtIOB2_Write |= (1 << (PortPin-40));  // set pin output high
		else  ExtIOB2_Write &= ~(1<<(PortPin-40));  // set low
		SPI_Write(GPIOB,ExtIOB2_Write);  // write to mcp23s
	}
	if ((PortPin>31) && (PortPin<40))  // Poort A van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		if (Value == 1) ExtIOA2_Write |= (1 << (PortPin-32));  // set pin output high
		else  ExtIOA2_Write &= ~(1<<(PortPin-32));  // set low
		SPI_Write(GPIOA,ExtIOA2_Write);  // write to mcp23s17
	}
	if ((PortPin>23) && (PortPin<32)) // poort B van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		if (Value == 1) ExtIOB1_Write |= (1 << (PortPin-24));  // set pin output high
		else  ExtIOB1_Write &= ~(1<<(PortPin-24));  // set low
		SPI_Write(GPIOB,ExtIOB1_Write);  // write to mcp23s17
	}
	if ((PortPin>15) && (PortPin<24))  // Poort A van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		if (Value == 1) ExtIOA1_Write |= (1 << (PortPin-16));  // set pin output high
		else  ExtIOA1_Write &= ~(1<<(PortPin-16));  // set low
		SPI_Write(GPIOA,ExtIOA1_Write);  // write to mcp23s17
	}
	if ((PortPin>7) && (PortPin<16))  // PoortB van extio met adress 0
	{
		SPI_SLAVE_ADDR=0;
		if (Value == 1) ExtIOB0_Write |= (1 << (PortPin-8));  // set pin output high
		else  ExtIOB0_Write &= ~(1<<(PortPin-8));  // set low
		SPI_Write(GPIOB,ExtIOB0_Write);  // write to mcp23s17	
	}
	if ((PortPin<8))  //poortA van extio met adress 0
	{
		SPI_SLAVE_ADDR=0;
		if (Value == 1) ExtIOA0_Write |= (1 << PortPin);  // set pin output high
		else  ExtIOA0_Write &= ~(1<<PortPin);  // set low
		SPI_Write(GPIOA,ExtIOA0_Write);  // write to mcp23s17	
	}
}

void Set_Pinmode_Ext_IO(uint8_t PortPin,uint8_t IO_Choice)
// Set the Pin of port PB on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
// Adressering 0 -15 is A0-B7
{
	if (ExtIO_is_Set==0) INIT_SPI_EXT_IO();	//eerste keer initialisatie uitvoeren
	if ((PortPin>39) && (PortPin<48)) // poort B van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		if (IO_Choice == 1)		// controle of output
		{
			ExtIOB2_ddr &= ~(1<<(PortPin-40));  // 0 is output

		}
		else
		{
			ExtIOB2_ddr |= (1<<((PortPin-40)));  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOB2_PullUp |= (1<<(PortPin-40));  //pullup
			else ExtIOB2_PullUp &= ~(1<<(PortPin-40));  //geen pullup
		}
		SPI_Write(IODIRB,ExtIOB2_ddr);	//write ddr in/uit
		SPI_Write(GPPUB,ExtIOB2_PullUp);  //write pullup niet/wel
	}
	if ((PortPin>31) && (PortPin<40))  // Poort A van extio adres 2
	{
		SPI_SLAVE_ADDR=2;
		if (IO_Choice == 1)// controle of output
		{
			ExtIOA2_ddr &= ~(1<<(PortPin-32));  // 0 is output van adres 1

		}
		else   // ingang
		{
			ExtIOA2_ddr |= (1<<(PortPin-32));  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOA2_PullUp |= (1<<(PortPin-32));  //pullup
			else ExtIOA2_PullUp &= ~(1<<(PortPin-32));  //geen pullup
		}
		SPI_Write(IODIRA,ExtIOA2_ddr);	//write ddr in/uit
		SPI_Write(GPPUA,ExtIOA2_PullUp);   //write pullup niet/wel
	}
	if ((PortPin>23) && (PortPin<32)) // poort B van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		if (IO_Choice == 1)		// controle of output
		{
			ExtIOB1_ddr &= ~(1<<(PortPin-24));  // 0 is output

		}
		else
		{
			ExtIOB1_ddr |= (1<<((PortPin-24)));  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOB1_PullUp |= (1<<(PortPin-24));  //pullup
			else ExtIOB1_PullUp &= ~(1<<(PortPin-24));  //geen pullup
		}
		SPI_Write(IODIRB,ExtIOB1_ddr);	//write ddr in/uit
		SPI_Write(GPPUB,ExtIOB1_PullUp);  //write pullup niet/wel
	}
	if ((PortPin>15) && (PortPin<24))  // Poort A van extio adres 1
	{
		SPI_SLAVE_ADDR=1;
		if (IO_Choice == 1)// controle of output
		{
			ExtIOA1_ddr &= ~(1<<(PortPin-16));  // 0 is output van adres 1

		}
		else   // ingang
		{
			ExtIOA1_ddr |= (1<<(PortPin-16));  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOA1_PullUp |= (1<<(PortPin-16));  //pullup
			else ExtIOA1_PullUp &= ~(1<<(PortPin-16));  //geen pullup
		}
		SPI_Write(IODIRA,ExtIOA1_ddr);	//write ddr in/uit
		SPI_Write(GPPUA,ExtIOA1_PullUp);   //write pullup niet/wel
	}
	if ((PortPin > 7)  && (PortPin <16))// 0-7 = ExtIO_A 8-15 ExtIO_B van adres 0
	{
		SPI_SLAVE_ADDR=0;
		if (IO_Choice == 1)		// controle of output
		{
			ExtIOB0_ddr &= ~(1<<(PortPin-8));  // 0 is output

		}
		else
		{
			ExtIOB0_ddr |= (1<<((PortPin-8)));  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOB0_PullUp |= (1<<(PortPin-8));  //pullup
			else ExtIOB0_PullUp &= ~(1<<(PortPin-8));  //geen pullup
		}	
		SPI_Write(IODIRB,ExtIOB0_ddr);	//write ddr in/uit
		SPI_Write(GPPUB,ExtIOB0_PullUp);  //write pullup niet/wel
	}
	if (PortPin < 7)  // poortA extio adres 0
	{	
		SPI_SLAVE_ADDR=0;
		if (IO_Choice == 1)// controle of output
		{
			ExtIOA0_ddr &= ~(1<<PortPin);  // 0 is output van adres 0

		}
		else   // ingang
		{
			ExtIOA0_ddr |= (1<<PortPin);  // 1 = input
			//Kijken of Pullup
			if (IO_Choice == 2) ExtIOA0_PullUp |= (1<<PortPin);  //pullup
			else ExtIOA0_PullUp &= ~(1<<PortPin);  //geen pullup
		}
		SPI_Write(IODIRA,ExtIOA0_ddr);	//write ddr in/uit
		SPI_Write(GPPUA,ExtIOA0_PullUp);   //write pullup niet/wel
	}	
}

void Delay_ms(uint16_t Hoeveelmsdelay)
// Deze functie bepaald een delay in msec , Het aantal ms is afhankelijk van de parameter Hoeveelmsdelay. max delay 65000ms
{
	for (uint16_t i=0; i<Hoeveelmsdelay;i++) _delay_us(1000); // 1ms  keer het aantal in Hoeveelmsdelay
}

void AnaloogInSetup(void)
// Deze routine stelt de analoge poorten in voor de zes adc ingangen (AD0-AD5)
// Geen parameters nodig
{
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);	// Analoge inputs activeren en Prescale op 128 zetten d.w.z 125kHz klokfrequentie
	ADMUX = (1<<REFS0) | (1<<ADLAR);				// Stel in op externe referentie spanning via pin 21
	AnaloogIn_is_Set=1;  //setup één keer na aanroep analog
}

uint8_t Analoog_Read_8bit (uint8_t kanaal)
//Inlezen analoge ingang en omzetten in een uint8_t variabele 
// PreParameter kanaal is de uint8_t variabele waarmee gekozen kan worden welk kanaal (0-5) wordt uitgelezen
{ 
	if (AnaloogIn_is_Set==0) AnaloogInSetup();	//eerste keer initialisatie uitvoeren
	//uitlezen van een waarde tussen 0 en 255 , 0 = 0V en 255 = ref. voltage (vaak 5V)
	ADMUX &= 0xF0;									// mux0 tot 3 op nul zetten
	ADMUX |= (kanaal & 0x0F);						// Kanaal ADC instellen: Van kanaal bit3 t/m 0 instellen en overzetten naar MUX3 t/ MUX 0
	ADCSRA |= (1<<ADSC);							// Één ADC-omzetting starten
	while ( ADCSRA & (1<<ADSC) )					// Wachten op omzetting gereed
	{
		;											// niets doen alleen wachten
	}
	return (ADCH)	;								// hoogste 8 bits opvragen van analoge meting en doorgeven aan routine
}

void TekstNaarDisplay(char tekst[40], uint8_t xpos, uint8_t ypos)
// Deze routine stuurt een tekststring (woord of string van max. 40 karakters) naar de gewenste locatie op het display
// Pre-parameters zijn de tekststring, gewenste xpositie (0 t/m 15) en y-positie(0 of1 tbv regel)
// Deze routine heeft geen post-parameter.
{
	if (LCD_is_Set==0) //eerste keer initialisatie uitvoeren
	{
		LCDinit();
		LCDclr();
		LCD_is_Set=1;
	}
	if (ypos>1)
	{
		xpos+=16;  // bij 4 regelig display gaat regel 0 over in regel2 16+ 16 karakters. regel gaat over in regel 3
		ypos-=2;	// We zetten de cursor via regel 0 of 1 naar 2 of 3
	}
	LCDGotoXY(xpos, ypos);	// cursor op de juiste plaats zetten
	size_t lengte = strlen(tekst); //bepalen van lenhte van de tekst, voor het aantal te versturen karakters
	for(uint8_t teller=0;teller<lengte;teller++)  // aantal keren (AantalKar) een karakter op het display plaatsen
	{
		LCDsendChar(tekst[teller]);			//verzend karakter per karakter
	}
}

void Analoog_naar_LCD(uint8_t AnaWaarde,uint32_t MaxWaarde, uint32_t MinWaarde, uint8_t xpos, uint8_t ypos)
// maximale waarde en minimale waarde  moet een geheel getal zijn kleiner dan 999
// Deze waarde zet een analoge waarde op een gewenste plek naar het display.
// De analogewaarde 8-bits (o-255) wordt omgezet naar de gewenst max-woorden
// pre-parameter zijn de 8-bits analogewaarde, de maxwaarden (tbv omzetting naar 0-maxwaarden), gewenste xpositie (0 t/m 15) en y-positie(0 of1 tbv regel)
// Geen post-parameters in deze routine.
{
	uint16_t AnaloogDummy1=0;
	AnaloogDummy1 = (((AnaWaarde)* (MaxWaarde-MinWaarde)/5 *10) /51) + (MinWaarde*10);		// Analoge waarde converteren    255x999/5 x10/51
	if (AnaloogDummy1>990) 
	{
		Verschuif_1_plaats =1;
	}
	else
	{
		Verschuif_1_plaats =0;
	}

	utoa(AnaloogDummy1,Analoogtekst,10);						//analogewaarde omzetten in tekst ( 3 cijfers, laatste is tiende)
	if (AnaloogDummy1<10)				// kijkien of alleen getal achter de komma
	{
			Analoogtekst[3]=Analoogtekst[0];					//Getal achter komma invullen op vierde karakterpos. van tekststring
			Analoogtekst[2]=44;									// Komma plaatsen op derde karakerpos tekststring
			Analoogtekst[0]=48;									// 0 plaatsen op eerste karakterpos tekststring
			Analoogtekst[1]=48;									// 0 plaatsen op tweede karakterpos tekststring
			TekstNaarDisplay(Analoogtekst,xpos,ypos);			// Tekststringplaatsen op lcd			
	}
	else
	{
			if (AnaloogDummy1<100)		// getal >0 en < 10
			{
				Analoogtekst[3]=Analoogtekst[1];				//Getal achter komma invullen op derde karakterpos. van tekststring
				Analoogtekst[1]=Analoogtekst[0];				//Getal voor komma invullen op eerste karakterpos. van tekststring
				Analoogtekst[2]=44;									// Komma plaatsen op tweede karakerpos tekststring
				Analoogtekst[0]=48;									// 0 plaatsen op eerste karakterpos tekststring
				TekstNaarDisplay(Analoogtekst,xpos,ypos);			// Tekststringplaatsen op lcd	
			}
	 		else 
			{
        		 if (AnaloogDummy1<1000)			// getal >9 en <100
         		 {
        	 	 	Analoogtekst[3]=Analoogtekst[2];				//Getal achter komma invullen op vierde karakterpos. van tekststring
        	 	 	Analoogtekst[1]=Analoogtekst[1];				//eenheid voor komma invullen op tweede karakterpos. van tekststring = eenheden
        	 	 	Analoogtekst[0]=Analoogtekst[0];				//tientall voor komma invullen op eerste karakterpos. van tekststring = 10 tallen
        	 	 	Analoogtekst[2]=44;									// Komma plaatsen op derde karakerpos tekststring
        	 	 	TekstNaarDisplay(Analoogtekst,xpos,ypos);			// Tekststringplaatsen op lcd
         		 }
         		 else												// getal >99
         		 {
				 	if (AnaloogDummy1<10000)			// getal >99 en <999
				 	{
				 		if ((Analoogtekst[3]> 52) && (AnaloogDummy1<999)) utoa(AnaloogDummy1+1,Analoogtekst,10);					//eenheid naarboven afronden  invullen op derde plaats
				 		Analoogtekst[3]=32;									// spatie plaatsen op vierde karakterpos tekststring
				 		TekstNaarDisplay(Analoogtekst,xpos,ypos);				// Tekststringplaatsen op lcd
				 	}			 
				 }
			}
	}	
}

//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PB(uint8_t PortPin)
{
	return !((PINB & (1<<PortPin))==0);
}

//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PC(uint8_t PortPin)
{
	return !((PINC & (1<<PortPin))==0);
}


//Uitlezen van een poortpin PB* (ingang)
uint8_t Digital_Read_PD(uint8_t PortPin)
{
	return !((PIND & (1<<PortPin))==0);
}


//Wegschrijven van een 1 of 0 naar een output pin van PB
void Digital_Write_PB(uint8_t PortPin,uint8_t Value)
{
	if (Value == 1) PORTB |= (1 << PortPin);
	else PORTB &= ~(1<<PortPin);
}


//Wegschrijven van een 1 of 0 naar een output pin van PC
void Digital_Write_PC(uint8_t PortPin,uint8_t Value)	
{ 
	if (Value == 1) PORTC |= (1 << PortPin);
	else PORTC &= ~(1<<PortPin);
}

//Wegschrijven van een 1 of 0 naar een output pin van PD
void Digital_Write_PD(uint8_t PortPin,uint8_t Value)	
{
	if (Value == 1) PORTD |= (1 << PortPin);
	else PORTD &= ~(1<<PortPin);
}

// Set the Pin of port PB on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PB(uint8_t PortPin,uint8_t IO_Choice)
{
	if (IO_Choice == 1) DDRB |= (1<<PortPin);
	else 
	{
		DDRB &= ~(1<<PortPin);
		if (IO_Choice == 2) PORTB |= (1<<PortPin);
		else PORTB &= ~(1<<PortPin);
	}
}

// Set the Pin of port PC on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PC(uint8_t PortPin,uint8_t IO_Choice)
{
	if (IO_Choice == 1) DDRC |= (1<<PortPin);
	else 
	{
		DDRC &= ~(1<<PortPin);
		if (IO_Choice == 2) PORTC |= (1<<PortPin);
		else PORTC &= ~(1<<PortPin);
	}
}

// Set the Pin of port PD on Input(INPUT), Input with pull_UP (PULL_UP) or Output(OUTPUT)
void Set_Pinmode_PD(uint8_t PortPin,uint8_t IO_Choice)
{
	if (IO_Choice == 1) DDRD |= (1<<PortPin); //output
	else 
	{
		DDRD &= ~(1<<PortPin);
		if (IO_Choice == 2) PORTD |= (1<<PortPin);
		else PORTD &= ~(1<<PortPin);
	}
}

// Een Decimaal (komma of punt)  invoegen bij een tekststring 
// bv. aanroep in main:
//	AnaloogString='1295';
//	Decimaal_Invoegen(AnaloogString,1,',');
// Analoogstring wordt "129,5"	
void Decimaal_Invoegen(char Tekststring[17], uint8_t Decimaals_na_decimaal_kar, char Decimaal_kar)
{
	uint8_t lengte = strlen(Tekststring);
	for (uint8_t i=lengte; i>(lengte-Decimaals_na_decimaal_kar);i--) 
	{
		Tekststring[i] =Tekststring[i-1];
	}
	Tekststring[lengte-Decimaals_na_decimaal_kar] =Decimaal_kar;
}


// zet een Analogewaarde om in een tekststring van max 16 karakters, geef aan hoeveel karakters moeten worden weergegeven ( 0, is geen aanvulling, 1  is 
// geef ook aan wat als aanvulling voor het getal (' ','0','_' moet worden weergegeven bv "  321""  of "00321" of "__321"
// kleinste tekst een één 0
void Analoog_naar_string(char Tekststring[16], uint32_t Getal, uint8_t MinAantalKar, char Aanvulling)
{	
	char TmpTekststring[16]="";
	utoa(Getal,TmpTekststring,10);
	size_t lengte = strlen(TmpTekststring);

	if (MinAantalKar>lengte)
	{
		for (uint8_t i=lengte;i>0;i--)
		{
			TmpTekststring[(MinAantalKar - lengte + i - 1)] = TmpTekststring[(i-1)];
		}
		for (uint8_t i=MinAantalKar-lengte;i>0;i--)
		{
			TmpTekststring[i-1] = Aanvulling;			
		}
	}
	for (uint8_t j=16;j>0;j--) Tekststring[j-1] = TmpTekststring[j-1];
}

// Functie verschaald een Waarde van tussen de ValueMin en ValueMax naar een outputparameter (Verschaal) 
// tussen RangeMin en RangeMax. both max's and Value min = 0 and max =1024
uint32_t Verschaal(uint32_t Value,uint32_t ValueMin,uint32_t ValueMax,uint32_t RangeMin,uint32_t RangeMax)
{
	uint32_t TmpVerschaal=0;
	TmpVerschaal=(RangeMax - RangeMin +1)*1000;	// Gewenste range bepalen * 100 compenseren voor waardes achter de komma
	TmpVerschaal /=  (ValueMax - ValueMin + 1);		// Value reeks bepalen en delen 
	
	TmpVerschaal *=  (Value-ValueMin);		// de verhouding vermenigvuldigen met de  Value
	TmpVerschaal /= 1000;	// delen door 100 om komma compensatie weg halen
	TmpVerschaal += RangeMin;   //
	return TmpVerschaal;
}

void SetmsTeller()  // instellen en activeren msTeller via timer2. Stelt ook andere timers in voor PWM en toon().
{
	if (PWM_is_Set==0) SetPWM();//eerste keer initialisatie uitvoeren van timers							
}

void SetToon()  // instellen en activeren van de toon functie via timer2. Stelt ook andere timers in voor PWM en msTeller.
{
	if (PWM_is_Set==0) SetPWM();//eerste keer initialisatie uitvoeren van timers
}

void SetPWM()		// instellen PWM via timer0. Stelt ook andere timers in voor toon() en msTeller.
{	
	Set_Pinmode_PWM0 |= (1<<PWM0_uitgang); // PWM0 set as output
	Set_Pinmode_PWM1 |= (1<<PWM1_uitgang); // PWM1 set as output
	Set_Pinmode_PWM2 |= (1<<PWM2_uitgang); // PWM1 set as output
	Set_Pinmode_TOON |= (1<<TOON_uitgang); // TOON_uitgang set as output
	/*
	 MODE	WGM02	WGM01	WGM00	 DESCRIPTION					TOP
	 0		0 		0		0		 Normal 						0xFF
	 1		0		0		1		PWM, Phase Corrected			0xFF
	 2		0		1		0		CTC								OCR0A
	 */
	TCCR0A = (0<<WGM01)|(0<<WGM11)|(0<<WGM01);					// timer0 in normal mode, interrupt bij overflow
	OCR0A = 0;					// PWM0
	OCR0B =0;					// PWM1
	OCR2A = 199;				// msteller
	OCR2B = 0;					// PWM2
	/*
	CS02	 CS01 	 CS00 	 DESCRIPTION
	0		0 		0 	 Timer/Counter0 Disabled
	0		0 		1 	 No Prescaling
	0		1 		0 	 Clock / 8
	0		1 		1 	 Clock / 64
	1		0 		0 	 Clock / 256
	1		0 		1 	 Clock / 1024
	*/
	TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00);		// Prescale timer = 8, overflow = 255, dus PWM frequentie = 7,8kHz
	TIMSK0 = (1<< OCIE0A) | (1<< OCIE0B)|(1 << TOIE0);					// configureren van interrupt
	/*
	 MODE	WGM21	WGM20	 DESCRIPTION			TOP
	 0		0		0		Normal 					0xFF
	 1		0		1		PWM Phase Corrected
	 2		1		0		CTC						OCR2
	*/
	TCCR2A = (1<<WGM21)|(0<<WGM20);					// timer0 in crc mode, interrupt bij overflow
	/*
	 CS22	 CS21 	 CS20 	 DESCRIPTION
	 0		0 		0 	 Timer/Counter2 Disabled
	 0		0 		1 	 No Prescaling
	 0		1 		0 	 Clock / 8
	 0		1 		1 	 Clock / 32
	 1		0 		0 	 Clock / 64
	 1		0 		1 	 Clock / 128
	 1		1 		0 	 Clock / 256
	 1		1 		1 	 Clock / 1024
	 */
	TCCR2B |= (0<<CS22)|(1<<CS21)|(0<<CS20);		// Prescale timer = 8, overflow = 200, dus PWM frequentie = 10kHz
	TIMSK2 |= (1<<OCIE2A) | (1<<OCIE2B)|(1<<TOIE2);					// configureren van interrupt
	/*  Timer1
	 			7 bit	 6 bit 	 5 bit 	 4 bit 	 3 bit 	 2 bit 	 1 bit 	 0 bit
	 	 TCCR1A	COM1A1	COM1A0	COM1B1	COM1B0		-		-	WGM11	WGM10
	 	 Timer/Counter Control Register 1 A
	

	 			 7 bit	 6 bit 	 5 bit 	 4 bit 	 3 bit 	 2 bit 	 1 bit 	 0 bit
	 	 TCCR1B	ICNC1	ICES1		-	WGM13	  WGM12	 CS12	 CS11	 CS10
	 	 Timer/Counter Control Register 1 B

	 	 MODE	WGM13	WGM12	WGM11	WGM10	 DESCRIPTION	 TOP
	 	 0		0		0 		0		0		Normal 			0xFFFF
	 	 4		0		1		0		0		CTC				OCR1A
	 	 5		0		1		0		1	 Fast PWM, 8bit		0x00FF
	 	 6		0		1		1		0	 Fast PWM, 9bit 	0x01FF
	 	 7		0		1		1		1	 Fast PWM, 10bit 	0x03FF
	 	 11		1		0		1		1	 PWM, Phase Correct  OCR1A
	 	 15		1		1		1		1		Fast PWM		OCR1A
	 	 Waveform Generator Mode bits

	 	 CS12	 CS11 	 CS10 	 DESCRIPTION
	 	 0		0 		0 		Timer/Counter1 Disabled
	 	 0		0 		1 		No Prescaling
	 	 0		1 		0 		Clock / 8
	 	 0		1 		1 		Clock / 64
	 	 1		0 		0 		Clock / 256
	 	 1		0 		1 		Clock / 1024
	 	 1		1 		0 		External clock source on T1 pin, Clock on Falling edge
	 	 1		1 		1 		External clock source on T1 pin, Clock on rising edge
	*/
	TCCR1A = (0<<WGM11)|(0<<WGM10);					// timer1 in crc mode en
	TCCR1B = (0<<WGM13)|(1<<WGM12);				// en interrupt bij overflow
	TCCR1B |= (0<<CS12)|(1<<CS11)|(1<<CS10);		// Prescale timer = 64, Toonfrequentie = 250kHz, var 0-20kHz afw 1%. abslote max = 55kHz
	TIMSK1 |= (0<<OCIE1A) | (0<<OCIE1B);					// configureren van interrupt
	sei();									// activeren van interrupts
	PWM_is_Set=1;
}

void Toon(uint16_t Frequentie)
// Elke 0,5usec wordt TCNT1 1 hoger, afhankelijk van gewenste frequentie wordt bij een bepaalde waarde de uitgang hoog en laag
// De dutycycle == 50%. De ingestelde waarde is 2000000/frequentie. maar  bij f<100Hz 15625/frequentie.
//Maximale frequentie 20kHz en min. 5Hz
{
	if (PWM_is_Set==0) SetPWM();//eerste keer initialisatie uitvoeren
	if (!(TempFrequentie == Frequentie))  // berekening uitvoeren als frequentie is aangepast
	{	
		TempFrequentie = Frequentie;  // de controle freq vullen met de gewenste freq
		if (Frequentie>4)
		{	
			TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); //reset timer
			TCCR1B |= (1<<CS11)|(1<<CS10);		// Prescale timer = 64, Toonfrequentie = 250kHz, var 5-20kHz afw 1%. absolote max = 55kHz
			TOON_Aan=1;  //Uitgang voor toon mag worden aangepast
			TempFrequentie = Frequentie;
			uint32_t Timer1Vergelijk = 250000 / Frequentie;  //timer1 vergelijker berekenen
			OCR1A = Timer1Vergelijk;   // uitgang wordt weer hoog
			OCR1B = (Timer1Vergelijk/2);  // uitgang wordt weer laag
			TCNT1L=0;TCNT1H=0;  //init timercnter op0
			TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);  //activeren interrupt timer
		} 
		else if (Frequentie>0)  
		{		
			TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));  //reset timer
			TCCR1B |= (1<<CS12)|(1<<CS10);		// Prescale timer = 1024, Toonfrequentie = 15625Hz, var 1-4kHz
			TOON_Aan=1;  //Uitgang voor toon mag worden aangepast
			TempFrequentie = Frequentie;
			uint32_t Timer1Vergelijk = 15625 / Frequentie;  //timer1 vergelijker berekenen
			OCR1A = Timer1Vergelijk;   // uitgang wordt weer hoog
			OCR1B = (Timer1Vergelijk/2);  // uitgang wordt weer laag
			TCNT1L=0;TCNT1H=0; //init timercnter op 0
			TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);
		}
		if (Frequentie == 0)  							// of niet
		{
			TOON_Aan=0;				
			TOON_PORT &= ~(1<<PWM0_uitgang);  // toon uitgang low
			TCNT1L=0;TCNT1H=0; //init timercnter op 0
			TIMSK1 |= (0<<OCIE1A) | (0<<OCIE1B);
		}
	}
}

void Analog_Write_PWM(uint8_t PWMIndex,uint8_t Value)
{	// er zijn drie PWM-kanalen PWM0, PWM1 en PWM2. In LIB_ED_JV.H kan je aangeven welke uitgangen bij deze kanalen horen
	// default PB2, PC3 en PB5(op testboard PC3)	
	Value = 255 - Value;
	if (PWM_is_Set==0) SetPWM();//eerste keer initialisatie uitvoeren
	if (PWMIndex ==PWM0) 
	{
			if (Value<251) 
			{
				PWM0_Aan =HIGH;
			}
			else 
			{		
				if (PWM0_Aan) PWM0_PORT &= ~(1<<PWM0_uitgang);
				PWM0_Aan=LOW;
				Value=250;
			}
			OCR0A=Value;
	}
	if (PWMIndex ==PWM1) 
	{
			if (Value<251) PWM1_Aan =HIGH;
			else 
			{		
				if (PWM1_Aan) PWM1_PORT &= ~(1<<PWM1_uitgang);
				PWM1_Aan=LOW;
				Value=250;
			}
			OCR0B=Value;
	}
	if (PWMIndex ==PWM2) 
	{
			if (Value<255) PWM2_Aan =HIGH;
			else 
			{		
				if (PWM2_Aan) PWM2_PORT &= ~(1<<PWM2_uitgang);
				PWM2_Aan=LOW;
				Value++;
			}
			OCR2B=((Value*199)/255);
	}		
}

ISR(TIMER0_COMPA_vect)
//  Deze routine uitvoeren wordt gebruikt bij PWM en zet de uitgang laag
{

		if (PWM0_Aan) 
				PWM0_PORT |= (1<<PWM0_uitgang);
		//else PWM0_PORT &= ~(1<<PWM0_uitgang);  //uncommand when else is necessary

}

ISR(TIMER0_COMPB_vect)
//  Deze routine uitvoeren wordt gebruikt bij PWM en zet de uitgang laag
{

		if (PWM1_Aan) 
				PWM1_PORT |= (1<<PWM1_uitgang);
		//else PWM1_PORT &= ~(1<<PWM1_uitgang);//uncommand when else is necessary
		if (DirStepper) PulsPos++;
		else PulsPos--;

}

ISR(TIMER2_COMPB_vect)  //  Deze routine uitvoeren wordt gebruikt bij PWM en zet de uitgang hoog
{

		if (PWM2_Aan) PWM2_PORT |= (1<<PWM2_uitgang);
		//else PWM2_PORT &= ~(1<<PWM2_uitgang);  //uncommand when else is necessary

}

ISR(TIMER2_COMPA_vect)  // Deze routine wordt uitgevoerd bij interuptrequest van timer2 (0,1msec puls)
{
	T10Teller++;				// msteller ophogen met 1
	if (T10Teller == 10)   // indien 10 pulsje is 1ms
	{
			msTeller++;		//	ms met één verhogen
			T10Teller=0;
	}
	if (PWM2_Aan) PWM2_PORT &= ~(1<<PWM2_uitgang);
}
ISR(TIMER0_OVF_vect) // set LOW PWM0 and PWM1
{
	if (PWM0_Aan) PWM0_PORT &= ~(1<<PWM0_uitgang);
	if (PWM1_Aan) PWM1_PORT &= ~(1<<PWM1_uitgang);
}
ISR(TIMER2_OVF_vect) // set LOW PWM0 and PWM1
{
	if (PWM1_Aan) PWM2_PORT &= ~(1<<PWM2_uitgang);
}

//Toon functie 
ISR(TIMER1_COMPA_vect)
{
	if (TOON_Aan)
	TOON_PORT |= (1<<TOON_uitgang);  // toon uitgang High
	

}

ISR(TIMER1_COMPB_vect)
{
	if (TOON_Aan)
	TOON_PORT &= ~(1<<TOON_uitgang);  // toon uitgang low
}