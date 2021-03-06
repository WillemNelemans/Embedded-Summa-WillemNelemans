/*
 * Langkort_knop_ingedrukt.c
 *
 * Created: 19-5-2021 20:36:19
 * Author : Willem
 */ 

#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <LIB_ED_JV/LIB_ED_JV.h>

uint16_t Kort_ingedrukt = 500; // tot hoelang kan jij hem indrukken tot het nog geldig is als kort indrukken 500 ms
uint16_t Lang_Ingehouden = 1000;
bool OudeStatusPC0, StatusPC0, StatusPC1 = 0;
uint16_t TijdIngedruktPC0, TijdIngedruktPC1= 0;
uint16_t TijdLosgelatenPC0, TijdLosgelatenPC1 = 0;
uint16_t PC0Voormsingedrukt, PC1Voormsingedrukt= 0;
uint8_t debounce = 10; // debounce van 10 ms je kan zien hoe kart dat is. je zult niet onder de 20ms kommen.
char tijdPC0[8]= {0};
char tijdPC1[8]= {0};
		
int main(void)
{
	Set_Pinmode_PC(PC0,PULL_UP);
	Set_Pinmode_PC(PC1,PULL_UP);
	Set_Pinmode_PC(PC1,PULL_UP);
	
	Set_Pinmode_PB(PB2,OUTPUT);	
	Set_Pinmode_PC(PC3,OUTPUT);
	Set_Pinmode_PD(PD3,OUTPUT);
	
	SetPWM(); // dit start msteller en PWM. het beste is als je dit bekijkt in de LIB_ED_JV.C normaal het is ingeprogrameert om dit te doen bij Analog_Write_PWM(); maar dit gebruiken we nu niet. SetmsTeller() kun jet ook gebruiken maar dit is gewoon SetPWM(). dit zie je allemaal in de LIB_ED_JV.C
	
	TekstNaarDisplay("Knoppen functie",0,0);					// bij startup laaten zien. dan weten we altijd welk programa op jpu atmega328P staat en dat jou scherm werkt.
	_delay_ms(1000);
	LCDclr();
	
    while (1) 
    {

		StatusPC0 = Digital_Read_PC(PC0);																// kijk of PC0 ingdrukt is ja of nee.	
	
		if(OudeStatusPC0 == HIGH && StatusPC0 == LOW)		
		{
			
			TijdIngedruktPC0 = msTeller;																// je hebt de knop ingedrukt knop is ingedrukt.
		}
		
		
		else if(OudeStatusPC0 == LOW && StatusPC0 == HIGH)												// je hebt de knop losgelaten knop is los gelaten.
		{
			
			TijdLosgelatenPC0 = msTeller;
			PC0Voormsingedrukt = TijdLosgelatenPC0 - TijdIngedruktPC0;
			
			sprintf(tijdPC0, "%0d ms", PC0Voormsingedrukt);
			
			if((PC0Voormsingedrukt > debounce) &&  (PC0Voormsingedrukt < Kort_ingedrukt))				// tussen 5 en 500 ms kortingedrukt
			{
				LCDclr();
				TekstNaarDisplay("kort gedrukt",0,1);
				Digital_Write_PC(PC3,HIGH);
				PC0Voormsingedrukt = 0;
			}
			
			if(PC0Voormsingedrukt >= Kort_ingedrukt)													// hoger dan 500 ms lang ingedrukt
			{
				LCDclr();
				TekstNaarDisplay("lang gedrukt",0,1);
				Digital_Write_PB(PB2,HIGH);
				PC0Voormsingedrukt = 0;
			}

		}
		OudeStatusPC0 = StatusPC0; // sla de posietie van knop op


		if(Digital_Read_PC(PC1) == LOW) 	// button is pressed
		{
			TijdIngedruktPC1 = msTeller;
			PC1Voormsingedrukt = TijdIngedruktPC1 - TijdLosgelatenPC1;
			
			if (PC1Voormsingedrukt > Lang_Ingehouden)
			{
				Digital_Write_PB(PB2,LOW);
				Digital_Write_PC(PC3,LOW);
				Digital_Write_PD(PD3,LOW);
			}
			else Digital_Write_PD(PD3,HIGH);
		}
		else if(Digital_Read_PC(PC1) == HIGH)					// button is released
		{
			TijdLosgelatenPC1 = msTeller;
			PC1Voormsingedrukt = 0;
		}
		

		sprintf(tijdPC1, "%0d ms", PC1Voormsingedrukt);
		
		TekstNaarDisplay(tijdPC0,0,0);
		TekstNaarDisplay(tijdPC1,8,0);
    }
}

