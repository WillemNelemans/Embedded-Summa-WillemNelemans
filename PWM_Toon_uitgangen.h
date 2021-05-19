/* 
 * Summa Embedded Design Library vs 31.3-2021
 * LIB_ED_JV.c
 *
 * Created: 31-3-21
 * Author : john
 */ 

// Hieronder kan je aangeven welke uitgangen gebruikt kunnen worden voor de functies PWM en Toon

//init defines PWM0  default PB2
#define PWM0_PORT PORTB
#define PWM0_uitgang 2
#define Set_Pinmode_PWM0 DDRB
//init defines PWM1  default PC3
#define PWM1_PORT PORTC
#define PWM1_uitgang 3
#define Set_Pinmode_PWM1 DDRC
//init defines PWM2	default PB5 -> red board = PD3
#define PWM2_PORT PORTD
#define PWM2_uitgang 3
#define Set_Pinmode_PWM2 DDRD
//init defines Toon output	red board = PC5
#define TOON_PORT PORTC
#define TOON_uitgang 5
#define Set_Pinmode_TOON DDRC

// definieren pwm uitgangen aanroep   - niet aanpassen
#define PWM0 0
#define PWM1 1
#define PWM2 2