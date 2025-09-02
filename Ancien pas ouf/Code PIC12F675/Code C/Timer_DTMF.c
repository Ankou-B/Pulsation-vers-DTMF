/*
 * File:   Timer dtmf.c
 * Author: Aelig Bedouet
 *
 * Created on 28 juillet 2024, 14:00
 */


#include "pic12f675.h"
#include <xc.h>
#define _XTAL_FREQ 4000000


// CONFIG1
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF    // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

unsigned long millis =0;
//unsigned long currentMillis = 0;
//unsigned long duree = 0;
//unsigned long PrevMillis = 0;
int Impuls = 0 ;
int Mem_haut = 0;
int note = 0;
void __interrupt()   ISR  (void);

void main(void)
{  
    //OSCCAL = 0x68;        // Choix 4Mhz  // 
    ANSEL = 0b00000000;        //  GP5 GP4 GP3 GP2 GP1 GP0 digital 0b00000000
    TRISIO =  0b00001000;        //  GP5 GP2 GP4 GP1 GP0 out digital,  GP3 in 0b00001000
    
    OPTION_REG = 0b10000001;
    CMCON =0x07;// digital out gpio
    GPIO = 0;
    INTCONbits.TMR0IF = 0; /////
    INTCONbits.TMR0IE = 1; /// 
    INTCONbits.GIE = 1; //Enable global interrupt 
    
   
    
    while (1)
    {                   
       //////////////////  G n ration DTMF ///////////////////////////
       /*
        currentMillis = millis; // Chrono 1
       duree = currentMillis - PrevMillis; // Tempo*/
       if ((millis > 220) && (note == 1)) // Suivant le nombre d'impulsion la dur e de la derni re d passera les 100ms
         { 
             if (Impuls == 10) //un zero compos  vaut dix impulsions 
             {                //on remet Impuls   z ro 
                 Impuls = 0;
             }
        
		switch (Impuls)
		{
		 case 0: // D          
			GP5 = 1;//=10 pulses =A =b 1010
			GP4 = 0;
			GP0 = 1;
			GP1 = 0;
			break;
	   
		  case 1: // 1
			GP5 = 0;//=b 0001
			GP4 = 0;
			GP0 = 0;
			GP1 = 1;
              
			break;

		  case 2: // 2
            GP5 = 0;//=b 0010
			GP4 = 0;
			GP0 = 1;
			GP1 = 0;
			break;

		  case 3: // 3
            GP5 = 0;//=b 0011
			GP4 = 0;
			GP0 = 1;
			GP1 = 1;
			break;

		  case 4: // 4
            GP5 = 0;//=b 0100
			GP4 = 1;
			GP0 = 0;
			GP1 = 0;
			break;

		  case 5: // 5
            GP5 = 0;//=b 0101
			GP4 = 1;
			GP0 = 0;
			GP1 = 1;
			break;

		  case 6: // 6
            GP5 = 0;//=b 0110
			GP4 = 1;
			GP0 = 1;
			GP1 = 0;
			break;

		  case 7: // 7
            GP5 = 0;//=b 0111
			GP4 = 1;
			GP0 = 1;
			GP1 = 1; 		
			break;

		  case 8: // 8
            GP5 = 1;//=b 1000
			GP4 = 0;
			GP0 = 0;
			GP1 = 0;			
			break;

		  case 9: // 9
            GP5 = 1;//=b 1001
			GP4 = 0;
			GP0 = 0;
			GP1 = 1; 			
			break;
		}
        __delay_us(50);    
         GP2 = 1; // Active MP5088 
        __delay_ms(500); //genere pendant 1/2 seconde la frequence 
         
        GPIO = 0; //GP2 = 0 also
        note = 0;
        Impuls = 0; 
         
        }
        //////////////////////// Num rotation //////////////////////////////
       //// Lire l' tat sur l'entr e des IMPULSIONS
       if ((GP3 == 1) && ( Mem_haut == 0))
           
           { 
            Mem_haut = 1;                  //M moriser l' tat haut
            note = 0;                      //Blocage tempo  
           }
       
       if  ((GP3 == 0) && (Mem_haut == 1))  //Sinon pas d'impulsion
           
            {
            Mem_haut = 0;                 //memoire de l' tat haut = 0
            millis = 0 ;
           
            Impuls ++; //Comptage des impulsions du cadran
            /*
            PrevMillis = currentMillis;    //duree   z ro ,la p riode commence    tre compt e 
            */
            note = 1;                       // validation d'au moins une impulsion            
            }
         
    } //while principale
 
    return;
}//MAIN

void __interrupt()   ISR  (void)
{
if(TMR0IE && TMR0IF)
    {
    millis ++;// 1 ms   chaque flag/debordement
    //GP5 =~ GP5;
    INTCONbits.TMR0IF = 0;
    }


}

/*
GP5= 1;
__delay_ms(10);
GP5= 0;
 */