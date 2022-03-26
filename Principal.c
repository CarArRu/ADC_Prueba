/*
 * File:   Principal.c
 * Author: cesar
 *
 * Created on 27 de agosto de 2021, 03:31 PM
 */

//  Librerías
#include <xc.h>
#include "ConfiguracionDeBits.h"

//  Definiciones
#define Salida1ON PORTCbits.RC0=1
#define Salida1OFF PORTCbits.RC0=0

#define Salida2ON PORTCbits.RC1=1
#define Salida2OFF PORTCbits.RC1=0

#define Salida3ON PORTCbits.RC2=1
#define Salida3OFF PORTCbits.RC2=0

#define _XTAL_FREQ 20000000

#define m 0.097751710;

//  Prototipos de las Funciones
void ConfiguracionRegistros(void);
void Inicio(void);


//  Estructuras
struct Variables{
    unsigned int ValorADC, Porcentaje;         
    unsigned char Con, tbajo;    
}Variable;




void main(void) {
    
    
    Primer Modificación Commit 2
    Segunda Modificación Commit 3
    
    ConfiguracionRegistros();
    Inicio();
    
    
    
    
    
    while(1){                          
        ADCON0bits.GO_NOT_DONE = 1;  //  Conversión en progreso        
        while(ADCON0bits.GO_NOT_DONE==1);                
        Variable.ValorADC = 0;       
        Variable.ValorADC = Variable.ValorADC | ADRESH;        
        Variable.ValorADC = Variable.ValorADC << 8;        
        Variable.ValorADC = Variable.ValorADC | ADRESL;        
        //Variable.Porcentaje --> 0 -- 100
        Variable.Porcentaje = Variable.ValorADC * m;                
        Variable.tbajo = 100 - Variable.Porcentaje;        
        PORTD = Variable.Porcentaje;                       
    }   //  Bucle While Principal
    
    
    
    //return;
}   //  MAIN 



void __interrupt() Interrupciones(){
    
    if(INTCONbits.TMR0IF==1){    //    TMR0 register has overflowed (must be cleared in software)
        
        if(PORTCbits.RC0==1){Salida1OFF;}
        else{Salida1ON;}     
                        
        if(Variable.Con>=Variable.tbajo){Salida2ON;}
        else{Salida2OFF;}                
        
        if(Variable.Con>=100){Variable.Con=0;}
        else{Variable.Con = Variable.Con + 1;}                                        
        
        //  Fin de Interrupción Timer 0
        TMR0L = 0b00111011;
        TMR0H = 0b11110110;
        INTCONbits.TMR0IF=0;        
    }            
}   //  



void ConfiguracionRegistros(void){
    
    //  Puerto C
    PORTC = 0x00;
    TRISC = 0x00; 
    //  PUERTO D
    PORTD = 0x00;
    TRISD = 0x00;
    //  Timer 0
    T0CONbits.TMR0ON = 1;   // Enables Timer0    
    T0CONbits.T08BIT = 0;   //  Timer0 is configured as a 16-bit timer/counter    
    T0CONbits.T0CS = 0; //  Internal instruction cycle clock (CLKO)
    
    T0CONbits.PSA = 0;  //   Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
    
    T0CONbits.T0PS = 0;    
             
    INTCONbits.GIE = 1; //  Enables all unmasked interrupts
    INTCONbits.TMR0IE = 1;  //  Enables the TMR0 overflow interrupt    
    //INTCONbits.TMR0IF = 0;    //    TMR0 register has overflowed (must be cleared in software)
    //INTCON2
    
    //  ADC
    PORTA = 0x00;
    TRISA = 0x01;
    
    
    ADCON0bits.CHS = 0; //  Canal 0 (AN0)        
    ADCON0bits.ADON = 1;    //  converter module is enabled    
    ADCON1bits.VCFG0 = 0;   // VSS Voltage Reference Configuration bit (VREF- source)    
    ADCON1bits.PCFG = 0b1110;
    
    ADCON2bits.ADFM = 1;    //  Right justified
    //ADRESH
    //ADRESL
    
    
}   //  Configuración de Registros


void Inicio(void){
    Variable.Con=0;
    Variable.Porcentaje=0;
    Variable.ValorADC=0;
    Variable.tbajo=0;
}