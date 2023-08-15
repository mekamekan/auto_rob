/*
 * File:   main.c
 * Author: hirot
 *
 * Created on 2023/08/15, 12:32
 */

// PIC16F1778 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 32000000
#include <xc.h>

void motorA(int duty);
void motorB(int duty);
void motorC(int duty);
void motorD(int duty);
unsigned int PushSwitchRead(void); 


void main(void) {
    //システム周波数設定
    OSCCON = 0b11110000;
    
    //IO設定
    ANSELA = 0x00;
    ANSELB = 0b00001111;
    ANSELC = 0x00;
    TRISA = 0b00001111;
    TRISB = 0b00101111;
    TRISC = 0x00;
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    
    //プルアップ設定
    OPTION_REGbits.nWPUEN = 0;
    WPUA = 0b00001111;
    WPUB = 0x00;
    WPUC = 0x00;
    
    //TMR2設定
    T2CLKCONbits.CS = 0b0001;
    T2CON = 0b10000000;
    T2PR = 0xFF;
    
    //PWM3設定
    RA5PPS = 0b011001;
    PWM3CON = 0b10000000;
    CCPTMRS2bits.P3TSEL = 0b00;
    PWM3DCH = 0x00;
    PWM3DCL = 0x00;
    
    //PWM4設定
    RA7PPS = 0b011010;
    PWM4CON = 0b10000000;
    CCPTMRS2bits.P4TSEL = 0b00;
    PWM4DCH = 0x00;
    PWM4DCL = 0x00;
    
    //PWM9設定
    RC1PPS = 0b011011;
    PWM9CON = 0b10000000;
    CCPTMRS2bits.P9TSEL = 0b00;
    PWM9DCH = 0x00;
    PWM9DCL = 0x00;
    
    //CCP1(PWM利用)設定
    RC3PPS = 0b010101;
    CCP1CON = 0b10011100;
    CCPTMRS1bits.C1TSEL = 0b00;
    CCPR1H = 0x00;
    CCPR1L = 0x00;
    
    
    
    while(1){
        
        for(int i = -600; i <= 600; i++){
            
            motorA(i);
            motorB(i);
            if(PushSwitchRead()){
                motorD(i);
                motorC(i);
            }
            else;
            __delay_ms(10);
        }
        for(int i = 600; i >= -600; i--){
            motorA(i);
            motorB(i);
            if(PushSwitchRead()){
                motorD(i);
                motorC(i);
            }
            else;
            __delay_ms(10);
        }
          
    }
    return;
}

void motorA(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        PWM3DCL = (duty << 8) & 0x00FF;
        PWM3DCH = (duty >> 2) & 0x00FF;
        RA5PPS = 0b011001;
        RA4PPS = 0x00;
        LATAbits.LATA4 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        PWM3DCL = (duty << 8) & 0x00FF;
        PWM3DCH = (duty >> 2) & 0x00FF;
        RA4PPS = 0b011001;
        RA5PPS = 0x00;
        LATAbits.LATA5 = 0;
    }
    else{
        RA4PPS = 0x00;
        RA5PPS = 0x00;
        LATAbits.LATA4 = 0;
        LATAbits.LATA5 = 0;
    }
    
    return;
    
}

void motorB(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        PWM4DCL = (duty << 8) & 0x00FF;
        PWM4DCH = (duty >> 2) & 0x00FF;
        RA7PPS = 0b011010;
        RA6PPS = 0x00;
        LATAbits.LATA6 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        PWM4DCL = (duty << 8) & 0x00FF;
        PWM4DCH = (duty >> 2) & 0x00FF;
        RA6PPS = 0b011010;
        RA7PPS = 0x00;
        LATAbits.LATA7 = 0;
    }
    else{
        RA6PPS = 0x00;
        RA7PPS = 0x00;
        LATAbits.LATA6 = 0;
        LATAbits.LATA7 = 0;
    }
    
    return;
    
}

void motorC(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        PWM9DCL = (duty << 8) & 0x00FF;
        PWM9DCH = (duty >> 2) & 0x00FF;
        RC1PPS = 0b011011;
        RC0PPS = 0x00;
        LATCbits.LATC0 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        PWM9DCL = (duty << 8) & 0x00FF;
        PWM9DCH = (duty >> 2) & 0x00FF;
        RC0PPS = 0b011011;
        RC1PPS = 0x00;
        LATCbits.LATC1 = 0;
    }
    else{
        RC0PPS = 0x00;
        RC1PPS = 0x00;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;
    }
    
    return;
    
}

void motorD(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        CCPR1L = (duty << 8) & 0x00FF;
        CCPR1H = (duty >> 2) & 0x00FF;
        RC3PPS = 0b010101;
        RC2PPS = 0x00;
        LATCbits.LATC2 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        CCPR1L = (duty << 8) & 0x00FF;
        CCPR1H = (duty >> 2) & 0x00FF;
        RC2PPS = 0b010101;
        RC3PPS = 0x00;
        LATCbits.LATC3 = 0;
    }
    else{
        RC2PPS = 0x00;
        RC3PPS = 0x00;
        LATCbits.LATC2 = 0;
        LATCbits.LATC3 = 0;
    }
    
    return;
    
}

unsigned int PushSwitchRead(void){
    return PORTAbits.RA3;
}
