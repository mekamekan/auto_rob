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
#include <conio.h>
#include <stdio.h>

void motorA(int duty);
void motorB(int duty);
void motorC(int duty);
void motorD(int duty);
unsigned int PushSwitchRead(void); 
unsigned int switchA_Read(void);
unsigned int switchB_Read(void);
unsigned int switchC_Read(void);
void Servo5(double angle);
void Servo12(double angle);
void DataWrite(unsigned char data);
void putch(unsigned char data);
unsigned int ADC_result(unsigned char ch);
unsigned int sensorA_Read(void);
unsigned int sensorB_Read(void);
unsigned int sensorC_Read(void);
unsigned int sensorD_Read(void);
void __interrupt() ISR(void);
//グローバル変数
unsigned char g_ReadData;
//unsigned char g_ReadStr[20];

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
    
    //PWM5(16bit)設定
    RC5PPS = 0b011101;
    PWM5CON = 0b10000000;
    PWM5CLKCON = 0b01000000;
    PWM5LDCON = 0x00;
    PWM5OFCON = 0x00;
    PWM5PHH = 0x00;
    PWM5PHL = 0x00;
    PWM5DCH = (2899 >> 8) & 0x00FF; //初期位置設定
    PWM5DCL = 2899 & 0x00FF;
    PWM5PRH = (39999 >> 8) & 0x00FF;
    PWM5PRL = 39999 & 0x00FF;
    PWM5OFH = 0x00;
    PWM5OFL = 0x00;
    PWM5TMRH = 0x00;
    PWM5TMRL = 0x00;
    
    //PWM6(16bit)設定
    __delay_ms(500);
    RC4PPS = 0b011110;
    PWM6CON = 0b10000000;
    PWM6CLKCON = 0b01000000;
    PWM6LDCON = 0x00;
    PWM6OFCON = 0x00;
    PWM6PHH = 0x00;
    PWM6PHL = 0x00;
    PWM6DCH = (2999 >> 8) & 0x00FF; //初期位置設定
    PWM6DCL = 2999 & 0x00FF;
    PWM6PRH = (39999 >> 8) & 0x00FF;
    PWM6PRL = 39999 & 0x00FF;
    PWM6OFH = 0x00;
    PWM6OFL = 0x00;
    PWM6TMRH = 0x00;
    PWM6TMRL = 0x00;
    
    //EUSART設定
    RXPPS = 0b001101;
    RB4PPS = 0b100100;
    TX1STA = 0b00100100;
    RC1STA = 0b10010000;
    BAUD1CON = 0b00001000;
    SP1BRGL = 416 & 0x00FF;
    SP1BRGH = (416 >> 8) & 0x00FF;
    
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    //ADC設定
    ADCON0 = 0b10101100;
    ADCON1 = 0b10100000;
    
    //変数宣言
    unsigned char str[] = "Please enter a string\r\n";
    while(1){
        
        //モータ＆スイッチテスト用プログラム
        /*for(int i = -600; i <= 600; i++){
 
            if(switchC_Read()){
                motorA(i);
                motorB(i);
                motorD(i);
                motorC(i);
            }
            else{
                motorA(0);
                motorB(0);
                motorD(0);
                motorC(0);
            }
            __delay_ms(10);
        }
        for(int i = 600; i >= -600; i--){
            
            if(PushSwitchRead()){
                motorA(i);
                motorB(i);
                motorD(i);
                motorC(i);
            }
            else{
                motorA(0);
                motorB(0);
                motorD(0);
                motorC(0);
            }
            __delay_ms(10);
        }*/
        
        //サーボテスト用プログラム
        /*for(int i = 0; i <= 270; i++){
            Servo5(i);
            Servo12(i);
            __delay_ms(100);
        }
        for(int i = 270; i >= 0; i--){
            Servo5(i);
            Servo12(i);
            __delay_ms(100);
        }*/
        
        //EUSARTテスト用プログラム
        /*for(int i = 0; str[i] != NULL; i++){
            DataWrite(str[i]);
        }
        DataWrite(g_ReadData);
        __delay_ms(100);*/
        
        //ADCテスト用プログラム
        printf("Vol:%u\r\n", sensorC_Read());
          
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

unsigned int switchA_Read(void){
    return PORTAbits.RA0;
}

unsigned int switchB_Read(void){
    return PORTAbits.RA1;
}

unsigned int switchC_Read(void){
    return PORTAbits.RA2;
}

void Servo5(double angle){
    
    double duty;
    
    angle > 180 ? (angle = 180) : angle;
    angle < 0 ? (angle = 0) : angle;
    
    duty = (3800 * (angle / 180)) + 999;
    
    
    PWM5DCH = ((int)duty >> 8) & 0x00FF;
    PWM5DCL = (int)duty & 0x00FF;
    PWM5LDCONbits.LDA = 1;
    
    return;
}

void Servo12(double angle){
    
    double duty;
    
    angle > 270 ? (angle = 270) : angle;
    angle < 0 ? (angle = 0) : angle;
    
    duty = (3200 * (angle / 270)) + 1399;
    
    PWM6DCH = ((int)duty >> 8) & 0x00FF;
    PWM6DCL = (int)duty & 0x00FF;
    PWM6LDCONbits.LDA = 1;
    
    return;
}

void DataWrite(unsigned char data){
    while(!PIR1bits.TXIF);
    PIR1bits.TXIF = 0;
    TX1REG = data;
    
    return;
}

void putch(unsigned char data){
    
    DataWrite(data);
    
    return;
}

unsigned int ADC_result(unsigned char ch){
    
    unsigned int adcValue;
    
    ADCON0bits.CHS = ch;
    ADCON0bits.ADON = 1;
    __delay_us(5);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);
    adcValue = ADRESL + (256 * ADRESH);
    ADCON0bits.ADON = 0;
    return adcValue;
    
}

unsigned int sensorA_Read(void){
    return ADC_result(12);
}

unsigned int sensorB_Read(void){
    return ADC_result(10);
}

unsigned int sensorC_Read(void){
    return ADC_result(8);
}

unsigned int sensorD_Read(void){
    return ADC_result(9);
}

void __interrupt() ISR(void){
    if(PIR1bits.RCIF){
        PIR1bits.RCIF = 0;
        if(RC1STAbits.FERR || RC1STAbits.OERR){
            RC1STA = 0x00;
            RC1STA = 0x90;
        }
        else{
            g_ReadData = RC1REG;
        }
    }
    else;
}
