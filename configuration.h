/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

/*
 * This file is for configuring the pins on the PIC24, and well as importing 
 * program wide libraries.
 */

#ifndef PIN_CONFIGURATION_H
#define	PIN_CONFIGURATION_H
    
#include <stdlib.h> // Used for fabs()
#include <math.h> // Used for round() and INFINITY
#include <stdbool.h> // Adds bools to c

#define postscaling 1
void set_postscaling(void) {
    switch(postscaling) {
        case(1):
            _RCDIV = 0b000; // 1:1 postscaling
            break;
        case(2):
            _RCDIV = 0b001; // 1:2 postscaling
            break;
        case(4):
            _RCDIV = 0b010; // 1:4 postscaling
            break;
        case(8):
            _RCDIV = 0b011; // 1:8 postscaling
            break;
        case(16):
            _RCDIV = 0b100; // 1:16 postscaling
            break;
        case(32):
            _RCDIV = 0b101; // 1:32 postscaling
            break;
        case(64):
            _RCDIV = 0b110; // 1:64 postscaling
            break;
        case(256):
            _RCDIV = 0b111; // 1:256 postscaling
            break;
        default:
            _RCDIV = 0b000; // 1:1 postscaling
            break;
    }
}

#define left_motor_in_1 _LATB2
#define left_motor_in_2 _LATA2
#define left_motor_pwm_period OC3RS // RB1
#define left_motor_pwm_duty_cycle OC3R // RB1
#define right_motor_in_3 _LATA0
#define right_motor_in_4 _LATA1
#define right_motor_pwm_period OC2RS // RB0
#define right_motor_pwm_duty_cycle OC2R // RB0
#define timer_1_period PR1
#define timer_1 TMR1
#define timer_1_enable T1CONbits.TON
#define timer_1_interrupt_enable _T1IE
#define timer_1_interrupt_flag _T1IF
#define enable_ad_auto_sample _ADON
#define right_qrd ADC1BUF10 // RB14
#define left_qrd ADC1BUF9 // RB15
#define center_qrd ADC1BUF11 // RB13
#define task_qrd ADC1BUF14 // RBA3
#define forwards_ir_range_sensor _RB7 // pin 11
#define left_ir_range_sensor _RB8 // pin 12
#define servo_pwm_period OC1RS
#define servo_pwm_duty_cycle OC1R
#define ball_qrd ADC1BUF15 // RB4
    
enum task_type {TEST, IDLE, STARTUP, LINE_FOLLOW, SAMPLE_COLLECTION, SAMPLE_RETURN, CANYON_NAVIGATION, EQUIPMENT_SERVICING, DATA_TRANSMISSION};
    
// All pin registers are set to output, digital, and off by default
void initialize_registers() {
    ANSA = 0;
    ANSB = 0;
    TRISA = 0;
    TRISB = 0;
    LATA = 0;
    LATB = 0;
}

#endif	/* PIN_CONFIGURATION_H */

