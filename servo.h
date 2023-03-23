/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

#ifndef SERVO_H
#define	SERVO_H
    
#include "configuration.h"

#define _SERVO_PWM_PERIOD (65535/postscaling); // 50 Hz

void configure_servo(void) {
    _TRISA6 = 0;    // pin 17 RA6
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON1bits.OCM = 0b110;
    OC1CON2bits.OCTRIG = 0;
    OC1CON2bits.SYNCSEL = 0b11111;
    servo_pwm_period = _SERVO_PWM_PERIOD; 
    servo_pwm_duty_cycle = (.0005/0.0163398693)*_SERVO_PWM_PERIOD + (50.0/180.0)*(0.002/0.0163398693)*_SERVO_PWM_PERIOD;
}

void set_door_servo(double angle) {
    if (angle >= 180 || angle <= 0) {
        return;
    }
    servo_pwm_duty_cycle = (.0005/0.0163398693)*_SERVO_PWM_PERIOD;
    servo_pwm_duty_cycle += (angle/180.0)*(0.002/0.0163398693)*_SERVO_PWM_PERIOD;
}

#endif	/* SERVO_H */