/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef SERVO_H
#define	SERVO_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"

#define _PWM_PERIOD = (89/postscaling);

void configure_servo(void) {
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON1bits.OCM = 0b110;
    OC2CON2bits.OCTRIG = 0;
    OC2CON2bits.SYNCSEL = 0b11111;
    servo_pwm_period = _PWM_PERIOD; // 50 Hz
    servo_pwm_duty_cycle = 0;
}

void set_door_servo(double angle) {
    if (angle > 180 || angle < 0) {
        return;
    }
    servo_pwm_duty_cycle = (angle/180.0)*_PWM_PERIOD;
}

#endif	/* SERVO_H */