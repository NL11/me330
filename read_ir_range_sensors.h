/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef READ_IR_RANGE_SENSORS_H
#define	READ_IR_RANGE_SENSORS_H
    
#include "configuration.h"

void config_ir_range_finders(void) {
    _TRISB8 = 1;    // pin 12
    _TRISB7 = 1;    // pin 11
}

bool check_forwards_obstacle(void) {
    if (forwards_ir_range_sensor == 1) {
        return true;
    }
    return false;
}

bool check_left_obstacle(void) {
    if (left_ir_range_sensor == 1) {
        return true;
    }
    return false;
}

#endif	/* READ_IR_RANGE_SENSORS_H */

