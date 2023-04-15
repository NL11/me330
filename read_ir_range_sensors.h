/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
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
    _TRISA4 = 1;    // pin 10
//    _TRISB12 = 1;    // pin 15 --> Taken for Left Task QRD
}

bool check_forwards_obstacle(void) {
    if (forwards_ir_range_sensor == 1) {
        return true;
    }
    return false;
}

//bool check_left_obstacle(void) {
//    if (left_ir_range_sensor == 1) {
//        return true;
//    }
//    return false;
//}

bool check_right_obstacle(void) {
    if (right_ir_range_sensor == 1) { //left_ir_range_sensor moved to the Right side on 4/14/2023  
        return true;
    }
    return false;
}

bool check_satallite_laser(void) {
    if (satallite_ir_sensor == 0) {
        return true;
    }
    return false;
}

bool check_equipment_servicing(void) {
    if (equipment_servicing_ir_sensor == 0) {
        return true;
    }
    return false;
}


#endif	/* READ_IR_RANGE_SENSORS_H */

