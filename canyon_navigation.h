/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef NAVIGATE_CANYON_H
#define	NAVIGATE_CANYON_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "read_ir_range_sensors.h"

void enable_right_ir_sensor() {
    // enable right ir
    _SMPI = 4;
    _CSS12 = 0; // AN12, Pin 15
    _TRISB12 = 0;
    _ANSB12 = 0;
    enable_right_ir = 1;
}

void disable_right_ir_sensor() {
    enable_right_ir = 0;
    // Lander QRD
    _TRISB12 = 1;
    _ANSB12 = 1; 
    _SMPI = 5;
    _CSS12 = 1; // AN12, Pin 15
}

enum task_type navigate_canyon(void) {
    enable_right_ir_sensor();
    if (check_forwards_obstacle() == true) {
        move_linear_at_velocity(0.58);
    }
    else if (check_right_obstacle() == true) {
         pivot_to_angle(140, 102, true);
    }
    else {
         pivot_to_angle(-140, -102, true);
    }
    if (read_left_qrd() > QRD_THRESHOLD && read_right_qrd() > QRD_THRESHOLD) {
        move_linear_to_position(0.5, 0.05, true);
        if (check_right_obstacle() == true) {
            pivot_at_angular_velocity(160);
            while (read_right_qrd() <= QRD_THRESHOLD) {
                Nop();
            }
            pivot_at_angular_velocity(0);
        }
        else {
            pivot_at_angular_velocity(-160);
            while (read_left_qrd() <= QRD_THRESHOLD) {
                Nop();
            }
            pivot_at_angular_velocity(0);
        }
        reset_line_follow_errors();
        disable_right_ir_sensor();
        return LINE_FOLLOW;
    }
    return CANYON_NAVIGATION;
}

#endif	/* NAVIGATE_CANYON_H */

