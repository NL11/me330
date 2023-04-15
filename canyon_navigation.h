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

enum task_type navigate_canyon(void) {
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
            pivot_at_angular_velocity(150);
            while (read_right_qrd() <= QRD_THRESHOLD) {
                Nop();
            }
            pivot_at_angular_velocity(0);
        }
        else {
            pivot_at_angular_velocity(-150);
            while (read_left_qrd() <= QRD_THRESHOLD) {
                Nop();
            }
            pivot_at_angular_velocity(0);
        }
        reset_line_follow_errors();
        return LINE_FOLLOW;
    }
    return CANYON_NAVIGATION;
}

#endif	/* NAVIGATE_CANYON_H */

