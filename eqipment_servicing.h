/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef EQUIPMENT_SERVICING_H
#define	EQUIPMENT_SERVICING_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "servo.h"

enum task_type service_equipment(void) {
    unsigned int service_equipment_interations_count = 0;
    while (service_equipment_interations_count <= 100) {
        line_follow();
        service_equipment_interations_count++;
    }
    
    move_linear_at_velocity(0);
    wait(0.10);
    pivot_to_angle(-140, -100, true);  // 90 deg turn counterclockwise
    move_linear_at_velocity(-0.5);
    wait(0.85);
    move_linear_at_velocity(0.0);
    wait(0.75);
    
    move_linear_at_velocity(0.5);
    while (read_left_qrd() <= QRD_THRESHOLD || read_right_qrd() <= QRD_THRESHOLD) {
        Nop();
    }
    move_linear_to_position(0.5, 0.05, true);
    pivot_at_angular_velocity(140);
    while (read_left_qrd() <= QRD_THRESHOLD) {
        Nop();
    }
    pivot_at_angular_velocity(0);
//    pivot_to_angle(140, 124, true);  // 90 deg turn clockwise
    reset_line_follow_errors();
    return LINE_FOLLOW;
}

#endif	/* EQUIPMENT_SERVICING_H */

