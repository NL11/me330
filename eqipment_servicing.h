/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
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

void service_equipment(void) {
    move_linear_at_velocity(0.30);
    wait(0.10);
    move_linear_at_velocity(0.10);
    wait(0.10);
    move_linear_at_velocity(0);
    wait(0.10);
    
    move_linear_at_velocity(0.35);
    wait(0.1);
    pivot_to_angle(-150, -102, true);  // 90 deg turn counterclockwise
    move_linear_at_velocity(-0.35);
    wait(1.2);
    move_linear_at_velocity(0.0);
    wait(1);
    
    move_linear_at_velocity(0.35);
    while (read_left_qrd() <= QRD_THRESHOLD || read_right_qrd() <= QRD_THRESHOLD) {
        Nop();
    }
    move_linear_to_position(0.5, 0.05, true);
    pivot_to_angle(120, 124, true);  // 90 deg turn clockwise
}

#endif	/* EQUIPMENT_SERVICING_H */
