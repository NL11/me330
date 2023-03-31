/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef LEAVE_LANDER_H
#define	LEAVE_LANDER_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "servo.h"

void leave_lander(void) {
    move_linear_at_velocity(0.35);
    wait(0.1);
    while (read_task_qrd() != black) {
        line_follow();
    }
    move_linear_at_velocity(-0.35);
    wait(0.05);
    pivot_to_angle(-140, -124, true);  // 90 deg turn clockwise
}

#endif	/* LEAVE_LANDER_H */

