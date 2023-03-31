/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
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

enum task_type leave_lander(void) {
    move_linear_at_velocity(0.35);
    wait(0.1);
    while (read_task_qrd() != black) {
        line_follow();
    }
    move_linear_at_velocity(-0.35);
    wait(0.05);
    pivot_to_angle(-140, -124, true);  // 90 deg turn clockwise
    reset_line_follow_errors();
    return LINE_FOLLOW;
}

unsigned int interations_count = 0;
enum task_type return_to_lander(void) {
    if (interations_count == 0) {
        move_linear_at_velocity(-0.50);
        wait(0.25);
        pivot_to_angle(-140, -104, true);  // 90 deg turn clockwise
        reset_line_follow_errors();
    }
    interations_count++;
    if (interations_count >= 2300) {
        move_linear_at_velocity(0);
        interations_count = 0;
        return DATA_TRANSMISSION;
    }
    else if (interations_count >= 700) {
        move_linear_at_velocity(0.40);
    }
    else {
        line_follow();
    }
    return RETURN_TO_LANDER;
}

#endif	/* LEAVE_LANDER_H */

