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

#define NUM_READINGS_FOR_LINE_DETECTED 50 // 125

enum task_type leave_lander(void) {
    move_linear_at_velocity(0.40);
    wait(0.15);
    int num_task_readings = 0;
    while (true) {
        line_follow();
        if (read_task_qrd() == black) {
            num_task_readings++;
        }
        if (num_task_readings >= NUM_READINGS_FOR_LINE_DETECTED) {
            break;
        }
    }
    move_linear_at_velocity(-0.5);
    wait(0.1);
    pivot_at_angular_velocity(-140);
    while (read_right_qrd() <= QRD_THRESHOLD) {
        Nop();
    }
    pivot_at_angular_velocity(0);
//    move_linear_at_velocity(0.35);
//    wait(0.15);
    reset_line_follow_errors();
    return LINE_FOLLOW;
}

static unsigned int lander_iterations_count = 0;
enum task_type return_to_lander(void) {
    if (lander_iterations_count == 0) {
        move_linear_at_velocity(-0.50);
        wait(0.1);
        pivot_to_angle(-140, -70, true);  // something deg turn clockwise
        pivot_at_angular_velocity(-150);
        while (read_right_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        pivot_at_angular_velocity(0);
//        pivot_to_angle(-140, -104, true);  // 90 deg turn clockwise
        reset_line_follow_errors();
    }
    lander_iterations_count++;
    if (lander_iterations_count >= 2600) {
        move_linear_at_velocity(0);
        lander_iterations_count = 0;
        return DATA_TRANSMISSION;
    }
    else if (lander_iterations_count >= 800) {
        move_linear_at_velocity(0.40);
    }
    else {
        line_follow();
    }
    return RETURN_TO_LANDER;
}

#endif	/* LEAVE_LANDER_H */

