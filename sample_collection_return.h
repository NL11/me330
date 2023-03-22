/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef SAMPLE_COLLECTION_RETURN_H
#define	SAMPLE_COLLECTION_RETURN_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "servo.h"

void collect_sample(void) {
//    move_linear_to_position(0.5, 0.15, true); // 1.5 tiles
    move_linear_at_velocity(0.5);
    wait(0.5);
    pivot_to_angle(-150, -102, true);  // 90 deg turn counterclockwise
//    move_linear_to_position(0.5, -0.15, true); // 1.5 tiles
    move_linear_at_velocity(-0.5);
    wait(0.5);
    move_linear_at_velocity(0.0);
    wait(1);
//    move_linear_to_position(0.5, 0.2, true); // 1.5 tiles
    move_linear_at_velocity(0.5);
    wait(0.5);
    pivot_to_angle(150, 104, true);  // 90 deg turn counterclockwise
}

void return_sample(void) {
    move_linear_to_position(0.5, -0.075, true); // 0.5 tiles
    bool ball_color = read_ball_qrd();
    if (ball_color == white) {
        pivot_to_angle(150, 104, true);  // 90 deg turn clockwise
        set_door_servo(165); // open
        wait(0.5);
        set_door_servo(40); // closed
        pivot_to_angle(-150, -102, true);
    }
    else {
        pivot_to_angle(-150, -102, true);  // 90 deg turn counterclockwise
        set_door_servo(165); // open
        wait(0.5);
        set_door_servo(40); // closed
        pivot_to_angle(150, 104, true);
    }
}

#endif	/* SAMPLE_COLLECTION_RETURN_H */

