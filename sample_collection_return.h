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
    pivot_to_angle(180, -102, true);  // 90 deg turn counterclockwise
    move_linear_to_position(0.5, -0.15, true); // 1.5 tiles
    wait(2);
    // TODO: Do arc drive away from collection to save time
    move_linear_to_position(0.5, 0.2, true); // 1.5 tiles
    pivot_to_angle(180, 104, true);  // 90 deg turn counterclockwise
}

void return_sample(void) {
    move_linear_to_position(0.5, -0.075, true); // 0.5 tiles
    bool ball_color = read_ball_qrd();
    if (ball_color == white) {
        pivot_to_angle(180, 104, true);  // 90 deg turn clockwise
        set_door_servo(90);
        pivot_to_angle(180, -102, true);
    }
    else {
        pivot_to_angle(180, -102, true);  // 90 deg turn counterclockwise
        set_door_servo(90);
        pivot_to_angle(180, 104, true);
    }
}

#endif	/* SAMPLE_COLLECTION_RETURN_H */

