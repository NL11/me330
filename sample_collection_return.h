/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
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
    pivot_to_angle(140, 124, true);  // 90 deg turn clockwise
    reset_line_follow_errors();
    return LINE_FOLLOW;
}

unsigned int interations_count = 0;
enum task_type return_sample(void) {
    interations_count++;
    if (interations_count >= 450) {
        place_sample_in_box();
        reset_line_follow_errors();
        interations_count = 0;
        return LINE_FOLLOW;
    }
    line_follow();
    return SAMPLE_RETURN;
}

void place_sample_in_box(void) {
    move_linear_at_velocity(0);
    wait(0.15);
    
    bool ball_color = read_ball_qrd();
    if (ball_color == white) {
        pivot_to_angle(150, 50, true);  // 45 deg turn clockwise
        
        move_linear_at_velocity(0);
        wait(0.15);
        move_linear_at_velocity(-0.40);
        wait(0.55);
        move_linear_at_velocity(0);
        wait(0.35);
        set_door_servo(165); // open
        wait(0.5);
        set_door_servo(50); // closed
        
        move_linear_at_velocity(0.35);
        while (read_right_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        
        move_linear_at_velocity(0.40);
        wait(0.25);
        pivot_to_angle(-150, -50, true);
    }
    else {
        pivot_to_angle(-150, -60, true);  // 45 deg turn counterclockwise
        
        move_linear_at_velocity(0);
        wait(0.15);
        move_linear_at_velocity(-0.40);
        wait(0.50);
        move_linear_at_velocity(0);
        wait(0.35);
        set_door_servo(165); // open
        wait(0.5);
        set_door_servo(50); // closed
        
        move_linear_at_velocity(0.35);
        while (read_left_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        
        move_linear_at_velocity(0.40);
        wait(0.25);
        pivot_to_angle(150, 60, true);
    }
}

#endif	/* SAMPLE_COLLECTION_RETURN_H */

