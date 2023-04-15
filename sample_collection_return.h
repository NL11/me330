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


enum task_type collect_sample(void) {
    unsigned int collect_sample_interations_count = 0;
    while (collect_sample_interations_count <= 180) {
        line_follow();
        collect_sample_interations_count++;
    }
    
    move_linear_at_velocity(0);
    wait(0.10);
    pivot_to_angle(-140, -100, true);  // 90 deg turn counterclockwise
    move_linear_at_velocity(-0.5);
    wait(0.85);
    move_linear_at_velocity(0.0);
    wait(1);
    
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
        wait(0.65);
        set_door_servo(50); // closed
        
        move_linear_at_velocity(0.35);
        while (read_right_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        
        move_linear_at_velocity(0.40);
        wait(0.35);
        pivot_at_angular_velocity(-140);
        while (read_left_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        pivot_at_angular_velocity(0);
    }
    else {
        pivot_to_angle(-150, -50, true);  // 45 deg turn clockwise
        
        move_linear_at_velocity(0);
        wait(0.15);
        move_linear_at_velocity(-0.40);
        wait(0.55);
        move_linear_at_velocity(0);
        wait(0.35);
        set_door_servo(165); // open
        wait(0.65);
        set_door_servo(50); // closed
        
        move_linear_at_velocity(0.35);
        while (read_right_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
//        
        move_linear_at_velocity(0.40);
        wait(0.35);
        pivot_at_angular_velocity(140);
        while (read_right_qrd() <= QRD_THRESHOLD) {
            Nop();
        }
        pivot_at_angular_velocity(0);
    }
}

static unsigned int return_sample_interations_count = 0;
enum task_type return_sample(void) {
    return_sample_interations_count++;
    if (return_sample_interations_count >= 450) {
        place_sample_in_box();
        reset_line_follow_errors();
        return_sample_interations_count = 0;
        return LINE_FOLLOW;
    }
    line_follow();
    return SAMPLE_RETURN;
}

#endif	/* SAMPLE_COLLECTION_RETURN_H */

