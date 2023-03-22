/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef LINE_FOLLOW_H
#define	LINE_FOLLOW_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"

// Weights of sensors
#define RIGHT_QRD_WEIGHT 3.0
#define CENTER_QRD_WEIGHT 2.0
#define LEFT_QRD_WEIGHT 1.0
#define AVERAGE_WEIGHT 2.0

// PID tuning
#define Kp 180.0
#define Ki 0.0
#define Kd 40.0
#define motor_base_speed 70.0 // rev/min

static double current_error = 0;
static double error_integral = 0;
static double last_error = 0;
// compute weighted average of sensor readings
void update_error(void) {
    int right_qrd_value = read_right_qrd();
    int center_qrd_value = read_center_qrd();
    int left_qrd_value = read_left_qrd();
    if (right_qrd_value <= QRD_THRESHOLD && center_qrd_value <= QRD_THRESHOLD && left_qrd_value <= QRD_THRESHOLD) {
        current_error = last_error;
    }
    else {
        current_error = (right_qrd_value*RIGHT_QRD_WEIGHT + center_qrd_value*CENTER_QRD_WEIGHT + left_qrd_value*LEFT_QRD_WEIGHT) /
            ((double)(right_qrd_value + center_qrd_value + left_qrd_value));
        current_error = current_error - AVERAGE_WEIGHT;
    }
}

double compute_pid(void) {
    double P = current_error;
    double I = error_integral + current_error;
    double D = current_error - last_error;
    return (Kp*P) + (Ki*I) + (Kd*D);
}

void line_follow(void) {
    update_error();
    double pid_value = compute_pid();
    last_error = current_error;
    int left_wheel_speed = motor_base_speed;
    int right_wheel_speed = motor_base_speed;
    if (pid_value > 0) {
        left_wheel_speed = left_wheel_speed + (int)pid_value;
        right_wheel_speed = right_wheel_speed - (int)(pid_value/1.75);
    }
    else {
        left_wheel_speed = left_wheel_speed + (int)(pid_value/1.75);
        right_wheel_speed = right_wheel_speed - (int)pid_value;
    }
    turn_motors_at_speed(left_wheel_speed, right_wheel_speed);
}

#endif	/* LINE_FOLLOW_H */

