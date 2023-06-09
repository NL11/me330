/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller and 

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

#define BASE_DEFAULT_SPEED 130 

#define DEAD_ZONE_MAX 0.1 // 0.005

// PID tuning for controlled zone
#define Kp_controlled 425.0 // mid 425 // old 420
#define Ki_controlled 0.0 // mid 0 // old 35
#define Kd_controlled 315.0 // mid 315 // old 265

static double current_error = 0;
static double error_integral = 0;
static double last_error = 0;
static bool on_line = true;
void reset_line_follow_errors() {
    current_error = 0;
    error_integral = 0;
    last_error = 0;
}

// compute weighted average of sensor readings
void update_error(void) {
    int right_qrd_value = read_right_qrd();
    int center_qrd_value = read_center_qrd();
    int left_qrd_value = read_left_qrd();
    if (right_qrd_value <= QRD_THRESHOLD && center_qrd_value <= QRD_THRESHOLD && left_qrd_value <= QRD_THRESHOLD) {
        on_line = false;
        current_error = last_error;
    }
    else {
        on_line = true;
        current_error = (right_qrd_value*RIGHT_QRD_WEIGHT + center_qrd_value*CENTER_QRD_WEIGHT + left_qrd_value*LEFT_QRD_WEIGHT) /
            ((double)(right_qrd_value + center_qrd_value + left_qrd_value));
        current_error = current_error - AVERAGE_WEIGHT;
    }
}

double compute_pid(void) {
    if (fabs(current_error) < DEAD_ZONE_MAX) {
        return 0;
    }
    double P = current_error;
    double I = error_integral + current_error;
    if (I > Kp_controlled) {
        I = Kp_controlled;
    }
    double D = current_error - last_error;
    return (Kp_controlled*P) + (Ki_controlled*I) + (Kd_controlled*D);
}

void line_follow(void) {
    update_error();
    double pid_value = compute_pid();
    last_error = current_error;
    int left_wheel_speed = BASE_DEFAULT_SPEED;
    int right_wheel_speed = BASE_DEFAULT_SPEED;
    if (pid_value > 0) {
        left_wheel_speed = left_wheel_speed;
        right_wheel_speed = right_wheel_speed - (int)(pid_value);
    }
    else {
        left_wheel_speed = left_wheel_speed + (int)(pid_value);
        right_wheel_speed = right_wheel_speed;
    }
    turn_motors_at_speed(left_wheel_speed, right_wheel_speed);
}

#endif	/* LINE_FOLLOW_H */

