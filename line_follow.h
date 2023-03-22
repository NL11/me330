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

static double motor_base_speed = 150;
void set_line_follow_speed(double speed) {
    motor_base_speed = speed;
}

#define DEAD_ZONE_MAX 0.1
#define CONTROLLED_ZONE_MAX 0.5

#define WHEEL_SPEED_CONTROL_RATIO 1

// PID tuning for controlled zone
#define Kp_controlled 90.0
#define Ki_controlled 0.0
#define Kd_controlled 25.0

// PID tuning for ballistic zone
#define Kp_balistic 260.0
#define Ki_balistic 0.0
#define Kd_balistic 50.0

static double current_error = 0;
static double error_integral = 0;
static double last_error = 0;
// compute weighted average of sensor readings
void update_error(void) {
    int right_qrd_value = read_right_qrd();
    int center_qrd_value = read_center_qrd();
    int left_qrd_value = read_left_qrd();
    current_error = (right_qrd_value*RIGHT_QRD_WEIGHT + center_qrd_value*CENTER_QRD_WEIGHT + left_qrd_value*LEFT_QRD_WEIGHT) /
		((double)(right_qrd_value + center_qrd_value + left_qrd_value));
    current_error = current_error - AVERAGE_WEIGHT;
}

double compute_pid(void) {
    if (fabs(current_error) < DEAD_ZONE_MAX) {
        return 0;
    }
    double P = current_error;
    double I = error_integral + current_error;
    double D = current_error - last_error;
    if (fabs(current_error) < CONTROLLED_ZONE_MAX) {
        return (Kp_controlled*P) + (Ki_controlled*I) + (Kd_controlled*D);
    }
    else {
        return (Kp_balistic*P) + (Ki_balistic*I) + (Kd_balistic*D);
    }
}

void line_follow(void) {
    update_error();
    double pid_value = compute_pid();
    last_error = current_error;
    int left_wheel_speed = motor_base_speed;
    int right_wheel_speed = motor_base_speed;
    if (pid_value > 0) {
        left_wheel_speed = left_wheel_speed + (int)pid_value;
        right_wheel_speed = right_wheel_speed - (int)(pid_value/WHEEL_SPEED_CONTROL_RATIO);
    }
    else {
        left_wheel_speed = left_wheel_speed + (int)(pid_value/WHEEL_SPEED_CONTROL_RATIO);
        right_wheel_speed = right_wheel_speed - (int)pid_value;
    }
    turn_motors_at_speed(left_wheel_speed, right_wheel_speed);
}

#endif	/* LINE_FOLLOW_H */

