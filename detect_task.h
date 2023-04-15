/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef DETECT_TASK_H
#define	DETECT_TASK_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "read_ir_range_sensors.h"
#include "set_laser.h"

static int tasks_completed = 0;

// Weights of sensors
#define MIN_READINGS_FOR_TASK_LINE_DETECTED 7 // readings 9
#define MIN_READINGS_FOR_WHITE_BREAK_DETECTED 4 // readings 3
#define MAX_READINGS_FOR_TASK_DETECTED 90 // readings 90

static int num_readings_black = 0;
static int num_readings_white = 0;
static int num_lines_detected = 0;
static bool task_line_already_detected = false;
static bool task_line_detection_started = false;

enum task_type detect_task_lines() {
    if (read_task_qrd() == black) {
        num_readings_black++;
        num_readings_white = 0;
    }
    if (read_task_qrd() == white) {
        num_readings_white++;
        num_readings_black = 0;
    }
    
    if (num_readings_black >= MIN_READINGS_FOR_TASK_LINE_DETECTED) {
        task_line_detection_started = true;
    }
    else if (num_readings_white >= MIN_READINGS_FOR_WHITE_BREAK_DETECTED) {
        task_line_already_detected = false;
    }
    
    if (task_line_detection_started) {
        if (!task_line_already_detected && num_readings_black >= MIN_READINGS_FOR_TASK_LINE_DETECTED) {
            num_lines_detected++;
            task_line_already_detected = true;
        }
        if (num_readings_white >= MAX_READINGS_FOR_TASK_DETECTED) {
            task_line_detection_started = false;
            task_line_already_detected = false;
            int num_lines_detected_temp = num_lines_detected;
            num_readings_black = 0;
            num_readings_white = 0;
            num_lines_detected = 0;
            switch(num_lines_detected_temp){
                case (1):
                    return LINE_FOLLOW;
                    break;
                case (2):
                    tasks_completed++;
                    return SAMPLE_COLLECTION;
                    break;
                case (3):
                    tasks_completed++;
                    return SAMPLE_RETURN;
                    break;
                case(4):
                    tasks_completed++;
                    return CANYON_NAVIGATION;
                    break;
                default:
                    return LINE_FOLLOW;
                    break;
            }
        }
    }
    
    return LINE_FOLLOW;
}

#define NUM_READINGS_FOR_LANDER_LINE_DETECTED 60
static int num_lander_line_readings = 0;

bool check_lander() {
    if (read_lander_qrd() == black) {
        num_lander_line_readings++;
    }
    else {
        num_lander_line_readings = 0;
    }
    if (num_lander_line_readings >= NUM_READINGS_FOR_LANDER_LINE_DETECTED) {
        return true;
    }
    return false;
}

enum task_type detect_task() {
    enum task_type new_state = detect_task_lines();
    if (new_state != LINE_FOLLOW) {
        return new_state;
    }
//    if (check_equipment_servicing()) {
//        tasks_completed++;
//        return EQUIPMENT_SERVICING;
//    }
    if (check_lander() && tasks_completed >= 3) {
        return RETURN_TO_LANDER;
    }
    return new_state;
}

#endif	/* DETECT_TASK_H */

