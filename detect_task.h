/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef DETECT_TASK_H
#define	DETECT_TASK_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"

// Weights of sensors
#define MIN_READINGS_FOR_TASK_LINE_DETECTED 5 // readings
#define MAX_READINGS_FOR_TASK_DETECTED 50 // readings

static int num_readings_black = 0;
static int num_readings_white = 0;
static int num_lines_detected = 0;
static bool task_line_already_detected = false;
static bool task_line_detection_started = false;
// Get current task type
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
    if (num_readings_white >= MIN_READINGS_FOR_TASK_LINE_DETECTED) {
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
            switch(num_lines_detected){
                case (2):
                    num_readings_black = 0;
                    num_readings_white = 0;
                    num_lines_detected = 0;
                    return SAMPLE_COLLECTION;
                    break;
                case (3):
                    num_readings_black = 0;
                    num_readings_white = 0;
                    num_lines_detected = 0;
                    return SAMPLE_RETURN;
                    break;
                case(4):
                    num_readings_black = 0;
                    num_readings_white = 0;
                    num_lines_detected = 0;
                    return CANYON_NAVIGATION;
                    break;
                default:
                    num_readings_black = 0;
                    num_readings_white = 0;
                    num_lines_detected = 0;
                    return LINE_FOLLOW;
                    break;
            }
        }
    }
    
    return LINE_FOLLOW;
}

enum task_type detect_equipment_servicing(void) {
    // TODO: check IR sensor, code is very similar to read_qrd
    return LINE_FOLLOW;
}

static int num_tiles_traveled = 0;
void update_distance_traveled(void) {
    // TODO: Code this function
    // Canyon = 6 tiles
    // Sample collection = 1 tile
    // Sample return = 1 tile but maybe only 1/2 traveled
    // Equipment servicing = 1 tile
    // Other tiles = 12 tiles
    // Total = 22 tiles
}

enum task_type detect_data_transmission(void) {
    if (num_tiles_traveled == 22) {
        return DATA_TRANSMISSION;
    }
    return LINE_FOLLOW;
}

enum task_type detect_task() {
    enum task_type new_state = detect_task_lines();
//    if (new_state != LINE_FOLLOW) {
//        return new_state;
//    }
//    new_state = detect_equipment_servicing();
//    if (new_state != LINE_FOLLOW) {
//        return new_state;
//    }
//    new_state = detect_data_transmission();
    return new_state;
}

#endif	/* DETECT_TASK_H */

