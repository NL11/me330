/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes the logic for line following using a PID controller

#ifndef SATILLITE_TRANSMISSION_H
#define	SATILLITE_TRANSMISSION_H
    
#include "configuration.h"
#include "read_qrd.h"
#include "robot_motion.h"
#include "servo.h"

#define door_starting_angle 60
#define door_max_angle 130
#define offset_to_center_of_ir_beam 15

enum task_type transmit_to_satilite(void) {
    int door_angle = door_starting_angle;
    while(door_angle <= door_max_angle && !check_satallite_laser()) {
        set_door_servo(door_angle);
        door_angle += 1;
        wait(0.04);
    }
    set_door_servo(door_angle + offset_to_center_of_ir_beam);
    wait(0.1);
    laser_on();
    return IDLE;
}

#endif	/* SATILLITE_TRANSMISSION_H */

