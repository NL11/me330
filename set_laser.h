/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

// File that includes code to read the current state of a QRD sensor

#ifndef SET_LASER_H
#define	SET_LASER_H

#include "configuration.h"

void config_laser(void) {
    _TRISB9 = 0;    // pin 17 AN10
}


void laser_on(void) {
    laser = 1;
}


void laser_off(void) {
    laser = 0;
}

#endif	/* SET_LASER_H */

