#ifndef THRESHOLDS_H
#define THRESHOLDS_H

//TODO set values for the thresholds
float coast_time_thresh = 500; // .5 seconds, probably maybe
float coast_thresh = -9; //"-9" because below -9.81 must be freefall, add some buffer for data noise
float apogee_thresh = 0;
float coast_timer;
float apogee_timer;
float descent_timer;
float burn_timer; //Measuring how long the burn happens
float launch_az_thresh = 3500; //Minimum vert acc. for launch detection
float launch_time_thresh = 300; //Minimum time to confirm launch
float apogee_time_thresh = 500;
bool launch_init = false;
bool coast_init = false;

#endif
