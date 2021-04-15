#ifndef MAIN_CPP
#define MAIN_CPP

#include <ChRt.h>

static THD_WORKING_AREA(pulse_sender_WA, 512);
static THD_WORKING_AREA(pulse_receiver_WA, 512);

bool write_val;
static virtual_timer_t write_vt;
static void toggle_write() {
  digitalWrite(/*INSERT PIN*/ 1, write_val);
  write_val = !write_val;
  chSysLockFromISR();
  chVTSetI(&write_vt, TIME_MS2I(100), toggle_write, NULL);
  chSysUnlockFromISR();
}
static THD_FUNCTION(pulse_sender_THD, arg) {
  write_val = 0;
  chVTObjectInit(&write_vt);
  chVTSet(&write_vt, TIME_MS2I(100), toggle_write, NULL);
}

bool read_val;
int num_missed;
static virtual_timer_t read_vt;
static void toggle_read() {
  int received = digitalRead(/*INSERT PIN*/ 1);
  if (received != read_val) {
    num_missed++;
  } else {
    num_missed = 0;
    read_val = !read_val;
  }
  if (num_missed > /*SOME THRESHOLD*/ 20) {
    /*ERROR HERE*/
    return;
  }
  chSysLockFromISR();
  chVTSetI(&read_vt, TIME_MS2I(100), toggle_read, NULL);
  chSysUnlockFromISR();
}
static THD_FUNCTION(pulse_receiver_THD, arg) {
  read_val = 0;
  chVTObjectInit(&read_vt);
  chVTSet(&read_vt, TIME_MS2I(100), toggle_read, NULL);
}

/**
 * @brief Starts all of the threads.
 * 
 */
void chSetup(){ 
  chThdCreateStatic(pulse_sender_WA, sizeof(pulse_sender_WA), NORMALPRIO, pulse_sender_THD, NULL);
  chThdCreateStatic(pulse_receiver_WA, sizeof(pulse_receiver_WA), NORMALPRIO, pulse_receiver_THD, NULL);

  while(true);
}

void setup() {
  Serial.println("Starting ChibiOS");
  chBegin(chSetup);

  while(true);
}

void loop() {
  // not used
}

#endif