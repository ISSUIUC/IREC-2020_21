#ifndef MAIN_CPP
#define MAIN_CPP

#include <ChRt.h>
#include <Arduino.h>
#define INTERVAL_MS 10
#define ERR_THRESHOLD 60
// #define READ_PORT 27
// #define WRITE_PORT 26

#define READ_PORT 26
#define WRITE_PORT 27

static THD_WORKING_AREA(pulse_sender_WA, 512);
static THD_WORKING_AREA(pulse_receiver_WA, 512);

bool write_val;
static virtual_timer_t write_vt;
static void toggle_write() {
  chSysLockFromISR();
  chVTSetI(&write_vt, TIME_MS2I(INTERVAL_MS), toggle_write, NULL);
  chSysUnlockFromISR();
  digitalWrite(/*INSERT PIN*/ WRITE_PORT, write_val);
  Serial.printf("writing %d\n\r", write_val);
  write_val = !write_val;
}
static THD_FUNCTION(pulse_sender_THD, arg) {
  write_val = 0;
  chVTObjectInit(&write_vt);
  chVTSet(&write_vt, TIME_MS2I(INTERVAL_MS), toggle_write, NULL);
}

bool read_val;
int num_missed;
static virtual_timer_t read_vt;
static void toggle_read() {
  chSysLockFromISR();
  chVTSetI(&read_vt, TIME_MS2I(INTERVAL_MS), toggle_read, NULL);
  chSysUnlockFromISR();
  int received = digitalRead(/*INSERT PIN*/ READ_PORT);
  Serial.printf("read %d\n\r", received);
  if (received != read_val) {
    num_missed++;
  } else {
    num_missed = 0;
    read_val = !read_val;
  }
  if (num_missed > /*SOME THRESHOLD*/ ERR_THRESHOLD) {
    /*ERROR HERE*/
    Serial.println("ERROR!");
    return;
  }
}
static THD_FUNCTION(pulse_receiver_THD, arg) {
  read_val = digitalRead(/*INSERT PIN*/ READ_PORT);
  read_val != read_val;
  chVTObjectInit(&read_vt);
  chVTSet(&read_vt, TIME_MS2I(INTERVAL_MS), toggle_read, NULL);
}

/**
 * @brief Starts all of the threads.
 *
 */
void chSetup(){
  Serial.println("In ChSetup");
  chThdCreateStatic(pulse_sender_WA, sizeof(pulse_sender_WA), NORMALPRIO, pulse_sender_THD, NULL);
  chThdCreateStatic(pulse_receiver_WA, sizeof(pulse_receiver_WA), NORMALPRIO, pulse_receiver_THD, NULL);

  while(true);
}

void setup() {
  Serial.begin(115200);

  // while (!Serial) {}

  // pinMode(LED_WHITE, OUTPUT);
  // digitalWrite(LED_WHITE_HIGH);

  Serial.println("Starting ChibiOS");
  pinMode(READ_PORT, INPUT);
  pinMode(WRITE_PORT, OUTPUT);
  chBegin(chSetup);

  while(true);
}

void loop() {
  // not used
}

#endif
