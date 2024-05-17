/*
 * @author("Raymond Blum" <raygeeknyc@gmail.com>)
 * targeted for an ATTiny85 @ 1,8 Mhz but should work on any Arduino compatible controller
 *
 * Copyright (c) 2015, 2016, 2024 by Raymond Blum
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 **/

#include <Adafruit_SoftServo.h> 
#include <NewPing.h>

// Hardware pin assignment on the ATTiny85
#define PIN_LED 0
#define PIN_SERVO 1
#define PIN_PING_TRIG 3
#define PIN_PING_ECHO 4

// Servo declarations
#define SERVO_POS_MAX 180
#define SERVO_POS_MIN 0
#define SERVO_MOVEMENT_DELAY_MS 15
#define SERVO_SWEEP_STEP 5
#define SWEEP_COUNT 2
unsigned long int lastServoPointAt;
Adafruit_SoftServo myServo;

// LED declarations
#define LED_MIN 20
#define LED_MAX 200
#define LED_STEP 5
#define LED_PULSE_DUR_MS 15
#define LONG_PULSE_DUR_MS 2000
int LEDLevel;
int LEDStep;
unsigned long int lastLEDPulseAt;

// Ping sensor declarations
#define PING_SAMPLES 5
#define MAX_DISTANCE_CM 200
#define DISTANCE_DELTA_THRESHOLD_CM 8
#define LOOP_MIN_TIME_MS 5000
#define LOOP_TIMER_STEP_MS 5
int dist, prev_dist;
const int DEFAULT_LOCATION = (SERVO_POS_MAX - SERVO_POS_MIN) / 2;
int min_distance;  // min distance seen during a sweep()
int min_location;  // Servo position at min distance reading during a sweep()
NewPing sonar(PIN_PING_TRIG, PIN_PING_ECHO, MAX_DISTANCE_CM); // NewPing setup of pins and maximum distance.


void startLEDPulsing(bool startHigh=false) {
  if (startHigh) {
    LEDLevel = LED_MAX;
    LEDStep = LED_STEP*-1;
  } else {
    LEDLevel = LED_MIN;
    LEDStep = LED_STEP;
  }
  lastLEDPulseAt = 0;
}

void stopLEDPulsing(int offVal=0) {
  LEDLevel = LED_MIN;
  analogWrite(PIN_LED, offVal);
}

void refreshPulsingLED() {
  if ((millis() - lastLEDPulseAt) > LED_PULSE_DUR_MS) {
    LEDLevel += LEDStep;
    if (LEDLevel < LED_MIN || LEDLevel > LED_MAX) {
      LEDStep *= -1;
      LEDLevel += LEDStep;
    }
    analogWrite(PIN_LED, LEDLevel);
    lastLEDPulseAt = millis();
  }
}

int getDistanceCM() {
// Return smoothed ping sensor reading
  int sum = 0, min = 9999, max = -1;
  for (int i = 0; i < PING_SAMPLES; i++) {
    int sample = sonar.ping_cm();
    if (sample < min) min = sample;
    if (sample > max) max = sample;
    sum += sample;
  }
  sum -= (min + max);
  return (sum / (PING_SAMPLES - 2));
}

void pointAt(int pos) {
  long remainingServoDelay = SERVO_MOVEMENT_DELAY_MS - (millis() - lastServoPointAt);
  if (remainingServoDelay > 0) {
    delay(remainingServoDelay);
  }
  myServo.write(pos);
  myServo.refresh();
  lastServoPointAt = millis();
}

void setup() {
  long timerStart = millis();
  lastServoPointAt = 0;

  // Pulse LED for 2 seconds
  startLEDPulsing();
  while (millis() < (timerStart+LONG_PULSE_DUR_MS)) {
   refreshPulsingLED();
  }
  stopLEDPulsing();

  myServo.attach(PIN_SERVO);
  // Sweep the servo back and forth
  pointAt(SERVO_POS_MIN);
  for (int i=SERVO_POS_MIN; i<=SERVO_POS_MAX; i+=SERVO_SWEEP_STEP) {
    pointAt(i);
  }
  for (int i=SERVO_POS_MAX; i>=SERVO_POS_MIN; i-=SERVO_SWEEP_STEP) {
    pointAt(i);
  }

  pinMode(PIN_PING_TRIG, OUTPUT);
  pinMode(PIN_PING_ECHO, INPUT);

  startLEDPulsing(true);
  // Store an initial distance reading
  dist = getDistanceCM();
};

void loop() {
  unsigned long int loop_timer = millis();

  for (int sweep=0; sweep<SWEEP_COUNT; sweep++) {
    for (int pos=SERVO_POS_MIN; pos<SERVO_POS_MAX; pos+=SERVO_SWEEP_STEP) {
      refreshPulsingLED();
      pointAt(pos);
    }
  }
  // Guarantee a minimum time spent in each loop iteration
  if (millis() > loop_timer) {  // the millis() counter resets to 0
    while ((millis() - loop_timer) < LOOP_MIN_TIME_MS) {
      refreshPulsingLED();
      delay(LOOP_TIMER_STEP_MS);
    }
  } else {
    refreshPulsingLED();
    delay(LOOP_TIMER_STEP_MS);  // easy guess
  }
}
