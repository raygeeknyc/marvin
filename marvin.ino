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

const unsigned long int FOREVER_MS = (unsigned long int) -1;

// Servo and behavior declarations
#define SERVO_POS_MAX 180
#define SERVO_POS_MIN 0
#define SERVO_MOVEMENT_DELAY_MS 25
#define LOOP_MIN_TIME_MS 10000
#define LOOP_TIMER_STEP_MS 5
#define POINT_DUR_MS 3000
#define POST_POINT_WAIT_MS 4000
#define SERVO_SWEEP_STEP 5
#define SWEEP_COUNT 2
unsigned long int lastServoPointAt;
Adafruit_SoftServo myServo;

// LED declarations
#define LED_PULSE_MIN 20
#define LED_PULSE_MAX 200
#define LED_MIN 0
#define LED_MAX 255

#define LED_STEP 5
#define LED_PULSE_DUR_MS 15
#define LONG_PULSE_DUR_MS 2000
#define LED_BLINK_DUR_MS 300
int LEDLevel;
int LEDStep;
unsigned long int lastLEDPulseAt;

// Ping sensor declarations
#define PING_SAMPLES 5
#define MAX_DISTANCE_CM 200
#define MIN_POS_DELTA_CM_THRESHOLD 6
#define PING_DELAY_MS 80
const int MIN_POS_DELTA_THRESHOLD = SERVO_SWEEP_STEP * 2;
unsigned long int lastPingAt;


const int DEFAULT_LOCATION = (SERVO_POS_MAX - SERVO_POS_MIN) / 2;
int minDistance;  // min distance seen during a sweep()
int minLocation;  // Servo position at min distance reading during a sweep()
int prevLocation;  // The last sweep's closest object's heading
NewPing sonar(PIN_PING_TRIG, PIN_PING_ECHO, MAX_DISTANCE_CM); // NewPing setup of pins and maximum distance.


void startLEDPulsing(bool startHigh=false) {
  if (startHigh) {
    LEDLevel = LED_PULSE_MAX;
    LEDStep = LED_STEP*-1;
  } else {
    LEDLevel = LED_PULSE_MIN;
    LEDStep = LED_STEP;
  }
  lastLEDPulseAt = 0;
}

void stopLEDPulsing(int offVal=LED_MIN) {
  LEDLevel = LED_PULSE_MIN;
  analogWrite(PIN_LED, offVal);
}

void refreshPulsingLED() {
  if (lastLEDPulseAt> millis()) 
    return;
  
  if ((millis() - lastLEDPulseAt) > LED_PULSE_DUR_MS) {
    LEDLevel += LEDStep;
    if (LEDLevel < LED_PULSE_MIN || LEDLevel > LED_PULSE_MAX) {
      LEDStep *= -1;
      LEDLevel += LEDStep;
    }
    analogWrite(PIN_LED, LEDLevel);
    lastLEDPulseAt = millis();
  }
}

void shineLED(int brightness=LED_MAX) {
  analogWrite(PIN_LED, brightness);
  lastLEDPulseAt = FOREVER_MS;
}

int getPingReading() {
  long remainingPingDelay = PING_DELAY_MS - (millis() - lastPingAt);
  if (remainingPingDelay > 0) {
    delay(remainingPingDelay);
  }
  int distance = sonar.ping_cm();
  lastPingAt = millis();
  return distance;
}

int getDistanceCM(bool smooth=true) {
  if (!smooth) {
    return getPingReading();
  }
  // Return smoothed ping sensor reading
  int sum = 0, min = 99999, max = -99999;
  for (int i = 0; i < PING_SAMPLES; i++) {
    refreshPulsingLED();
    int sample = getPingReading();
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
  refreshServo();
  lastServoPointAt = millis();
}

void refreshServo() {
    myServo.refresh();
}

void setup() {
  long timerStart = millis();
  lastServoPointAt = 0;
  lastPingAt = 0;
  prevLocation = 0;

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
};

void loop() {
  unsigned long int loop_timer = millis();
  int currentDistance, prevDistance, distanceDelta;
  int pos;

  // Wait until we see a change in distance at the current position
  currentDistance = getDistanceCM(false);
  prevDistance = currentDistance;
  while (distanceDelta=(currentDistance - prevDistance),abs(distanceDelta) < MIN_POS_DELTA_CM_THRESHOLD) {
    currentDistance = getDistanceCM(false);
    refreshPulsingLED();
    delay(LOOP_TIMER_STEP_MS);
  }

  stopLEDPulsing(LED_MIN);

  shineLED();
  delay(LED_BLINK_DUR_MS);

  stopLEDPulsing(LED_MIN);
  delay(LED_BLINK_DUR_MS);

  shineLED();
  delay(LED_BLINK_DUR_MS);

  startLEDPulsing();
  // Sweep and record the servo position where we see the closest object
  minDistance = MAX_DISTANCE_CM;
  minLocation = DEFAULT_LOCATION;
  for (int sweep=0; sweep<SWEEP_COUNT; sweep++) {
    for (pos=SERVO_POS_MIN; pos<SERVO_POS_MAX; pos+=SERVO_SWEEP_STEP) {
      pointAt(pos);
      refreshPulsingLED();
      currentDistance = getDistanceCM(false);
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minLocation = pos;
      }
    }
    for (pos=SERVO_POS_MAX; pos>SERVO_POS_MIN; pos-=SERVO_SWEEP_STEP) {
      pointAt(pos);
      refreshPulsingLED();
      currentDistance = getDistanceCM(false);
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minLocation = pos;
      }
    }
  }
  
  // Point to the closest thing and shine at it
  prevLocation = minLocation;
  stopLEDPulsing();
  int dir=(minLocation<pos?-1:1);
  shineLED();
  while (pos != minLocation) {
    pos+=dir;
    pointAt(pos);
  }
  delay(POINT_DUR_MS);
  unsigned long int pointFinishedAt = millis();
  while ((millis() - pointFinishedAt) < POST_POINT_WAIT_MS) {
    delay(LOOP_TIMER_STEP_MS);
  }
  startLEDPulsing();
  // Guarantee a minimum time spent in each loop iteration
  while ((millis() - loop_timer) < LOOP_MIN_TIME_MS) {
   refreshPulsingLED();
   delay(LOOP_TIMER_STEP_MS);
  }
}
