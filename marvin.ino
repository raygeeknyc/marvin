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

#include <Adafruit_SoftServo.h>  // SoftwareServo (works on non PWM pins)

Adafruit_SoftServo myServo;  // create servo object to control a servo 

// Hardware pin on the ATTiny85
#define PIN_LED 0
#define PIN_SERVO 1
#define PIN_PING_TRIG 3
#define PIN_PING_ECHO 4

// Servo declarations
#define SERVO_MAX_POS 180
#define SERVO_MIN_POS 0
#define SERVO_MOVEMENT_DELAY_MS 15
#define SERVO_SWEEP_STEP 5

// LED declarations
#define LED_MIN 0
#define LED_MAX 255
#define LED_STEP 5
#define LED_PULSE_DUR_MS 40
int LEDLevel;
unsigned long int lastLEDPulse;

// Ping sensor declarations
#define PING_SAMPLES 5
#define MAX_DISTANCE_CM 200
#define DISTANCE_DELTA_THRESHOLD_CM 8
#define LOOP_MIN_TIME_MS 80
#define LOOP_TIMER_STEP_MS 5
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_CM); // NewPing setup of pins and maximum distance.
int dist, prev_dist;
const int DEFAULT_LOCATION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2;
int min_distance;  // min distance seen during a sweep()
int min_location;  // Servo position at min distance readings during a sweep()


void startLEDPulsing() {
  LEDLevel = LED_MIN;
  LEDStep = LED_STEP;
  lastLEDPulse = 0;
}

void stopLEDPulsing() {
  LEDLevel = LED_MIN;
  analogWrite(PIN_LED, LEDLevel);
}

void refreshPulsingLED() {
  if ((millis() - lastLEDPulse) > LED_PULSE_DUR_MS) {
    lEDLevel += LEDStep;
    if (LEDLevel < LED_MIN || LEDLevel > LED_MAX) {
      LEDStep *= -1;
      LEDLevel += LEDStep;
    }
    analogWrite(PIN_LED, LEDLevel);
    lastLEDPulse = millis();
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
  myServo.write(pos);
  myServo.refresh();
  delay(SERVO_MOVEMENT_DELAY_MS);}
}

void setup() {
  long timerStart = millis();
  // Pulse LED for 2 seconds
  startLEDPulsing();
  while (millis() < timerStart+2000) {
    refreshPulsingLED();
  }

  myServo.attach(SERVO_PIN);
  // Sweep the servo back and forth
  pointAt(SERVO_MIN_POS);
  for (int i=SERVO_MIN_POS; i<=SERVO_MAX_POS; i+=SERVO_SWEEP_STEP) {
    pointAt(i);
  }
  for (; i>=SERVO_MIN_POS; i-=SERVO_SWEEP_STEP) {
    pointAt(i);
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Store an initial distance reading
  dist = getDistanceCM();
}

void loop() {
  int loop_timer = millis();

  // Guarantee a minimum time spent in each loop iteration
  while ((millis() - loop_timer) < LOOP_MIN_TIME_MS) {
    delay(LOOP_TIMER_STEP_MS);
  }
}
