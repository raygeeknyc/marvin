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

void pointAt(int loc) {
#ifdef _DEBUG
  Serial.print("pointing to ");
  Serial.println(loc);
#endif
  if (sweepPos > loc) {
#ifdef _DEBUG
    Serial.print("down from ");
    Serial.println(sweepPos);
#endif
    for (int pos=sweepPos; pos>loc; pos--) {
      point(pos);
    }
  } else if (sweepPos < loc) {
#ifdef _DEBUG
    Serial.print("up from ");
    Serial.println(sweepPos);
#endif
    for (int pos=sweepPos; pos<loc; pos++) {
      point(pos);
    }
  }
  pointed_at = millis();
}


void point(int loc) {
  myServo.write(loc);
  delayMicroseconds(MOVEMENT_DELAY);     // waits 15ms for the servo to reach the position 
}

void startSweeping() {
  min_distance = DEFAULT_DISTANCE;
  max_distance = DEFAULT_DISTANCE;
  min_location = DEFAULT_LOCATION;
  max_location = DEFAULT_LOCATION;
  sweep_swing_remaining_count = SWEEPS_COUNT;
  pointed_at = 0L;
  stopLedPulsing();
}

void stopLedPulsing() {
  led_level = LED_MIN;
  analogWrite(PIN_LED, led_level);
}

void sweep() {
  if (!isSweeping()) {
    return;
  }

  point(sweepPos);
  delayMicroseconds(MOVEMENT_DELAY);           // Give servo some time to move before giving it a new position

  if (Dir == 1) {
    if (sweepPos < MAX_POS) {
      sweepPos+=SWEEP_STEP;                     // Rotate servo to 180 degrees
    } else {                                  // Servo hit upper limit
      sweepPos = MAX_POS;                     // Keep servo angle in bounds
      Dir=!Dir;                               // Switch direction
      sweep_swing_remaining_count--;
#ifdef _DEBUG
      Serial.print("bounce ");
      Serial.println(sweep_swing_remaining_count);
#endif
    }
  } else {
    if (sweepPos > MIN_POS) {
      sweepPos-=SWEEP_STEP;                     // Rotate servo to 0 degrees
    } else {                                  // Servo hit lower limit
      sweepPos = MIN_POS;                     // Keep servo angle in bounds
      Dir=!Dir;                               // switch direction
      sweep_swing_remaining_count--;
#ifdef _DEBUG
      Serial.print("bounce ");
      Serial.println(sweep_swing_remaining_count);
#endif
    }
  }

  last_distance = getDistance();
  if (last_distance<min_distance) {
    min_distance=last_distance;
    min_location=sweepPos;
#ifdef _DEBUG
      Serial.print("New min ");
      Serial.print(last_distance);
      Serial.print(" at ");
      Serial.println(sweepPos);
#endif
    blinkLed();
  }
  if (last_distance>max_distance) {
    max_distance=last_distance;
    max_location=sweepPos;
  }
}

boolean isPointed() {
  return pointed_at != 0;
}

void setup() { 
  pinMode(PIN_PING_TRIG, OUTPUT);
  pinMode(PIN_PING_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  myServo.attach(PIN_SERVO);

  servoSweepPos = SERVO_MIN_POS;
  led_level = LED_MIN;
  lastLEDPulse = 0;

  // blink confirmation sequence
  for (int i=0; i<4; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(50);
  }
  analogWrite(PIN_LED, LED_MIN);
  Dir = 1;
  sweep_swing_remaining_count = 0;
  pointed_at = 0L;
  shine_end_at = 0;
  startLedPulsing();
  prev_distance = getDistance();
  isDistanceChanged();
#ifdef _DEBUG
  Serial.println("/setup");
#endif
}

void loop() {  
  expireLed();
  sweep();

  if (!isSweeping()) {
    if (!isPointed()) {
#ifdef _DEBUG
      Serial.println("Stopped sweeping");
#endif
      pointAt(min_location);
      getDistance();  // consume one distance reading to eliminate a spurious distance change detecti
      startLedPulsing();
    }
  }

  if (isResting()) {
#ifdef _DEBUG
    //Serial.println("*");
#endif
   if (isDistanceChanged()) {
     startSweeping();
     return;
    }
  }
}


// We'll take advantage of the built in millis() timer that goes off
// to keep track of time, and refresh the servo every 20 milliseconds
volatile uint8_t counter = 0;
SIGNAL(TIMER0_COMPA_vect) {
  // this gets called every 2 milliseconds
  counter += 2;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;
    myServo.refresh();
  }
}
