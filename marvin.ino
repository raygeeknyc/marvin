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

#define PIN_LED 0
#define PIN_SERVO 1
#define PIN_PING_TRIG 3
#define PIN_PING_ECHO 2

#define SWEEPS_COUNT 3

#define POINTED_DELAY 3000

#define MAX_POS 180
#define MIN_POS 0
#define MOVEMENT_DELAY 3000
#define SWEEP_STEP 5

#define DISTANCE_CHANGE_THRESHOLD_CM 8
#define DEFAULT_DISTANCE 100
#define DEFAULT_LOCATION 90

//#define _DEBUG

int sweepPos        =   MIN_POS;
bool Dir;                         // Servo direction
int max_distance, min_distance, last_distance, prev_distance;  // min and max distance seen during a sweep() and the last reading
int max_location, min_location;  // Servo position at max and min distance readings during a sweep()

int current_sleep_distance;
int previous_sleep_distance;

boolean sweep_complete;
unsigned long pointed_at;

int sweep_swing_remaining_count;

unsigned long int led_step_at;
unsigned long int shine_end_at;

int led_level;
bool led_dir;
#define LED_MIN 0
#define LED_MAX 12
#define LED_STEP_DURATION 100
#define SHINE_DURATION 3000
#define BLINK_DURATION 100
#define LED_SHINE_BRIGHTNESS 255

void startLedPulsing() {
  led_level = LED_MAX;
  led_dir = false;
  led_step_at = 0;
  shine_end_at = 0;
  analogWrite(PIN_LED, led_level);
}

void expireLed() {
  if (isShining()) {
    return;
  }
  if (shine_end_at != 0) {
    shine_end_at = 0;
    analogWrite(PIN_LED, LED_MIN);
    return;
  }
  if (isSweeping()) {
    return;
  }  
  if (millis() <= led_step_at) {
    return;
  }
  led_step_at = millis() + LED_STEP_DURATION;

  if (led_dir) {
    if (led_level < LED_MAX) {
      led_level++;
    } 
    else { 
      led_dir = false;
    }
  } 
  else {
    if (led_level > LED_MIN) {
      led_level--;
    } 
    else { 
      led_dir = true;
    }
  }
  analogWrite(PIN_LED, led_level);
}

boolean isShining() {
  return (shine_end_at != 0) && (shine_end_at > millis());
}

void blinkLed() {
  if (! isShining()) {
    analogWrite(PIN_LED, LED_SHINE_BRIGHTNESS);
    shine_end_at = millis() + BLINK_DURATION;
  }
}

int getDistance() {
  digitalWrite(PIN_PING_TRIG, LOW); 
  delayMicroseconds(2); 

  digitalWrite(PIN_PING_TRIG, HIGH);
  delayMicroseconds(10); 

  digitalWrite(PIN_PING_TRIG, LOW);

  int duration = pulseIn(PIN_PING_ECHO, HIGH);

  if (duration <= 0) {
    return DEFAULT_DISTANCE;
  }
  //Calculate the distance (in cm) based on the speed of sound.
  float HR_dist = duration/58.2;
  //Serial.println(HR_dist);
  return int(HR_dist);
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

boolean isDistanceChanged() {
  current_sleep_distance = getDistance();

  boolean movement = abs(current_sleep_distance - previous_sleep_distance) > (DISTANCE_CHANGE_THRESHOLD_CM);
  previous_sleep_distance = current_sleep_distance;
  if (movement) {
 #ifdef _DEBUG
  Serial.println("moved"); 
#endif
  }
  return movement;
}

void point(int loc) {
  myServo.write(loc);
  delayMicroseconds(MOVEMENT_DELAY);     // waits 15ms for the servo to reach the position 
}

void startSweeping() {
#ifdef _DEBUG
  Serial.println("StartSweeping");
#endif
  min_distance = DEFAULT_DISTANCE;
  max_distance = DEFAULT_DISTANCE;
  min_location = DEFAULT_LOCATION;
  max_location = DEFAULT_LOCATION;
  sweep_swing_remaining_count = SWEEPS_COUNT;
  pointed_at = 0L;
  stopLedPulsing();
#ifdef _DEBUG
  Serial.print("Sweeping from ");
  Serial.println(sweepPos);
#endif

}

void stopLedPulsing() {
  led_level = LED_MIN;
  analogWrite(PIN_LED, led_level);
}

boolean isSweeping() {
  return sweep_swing_remaining_count > 0;
}

boolean isResting() {
  return (!isSweeping() && ((pointed_at + POINTED_DELAY) < millis()));
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
#ifdef _DEBUG
  Serial.begin(9600);
  Serial.println("setup");
#endif
  pinMode(PIN_PING_TRIG, OUTPUT);
  pinMode(PIN_PING_ECHO, INPUT);
  /** software servo **/
  OCR0A = 0xAF;            // any number is OK
  TIMSK |= _BV(OCIE0A);    // Turn on the compare interrupt (below!)
  /** /software servo **/

  myServo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);
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
