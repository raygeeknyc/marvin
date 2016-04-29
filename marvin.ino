/*
 * @author("Raymond Blum" <raymond@insanegiantrobots.com>)
 * targeted at an Adafruit Metro Mini should work on any Arduino 
 * compatible controller
 *
 * Copyright (c) 2015 by Raymond Blum
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 **/

// Build configuration flags
// #define _DEBUG
#define _USE_NEWPING_LIBRARY

#ifdef _USE_NEWPING_LIBRARY
#include <NewPing.h>
#endif

#include <TimerFreeTone.h>
#include <Servo.h> 

#define PIN_LED 5
#define PIN_SERVO 11
#define PIN_PING_TRIG 7
#define PIN_PING_ECHO 8
#define PIN_SPEAKER 2

#define PING_SAMPLES 3
#define PING_MIN_INTERVAL_MS 20

#define DISTANCE_CHANGE_THRESHOLD_CM 5
#define DEFAULT_LOCATION 90
#define MAX_DISTANCE 200
#define MIN_DISTANCE 0

#define SWEEPS_COUNT 3

#define BLEEP_FREQUENCY 450
#define BLEEP_DURATION_MS 500

#define POINTED_DELAY 3000

#define MAX_POS 180
#define MIN_POS 0
#define MOVEMENT_DELAY 15
#define SWEEP_STEP 5

int current_distance, last_ping_duration;

Servo myservo;  // create servo object to control a servo 

#ifdef _USE_NEWPING_LIBRARY
NewPing sonar(PIN_PING_TRIG, PIN_PING_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
#endif

int sweepPos        =   MIN_POS;
bool Dir;                         // Servo direction
int max_distance, min_distance, last_distance, prev_distance;  // min and max distance seen during a sweep() and the last reading
int max_location, min_location;  // Servo position at max and min distance readings during a sweep()

int current_sleep_distance;
int previous_sleep_distance;

boolean sweep_complete;
unsigned long pointed_at;
unsigned long next_ping_at;

int sweep_swing_remaining_count;

unsigned long int led_step_at;
unsigned long int shine_end_at;

int getDistance() {
  return getPing();
}

int led_level;
bool led_dir;
#define LED_OFF 0
#define LED_MIN 3
#define LED_MAX 12
#define LED_STEP_DURATION 200    
#define SHINE_DURATION 3000
#define BLINK_DURATION 100
#define LED_SHINE_BRIGHTNESS 255

void startLedPulsing() {
  led_level = LED_MAX;
  led_dir = false;
  led_step_at = 0L;
  shine_end_at = 0L;
  analogWrite(PIN_LED, led_level);
}

void expireLed() {
  if (isShining()) {
    return;
  }
  if (shine_end_at != 0L) {
    shine_end_at = 0L;
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

int getPing() {
 if (next_ping_at > millis()) {
   #ifdef _DEBUG
   Serial.print("Reusing old distance: ");
   Serial.println(current_distance);
   #endif
   return current_distance;
 }
  current_distance = getPingSensorReading();
  return current_distance;
}

int getPingSensorReading() {
#ifdef _USE_NEWPING_LIBRARY
 int cm = 0;
 while (cm==0) {
  int echoTime = sonar.ping_median(PING_SAMPLES);
  cm = sonar.convert_cm(echoTime);
 }
#ifdef _DEBUG
  Serial.print("Distance ");
  Serial.println(cm);
#endif 
 return cm;
#else
  int sum = 0, min_sample = 9999, max_sample = -1;
  for (int i = 0; i < PING_SAMPLES; i++) {

    digitalWrite(PIN_PING_TRIG, LOW); 
    delayMicroseconds(2); 

    digitalWrite(PIN_PING_TRIG, HIGH);
    delayMicroseconds(10); 

    digitalWrite(PIN_PING_TRIG, LOW);

    int duration = pulseIn(PIN_PING_ECHO, HIGH);

    if (duration < 0) {
      Serial.println("Impossible error reading distance!");
      duration = last_ping_duration;
    }
    last_ping_duration = duration;
    if (duration < min_sample) min_sample = duration;
    if (duration > max_sample) max_sample = duration;
    sum += duration;
  }
  sum -= (min_sample + max_sample);
  current_distance = sum / (PING_SAMPLES - 2);

  //Calculate the distance (in cm) based on the speed of sound.
  float HR_dist = current_distance/58.2;
  next_ping_at = millis() + PING_MIN_INTERVAL_MS;
  current_distance = int(HR_dist);
  #ifdef _DEBUG
  Serial.print("Distance: ");
  Serial.println(HR_dist);
  #endif
  return int(HR_dist);
#endif
}

void pointAt(int loc) {
#ifdef _DEBUG
  Serial.print("pointAt ");
  Serial.print(loc);
#endif
  if (sweepPos > loc) {
#ifdef _DEBUG
    Serial.print(" down from ");
    Serial.println(sweepPos);
#endif
    for (int pos=sweepPos; pos>loc; pos--) {
      point(pos);
    }
  } else if (sweepPos < loc) {
#ifdef _DEBUG
    Serial.print(" up from ");
    Serial.println(sweepPos);
#endif
    for (int pos=sweepPos; pos<loc; pos++) {
      point(pos);
    }
  }
  pointed_at = millis();
}

boolean isDistanceChanged() {
  #ifdef _DEBUG
  Serial.println("isDistanceChanged()");
  #endif
  current_sleep_distance = getDistance();
  #ifdef _DEBUG
  Serial.print("current: ");
  Serial.print(current_sleep_distance);
  Serial.print(" prev: ");
  Serial.println(previous_sleep_distance);
  #endif

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
  #ifdef _DEBUG
  Serial.print("Servo ");
  Serial.println(loc);
  #endif
  myservo.write(loc);
  delay(MOVEMENT_DELAY);     // waits 15ms for the servo to reach the position 
}

void startSweeping() {
#ifdef _DEBUG
  Serial.println("StartSweeping");
#endif
  min_distance = MAX_DISTANCE;
  max_distance = MIN_DISTANCE;
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
  led_level = LED_OFF;
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

  if (Dir) {
    if (sweepPos < MAX_POS) {
      #ifdef _DEBUG
      Serial.println("+");
      #endif
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
      #ifdef _DEBUG
      Serial.println("-");
      #endif
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
  next_ping_at = millis();
  last_ping_duration = 0;
  #ifdef _DEBUG
  Serial.begin(115200);
  Serial.println("setup");
  #endif
  pinMode(PIN_PING_TRIG,OUTPUT);
  pinMode(PIN_PING_ECHO,INPUT);
  
  myservo.attach(PIN_SERVO);
  //myservo.setMaximumPulse(2200);
  pinMode(PIN_LED, OUTPUT);
  // blink confirmation sequence
  for (int i=0; i<4; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  analogWrite(PIN_LED, LED_MIN);
  Dir = true;
  sweep_swing_remaining_count = 0;
  pointed_at = 0L;
  shine_end_at = 0;
  startLedPulsing();
  getPing();
  prev_distance = getDistance();
  isDistanceChanged();
  #ifdef _DEBUG
  Serial.println("/setup");
  #endif
}

void bleep() {
  TimerFreeTone(PIN_SPEAKER, BLEEP_FREQUENCY, BLEEP_DURATION_MS);
}

void loop() {  
  expireLed();
  sweep();

  if (!isSweeping()) {
    #ifdef _DEBUG
    Serial.println("not sweeping");
    #endif
    if (!isPointed()) {
      #ifdef _DEBUG
      Serial.println("Stopped sweeping");
      #endif
      bleep();
      pointAt(min_location);
      delay(100);
      previous_sleep_distance = current_sleep_distance = current_distance = getDistance();
      #ifdef _DEBUG
      Serial.print("previous sleep: ");
      Serial.print(previous_sleep_distance);
      Serial.print(" current sleep: ");
      Serial.print(current_sleep_distance);
      Serial.print(" current: ");
      Serial.println(current_distance);
      #endif
      startLedPulsing();
    }
  }

  if (isResting()) {
    #ifdef _DEBUG
    Serial.println("*");
    #endif
   if (isDistanceChanged()) {
     startSweeping();
    }
  }
}
