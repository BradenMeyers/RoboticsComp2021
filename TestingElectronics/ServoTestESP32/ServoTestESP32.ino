#include <Arduino.h>
#include <ESP32Servo.h>

int minUs = 500;
int maxUs = 2400;
ESP32PWM pwm;
Servo servo;
int servoPin = 26;

void setup() {
  ESP32PWM::allocateTimer(0);         //servo timers
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Serial.begin(115200);
  servo.attach(servoPin, minUs, maxUs);
  servo.write(90);
  servo.detach();
  // put your setup code here, to run once:
}

void loop() {
  servo.attach(servoPin, minUs, maxUs);
  servo.write(170);
  delay(1000);
  servo.write(90);
  delay(1000);
  servo.write(10);
  delay(1000);
  // put your main code here, to run repeatedly:
}
