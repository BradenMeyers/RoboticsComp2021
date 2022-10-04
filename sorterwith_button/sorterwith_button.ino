#include <ESP32Servo.h>
int newIrValue = 0;
int blackPong = 0;
int whitePong = 0;
int minUs = 500;
int maxUs = 2400;
unsigned long currentTime;
const int servoDelay = 500;//determine how long our servo will delay before proceeding to next movement
const int detectionDelay = 10;
const int wantedDetect = 5;
int blackValue = 700;
int whiteValue = 2000;
int noBallLight = 0;
bool started = false;
ESP32PWM pwm; 
    bool BallDetected = false;       //Variable that will count up everytime button is pressed(ball falls)
    //int BallSorted;
    //int AIN;            //GPIO for IR reader
    unsigned long startMTime;      //used for the delay of the servo
    int irAvg = 0;      //used in the detect value to get a sum of the 5 values collected
    int Irvalue = 0;    //averaged value from the IR reader
    char ballColor = 'N';
    char ballValue = 'N';
    unsigned long nextRead = millis() + detectionDelay;
    int detectCount = 0;
 Servo servo;
        int servoPin = 26;
        
        int buttonPin = 25;
        
        int AIN = 33;
    //do we want these here?
void left(){
        servo.attach(servoPin, minUs, maxUs);
        servo.write(170);
        if (started == false){
          startMTime = millis();// waits 100ms for the servo to reach the position
          started = true;
        }
        if(currentTime >= (startMTime + servoDelay)){
            servo.write(90);
            //servo.detach();
            ballValue = 'N';
            BallDetected = false;
            //BallSorted++;
            Serial.println("IM HERE 2");
            started = false;
        }
    }
void right(){
        servo.attach(servoPin, minUs, maxUs);
        servo.write(15);
         if (started == false){
          startMTime = millis();// waits 100ms for the servo to reach the position
          started = true;
        }
        if(currentTime >= (startMTime + servoDelay)){
            servo.write(90);
            //servo.detach();
            //BallSorted++;
            BallDetected = false;
            ballValue = 'N'; 
            Serial.println("IM HERE 1");
            started = false;           
        }
    }
    void sortBall(){
        if(BallDetected == true) {
            ballValue = detectValue();
            Serial.println("IM HERE 3");
            if (ballValue == 'N') {
                ballValue = detectValue();
                Serial.println("IM HERE 4");
            }
            if (ballValue == 'B'){
                right();
            }
            if (ballValue == 'W'){
                left();
            }
        }
        else {
          //Serial.println("IM HERE 5");
        }
    }
    char detectValue(){
        //currentTime = millis();
        if (currentTime > nextRead && detectCount < wantedDetect){
            irAvg += analogRead(AIN);
            detectCount++;
            nextRead = millis() + detectionDelay;
            Serial.println("IM HERE 6");
            Serial.print("counted: ");
            Serial.println(detectCount);
        }
        if (detectCount == wantedDetect){
            newIrValue = (irAvg/wantedDetect);
            irAvg = 0;
            if (newIrValue >= whiteValue) {
                ballColor = 'W';
            }
            else if (newIrValue <= blackValue) {
                ballColor = 'B';
            }
            else {
                ballColor = 'N';
            }
            Serial.print("Ball Color Detected: ");
            Serial.println(ballColor);
            detectCount = 0;
        }
        else{
            ballColor = 'N';
        }
        return ballColor;
    }
    void ballCount(){
        BallDetected = true;
        Serial.println(BallDetected);
    }

void setup() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.begin(115200);// attaches the servo on pin 9 to the servo object
    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin), ballCount, RISING);
    BallDetected = false;
    servo.write(90);
    //servo1.attach(servo1Pin, minUs, maxUs);
    //servo2.attach(servo2Pin, minUs, maxUs);
}

void loop(){
  currentTime = millis();
  sortBall();
  //Serial.println(newIrValue);
  //int buttonvalue = digitalRead(buttonPin);
  //Serial.println(buttonvalue);
  //delay(100);
}
