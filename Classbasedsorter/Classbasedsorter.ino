#include <ESP32Servo.h>

//Servo servo1;
//Servo servo2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

//int pos = 90;// variable to store the servo position
//const int IN = 33; //
 //
int blackPong = 0;
int whitePong = 0;
int minUs = 500;
int maxUs = 2400;
unsigned long currentTime = 0;
const int servoDelay = 300;//determine how long our servo will delay before proceeding to next movement
const int detectionDelay = 10;
const int wantedDetect = 5;
//int servo1Pin = 26;
//int servo2Pin = 16;
int blackValue = 400;
int whiteValue = 2000;
int noBallLight = 0;
ESP32PWM pwm;
Servo servo1;
Servo servo2;

class Sorter {
    int servoPin;       //GPIO pin for the servo
    int buttonPin;
    int BallDetected;       //Variable that will count up everytime button is pressed(ball falls)
    int BallSorted;
    int AIN;            //GPIO for IR reader
    unsigned long startMTime;      //used for the delay of the servo
    int irAvg = 0;      //used in the detect value to get a sum of the 5 values collected
    int Irvalue = 0;    //averaged value from the IR reader
    char ballColor = 'N';
    char ballValue = 'N';
    unsigned long nextRead = currentTime + detectionDelay;
    int detectCount = 0;
    bool start;
    Servo servo;

public:
    Sorter(Servo myServo, int sPin,int bPin, int inputPin) {
        servo = myServo;
        servoPin = sPin;
        //buttonPin = bPin;
        servo.attach(servoPin, minUs, maxUs);
        pinMode(buttonPin, INPUT_PULLUP);
        //attachInterrupt(digitalPinToInterrupt(bPin), ballCount, FALLING);
        AIN = inputPin;
        start = true;
        //do we want these here?
    }
    void left() {
        currentTime = millis();
        if (start == true) {
            servo.write(170);
            
            startMTime = currentTime;// waits 100ms for the servo to reach the position
            start = false;
        }
        if (currentTime >= (startMTime + servoDelay)) {
            servo.write(90);
            
            if (currentTime >= (startMTime + (2*servoDelay))){
                ballValue = 'N';
                start = true;  
                
            }
          }
    }
    void right() {
        currentTime = millis();
        if (start == true) {
            servo.write(10);
            Serial.println("I'm here 1");
            startMTime = currentTime;// waits 100ms for the servo to reach the position
            start = false;
        }
        if (currentTime >= (startMTime + servoDelay)) {
            servo.write(90);
            Serial.println("I'm here 2");
            //BallSorted++;
            if (currentTime >= (startMTime + (2*servoDelay))){
            ballValue = 'N';
            start = true; 
            Serial.println("I'm here 3"); 
            }            
        }
    }
    void sortBall(){
        //ballValue = detectValue();
        //Serial.print("ball color ");
        //Serial.println(ballValue);
        if (ballValue == 'N') {
            ballValue = detectValue();
            Serial.println("I'm here 6");
        }
        if (ballValue == 'B'){
            right();
            Serial.println("I'm here 4");
        }
        if (ballValue == 'W'){
            left();
            Serial.println("I'm here 5");
        }
    }
    char detectValue(){
        int newIrValue = analogRead(AIN);
        //currentTime = millis();
        //Serial.print("ball value: ");
        //Serial.println(newIrValue);
        if (newIrValue >= whiteValue) {
                ballColor = 'W';
        }
        else if (newIrValue <= blackValue) {
            ballColor = 'B';
        }
        else {
            ballColor = 'N';
        }
            //Serial.print("Ball Color Detected");
            //Serial.println(ballColor);
            //detectCount = 0;
    
        /*
        if (currentTime > nextRead && detectCount < wantedDetect){
            irAvg += analogRead(AIN);
            detectCount++;
            nextRead = currentTime + detectionDelay; 
        }
        if (detectCount = wantedDetect){
            int newIrValue = (irAvg/wantedDetect);
            //Serial.print("Ir value");
            //Serial.println(newIrValue);
            //irAvg = 0;
            if (newIrValue >= whiteValue) {
                ballColor = 'W';
            }
            else if (newIrValue <= blackValue) {
                ballColor = 'B';
            }
            else {
                ballColor = 'N';
            }
            //Serial.print("Ball Color Detected");
            //Serial.println(ballColor);
            detectCount = 0;
        }
        else{
            ballColor = 'N';
        }
        */
        
        return ballColor;
    }
    void ballCount(){
        BallDetected ++;
    }
};
Sorter sorter(servo1, 26, 25, 33);
Sorter sorter2(servo2, 14, 27, 35);

void setup() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.begin(115200);// attaches the servo on pin 9 to the servo object
    //servo1.attach(servo1Pin, minUs, maxUs);
    //servo2.attach(servo2Pin, minUs, maxUs);
}
void loop() {

    //Irvalue = analogRead(IN);
    //buttonValue = digitalRead(button);
    currentTime = millis();
    //Serial.print("IR Value: ");
    //Serial.println(Irvalue);
    //Irvalue = detectValue();

    sorter.sortBall();
    //sorter2.sortBall();
    //white ping pong ball


}
