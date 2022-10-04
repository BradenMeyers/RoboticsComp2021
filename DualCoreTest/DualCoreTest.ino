#include <ESP32Servo.h>
#include <Preferences.h>
Preferences preferences;
//Information from Sorter
int minUs = 500;
int maxUs = 2400;
unsigned long currentTime = 0;
const int servoDelay = 500;     //determine how long our servo will delay before proceeding to next movement
ESP32PWM pwm;
Servo servo1;
Servo servo2;

class Sorter {
    int AIN;            //GPIO for IR reader
    int servoPin;       //GPIO pin for the servo
    int buttonPin;
    unsigned long startMTime;      //used for the delay of the servo
    char ballColor = 'N';
    char ballValue = 'N';
    bool start;
    bool switchChanged;
    int blackValue = 400;           //change this depending on the ir value of black ball
    int whiteValue = 2000;          //change this depending on the ir value of white ball
    Servo servo;

public:
    Sorter(Servo myServo, int sPin, int bPin, int inputPin, const char* nSpace) {
        servo = myServo;
        servoPin = sPin;
        buttonPin = bPin;
        AIN = inputPin;
        pinMode(buttonPin, INPUT_PULLUP);
        start = true;
        //Move servo into starting position
        servo.attach(servoPin, minUs, maxUs);
        servo.write(90);
        servo.detach();
        /*
        //preferences values
        preferences.begin(nSpace, false);
        //Menu startup
        Serial.print("Enter 1 to set values for");
        Serial.println(nSpace);
        delay(3000);
        if(Serial.available()){
            int cereal = Serial.parseInt();
            while(cereal == 1){
                Serial.println("Enter 1 (Black) or 2 (White)");
                while(!Serial.available()){}
                int color = Serial.parseInt();
                if (color == 1){
                    int storedBlack = preferences.getInt("black", 400);
                    Serial.print("Stored black value is: ");
                    Serial.println(storedBlack);
                    int currentBlack = analogRead(inputPin);
                    Serial.print("Current black value is: ");
                    Serial.println(currentBlack);
                    Serial.println("Enter 1 to change value or anything else to continue");
                    while(!Serial.available()){}
                    int change = Serial.parseInt();
                    if (change == 1){
                        preferences.putInt("black", currentBlack);
                    }
                }
                if (color == 2){
                    int storedWhite = preferences.getInt("white", 400);
                    Serial.print("Stored white value is: ");
                    Serial.println(storedWhite);
                    int currentWhite = analogRead(AIN);
                    Serial.print("Current white value is: ");
                    Serial.println(currentWhite);
                    Serial.println("Enter 1 to change value or anything else to continue");
                    while(!Serial.available()){}
                    int change = Serial.parseInt();
                    if (change == 1){
                        preferences.putInt("white", currentWhite);
                    }
                }
            }
        }
        blackValue = preferences.getInt("black", 400);
        whiteValue = preferences.getInt("white", 2000);
        preferences.end();
         */
    }
    void left() {
        currentTime = millis();
        if (start == true) {
            servo.attach(servoPin, minUs, maxUs);
            servo.write(170);
            startMTime = currentTime;// waits 100ms for the servo to reach the position
            start = false;
        }
        if (currentTime >= (startMTime + servoDelay)) {
            servo.write(90);
            switchChanged = false;
            Serial.println("Switch changed back");
            if (currentTime >= (startMTime + (2 * servoDelay))) {
                ballValue = 'N';
                start = true;
                servo.detach();
            }
        }
    }
    void right() {
        currentTime = millis();     //update the time
        if (start == true) {        //will only run once on start of the function
            servo.attach(servoPin, minUs, maxUs);       //attach the servo to pwm pin
            servo.write(10);                            //tip servo right
            startMTime = currentTime;       //Set the start time of servo movement
            Serial.println("Im here 8");
            start = false;                  //set value false so that this function is not run until movement restarts
        }
        vTaskDelay(1);
        Serial.println("Im here 7");
        if (currentTime >= (startMTime + servoDelay)) {         //once time for the movement has completed
            servo.write(90);                                    //move servo back to position
            Serial.println("Im here 9");
            switchChanged = false;                              //allow switch to be pressed again
            if (currentTime >= (startMTime + (2 * servoDelay))){//true when double the delay time has passed since start
                ballValue = 'N';        //allow detection to begin again
                start = true;           //reset movement
                Serial.println("Im here 10");
                servo.detach();
            }
        }
    }
    void sortBallSorterRight() {
        if (ballValue == 'N') {
            Serial.println(switchChanged);
            if (digitalRead(buttonPin) == 1) {
                switchChanged = true;
            }
            if (switchChanged) {
                ballValue = detectValue();
            }
        }
        if (ballValue == 'W') {
            right();
        }
        if (ballValue == 'B') {
            left();
        }
    }
    void sortBallSorterLeft() {     //This function is for the left sorter that tips right
        if (ballValue == 'N') {     //This will be default ball color value and can only read color when in this part
            if (digitalRead(buttonPin) == 1) {      //Waits for ping pong ball to change switch
                Serial.println("Im here 3");
                switchChanged = true;               //allows detection to begin
            }
            if (switchChanged) {
                ballValue = detectValue();
                Serial.println("Im here 4");
            }
        }
        if (ballValue == 'B') {         //Black ball
            right();
            Serial.println("Im here 5");
        }
        if (ballValue == 'W') {         //White ball
            left();
            Serial.println("Im here 6");
        }
    }
    char detectValue() {
        int newIrValue = analogRead(AIN);       //read analog pin
        //Serial.println(newIrValue);
        if (newIrValue >= whiteValue) {         //compare value to global ball value
            ballColor = 'W';
        } else if (newIrValue <= blackValue) {
            ballColor = 'B';
        } else {
            ballColor = 'N';
        }
        return ballColor;
    }
};
Sorter sorter(servo1, 26, 25 , 34, "sortA");         //create sorter objects
Sorter sorter2(servo2, 33, 32, 35, "sortB");

//information for the robot control
unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt function LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt function RIGHT
//Pin Assignments
const int interruptPinR = 19;
const int interruptPinL = 18;
const int motorLS = 4;
const int motorLD = 2;
const int motorRS = 17;
const int motorRD = 16;
//Adjustable speed and movement properties
int leftDutyC = 300;
int rightDutyC = 300;
int encoderCountL = 0;
int encoderCountR = 0;
int freqSpeed = 100;
unsigned long nowTime = 0;
bool setTime = false;
// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int resolution = 10;
const int pwmChannelR = 1;

TaskHandle_t Task1;                     //create tasks to run on cores
TaskHandle_t Task2;

void Task1code( void * pvParameters ){          //define task1 function
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
      sorter.sortBallSorterLeft();
      vTaskDelay(1);
      //Serial.println("Im here 1");
      //sorter2.sortBallSorterRight();
  }
    vTaskDelete( NULL );
}

void Task2code( void * pvParameters ){          //define task2 function
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    moveForward(1000,1000,400);
    delay(100);
    moveBackward(1000,1000,400);
  }
    vTaskDelete( NULL );
}

void setup() {
    ESP32PWM::allocateTimer(0);         //servo timers
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.begin(115200);
    // set pin modes for the GPIO

    pinMode(motorLD, OUTPUT);
    pinMode(motorRD, OUTPUT);
    digitalWrite(motorLD, LOW);
    digitalWrite(motorLD, LOW);
    // configure LED PWM functionalities
    ledcSetup(pwmChannelL, freq, resolution);
    ledcSetup(pwmChannelR, freq, resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(motorLS, pwmChannelL);
    ledcAttachPin(motorRS, pwmChannelR);
    // pin modes and interrupts
    pinMode(interruptPinR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    pinMode(interruptPinL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    Serial.println("Setup Complete");


    xTaskCreatePinnedToCore(                        //pin tasks to the 2 cores and run them.
            Task1code, /* Task function. */
            "Task1",   /* name of task. */
            10000,     /* Stack size of task */
            NULL,      /* parameter of the task */
            5,         /* priority of the task */
            &Task1,    /* Task handle to keep track of created task */
            1);        /* pin task to core 0 */
    xTaskCreatePinnedToCore(
             Task2code,
             "Task2",
             10000,
             NULL,
             6,
             &Task2,
             0);

}
void checkSpeed(){
    if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
    }
    if ((millis() - nowTime) >= freqSpeed){
        encoderCountL = (totalLRot - pastLRot);
        encoderCountR = (totalRRot - pastRRot);
        if ((encoderCountL - encoderCountR) >= 2){
            rightDutyC++;
            //Serial.println("left moves faster");
        }
        if ((encoderCountR - encoderCountL) >= 2){
            rightDutyC--;
            //Serial.println("right moves faster");
        }
        /*Serial.print("Right Speed Set");
        Serial.println(rightDutyC);
        Serial.print("left rotations:");
        Serial.println(encoderCountL);
        Serial.print("Right rotations:");
        Serial.println(encoderCountR);
         */
        setTime = false;                  //Reset the whole process
    }
}
void moveBackward(int leftRotation, int rightRotation, int speed) {
    rightDutyC = speed;
    leftDutyC = speed;
    digitalWrite(motorLD,HIGH);                  //set direction as Backward
    digitalWrite(motorRD,HIGH);
    Serial.println("MOVING Backward");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        ledcWrite(pwmChannelL, leftDutyC);
        ledcWrite(pwmChannelR, rightDutyC);
        checkSpeed();
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            ledcWrite(pwmChannelR, 0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(rightCount);
            //Serial.println("Motor Right Stopped First");
        }
    }
    ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        ledcWrite(pwmChannelR, rightDutyC);
        //Serial.println("Right motor continue");
    }
    ledcWrite(pwmChannelL, 0);            //turn both motors off
    ledcWrite(pwmChannelR, 0);
    //Serial.print("Left Rotation Count:");
    //Serial.println(leftCount);
    //Serial.print("Right Rotation Count:");
    //Serial.println(rightCount);
    //Serial.println("Movement Completed");
}
void moveForward(int leftRotation, int rightRotation, int speed) {
    rightDutyC = speed;
    leftDutyC = speed;
    digitalWrite(motorLD,LOW);                  //set direction as forward
    digitalWrite(motorRD,LOW);
    Serial.println("MOVING FORWARD");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        ledcWrite(pwmChannelL, leftDutyC);
        ledcWrite(pwmChannelR, rightDutyC);
        checkSpeed();
        //Serial.println(rightDutyC);
        //Serial.print("Left Rotation Count:");
        //Serial.println(totalLRot);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            ledcWrite(pwmChannelR, 0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(totalRRot);
            //Serial.println("Motor Right Stopped First");
        }
    }
    ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        ledcWrite(pwmChannelR, rightDutyC);
        //Serial.println("Right motor continue");
    }
    ledcWrite(pwmChannelL, 0);            //turn both motors off
    ledcWrite(pwmChannelR, 0);
    Serial.println("Movement Completed");
}
void turnLeft(int turnRotate, int speed) {
    rightDutyC = speed;
    leftDutyC = speed;
    digitalWrite(motorLD, HIGH);      //set direction of the motors
    digitalWrite(motorRD, LOW);
    Serial.println("Moving LEFT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        ledcWrite(pwmChannelL, leftDutyC);
        ledcWrite(pwmChannelR, rightDutyC);
        checkSpeed();
        //Serial.print("Left Rotation Count:");
        //Serial.println(leftCount);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            ledcWrite(pwmChannelR, 0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(rightCount);
            //Serial.println("Motor Right Stopped First");
        }
    }
    ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        ledcWrite(pwmChannelR, rightDutyC);
        //Serial.println("Right motor continue");
    }
    ledcWrite(pwmChannelL, 0);            //turn both motors off
    ledcWrite(pwmChannelR, 0);
    Serial.println("turn completed");
}
void turnRight(int turnRotate, int speed) {
    rightDutyC = speed;
    leftDutyC = speed;
    digitalWrite(motorLD, LOW);      //set direction of the motors
    digitalWrite(motorRD, HIGH);
    Serial.println("Moving RIGHT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        ledcWrite(pwmChannelL, leftDutyC);
        ledcWrite(pwmChannelR, rightDutyC);
        checkSpeed();
        //Serial.print("Left Rotation Count:");
        //Serial.println(leftCount);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            ledcWrite(pwmChannelR, 0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(rightCount);
            //Serial.println("Motor Right Stopped First");
        }
    }
    ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        ledcWrite(pwmChannelR, rightDutyC);
        //Serial.println("Right motor continue");
    }
    ledcWrite(pwmChannelL, 0);            //turn both motors off
    ledcWrite(pwmChannelR, 0);
    //leftCount = 0;                        //Reset the rotation count values
    //Serial.print("Left Rotation Count:");
    //Serial.println(leftCount);
    //rightCount = 0;
    //Serial.print("Right Rotation Count:");
    //Serial.println(rightCount);
    Serial.println("turn completed");
}
void addRotR(){
    totalRRot += 1;
}
void addRotL(){
    totalLRot += 1;
}
void loop() {
    vTaskDelete(NULL);
}
