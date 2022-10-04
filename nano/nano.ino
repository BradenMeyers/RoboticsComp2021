
//Sorter sorter(servo1, 6, 25 , A0, "sortA");         //create sorter objects
//Sorter sorter2(servo2, 5, 32, A1, "sortB");

//information for the robot control
int checkStallCount = 20;
unsigned long currentTime = 0;
unsigned long triggerTime;
unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt function LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt function RIGHT
//Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorLD = 9;
const int motorLS = 10;
const int motorRD = 12;
const int motorRS = 11;
//Adjustable speed and movement properties
int leftDutyC = 100;
int rightDutyC = 100;
int encoderCountL = 0;
int encoderCountR = 0;
int freqSpeed = 100;
unsigned long nowTime = 0;
bool setTime = false;
bool trigger = false;
bool directionState = LOW;
int speedMotor = 50;
// Setting PWM properties
//const int freq = 30000;
//const int pwmChannelL = 15;
//const int resolution = 10;
//const int pwmChannelR = 14;

void checkStall(){
    if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
        Serial.println("set time");
    }
    if ((millis() - nowTime) >= freqSpeed){
        encoderCountL = (totalLRot - pastLRot);
        encoderCountR = (totalRRot - pastRRot);
        Serial.println(encoderCountR);
        if(millis() - triggerTime >= 2000);{
          if (encoderCountL <= checkStallCount || encoderCountL <= checkStallCount){
            triggerTime = millis();
            trigger = true;
            //Serial.println("trigger on");
          }
        }
        setTime = false;
        Serial.print("trueee");
    }
}
/*void checkSpeed(){
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
         
        setTime = false;                  //Reset the whole process
    }
}
*/
void moveForward(int leftRotation, int rightRotation, int speedL, int speedR) {
    rightDutyC = speedR;
    leftDutyC = speedL;
    digitalWrite(motorLD,LOW);                  //set direction as forward
    digitalWrite(motorRD,LOW);
    Serial.print("Moving Forward");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        //Serial.println("i am here2");
        //checkSpeed();
        //sorter.sortBallSorterRight();
        //sorter2.sortBallSorterLeft();
        //Serial.println(rightDutyC);
        //Serial.print("Left Rotation Count:");
        //Serial.println(totalLRot);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            //ledcWrite(pwmChannelR, 0);
            analogWrite(motorRS,0);
            //sorter.sortBallSorterRight();
            //sorter2.sortBallSorterLeft();
            //Serial.print("Right Rotation Count:");
            //Serial.println(totalRRot);
            Serial.println("Motor Right Stopped First");
        }
    }
     analogWrite(motorLS, 0);
    //ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorRS,rightDutyC);
        //sorter.sortBallSorterRight();
        //sorter2.sortBallSorterLeft();
        //checkStall();
        if (trigger == true){
          leftCount = leftCount - leftRotation;
          rightCount = rightCount - rightRotation;
        }
        //Serial.println("Right motor continue");
    }
    //ledcWrite(pwmChannelL, 0);            //turn both motors off
    //ledcWrite(pwmChannelR, 0);
    analogWrite(motorLS, 0);
    //Serial.println("i am here3");
    analogWrite(motorRS,0);
    Serial.println("Movement Completed");
}
void moveBackward(int leftRotation, int rightRotation, int speedL, int speedR) {
    rightDutyC = speedR;
    leftDutyC = speedL;
    digitalWrite(motorLD,HIGH);                  //set direction as forward
    digitalWrite(motorRD,HIGH);
    Serial.print("Moving Forward");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        //Serial.println("i am here2");
        //checkSpeed();
        //sorter.sortBallSorterRight();
        //sorter2.sortBallSorterLeft();
        //Serial.println(rightDutyC);
        //Serial.print("Left Rotation Count:");
        //Serial.println(totalLRot);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            //ledcWrite(pwmChannelR, 0);
            analogWrite(motorRS,0);
            //sorter.sortBallSorterRight();
            //sorter2.sortBallSorterLeft();
            //Serial.print("Right Rotation Count:");
            //Serial.println(totalRRot);
            Serial.println("Motor Right Stopped First");
        }
    }
     analogWrite(motorLS, 0);
    //ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorRS,rightDutyC);
        //sorter.sortBallSorterRight();
        //sorter2.sortBallSorterLeft();
        //checkStall();
        if (trigger == true){
          leftCount = leftCount - leftRotation;
          rightCount = rightCount - rightRotation;
        }
        //Serial.println("Right motor continue");
    }
    //ledcWrite(pwmChannelL, 0);            //turn both motors off
    //ledcWrite(pwmChannelR, 0);
    analogWrite(motorLS, 0);
    //Serial.println("i am here3");
    analogWrite(motorRS,0);
    Serial.println("Movement Completed");
}
void turnLeft(int turnRotate, int speedM) {
    rightDutyC = speedM;
    leftDutyC = speedM;
    digitalWrite(motorLD, HIGH);      //set direction of the motors
    digitalWrite(motorRD, LOW);
    Serial.println("Moving LEFT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        //ledcWrite(pwmChannelL, leftDutyC);
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        //checkSpeed();
        //Serial.print("Left Rotation Count:");
        //Serial.println(leftCount);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            //ledcWrite(pwmChannelR, 0);  
            analogWrite(motorRS,0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(rightCount);
            //Serial.println("Motor Right Stopped First");
        }
    }
    analogWrite(motorLS,0);
    //ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorRS,rightDutyC);
        //Serial.println("Right motor continue");
    }
    //ledcWrite(pwmChannelL, 0);            //turn both motors off
    //ledcWrite(pwmChannelR, 0);
    analogWrite(motorLS, 0);
    analogWrite(motorRS,0);
    Serial.println("turn completed");
}
void turnRight(int turnRotate, int speedM) {
    rightDutyC = speedM;
    leftDutyC = speedM;
    digitalWrite(motorLD, LOW);      //set direction of the motors
    digitalWrite(motorRD, HIGH);
    Serial.println("Moving RIGHT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        //ledcWrite(pwmChannelL, leftDutyC);
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        //checkSpeed();
        //Serial.print("Left Rotation Count:");
        //Serial.println(leftCount);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            //ledcWrite(pwmChannelR, 0);
            analogWrite(motorRS, 0);
            //Serial.print("Right Rotation Count:");
            //Serial.println(rightCount);
            //Serial.println("Motor Right Stopped First");
        }
    }
    analogWrite(motorLS, 0);
    //ledcWrite(pwmChannelL, 0);                    //after while loop completed the left motor is turned off
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        //ledcWrite(pwmChannelR, rightDutyC);
        analogWrite(motorRS, rightDutyC);
        //Serial.println("Right motor continue");
    }
    //ledcWrite(pwmChannelL, 0);            //turn both motors off
    //ledcWrite(pwmChannelR, 0);
    analogWrite(motorLS, 0);
    analogWrite(motorRS, 0);
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

unsigned long compTime = 0;
bool firstRun = true;

void setup() {
    //ledcSetup(pwmChannelL, freq, resolution);
    //ledcSetup(pwmChannelR, freq, resolution);
    // attach the channel to the GPIO to be controlled
    //ledcAttachPin(motorLS, pwmChannelL);
    //analogReadResolution(12);
    pinMode(motorLS, OUTPUT);
    //ledcAttachPin(motorRS, pwmChannelR);
    pinMode(motorRS, OUTPUT);
    
    Serial.begin(115200);
    // set pin modes for the GPIO
    pinMode(motorLD, OUTPUT);
    pinMode(motorRD, OUTPUT);
    digitalWrite(motorLD, LOW);
    digitalWrite(motorLD, LOW);
    // configure LED PWM functionalities
    
    // pin modes and interrupts
    pinMode(interruptPinR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    pinMode(interruptPinL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    //sorter.calibrate();
    //sorter2.calibrate();
    Serial.println("Setup Complete");
    delay(5000);
    //xTaskCreatePinnedToCore(Task1code,"Task1", 10000, NULL, 5, &Task1, 0);
    //xTaskCreatePinnedToCore(Task2code,"Task2", 10000, NULL, 5, &Task2, 1);
}

void loop() {
    compTime = millis();
    Serial.println(compTime);
    if(firstRun == true){
      moveForward(1000,1000, 102, 85);
      moveBackward(1800,1800, 150, 117);
      firstRun = false;
    }
    while(millis() - compTime <= 60000){
        moveForward(1000, 1000, 103, 75);
        moveBackward(1800,1800, 142, 122);
        moveForward(1850, 1850, 103, 75);
        moveBackward(1700, 1700, 142, 122);
        moveForward(1700, 1700, 103, 75);
        moveBackward(1600, 1600, 142, 122);
        //moveForward(700, 0, 90);
        //moveBackWard(1850, 1850, 120);
    }
    while(millis() - compTime >= 60000){
      Serial.println("congrats");
    }
}
