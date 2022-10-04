#include <Preferences.h>
#include <ESP32Servo.h>
Preferences preferences;

//Information from Sorter
int minUs = 500;
int maxUs = 2400;
const int servoDelay = 275;     //determine how long our servo will delay before proceeding to next movement
ESP32PWM pwm;
Servo servo1;
Servo servo2;

class Sorter {
    int AIN;            //GPIO for IR reader
    int servoPin;       //GPIO pin for the servo
    int buttonPin;
    const char* nSpace;
    unsigned long startMTime;      //used for the delay of the servo
    char ballColor = 'N';
    char ballValue = 'N';
    bool start;
    bool stuck;
    bool startStuck;
    unsigned long stuckTime = 0;
    unsigned long currentTime = 0;
    int blackValue = 500;           //change this depending on the ir value of black ball
    int whiteValue = 3000;          //change this depending on the ir value of white ball
    int blackMargin = 800;
    int whiteMargin = 600;
    Servo servo;

public:
    Sorter(Servo myServo, int sPin, int bPin, int inputPin, const char* nameSpace) {
        servo = myServo;
        servoPin = sPin;
        buttonPin = bPin;
        AIN = inputPin;
        pinMode(buttonPin, INPUT_PULLUP);
        start = true;
        nSpace = nameSpace;
    }
    void calibrate(){
      //Move servo into starting position
      servo.attach(servoPin);
      servo.write(90);
      preferences.begin(nSpace, false);
      //Menu startup
      Serial.print("Enter 1 to set values for");
      Serial.println(nSpace);
      delay(3000);
      if(Serial.available()){
          int cereal = Serial.parseInt();
          while(cereal == 1){
              Serial.println("Enter 1 (Black) or 2 (White) or 3 (black margin)");
              while(!Serial.available()){}
              int color = Serial.parseInt();
              if (color == 1){
                  int storedBlack = preferences.getInt("black", 400);
                  Serial.print("Stored black value is: ");
                  Serial.println(storedBlack);
                  int currentBlack = analogRead(AIN);
                  Serial.print("Current black value is: ");
                  Serial.println(currentBlack);
                  Serial.println("Enter 1 to change value or anything else to continue");
                  while(!Serial.available()){}
                  int change = Serial.parseInt();
                  if (change == 1){
                      preferences.putInt("black", currentBlack);
                      Serial.println("Value Stored");
                      Serial.println("Enter 1 to end or anything to continue");
                      while(!Serial.available()){}
                    int clear = Serial.parseInt();
                    if(clear != 1){
                      cereal = 1;
                      color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
                    }
                  }
                  if(change != 1){
                    cereal = 0;
                    Serial.println("Value Not Stored");
                    Serial.println("Enter 1 to end or anything to continue");
                    while(!Serial.available()){}
                    int clear = Serial.parseInt();
                    if(clear != 1){
                      cereal = 1;
                      color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
                    }
                  }
                }
                if (color == 3){
                  int storedBlackM = preferences.getInt("blackM", 700);
                  Serial.print("Stored black margin value is: ");
                  Serial.println(storedBlackM);
                  int currentBlackM = analogRead(AIN);
                  Serial.print("Current empty value is: ");
                  Serial.println(currentBlackM);
                  Serial.println("Enter Black Margin");
                  while(!Serial.available()){
                    delay(5000);
                    }
                  int change = Serial.parseInt();
                  preferences.putInt("blackM", change);
                  Serial.println("Value Stored: ");
                  Serial.println(change);
                  Serial.println("Enter 1 to end or anything to continue");
                  while(!Serial.available()){}
                  int clear = Serial.parseInt();
                  if(clear != 1){
                      cereal = 1;
                      color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
                    }
                }
              if (color == 4){
                  int storedwhiteM = preferences.getInt("whiteM", 600);
                  Serial.print("Stored white margin value is: ");
                  Serial.println(storedwhiteM);
                  int currentwhiteM = analogRead(AIN);
                  Serial.print("Current empty value is: ");
                  Serial.println(currentwhiteM);
                  Serial.println("Enter White Margin");
                  while(!Serial.available()){
                    delay(5000);
                    }
                  int change = Serial.parseInt();
                  preferences.putInt("whiteM", change);
                  Serial.println("Value Stored: ");
                  Serial.println(change);
                  Serial.println("Enter 1 to end or anything to continue");
                  while(!Serial.available()){}
                  int clear = Serial.parseInt();
                  if(clear != 1){
                      cereal = 1;
                      color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
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
                      Serial.println("Value Stored");
                      Serial.println("Enter 1 to end or anything to continue");
                      while(!Serial.available()){}
                      int clear = Serial.parseInt();
                      if(clear != 1){
                      cereal = 1;
                      color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
                    }
                  }
                  if(change != 1){
                    cereal = 0;
                    Serial.println("Value Not Stored");
                    Serial.println("Enter 1 to end or anything to continue");
                    while(!Serial.available()){}
                    int clear = Serial.parseInt();
                    if(clear != 1){
                    cereal = 1;
                    color = 0;
                    }
                    if(clear == 1){
                      cereal = 0;
                    }
                  }
              }
          }
      }
      blackValue = preferences.getInt("black", 400);
      whiteValue = preferences.getInt("white", 2000);
      blackMargin = preferences.getInt("blackM", 700);
      whiteMargin = preferences.getInt("whiteM", 600);
      preferences.end();
    }
    void left() {
        currentTime = millis();
        if (start == true) {
            //servo.attach(servoPin, minUs, maxUs);
            servo.write(170);
            startMTime = currentTime;// waits 100ms for the servo to reach the position
            start = false;
        }
        if (currentTime >= (startMTime + servoDelay)) {
            servo.write(90);
            //switchChanged = false;
            //Serial.println("Switch changed back");
            if (currentTime >= (startMTime + (2 * servoDelay))) {
                ballValue = 'N';
                start = true;
                startStuck = false;
                //servo.detach();
            }
        }
    }
    void right() {
        currentTime = millis();     //update the time
        if (start == true) {        //will only run once on start of the function
            //servo.attach(servoPin, minUs, maxUs);       //attach the servo to pwm pin
            servo.write(10);                            //tip servo right
            startMTime = currentTime;       //Set the start time of servo movement
            //Serial.println("Im here 8");
            start = false;                  //set value false so that this function is not run until movement restarts
        }
        //Serial.println("Im here 7");
        if (currentTime >= (startMTime + servoDelay)) {         //once time for the movement has completed
            servo.write(90);                                    //move servo back to position
            //Serial.println("Im here 9");
            if (currentTime >= (startMTime + (2 * servoDelay))){//true when double the delay time has passed since start
                ballValue = 'N';        //allow detection to begin again
                startStuck = false;
                start = true;           //reset movement
                //Serial.println("Im here 10");
                //servo.detach();
            }
        }
    }
    void sortBallSorterRight() {
        if (ballValue == 'N') {
            ballValue = detectValue();
            if(startStuck == false){
              stuckTime = (millis() + 400);
              startStuck = true;
            }
            currentTime = millis();
            if (currentTime >= stuckTime){
              ballValue = 'W';
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
            ballValue = detectValue();
            if(startStuck == false){
              stuckTime = (millis() + 400);
              startStuck = true;
            }
            currentTime = millis();
            if (currentTime >= stuckTime){
              ballValue = 'W';
            }
        }
        if (ballValue == 'B') {         //Black ball
            right();
            //Serial.println("Im here 5");
        }
        if (ballValue == 'W') {         //White ball
            left();
            //Serial.println("Im here 6");
        }
    }
    char detectValue() {
        int newIrValue = analogRead(AIN);       //read analog pin
        Serial.println(newIrValue);
        //Serial.println(newIrValue);
        if (newIrValue >= (whiteValue-whiteMargin)) {         //compare value to global ball value
            ballColor = 'W';
            //Serial.println("White");
        } 
        else if (newIrValue <= (blackValue+blackMargin)) {
            ballColor = 'B';
            //Serial.println("Black");
        } else {
            ballColor = 'N';
        }
        return ballColor;
    }
};
Sorter sorter(servo1, 26, 25 , 34, "sortA");         //create sorter objects
Sorter sorter2(servo2, 33, 32, 35, "sortB");
unsigned long compTime = 0;

void setup() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    //ledcSetup(pwmChannelL, freq, resolution);
    //ledcSetup(pwmChannelR, freq, resolution);
    // attach the channel to the GPIO to be controlled
    //ledcAttachPin(motorLS, pwmChannelL);
    //analogReadResolution(12);
    //pinMode(motorLS, OUTPUT);
    //ledcAttachPin(motorRS, pwmChannelR);
    //pinMode(motorRS, OUTPUT);
    
    Serial.begin(115200);
    // set pin modes for the GPIO
    //pinMode(motorLD, OUTPUT);
    //pinMode(motorRD, OUTPUT);
    //digitalWrite(motorLD, LOW);
    //digitalWrite(motorLD, LOW);
    // configure LED PWM functionalities
    
    // pin modes and interrupts
    //pinMode(interruptPinR, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    //pinMode(interruptPinL, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    sorter.calibrate();
    sorter2.calibrate();
    Serial.println("Setup Complete");
    //delay(1000);
    //xTaskCreatePinnedToCore(Task1code,"Task1", 10000, NULL, 5, &Task1, 0);
    //xTaskCreatePinnedToCore(Task2code,"Task2", 10000, NULL, 5, &Task2, 1);
}

void loop() {
    compTime = millis();
    Serial.println(compTime);
    while(millis() - compTime <= 62000){
    sorter.sortBallSorterLeft();
    sorter2.sortBallSorterRight();  
    }
    while(millis() - compTime >= 62000){
      Serial.println("congrats");
    }
  }
