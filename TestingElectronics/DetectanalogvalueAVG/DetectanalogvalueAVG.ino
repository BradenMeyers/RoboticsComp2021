//arduino basic analog read and print with an averaged value.
const int IN = A0; //
int Irvalue = 0; //
int blackPong = 0;
int iravg = 0; 
int currentirValue;
int whitePong = 0;
int minUs = 500;
int maxUs = 2400;
int servo1Pin = 26;
int servo2Pin = 16;
int button1Pin = 0;
int buttonValue = 0;
const int blackValue = 0;
const int whiteValue = 0;
const int noBallLight = 0;
/*ESP32PWM pwm;

class Sorter {
  Servo servo;
  
  public: 
  Sorter(){
    servo.attach(servo1Pin, minUs, maxUs);
  }
  //do we want these here?
  void left(){
    servo.write(170);
    delay(160);             // waits 100ms for the servo to reach the position
    servo.write(90);
    delay(160); 
  }
  
  void right(){
      servo.write(10);
      delay(180);             // waits 100ms for the servo to reach the position
      servo.write(90);
      delay(180);                                    
    }
    
  void sortBall(){
    if (Irvalue >= 2000 && Irvalue <= 4000) {
        right();
        delay(360);
    }
    //black ping pong ball
    if (Irvalue >= 400 && Irvalue <= 700) {
      left();
      delay(360);
    }
  }
};
*/
int detectValue(){
  for (int i = 0; i < 10; i++) { 
    iravg += analogRead(IN);
    delay(10);
  }
 int newIrValue = (iravg/10);
 iravg = 0;
 return newIrValue;
  
}

void setup() {
  Serial.begin(9600);
  }
 /* ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Serial.begin(115200);// attaches the servo on pin 9 to the servo object
  //servo1.attach(servo1Pin, minUs, maxUs);
  servo2.attach(servo2Pin, minUs, maxUs);
}

Sorter sorter;
*/
void loop() {

  //Irvalue = analogRead(IN);
  //buttonValue = digitalRead(button);
  Serial.print("IR Value: ");
  Serial.println(Irvalue);
  Irvalue = detectValue();

  //sorter.sortBall();
  //white ping pong ball
  
 
}
