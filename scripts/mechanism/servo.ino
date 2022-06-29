#include <Servo.h>

#define ServoPinM 10 //Tengah
#define ServoPinL 11 //CW
#define ServoPinR 12 //CCW

Servo servoM,servoL,servoR;

//LED
unsigned long previousMillis = 0;
const long interval = 500; // 500 milisecond
int ledState = LOW;
String string_serial = "";
bool isParseStart = false;
bool isParseComplete = false;
String command_string="";
int command_int;

void setup (){

    Serial.begin(9600);
    servoM.attach(ServoPinM);
    servoL.attach(ServoPinL);
    servoR.attach(ServoPinR);
    delay(100);

    closeServos();

    delay(100);

    setupLED();

    Serial.println("[Arduino Ready]");

}

void loop(){

  updateLED();
  if (Serial.available()>0){
      char inData = Serial.read();
      if(inData=='<'){
        isParseStart = true;
      }
      if(isParseStart){
        string_serial+=inData;
        
        if(inData=='>'){
          isParseComplete=true;
        }
      }
    }
  if(isParseComplete){
    for(int i=1; i<string_serial.length()-1; i++){
      command_string += string_serial[i];
    }
    command_int = command_string.toInt();
    moveServo(command_int);
    resetParam();
  }
}

void resetParam(){
  string_serial = "";
  isParseStart = false;
  isParseComplete = false;
  command_string = "";
  command_int = 0;
}

void moveServo(int cmd_int){
  switch(cmd_int){

    case 1:
      moveServo1();
      break;
    case 2:
      moveServo2();
      break;
    case 3:
      moveServo3();
      break;
    case 4:
      openServos();
      break;
    case 5:
      closeServos();
      break;
  }

}


void moveServo1(){
    Serial.println("Servo1 180 degree");
    servoR.write(180);
    delay(1000);
    servoR.write(0);
    delay(300);
}

void moveServo2(){
    Serial.println("Servo2 180 degree");
    servoM.write(0);
    delay(1000);
    servoM.write(180);
    delay(300);
}

void moveServo3(){
    Serial.println("Servo3 180 degree");
    servoL.write(180);
    delay(1000);
    servoL.write(0);
    delay(300);
}

void setupLED(){
   pinMode(3, OUTPUT);
   pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
}

void updateLED(){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {    
       previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(3, ledState);
      digitalWrite(4, ledState);
      digitalWrite(5, ledState);
      digitalWrite(6, ledState);
    }  
 }

void openServos(){
  servoM.write(0);
  servoL.write(180);
  servoR.write(180);
}

void closeServos(){
  servoM.write(180);
  servoL.write(0);
  servoR.write(0);
}