//This is the code to put on the race car side- Mega

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include "pitches.h"

#define L_MOTORPIN1 6 //declare left motor PWM pin 1
#define L_MOTORPIN2 5 //declare left motor PWM pin 2
#define L_ONPIN1 37 //declare left H bridge 1A/4A pin
#define L_ONPIN2 39 //declare left H bridge 2A/3A pin
#define R_MOTORPIN1 4 //declare right motor PWM pin 1
#define R_MOTORPIN2 3 //declare right motor PWM pin 2
#define R_ONPIN1 35 //declare right H bridge 1A/4A pin
#define R_ONPIN2 33 //declare right H bridge 2A/3A pin
#define weaponPin1 8 //declare servo PWM pin 1
#define weaponPin2 9 //declare servo PWM pin 2
#define LED1 24 //declare the LED pin 1 
#define LED2 26//declare the LED pin 1 


Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
byte rsComMega[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //declare the accept array through UART
byte currentArray[30] = {0}; //declare the adjust array with 3 times length of the original one

const byte interruptPin = 2; //declare the interrupt pin
unsigned long record_time = 0; //declare the record time
volatile bool flag = false; //decare the flag

unsigned int interval = 0;
unsigned int falling = 0;
unsigned long time_record[7] = {0}; // record the time of this current falling
unsigned long time_diff[6] = {0}; // record the time difference between this current falling and its previous falling
unsigned int check[6] = {0}; // a boolean array, use 0 to indicate sync-sync interval, 1 to indicate others
unsigned long x_timeInterval = 0;
unsigned long y_timeInterval = 0;
unsigned int x_index = 0;
unsigned int y_index = 0;
int xJoystick = 0; //declare the global int num1 for speed of right recieving from remote controller
int yJoystick = 0; //declare the global int num2 for speed of right recieving from remote controller
int stop_switch = 0; //declare the global int num3 for on/off mode recieving from remote controller
int leftSpeed = 0; //declare the global int speed1 calculate from num1 for trasfer to the PWM
int rightSpeed = 0; //declare the global int speed2 calculate from num2 for trasfer to the PWM

int weaponAngle1 = 0; //declare the weapon angle 1 accepted
int weaponAngle2 = 0; //declare the weapon angle 2accepted
int weaponOn = 0; //declare the weapon status accepted

int health = 1; //declare the remain health accepted
int automomous = 1;//declare the auto mode accepted
int game = 1;//declare the game status accepted

int respawnTimer  = 1;//declare the remain respwawnTumer accepted

int music = 0;//declare the music flag
int death = 0;//declare whether the car is died or not
int autoEnd = 0;//declare the auto mode end flag

void fn() {
  flag = true;
  //Serial.println("inside fn()...");
}


int melody[] =
{
  NOTE_B5, NOTE_E5, NOTE_G5, NOTE_FS5, NOTE_E5, NOTE_B6, NOTE_A6, NOTE_FS5, NOTE_E5, NOTE_G5,
  NOTE_FS5,  NOTE_DS5, NOTE_F5, NOTE_B4, NOTE_B4, NOTE_E5, NOTE_G5, NOTE_FS5, NOTE_E5, NOTE_B6,
  NOTE_D6, NOTE_CS6, NOTE_C6, NOTE_GS6, NOTE_C6, NOTE_B6, NOTE_GS5, NOTE_GS4, NOTE_G5, NOTE_E5,
  //the melody of part of Hedwig's Theme

};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] =
{
  8, 6, 16, 8, 4, 8, 3, 3, 6, 16,
  8, 5, 8, 3, 8, 6, 16, 8, 4, 8,
  4, 8, 4, 8, 6, 16, 8, 4, 8, 3,
  //the node duration of the melofy
};


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1);
  pinMode(R_MOTORPIN2, OUTPUT); //set the motor PWM pin 1
  pinMode(R_MOTORPIN1, OUTPUT); //set the motor PWM pin 1
  pinMode(L_MOTORPIN2, OUTPUT); //set the motor PWM pin 1
  pinMode(L_MOTORPIN1, OUTPUT); //set the motor PWM pin 1
  pinMode(R_ONPIN1, OUTPUT); //set H bridge 1A/4A pin
  pinMode(R_ONPIN2, OUTPUT); //set H bridge 2A/3A pin
  pinMode(L_ONPIN1, OUTPUT); //set H bridge 1A/4A pin
  pinMode(L_ONPIN2, OUTPUT); //set H bridge 2A/3A pin
  myservo1.attach(weaponPin1);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(weaponPin2);  // attaches the servo on pin 9 to the servo object
  pinMode(weaponPin1, OUTPUT); //set the servo PWM pin 1
  pinMode(weaponPin2, OUTPUT); //set the servo PWM pin 2
  pinMode(LED1, OUTPUT); //set the LED pin 1
  pinMode(LED2, OUTPUT); //set the LED pin 2
  pinMode(interruptPin, INPUT_PULLUP);//set the interrupt pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), fn, RISING);
  delay(50);


}

void loop() {


  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);


  //recieve the message from ESP 32
  if (Serial2.available()) {
    Serial2.readBytes(rsComMega, 10);
    delay(7);

  }
  
  int shift = 6;//declare the shift index

//save the messages from the array    
  if (autoEnd == 0) {
      xJoystick =  rsComMega[0]; 
      yJoystick = rsComMega[1]; 
      stop_switch = rsComMega[2]; 
      weaponAngle1 =  rsComMega[3];
      weaponAngle2 =  rsComMega[4];
      weaponOn =  rsComMega[5];
      health = rsComMega[6];
      automomous = rsComMega[7];
      game = rsComMega[8];
      respawnTimer = rsComMega[9];

  //shift back the array index in case it will have self shift after the auto mode end
  } else if (autoEnd == 1){
    //make the array 3 times in a new array
    for (int i = 0; i < 10; i++) {
      currentArray[i] = rsComMega[i];
      currentArray[i + 10] = rsComMega[i];
      currentArray[i + 20] = rsComMega[i];
      //at the time auto mode end, auto mode will be 0 and game status will be 1
      if (rsComMega[i] == 0 && rsComMega[i + 1] == 1) {
        shift = i + 10;
      }
    }
    //save the numbers from the middle 10s of the numbers, 30 is to make sure there alwya some number after calculation of shift
    xJoystick =  currentArray[shift - 7]; //num1= 1st index of packetBuffer
    yJoystick = currentArray[shift - 6]; //num1= 2nd index of packetBuffer
    stop_switch = currentArray[shift - 5]; //num1= 2nd index of packetBuffer
    weaponAngle1 =  currentArray[shift - 4];
    weaponAngle2 =  currentArray[shift - 3];
    weaponOn =  currentArray[shift - 2];
    health = currentArray[shift - 1];
    automomous = currentArray[shift];
    game = currentArray[shift + 1];
    respawnTimer = currentArray[shift + 2];



    if (death == 0) {
      //save the messages from the array 
      xJoystick =  rsComMega[0]; 
      yJoystick = rsComMega[1]; 
      stop_switch = rsComMega[2]; 
      weaponAngle1 =  rsComMega[3];
      weaponAngle2 =  rsComMega[4];
      weaponOn =  rsComMega[5];
      health = rsComMega[6];
      automomous = rsComMega[7];
      game = rsComMega[8];
      respawnTimer = rsComMega[9];
    } else if (death == 1) {
      //    same logic as above
      //make the array 3 times in a new array
      for (int i = 0; i < 10; i++) {
        currentArray[i] = rsComMega[i];
        currentArray[i + 10] = rsComMega[i];
        currentArray[i + 20] = rsComMega[i];
        if (rsComMega[i] == 0 && rsComMega[i + 1] == 1) {
          shift = i + 10;
        }
      }
    //save the numbers from the middle 10s of the numbers, 30 is to make sure there alwya some number after calculation of shift
      xJoystick =  currentArray[shift - 7]; 
      yJoystick = currentArray[shift - 6]; 
      stop_switch = currentArray[shift - 5]; 
      weaponAngle1 =  currentArray[shift - 4];
      weaponAngle2 =  currentArray[shift - 3];
      weaponOn =  currentArray[shift - 2];
      health = currentArray[shift - 1];
      automomous = currentArray[shift];
      game = currentArray[shift + 1];
      respawnTimer = currentArray[shift + 2];

    }
  }


//
//  //  Serial.println("value is:");
//  //  Serial.print("x is :   ");
//  Serial.println(xJoystick); //Print data on Serial Monitor
//  //  Serial.print("y is :   ");
//  Serial.println(yJoystick); //Print data on Serial Monitor
//  //  Serial.print("stop is :   ");
//  Serial.println(stop_switch); //Print data on Serial Monitor
//  //  Serial.print("W1 is :   ");
//  Serial.println(weaponAngle1); //Print data on Serial Monitor
//  //  Serial.print("W2 is :   ");
//  Serial.println(weaponAngle2); //Print data on Serial Monitor
//  //  Serial.print("W on is :   ");
//  Serial.println(weaponOn); //Print data on Serial Monitor
//  //  Serial.print("Still remain health?   ");
//  Serial.println(health);
//  //  Serial.print("Automomous Start?   ");
//  Serial.println(automomous);
//  //  Serial.print("Game Start?   ");
//  Serial.println(game);
//  //  Serial.print("respawnTimer ");
//  Serial.println(respawnTimer);
//  Serial.println("");

  if (stop_switch == 1 || game == 0 || health == 0 || automomous == 1) { //if switch is shift, stop the car
    digitalWrite(R_ONPIN2, LOW);
    digitalWrite(R_ONPIN1, LOW);
    digitalWrite(L_ONPIN2, LOW);
    digitalWrite(L_ONPIN1, LOW);
  }

  if (weaponOn == 1 || game == 0) {
    digitalWrite(weaponPin1, LOW);
    digitalWrite(weaponPin2, LOW);

  }


  if (game == 1) {
    //
    if (automomous == 1) {


      //      while (flag == false) {
      //        // leaves the loop when a falling edge is detected
      //      }
      //      flag = false;
      //      //    Serial.print(micros());
      //      if (falling == 0) { // 0
      //        //      Serial.print("falling 1");
      //        time_record[falling] = micros();
      //        //delay(1);
      //        //Serial.println(time_record[falling]);
      //        falling++;
      //      } else if (falling < 6) {  // 1 2 3 4 5
      //        //      Serial.print("falling 2");
      //        time_record[falling] = micros();
      //        time_diff[falling - 1] = time_record[falling] - time_record[falling - 1];
      //        //Serial.println(time_record[falling]);
      //        falling++;
      //      } else { // 6
      //        Serial.print("falling 3");
      //        time_record[falling] = micros();
      //        time_diff[falling - 1] = time_record[falling] - time_record[falling - 1];
      //        //      Serial.println(time_diff[0]);
      //        //      Serial.println(time_diff[1]);
      //        //      Serial.println(time_diff[2]);
      //        //      Serial.println(time_diff[3]);
      //        //      Serial.println(time_diff[4]);
      //        //      Serial.println(time_diff[5]);
      //        //Serial.println(time_record[falling]);
      //
      //        // traverse time_diff, mark sync as 0, others as 1
      //        for (int i = 0; i < 6; i++) {
      //          interval = time_diff[i]; // in micro_seconds
      //          if (interval < 8000) {
      //            check[i] = 1;
      //          }
      //          else {
      //            check[i] = 0;
      //          }
      //        }
      //
      //        // traverse bool array "check" to find which is x_timeInterval and which is y_timeInterval
      //        for (int j = 0; j < 6; j++) {
      //          if (j == 0) {
      //            if (check[0] == 1 && check[5] == 0) {
      //              x_index = 0;
      //              y_index = 2;
      //            }
      //          } else {
      //            if (check[j] == 1 && check[j - 1] == 0) {
      //              x_index = j;
      //              y_index = (j + 2) % 6;
      //            }
      //          }
      //        }
      //
      //        x_timeInterval = time_diff[x_index]; // in us
      //        y_timeInterval = time_diff[y_index]; // in us
      //
      //        if (x_timeInterval < 8200 && y_timeInterval < 8200) {
      //          Serial.print("x:  ");
      //          Serial.println(x_timeInterval);
      //          Serial.print("y:  ");
      //          Serial.println(y_timeInterval);
      //        }
      //        falling = 0;
      //        //delay(3);
      //      }
    } else if (automomous == 0) {
      autoEnd = 1;//auto end flag on

      //if the game is in the remote control status
      if ((stop_switch == 2 || game == 1) && health != 0 && automomous == 0) { //start the car by switch on remote control side

        // move forward
        if (yJoystick > 130) {
          digitalWrite(R_ONPIN2, LOW);
          digitalWrite(R_ONPIN1, HIGH);
          digitalWrite(L_ONPIN2, LOW);
          digitalWrite(L_ONPIN1, HIGH);
          leftSpeed = map(yJoystick, 138, 255, 128, 254);  //map and read the ADC number as speed to go forward for right wheel
          rightSpeed = map(yJoystick, 138, 255, 128, 255);  //map and read the ADC number as speed to go forward for left wheel
        }

        //move backward
        else if (yJoystick < 108) {
          digitalWrite(R_ONPIN2, HIGH);
          digitalWrite(R_ONPIN1, LOW);
          digitalWrite(L_ONPIN2, HIGH);
          digitalWrite(L_ONPIN1, LOW);
          leftSpeed = map(yJoystick, 118, 0, 128, 254);  //map and read the ADC number as speed to go backward for right wheel
          rightSpeed = map(yJoystick, 118, 0, 128, 255);  //map and read the ADC number as speed to go backward for left wheel
        }

        else {
          rightSpeed = 0;
          leftSpeed = 0;
        }
        //move rightward, when speed of left wheel is higher than the right wheel
        if (xJoystick < 108) {
          int xMapped = map(xJoystick, 110, 1, 128, 255);
          rightSpeed -= xMapped;
          if (rightSpeed < 0) {
            rightSpeed = 0;
          }
          if (leftSpeed > 255) {
            leftSpeed = 255;
          }
        }

        //move leftward, when speed of right wheel is higher than the left wheel
        if (xJoystick > 126) {
          int xMapped = map(xJoystick, 126, 255, 128, 255);
          leftSpeed -= xMapped;
          if (rightSpeed > 255) {
            rightSpeed = 255;
          }
          if (leftSpeed < 0 ) {
            leftSpeed = 0;
          }
        }

        //use PWM to control the speed of the car
        analogWrite(L_MOTORPIN1, leftSpeed);
        analogWrite(L_MOTORPIN2, leftSpeed);
        analogWrite(R_MOTORPIN1, rightSpeed);
        analogWrite(R_MOTORPIN2, rightSpeed);
        Serial.println(leftSpeed);
        Serial.println(rightSpeed);
        Serial.println();
      }

// stop the car when the switch is off, game is end, health is 0, auto is on
      if (stop_switch == 1 || game == 0 || health == 0 || automomous == 1) { //if switch is shift, stop the car
        digitalWrite(R_ONPIN2, LOW);
        digitalWrite(R_ONPIN1, LOW);
        digitalWrite(L_ONPIN2, LOW);
        digitalWrite(L_ONPIN1, LOW);
      }
//weapon is on when the switch is on, game is start, health is not 0,  auto is off
      if (( weaponOn == 2 && game == 1) && health != 0 && automomous == 0) {
        myservo1.write(weaponAngle1);
        myservo2.write(weaponAngle2);
      }
      
//stop the weapon when the switch is off
      if (weaponOn == 1) {
        digitalWrite(weaponPin1, LOW);
        digitalWrite(weaponPin2, LOW);
      }
      
      if (health == 0) { //if switch is shift, stop the car
        digitalWrite(R_ONPIN2, LOW);
        digitalWrite(R_ONPIN1, LOW);
        digitalWrite(L_ONPIN2, LOW);
        digitalWrite(L_ONPIN1, LOW);
      }

// when health is 0, respawnTimer is 15 and music flag is 0, start play the music
      if (health == 0 && respawnTimer == 15 && music == 0) { //if switch is shift, stop the car
        // iterate over the notes of the melody:
        int size = sizeof(melody) / sizeof(int);
        for (int thisNote = 0; thisNote < size; thisNote++)
        {
          // to calculate the note duration, take one second
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1200 / noteDurations[thisNote];
          tone(50, melody[thisNote], noteDuration);


          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.8;
          delay(pauseBetweenNotes);
          // stop the tone playing:
          noTone(50);
          music = 1;
        }

        //death flag is on
        death = 1;

      }

      if (respawnTimer == 2) {
        music = 0;
//music flag is off when respawnTimer is 2
      }
    }
  } else {
    digitalWrite(R_ONPIN2, LOW);
    digitalWrite(R_ONPIN1, LOW);
    digitalWrite(L_ONPIN2, LOW);
    digitalWrite(L_ONPIN1, LOW);
    digitalWrite(weaponPin1, LOW);
    digitalWrite(weaponPin2, LOW);
//stop the car and weapon when game status is 0
  }

}
