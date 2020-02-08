
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define L_MOTORPIN1 6 //declare motor PWM pin 1
#define L_MOTORPIN2 5 //declare motor PWM pin 2
#define L_ONPIN1 37 //declare H bridge 1A/4A pin
#define L_ONPIN2 39 //declare H bridge 2A/3A pin
#define R_MOTORPIN1 4 //declare motor PWM pin 1
#define R_MOTORPIN2 3 //declare motor PWM pin 2
#define R_ONPIN1 35 //declare H bridge 1A/4A pin
#define R_ONPIN2 33 //declare H bridge 2A/3A pin


/* Autonomous Mode Variable */
/******************************************************************************************/
unsigned int x1 = 0; // reading of rear VIVE in x direction 
unsigned int x2 = 0; // reading of front VIVE in x direction 
unsigned int y1 = 0; // reading of rear VIVE in y direction 
unsigned int y2 = 0; // reading of front VIVE in y direction 
unsigned int vive_different_x = 0; // reading difference of two VIVE in x direction 
unsigned int vive_different_y = 0; // reading difference of two VIVE in y direction 
unsigned int vive_tolerance_x = 100; // tolerance in x direction
unsigned int vive_tolerance_y = 100; // tolerance in y direction 

unsigned int blue_x_target = 3552; // target blue button x coordinate 
unsigned int blue_y_target = 3932; // target blue button y coordinate 
unsigned int car_length = 100; // the length of two vive beacon 
int turn = 0; // flag of turn 
/******************************************************************************************/



/* xuejie's vive vode */
/******************************************************************************************/
const byte interruptPin_one = 18;
const byte interruptPin_two = 19;

volatile bool flag_one = false;
volatile bool flag_two = false;
int leftSpeed = 0; //declare the global int speed1 calculate from num1 for trasfer to the PWM
int rightSpeed = 0; //declare the global int speed2 calculate from num2 for trasfer to the PWM
unsigned long record_time_one = 0;
unsigned int interval_one = 0;
unsigned int falling_one = 0;
unsigned long time_record_one[7] = {0}; // record the time of this current falling
unsigned long time_diff_one[6] = {0}; // record the time difference between this current falling and its previous falling
unsigned int check_one[6] = {0}; // a boolean array, use 0 to indicate sync-sync interval, 1 to indicate others
unsigned long x_timeInterval_one = 0;
unsigned long y_timeInterval_one = 0; 
unsigned int x_index_one = 0;
unsigned int y_index_one = 0;

unsigned long record_time_two = 0;
unsigned int interval_two = 0;
unsigned int falling_two = 0;
unsigned long time_record_two[7] = {0}; // record the time of this current falling
unsigned long time_diff_two[6] = {0}; // record the time difference between this current falling and its previous falling
unsigned int check_two[6] = {0}; // a boolean array, use 0 to indicate sync-sync interval, 1 to indicate others
unsigned long x_timeInterval_two = 0;
unsigned long y_timeInterval_two = 0; 
unsigned int x_index_two = 0;
unsigned int y_index_two = 0;


void fn_one() {
  flag_one = true;
}

void fn_two() {
  flag_two = true;
}

void calculate_one(){
    if (falling_one == 0) { // 0
    time_record_one[falling_one] = micros();
    falling_one++;
  } else if (falling_one < 6) {  // 1 2 3 4 5
    time_record_one[falling_one] = micros();
    time_diff_one[falling_one - 1] = time_record_one[falling_one] - time_record_one[falling_one - 1];
    falling_one++;
  } else { // 6
    time_record_one[falling_one] = micros();
    time_diff_one[falling_one - 1] = time_record_one[falling_one] - time_record_one[falling_one - 1];
    
    for (int i = 0; i < 6; i++) {
      interval_one = time_diff_one[i]; // in micro_seconds
      if (interval_one < 8000) {
        check_one[i] = 1;
      }
      else {
        check_one[i] = 0;
      }
    }

    // traverse bool array "check" to find which is x_timeInterval and which is y_timeInterval
    for (int j = 0; j < 6; j++) {
      if (j == 0) {
        if (check_one[0] == 1 && check_one[5] == 0) {
          x_index_one = 0;
          y_index_one = 2;
        }
      } else {
        if (check_one[j] == 1 && check_one[j - 1] == 0) {
          x_index_one = j;
          y_index_one = (j + 2) % 6;
        }
      }
    }

    x_timeInterval_one = time_diff_one[x_index_one]; // in us
    y_timeInterval_one = time_diff_one[y_index_one]; // in us
    
    if (x_timeInterval_one > 1000 && x_timeInterval_one < 8200 && y_timeInterval_one > 1000 && y_timeInterval_one < 8200){
//      Serial.print("x1:  ");
//      Serial.print(x_timeInterval_one);
      x1 = x_timeInterval_one;
//      Serial.print("      ");
//      Serial.print("y1:  ");
//      Serial.print(y_timeInterval_one);
      y1 = y_timeInterval_one;
//      Serial.print("      ");
    }

    falling_one = 0;
  }
}

void calculate_two(){
    if (falling_two == 0) { // 0
    time_record_two[falling_two] = micros();
    falling_two++;
  } else if (falling_two < 6) {  // 1 2 3 4 5
    time_record_two[falling_two] = micros();
    time_diff_two[falling_two - 1] = time_record_two[falling_two] - time_record_two[falling_two - 1];
    falling_two++;
  } else { // 6
    time_record_two[falling_two] = micros();
    time_diff_two[falling_two - 1] = time_record_two[falling_two] - time_record_two[falling_two - 1];
    
    for (int i = 0; i < 6; i++) {
      interval_two = time_diff_two[i]; // in micro_seconds
      if (interval_two < 8000) {
        check_two[i] = 1;
      }
      else {
        check_two[i] = 0;
      }
    }

    // traverse bool array "check" to find which is x_timeInterval and which is y_timeInterval
    for (int j = 0; j < 6; j++) {
      if (j == 0) {
        if (check_two[0] == 1 && check_two[5] == 0) {
          x_index_two = 0;
          y_index_two = 2;
        }
      } else {
        if (check_two[j] == 1 && check_two[j - 1] == 0) {
          x_index_two = j;
          y_index_two = (j + 2) % 6;
        }
      }
    }

    x_timeInterval_two = time_diff_two[x_index_two]; // in us
    y_timeInterval_two = time_diff_two[y_index_two]; // in us
    
    if (x_timeInterval_two > 1000 && x_timeInterval_two < 8200 && y_timeInterval_two >1000 && y_timeInterval_two < 8200){
//      Serial.print("x2:  ");
//      Serial.print(x_timeInterval_two);
      x2 = x_timeInterval_two;
//      Serial.print("      ");
//      Serial.print("y2:  ");
//      Serial.print(y_timeInterval_two);
      y2 = y_timeInterval_two;
//      Serial.println("      ");
    }

    falling_two = 0;
  }
}
/******************************************************************************************/


// subroutine to make motor move forward 
void move_forward(){
    digitalWrite(R_ONPIN2, HIGH);
    digitalWrite(R_ONPIN1, LOW);
    digitalWrite(L_ONPIN2, HIGH);
    digitalWrite(L_ONPIN1, LOW);
    leftSpeed = 50;
    rightSpeed = 55;
    analogWrite(L_MOTORPIN1, leftSpeed);
    analogWrite(L_MOTORPIN2, leftSpeed); 
    analogWrite(R_MOTORPIN1, rightSpeed);
    analogWrite(R_MOTORPIN2, rightSpeed);
}

// subroutine to make motor turn right 
void turn_right(){
    digitalWrite(R_ONPIN2, HIGH);
    digitalWrite(R_ONPIN1, LOW);
    digitalWrite(L_ONPIN2, HIGH);
    digitalWrite(L_ONPIN1, LOW);
    leftSpeed = 50;
    rightSpeed = -50;
    analogWrite(L_MOTORPIN1, leftSpeed);
    analogWrite(L_MOTORPIN2, leftSpeed); 
    analogWrite(R_MOTORPIN1, rightSpeed);
    analogWrite(R_MOTORPIN2, rightSpeed);
}

// subroutine to make motor turn left 
void turn_left(){
    digitalWrite(R_ONPIN2, LOW);
    digitalWrite(R_ONPIN1, HIGH);
    digitalWrite(L_ONPIN2, LOW);
    digitalWrite(L_ONPIN1, HIGH);
    leftSpeed = -50;
    rightSpeed = 50;
    analogWrite(L_MOTORPIN1, leftSpeed);
    analogWrite(L_MOTORPIN2, leftSpeed); 
    analogWrite(R_MOTORPIN1, rightSpeed);
    analogWrite(R_MOTORPIN2, rightSpeed);
}

// subroutine to make motor stop 
void car_stop(){
    digitalWrite(R_ONPIN2, LOW);
    digitalWrite(R_ONPIN1, LOW);
    digitalWrite(L_ONPIN2, LOW);
    digitalWrite(L_ONPIN1, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(R_MOTORPIN2, OUTPUT); //set the motor PWM pin 1
  pinMode(R_MOTORPIN1, OUTPUT); //set the motor PWM pin 1
  pinMode(L_MOTORPIN2, OUTPUT); //set the motor PWM pin 1
  pinMode(L_MOTORPIN1, OUTPUT); //set the motor PWM pin 1
  pinMode(R_ONPIN1, OUTPUT); //set H bridge 1A/4A pin
  pinMode(R_ONPIN2, OUTPUT); //set H bridge 2A/3A pin
  pinMode(L_ONPIN1, OUTPUT); //set H bridge 1A/4A pin
  pinMode(L_ONPIN2, OUTPUT); //set H bridge 2A/3A pin

  pinMode(interruptPin_one, INPUT_PULLUP);
  pinMode(interruptPin_two, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin_one), fn_one, RISING); // vive interrupt pin 
  attachInterrupt(digitalPinToInterrupt(interruptPin_two), fn_two, RISING);
}

void loop() {

/* receive vive reading */
/*********************************************************************/
  while (flag_one == false) { 
    // leaves the loop when a rising edge is detected
  }

  // decides which pin (18 or 19) gets interrupt
  if (flag_one == true){
    // pin 18 catches an interrupt, go to calculate_one() to do calculation
    calculate_one();
    flag_one = false;
  }
  while (flag_two == false) { 
    // leaves the loop when a rising edge is detected
  }
  if (flag_two == true){
    // pin 19 catches an interrupt, go to calculate_two() to do calculation
    calculate_two();
    flag_two = false;
  }
/*********************************************************************/


/* calculate movement based on vive reading */
/*********************************************************************/
  vive_different_x = x2 - x1; // calculate vive reading difference in x direction 
  vive_different_y = y2 - y1; // calculate vive reading difference in y direction 

  if (x1 > 4300 && y1 < 2600) {
    if (abs(vive_different_x) > vive_tolerance_x || vive_different_y < car_length){ 
      if(vive_different_y < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }else if (abs(vive_different_x) > vive_tolerance_x && vive_different_y < car_length){ 
      move_forward(); // move forward if car is align with desired orientation 
    }
  }

  else if (x1 > 4300 && y1 > 2600){
    if (vive_different_x < car_length || abs(vive_different_y) > vive_tolerance_y){
      if(vive_different_x < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }else if (vive_different_x < car_length && abs(vive_different_y) > vive_tolerance_y){
      move_forward(); // move forward if car is align with desired orientation 
    }
  }
  
 else if (x1 < 4300 && x1 > 3800 && y1 > 2300){
    if (abs(vive_different_x) > vive_tolerance_x || vive_different_y < car_length){
      if(vive_different_y < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }
    else if(abs(vive_different_x) > vive_tolerance_x && vive_different_y < car_length){
      move_forward(); // move forward if car is align with desired orientation 
    }
 }
 
 else if (x1 < 4300 && x1 > 3800 && y1 < 2300){
    if (vive_different_x < car_length || abs(vive_different_y) > vive_tolerance_y){
      if(vive_different_y < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }
    else if(vive_different_x < car_length && abs(vive_different_y) > vive_tolerance_y){
      move_forward(); // move forward if car is align with desired orientation 
    }
 }

 else if (x1 > 3662 && x1 < 3800 && y1 < 2300){
    if (vive_different_x < car_length || abs(vive_different_y) > vive_tolerance_y){
      if(vive_different_y < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }
    else if (vive_different_x < car_length && abs(vive_different_y) > vive_tolerance_y){
      move_forward(); // move forward if car is align with desired orientation 
    }
 }

 else if (x1 < 3662){
    if (abs(vive_different_x) > vive_tolerance_x || vive_different_y < car_length){
      if(vive_different_x < 0) {
        turn_right(); // adjust orientation if car isn't align with desired orientation, turn right if y2 < y1
      } else {
        turn_left(); // adjust orientation if car isn't align with desired orientation, turn left if y2 > y1
      }
    }else if (abs(vive_different_x) > vive_tolerance_x && vive_different_y < car_length){
      move_forward(); // move forward if car is align with desired orientation 
    }
      else if (abs(x1 - blue_x_target) <= vive_tolerance_x && abs(y1 - blue_y_target) <= vive_tolerance_y){
       if (vive_different_x < car_length || vive_different_y > vive_tolerance_y){
            turn_left(); // turn left in the hill
        }
        else if (abs(vive_different_x) > vive_tolerance_x && vive_different_y < car_length){
          move_forward(); // move forward if car is align with desired orientation 
        }
    }
  }
}
