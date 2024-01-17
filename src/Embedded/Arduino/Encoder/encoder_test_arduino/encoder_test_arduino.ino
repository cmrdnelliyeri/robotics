#define USE_USBCON
#include <Encoder.h>

#define MOTOR1_PWM_PIN 8
#define MOTOR1_DIR_PIN_22 22
#define MOTOR1_DIR_PIN_23 23
#define MOTOR1_EN_A 24
#define MOTOR1_EN_B 25

Encoder motor1(MOTOR1_EN_A, MOTOR1_EN_B);
long prevpos1 =0;
long currentpos1= 0;

unsigned long prevtime = 0;

void setup()
{
    
  Serial.begin(9600);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_22, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_23, OUTPUT);
  pinMode(MOTOR1_EN_A, INPUT_PULLUP);
  pinMode(MOTOR1_EN_B,  INPUT_PULLUP);
  
}

void loop() {

  // put your main code here, to run repeatedly:
  rotate_motor1_ccw();
  set_motor1_speed(150);
  GetEncoderValues();

}

void set_motor1_speed(int motor_speed){

  analogWrite(MOTOR1_PWM_PIN,motor_speed);

}

void rotate_motor1_cw(){

  digitalWrite(MOTOR1_DIR_PIN_22,HIGH);
  digitalWrite(MOTOR1_DIR_PIN_23,LOW);

}

void rotate_motor1_ccw(){

  digitalWrite(MOTOR1_DIR_PIN_22,LOW);
  digitalWrite(MOTOR1_DIR_PIN_23,HIGH);

}

void GetEncoderValues(){

  //read velocity of selected motor
  //return velocity in rad/s
  const int encoder_1_round = 90; //define number of pulses in one round of encoder

  float motor1_speed;           //rotating speed in rad/s
  const int interval = 1000; //choose interval is 1 second (1000 milliseconds)

  if((millis()-prevtime) >= 50){
    currentpos1 = motor1.read();  
    prevtime = millis();    
    motor1_speed = (float)((currentpos1-prevpos1)*60 / (encoder_1_round));
    Serial.println(motor1_speed);
    prevpos1 = currentpos1;    
  }  
}