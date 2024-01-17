#define USE_USBCON

#define MOTOR1_PWM_PIN 8
#define MOTOR1_DIR_PIN_22 22
#define MOTOR1_DIR_PIN_23 23

void setup()
{
    
  Serial.begin(57600);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_22, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_23, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  rotate_motor1_cw();
  set_motor1_speed(110);

  delay(2000);

  rotate_motor1_ccw();
  set_motor1_speed(200);

  delay(2000);

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