#define USE_USBCON
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_robot/MotorPWM.h>

#define MOTOR1_PWM_PIN 8
#define MOTOR1_DIR_PIN_22 22
#define MOTOR1_DIR_PIN_23 23
#define MOTOR1_EN_A 24
#define MOTOR1_EN_B 25

Encoder motor1(MOTOR1_EN_A, MOTOR1_EN_B);
long prevpos1 =0;
long currentpos1= 0;
unsigned long prevtime = 0;
std_msgs::Int16MultiArray qecounts;
char dim0_label[] = "quad";
ros::Publisher quadenc("quadenc", &qecounts);

float motor_input_pwm = 0;

void get_motor_pwm( const nav_robot::MotorPWM& pwmValues);
ros::Subscriber<nav_robot::MotorPWM> pwm("/motor_pwm", &get_motor_pwm );

ros::NodeHandle nh;

void setup() {
  // put your setup code here, to run once:

  nh.getHardware()->setBaud(57600);
  Serial.begin(57600);

  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_22, OUTPUT);
  pinMode(MOTOR1_DIR_PIN_23, OUTPUT);
  pinMode(MOTOR1_EN_A, INPUT_PULLUP);
  pinMode(MOTOR1_EN_B,  INPUT_PULLUP);

  /* qe Initialization starts here */
  
    qecounts.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    qecounts.layout.dim[0].label = dim0_label;
    qecounts.layout.dim[0].size = 2;
    qecounts.layout.dim[0].stride = 1*2;
    qecounts.layout.data_offset = 0;
    qecounts.data_length = 2;
    qecounts.data = (short int *)malloc(sizeof(short int)*2);

  /* qe Initialization ends here */

  /**** Node initializers ****/     
    // nh. initNode();
    nh.advertise(quadenc);
    nh.subscribe(pwm);
  /**** End of Node initializers ****/

}

void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce();
  get_encoder_values();
  run_motor(motor_input_pwm);
}

void get_encoder_values(){

  //read velocity of selected motor
  //return velocity in rad/s
  const int encoder_1_round = 90; //define number of pulses in one round of encoder

  // float motor1_speed;           //rotating speed in rad/s
  const int interval = 1000; //choose interval is 1 second (1000 milliseconds)

  if((millis()-prevtime) >= interval){
    currentpos1 = motor1.read();  
    prevtime = millis();    
    qecounts.data[0] = (float)((currentpos1-prevpos1)*60 / (encoder_1_round));
    qecounts.data[1] = currentpos1-prevpos1;
    prevpos1 = currentpos1;    
  }  
  quadenc.publish(&qecounts);
}

void run_motor(int pwm){

  if (pwm > 0){ //clockwise rotation
    set_motor1_speed(pwm);
    rotate_motor1_cw();
  }else if (pwm < 0){ //anticlockwise rotation
    set_motor1_speed(abs(pwm));
    rotate_motor1_ccw();
  }else{  //Zero speed
    set_motor1_speed(0);
  }

}

void get_motor_pwm( const nav_robot::MotorPWM& pwmValues){
  
    motor_input_pwm = pwmValues.R_MOTOR_SPEED;
}

void set_motor1_speed(int motor_speed){

  analogWrite(MOTOR1_PWM_PIN,motor_speed);

}

void rotate_motor1_cw(){

  digitalWrite(MOTOR1_DIR_PIN_22,LOW);
  digitalWrite(MOTOR1_DIR_PIN_23,HIGH);

}

void rotate_motor1_ccw(){

  digitalWrite(MOTOR1_DIR_PIN_22,HIGH);
  digitalWrite(MOTOR1_DIR_PIN_23,LOW);

}