#define USE_USBCON
#include <ros.h>
#include "interrupt_pins.h"
#include <Encoder.h>
#include <std_msgs/Int16MultiArray.h>

#define MOTOR1_EN_A 22
#define MOTOR1_EN_B 23

Encoder motor1(MOTOR1_EN_A, MOTOR1_EN_B);
// Encoder motor2(24,25);
long prevpos2 =0;
long currentpos2= 0;
long prevpos1 =0;
long currentpos1= 0;

unsigned long prevtime = 0;
std_msgs::Int16MultiArray qecounts;
char dim0_label[] = "quad";
ros::Publisher quadenc("quadenc", &qecounts);

ros::NodeHandle nh;

void setup() {
  // put your setup code here, to run once:
   
  nh.getHardware()->setBaud(57600);
  Serial.begin(57600); 

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
  /**** End of Node initializers ****/


}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  GetEncoderValues();

}

void GetEncoderValues(){

  //read velocity of selected motor
  //return velocity in rad/s
  const int encoder_1_round = 90; //define number of pulses in one round of encoder

  float rot_speed;           //rotating speed in rad/s
  const int interval = 1000; //choose interval is 1 second (1000 milliseconds)

  if((millis()-prevtime) >= 50){
    currentpos1 = motor1.read();  
    // currentpos2 = motor2.read();
    prevtime = millis();    
    // qecounts.data[0] = -(currentpos1-prevpos1); //Left Motor
    qecounts.data[0] = (float)((currentpos1-prevpos1)*60 / (encoder_1_round));
    qecounts.data[1]= currentpos2-prevpos2;   //Right Motor  
    prevpos1 = currentpos1;
    // prevpos2 = currentpos2;
  }
  quadenc.publish(&qecounts);
}

