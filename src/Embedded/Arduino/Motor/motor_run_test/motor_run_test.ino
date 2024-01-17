#define USE_USBCON

#define MOTOR1_PWM_PIN 8
#define MOTOR1_DIR_PIN 3

void setup()
{
    
  Serial.begin(57600);            

  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);

  
}

void loop()
{  
  TestMotor();  
}

void TestMotor()
{       

    delay(5000);
    analogWrite(MOTOR1_PWM_PIN,100);
    digitalWrite(MOTOR1_DIR_PIN,HIGH);

    delay(2000);
    analogWrite(MOTOR1_PWM_PIN,150);
    digitalWrite(MOTOR1_DIR_PIN,LOW);

    delay(2000);
    analogWrite(MOTOR1_PWM_PIN,200);
    digitalWrite(MOTOR1_DIR_PIN,HIGH);

    delay(2000);
    analogWrite(MOTOR1_PWM_PIN,255);
    digitalWrite(MOTOR1_DIR_PIN,LOW);

    
}