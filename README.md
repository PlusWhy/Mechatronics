# Mechatronics
Mechatronics-CCA-Spring-2018


# week3
## video

https://youtu.be/ZzuQMN5Wqg8

## code

    const int motorPin = 3;
    const int switchPin = 2;
    const int lightSensorPin = A0;

    void setup() {
    pinMode (motorPin, OUTPUT);// Not necessary but good practice for clarity
    pinMode (switchPin, INPUT); // Not necessary but good practice for clarity
    pinMode (A0, INPUT); // Not necessary but good practice for clarity
    Serial.begin(9600);
         }

    void loop() {

    int sensorValue = analogRead(lightSensorPin);
    Serial.println(sensorValue); // Report the light sensor reading, so I can make adjustments

    if (HIGH == digitalRead(switchPin) ) { // read the pushbutton

    if ( sensorValue > 10 )  { // lots of light
      analogWrite(motorPin, 225);    // move the motor
    } else { // no light; mechanism must be blocking sensor
      analogWrite(motorPin, 0); // turn off motor
    }
    } else { // the switch must be off
    analogWrite(motorPin, 0); // so turn the motor off
    }
    }

# Drawing machine

I found a website to customize the gear. And slao, I can test it.
#The link:
http://geargenerator.com/#200,200,100,6,1,0,0,4,1,8,2,4,27,-90,0,0,16,4,4,27,-60,1,1,12,1,12,20,-60,2,0,60,5,12,20,0,0,0,2,-563

Then, I went to the Lazercut studio in CCA and cut the Gear.
## The gear's file:
[gear.ai.zip](https://github.com/PlusWhy/Mechatronics/files/1793743/gear.ai.zip)

## The image:

![img_6029](https://user-images.githubusercontent.com/35580394/37160272-b00f7524-22a4-11e8-9675-c661a1e2ef24.JPG)
![img_6030](https://user-images.githubusercontent.com/35580394/37160278-b36eeb78-22a4-11e8-9dc5-c0aef17f416f.JPG)
#The test:
## Test:

https://youtu.be/5Q2ABg-lE8A

# Arduino Car


## There are two reference:

1: Obstacle Avoiding Robot using L298N

http://mertarduinotutorial.blogspot.com.tr/2017/03/arduino-project-tutorial-17-obstacle.html

2: How To Make a Simple Autonomous Robot

http://www.evolutionandextinction.co.in/2017/02/making-of-simple-autonomous-robot.html

## Image

![img_6036](https://user-images.githubusercontent.com/35580394/37159302-558102f0-22a2-11e8-9240-d8b9ba82b754.JPG)

![img_6037](https://user-images.githubusercontent.com/35580394/37159319-5e8c214a-22a2-11e8-8467-a478d39fe9cd.JPG)

![img_6039](https://user-images.githubusercontent.com/35580394/37159328-65c84286-22a2-11e8-8366-ca421d5b5f8b.JPG)

## Test：
https://youtu.be/yfpSTMpGJLQ

## Code:

    #include <Servo.h>          //Servo motor library. This is standard library
    #include <NewPing.h>        //Ultrasonic sensor function library. You must install this library

    //our L298N control pins
    const int LeftMotorForward = 7;
    const int LeftMotorBackward = 6;
    const int RightMotorForward = 4;
    const int RightMotorBackward = 5;

    //sensor pins
    #define trig_pin A1 //analog input 1
    #define echo_pin A2 //analog input 2

    #define maximum_distance 200
    boolean goesForward = false;
    int distance = 100;

    NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
    Servo servo_motor; //our servo name


    void setup(){

    pinMode(RightMotorForward, OUTPUT);
    pinMode(LeftMotorForward, OUTPUT);
    pinMode(LeftMotorBackward, OUTPUT);
    pinMode(RightMotorBackward, OUTPUT);
  
    servo_motor.attach(10); //our servo pin

    servo_motor.write(115);
    delay(2000);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    }

    void loop(){

    int distanceRight = 0;
    int distanceLeft = 0;
    delay(50);

    if (distance <= 20){
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft){
      turnRight();
      moveStop();
     }
    else{
      turnLeft();
      moveStop();
    }
    }
    else{
    moveForward(); 
    }
    distance = readPing();
    }

    int lookRight(){  
    servo_motor.write(50);
    delay(500);
    int distance = readPing();
    delay(100);
    servo_motor.write(115);
    return distance;
    }

    int lookLeft(){
    servo_motor.write(170);
    delay(500);
    int distance = readPing();
    delay(100);
    servo_motor.write(115);
    return distance;
    delay(100);
    }

    int readPing(){
    delay(70);
    int cm = sonar.ping_cm();
    if (cm==0){
    cm=250;
    }
    return cm;
    }

    void moveStop(){
  
    digitalWrite(RightMotorForward, LOW);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(LeftMotorBackward, LOW);
    }

    void moveForward(){

    if(!goesForward){

    goesForward=true;
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
    }
    }

    void moveBackward(){

    goesForward=false;

    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
  
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorForward, LOW);
  
    }

    void turnRight(){

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW);
  
    delay(500);
  
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
 
  
  
    }

    void turnLeft(){

    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);

    delay(500);
  
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    }
# Final project
## Stabilizing machine

This is a machine that can automatically maintain its level. It can read the horizontal and vertical tilt angles, and then drive the servo rotation to maintain balance. It can help the restaurant attendant or can help the photographer stabilize the camera. In short, it is cool.

### Image

![img_6855](https://user-images.githubusercontent.com/35580394/39659406-4b5d0122-4fdc-11e8-842a-2e47523611e1.jpg)

![img_6857](https://user-images.githubusercontent.com/35580394/39659408-4e90b88e-4fdc-11e8-8319-c08a14f2ab42.jpg)

![img_6859](https://user-images.githubusercontent.com/35580394/39659410-4fdcbdbe-4fdc-11e8-88ab-6b5d2699f6be.jpg)

![img_6856](https://user-images.githubusercontent.com/35580394/39659411-57deeae6-4fdc-11e8-87b5-4986a0bc896e.jpg)

### Video

https://youtu.be/fkrfpTzuxd0

### Code
    #include <Servo.h>
    #include "I2Cdev.h"
    #include "MPU6050_6Axis_MotionApps20.h"
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
    #endif

    #define LED_PIN 13
    bool blinkState = true;

    Servo Servo1;
    Servo Servo2;

    int Servo1Pos = 0;
    int Servo2Pos = 0;
    
    float mpuPitch = 0;
    float mpuRoll = 0;
    float mpuYaw = 0;

    MPU6050 mpu;


    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];


    Quaternion q;
    VectorInt16 aa;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;
    float ypr[3];


    #define PITCH   1
    #define ROLL  2
    #define YAW   0



    void setup()
    {

      Servo1.attach(10);
    Servo2.attach(11);
    delay(50);
    Servo1.write(0);
    Servo2.write(60);
    delay(500);
    Servo1.write(180);
      Servo2.write(120);
      delay(500);
      Servo1.write(0);
      Servo2.write(90);
      delay(500);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(118);
    mpu.setYGyroOffset(-44);
    mpu.setZGyroOffset(337);
    mpu.setXAccelOffset(-651);
    mpu.setYAccelOffset(670);
    mpu.setZAccelOffset(1895);


    if (devStatus == 0)
    {

    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();


    packetSize = mpu.dmpGetFIFOPacketSize();
    }
    pinMode(LED_PIN, OUTPUT);

    } // setup()


    void loop(void)
    {
    processAccelGyro();
    }



    void processAccelGyro()
    {


    mpuIntStatus = mpu.getIntStatus();


    fifoCount = mpu.getFIFOCount();


    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {

    mpu.resetFIFO();
    return;
    }

    if (mpuIntStatus & 0x02)
    {

    if (fifoCount < packetSize)
      return;


    mpu.getFIFOBytes(fifoBuffer, packetSize);


    fifoCount -= packetSize;


    mpu.resetFIFO();


    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuYaw  = ypr[YAW] * 180 / M_PI;


    mpu.resetFIFO();


    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);


    mpu.resetFIFO();

    Servo1.write(-mpuPitch + 90);
    Servo2.write(mpuRoll + 90);

    mpu.resetFIFO();

    }
    }





