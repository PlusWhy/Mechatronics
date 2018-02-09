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
