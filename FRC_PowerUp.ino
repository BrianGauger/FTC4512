/* FRC RIODUINO CODE FOR 2018
   This sketch does the following:
   * 
*/

//
#include <Wire.h>
#include <NewPing.h>

// Constants  
const int frontLeftTrigPin = 2;
const int frontLeftEchoPin = 3;
const int frontRightTrigPin = 4;
const int frontRightEchoPin = 5;
const int backLeftTrigPin = 7;
const int backLeftEchoPin = 8;
const int backRightTrigPin = 12;
const int backRightEchoPin = 13;

const int maxDistance = 400;

// Distances calculated from ultrasonic sensor measurements
long frontLeftDistanceCm = 0;
long frontRightDistanceCm = 0;
long backLeftDistanceCm = 0;
long backRightDistanceCm = 0;

NewPing frontLeft(frontLeftTrigPin, frontLeftEchoPin, maxDistance);
NewPing frontRight(frontRightTrigPin, frontRightEchoPin, maxDistance);
NewPing backLeft(backLeftTrigPin, backLeftEchoPin, maxDistance);
NewPing backRight(backRightTrigPin, backRightEchoPin, maxDistance);

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onRequest(requestEvent);
}

void loop() {
  frontLeftDistanceCm = frontLeft.ping_cm();
  frontRightDistanceCm = frontRight.ping_cm();
  backLeftDistanceCm = backLeft.ping_cm();
  backRightDistanceCm = backRight.ping_cm();
}

// Handler for I2C request from the Roborio
void requestEvent() {  
  //Put upper byte
  Serial.print("Front Left Upper: ");
  unsigned int i = (frontLeftDistanceCm & 0xFF00) >> 8;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put lower byte
  Serial.print("Front Left Lower: ");
  i = frontLeftDistanceCm & 0xFF;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put upper byte
  Serial.print("Front Right Upper: ");
  i = (frontRightDistanceCm & 0xFF00) >> 8;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put lower byte
  Serial.print("Front Right Lower: ");
  i = frontRightDistanceCm & 0xFF;
  Serial.println(i);
  Wire.write((byte)i);

  //Put upper byte
  Serial.print("Back Left Upper: ");
  i = (backLeftDistanceCm & 0xFF00) >> 8;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put lower byte
  Serial.print("Back Left Lower: ");
  i = backLeftDistanceCm & 0xFF;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put upper byte
  Serial.print("Back Right Upper: ");
  i = (backRightDistanceCm & 0xFF00) >> 8;
  Serial.println(i);
  Wire.write((byte)i);
  
  //Put lower byte
  Serial.print("Back Right Lower: ");
  i = backRightDistanceCm & 0xFF;
  Serial.println(i);
  Wire.write((byte)i);
}
