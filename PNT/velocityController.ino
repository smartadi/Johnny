//*******************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include "SparkFun_TB6612.h"
#include <SoftwareSerial.h>
#include <DFRobot_QMC5883.h>
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 11
#define BIN1 6
#define AIN2 7
#define BIN2 13
#define PWMA 10
#define PWMB 5
#define STBY 12


// Serial port for Debug only
uint8_t ssRX = 2;
uint8_t ssTX = 3;
SoftwareSerial mySerial(ssRX, ssTX);
DFRobot_QMC5883 compass(&Wire, 0x0D);

float destinationX = 0;
float destinationY = 0;
//const float b = 0.074; //Distance between robot's wheels (meters?)

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = -1;
const int offsetB = 1;
int L = 0;
int R = 0;
// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

String receivedString = "";
float receivedValue;

//float b[2]; //Current vector
float b_d[2]; //Desired vector
sVector_t mag;

float pos[2];
float lastPos[2];

float heading;
float headingVect[2];
float headingVectMag;
float measuredHeadingVect[2];
float deltaPosMag;
float desiredHeading;
float deltaPos[2];

float b = 0.074; //Distance between robot's wheels (meters)
float c = 0.95;
float k_angle = 2;
float k_forwardVelocity = 0.1;
float maxVelocity = 0.13; // 0.13 m/s
float tol = 0.01;

void setup()
{
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  Serial.println("QMC5883 Initializing");
  float declinationAngle = (15.0 + (10.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  // start serial
  Serial.begin(115200); 
  mySerial.begin(115200);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);

  Serial.print("starting....");

  // while(!getPosition()){
  // }
  //getHeading();

  headingVect[0] = 1;
  headingVect[1] = 0;
  headingVect[0] = measuredHeadingVect[0];
  headingVect[1] = measuredHeadingVect[1];
}

void loop()
{
  mag = compass.readRaw();
  compass.getHeadingDegrees();
  heading = mag.HeadingDegress;
  Serial.print("Heading (deg): ");
  Serial.println(heading);
  heading *= 3.141592654/180;
  headingVect[0] = cos(heading);
  headingVect[1] = sin(heading);
  Serial.print("Heading (rad): ");
  Serial.println(heading);
  
  lastPos[0] = pos[0];
  lastPos[1] = pos[1];
  while(!getPosition()){
  }
  if ((pos[0] == 0) || (pos[1] == 0)) { //throw out zero zero
    pos[0] = lastPos[0];
    pos[1] = lastPos[1];
    Serial.println("Threw out pos");
  }

  b_d[0] = destinationX - pos[0];
  b_d[1] = destinationY - pos[1];

  float dot = (b_d[0] * headingVect[0]) + (b_d[1] * headingVect[1]);
  float b_dMag = sqrt(sq(b_d[0]) + sq(b_d[1]));
  float headingVectMag = sqrt(sq(headingVect[0]) + sq(headingVect[1]));
  float radDiff = acos(dot / (b_dMag * headingVectMag));
  radDiff = normalize(radDiff);
  Serial.print("Heading error: ");
  Serial.println(radDiff);

  float distanceToDestination = sqrt(sq(b_d[0]) + sq(b_d[1]));
  float angVel = k_angle * radDiff * distanceToDestination;
  float forwardVelocity = k_forwardVelocity * distanceToDestination;

  forwardVelocity = 0;
  
  if (forwardVelocity > maxVelocity) {
    forwardVelocity = maxVelocity;
  }

  float vR = ((2 * forwardVelocity) + (angVel * b)) / 2;
  float vL = ((2 * forwardVelocity) - (angVel * b)) / 2;

  if (vR > maxVelocity) {
    vR = maxVelocity;
  }
  if (vL > maxVelocity) {
    vL = maxVelocity;
  }

  int vR_pwm = vR * (255 / 0.13);
  int vL_pwm = vL * (255 / 0.13);

  motor1.drive(vL_pwm);
  motor2.drive(vR_pwm);

  Serial.print("Pos: ");
  Serial.print(pos[0]);
  Serial.print(", ");
  Serial.println(pos[1]);

  delay(100);
}

float normalize(float heading) {
  if (heading > 180) {
    heading -= 360;
  } else if (heading < -180) {
    heading += 360;
  }  return heading;
}

bool getPosition() {
  float x, y;
  if (mySerial.available() > 0) {
  receivedString = mySerial.readStringUntil('\n'); // Read the incoming string until newline
  int commaIndex = receivedString.indexOf(','); // Find the comma separator
  if (commaIndex > 0 && receivedString.length() == 14) {
    String xString = receivedString.substring(0, commaIndex); // Extract the X value string
    String yString = receivedString.substring(commaIndex + 1); // Extract the Y value string

    x = xString.toFloat(); // Convert the X value string to float
    y = yString.toFloat(); // Convert the Y value string to float  
    pos[0] = x;
    pos[1] = y;
    return true;
    }
  }
  return false;
}
