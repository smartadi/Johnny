//*******************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include "SparkFun_TB6612.h"
#include <XBee.h>
#include <SoftwareSerial.h>
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


XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
// Define NewSoftSerial TX/RX pins
// Connect Arduino pin 8 to TX of usb-serial device
uint8_t ssRX = 2;
// Connect Arduino pin 9 to RX of usb-serial device
uint8_t ssTX = 3;

SoftwareSerial nss(ssRX, ssTX);



// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = -1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
   // start serial
  Serial.begin(9600);
  xbee.setSerial(Serial);
  nss.begin(9600);
  
  nss.println("Starting up!");
  delay(10000);
}


void loop()
{
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor1.drive(240);
   motor2.drive(240);
   delay(5000);
   motor1.brake();
   motor2.brake();
     delay(1000);
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor1.drive(-240);
   motor2.drive(-240);
    delay(5000);
  motor1.brake();
   motor2.brake();
      delay(1000);
   
   
}
