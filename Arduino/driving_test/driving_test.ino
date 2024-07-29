//*******************************************************/


// To do

// Add forward velocity

// Regularize motor commands
// Validity check on data 




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
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();


// Serial port for Debug only
uint8_t ssRX = 2;
uint8_t ssTX = 3;
SoftwareSerial nss(ssRX, ssTX);



uint16_t v = 0;
uint16_t w = 0;
uint16_t z = 0;
uint8_t* temp[7];
char vt[4]={0};
char wt[4]={0}; 
char t[8]={0};
uint8_t vv=0;
uint8_t ww=0;
int l=0;
float theta = 0;
float Kwr = 1000;
float Kwl = 1000;

float Kvr = 1;
//float Kvl = 0.9;
float Kvl = 0.9;




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




void setup()
{
  // start serial
  Serial.begin(115200);  
  xbee.setSerial(Serial);
  
  nss.begin(9600);


  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
}


void loop()
{
   // Read Xbee
   // nss.println("looping1");


   xbee.readPacket();

    if (xbee.getResponse().isAvailable()) {

        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
        
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
      
        nss.println("Got an rx packet!");
            
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            // the sender got an ACK
            //nss.println("packet acknowledged");
        } else {
         // nss.println("packet not acknowledged");
        }
        //Read inits
        vv=0;
        ww=1;
        wt[0] = 0;
        vt[0] = 0;
        v = 0;
        w = 0;
        l = 0;
         
        for (int i = 0; i < rx.getDataLength(); i++) {
          
          t[i] = rx.getData()[i];
          if (t[i] != ',' && vv==0 && ww !=0){
            vt[i] = t[i];
            }
          else if (t[i] == ',' && vv==0){
            vv = 1;
            ww = 0;
            l=i;
            vt[i] = 0;
            }
          else if (t[i] != ',' && ww==0){
            wt[i-l-1] = t[i];
            }
          else {
          wt[i-l-1] = 0;  
          }
     
        }
        v = atoi(vt)-100;
        w = atoi(wt);


        // Convert data
        theta = w;
        //theta = (theta-100)/1800*3.14;
        theta = (theta-500)/100;


        
        nss.println("Velocity error : ");
        nss.print(v);
        // nss.print("vt : ");
        // nss.print(vt);

        nss.print("   Omega error : ");
        nss.print(theta);
        // nss.println(w);  

   
   L = Kvl*v - Kwl*theta;
   R = Kvr*v + Kwr*theta;

   if (L > 245){
     L = 245;
   }
   if (R > 245){
     R = 245;
   }
   if (L < -245){
     L = -245;
   }
   if (R < -245){
     R = -245;
   }

   if(L > R){
    digitalWrite(8,HIGH);
    digitalWrite(9,LOW);
    }else if(R>L){
      digitalWrite(9,HIGH);
    digitalWrite(8,LOW);
      }else{
        digitalWrite(9,LOW);
    digitalWrite(8,LOW);
        }
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
  //motor1.drive(L);
  //motor2.drive(R);

   nss.print("     L : ");
   nss.print(L);
   nss.print("     R : ");
   nss.println(R);
    }
    } else if (xbee.getResponse().isError()) {
     // nss.print("error code:");
     // nss.println(xbee.getResponse().getErrorCode());
    }
   
  motor2.drive(L);
  motor1.drive(R);   
   
//   
//   v=0;
//   L = Kv*v + Kp*theta;
//   R = Kv*v - Kp*theta;
// //   //Use of the drive function which takes as arguements the speed
// //   //and optional duration.  A negative speed will cause it to go
// //   //backwards.  Speed can be from -255 to 255.  Also use of the 
// //   //brake function which takes no arguements.
//   motor1.drive(L);
//   motor1.drive(R);



//
//   nss.print("     L : ");
//   nss.print(L);
//   nss.print("     R : ");
//   nss.println(R); 
////   motor1.brake();
////   delay(1000);
//   
//   //Use of the drive function which takes as arguements the speed
//   //and optional duration.  A negative speed will cause it to go
//   //backwards.  Speed can be from -255 to 255.  Also use of the 
//   //brake function which takes no arguements.
//   motor2.drive(255,1000);
//   motor2.drive(-255,1000);
//   motor2.brake();
//   delay(1000);
//   
//   //Use of the forward function, which takes as arguements two motors
//   //and optionally a speed.  If a negative number is used for speed
//   //it will go backwards
//   forward(motor1, motor2, 255);
//   delay(1000);
//   
//   //Use of the back function, which takes as arguments two motors 
//   //and optionally a speed.  Either a positive number or a negative
//   //number for speed will cause it to go backwards
//   back(motor1, motor2, -255);
//   delay(1000);
//   
//   //Use of the brake function which takes as arguments two motors.
//   //Note that functions do not stop motors on their own.
//   brake(motor1, motor2);
//   delay(1000);
//   
//   //Use of the left and right functions which take as arguements two
//   //motors and a speed.  This function turns both motors to move in 
//   //the appropriate direction.  For turning a single motor use drive.
//   left(motor1, motor2, 100);
//   delay(1000);
//   right(motor1, motor2, 100);
//   delay(1000);
//   
//   //Use of brake again.
//   brake(motor1, motor2);
//   delay(1000);
   
}
