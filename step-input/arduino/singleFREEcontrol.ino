//  Created: Summer 2017 by Clayton Baumgart
// Latest Revision: October 4, 2019 by Soheil Habibian
// Control rotation of a single FREE
// Arduino output routes to pressure transducer attached to FREE

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int out = 9; //output pin - attach to pressure transducer input

//*******************************************
double omega_d = 0.0; //desired angle value (deg) 
//******************************************

//** controller gains *******************************
double kp = 0.6; 
double ki=5.5;
double kd=0;
//***************************************************

double outV = 0; //output voltage
unsigned long lastTime;
unsigned long time;
double angle;
double postn;
double initial_angle;
double error, errSum, errD, lastError;
double timeChange;
int timestp=3000;
//*************************************************
void setup()
{
  Serial.begin(9600);
  // throw error if connection is not found with sensor
  if (!bno.begin())
  {
    Serial.println("Not connected.");
  }
  pinMode(out, OUTPUT);
  delay(800);
  imu:: Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initial_angle=euler.x();
}

double getAngle()
{
  imu:: Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle=euler.x();
  postn= angle-initial_angle;

  if(postn<0)
  {
    postn=-postn;
  }
  
  return postn;
}

double controlAction()
{
  unsigned long now = millis();
  timeChange = (double)(now-lastTime);
  if (now>timestp){
    omega_d=50;
  if (2*timestp<now)
    omega_d=20;
  if (3*timestp<now)
    omega_d=80;
 }
  error=omega_d-postn;
  
  error=map(error,0.0,180.0,0.0,255.0);
  errSum += error*(timeChange/1000);

  //fix error overshoots
  if (errSum>255)
  {
    errSum=255;
  }
  if (errSum<0)
  {
    errSum=0;
  }

  //define derivative error term
  errD = (error-lastError)/(timeChange/1000);

  if (error != 0)
  {
    outV = kp*error + ki*errSum + kd*errD;
    if (outV>255)
    {
      outV=255;
    }
  }

  lastError = error;
  lastTime = now;
  
  if (outV<0)
  {
    outV=0;
  }
  
  return outV;
}

void loop()
{
  postn=getAngle();
  outV=controlAction();
  analogWrite(out, outV);
  time = millis();


Serial.print(postn);
Serial.print("  ,  ");
//Serial.print(angle);
//Serial.print("  ,  ");
Serial.print(omega_d);
Serial.print("  ,  ");
Serial.println(time);


}
