/*
 * Riley Karp
 * CS483 Honors Research - Robotic Dog
 * Leg Control - Step to target location
 * 10 March 2019
 */

 #include <avr/io.h>
 #include "Adafruit_PWMServoDriver.h" // PWM Motor Driver
 #include "USART.h" //library w/ serial bus functions
 #include <stdio.h> // library w/ sprintf() string formatting

 /* Define constants */
 #define servoMin  170 // min pulse length out of 4096 (0 degrees)
 #define servoMed  370 // med pulse length out of 4096 (90 degrees)
 #define servoMax  510 // max pulse length out of 4096 (130 degrees)
 #define hipHL 0 // motor number for FRONT LEFT HIP
 #define kneeHL 1 // motor number for FRONT LEFT KNEE
 #define ankleHL 2 // motor number for FRONT LEFT ANKLE
 #define hipHR 4 // motor number for HIND LEFT HIP
 #define kneeHR 5 // motor number for HIND LEFT KNEE
 #define ankleHR 6 // motor number for HIND LEFT ANKLE
 #define hipFL 8 // motor number for FRONT RIGHT HIP
 #define kneeFL 9 // motor number for FRONT RIGHT KNEE
 #define ankleFL 10 // motor number for FRONT RIGHT ANKLE
 #define hipFR 12 // motor number for HIND RIGHT HIP
 #define kneeFR 13 // motor number for HIND RIGHT KNEE
 #define ankleFR 14 // motor number for HIND RIGHT ANKLE
 #define pi 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998

 /* Initialize global variables */
 const int numAngles = 8; // initialize number of waypoint angles
 Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver(); // motor driver object

// float hipAngles[numAngles] = {-107.53,-115.32, -120.90,-120.89,-122.7,-133.56,-132.03,-126.2,-117.23,-105,-79.74,-88.8,-104.19};
// float kneeAngles[numAngles] = {16.76,15.07,11.30,5.73,0.94,7.32,12.75,16.97,20,21.15,13.79,13.34,14.76};
// float ankleAngles[numAngles] = {31.44,35.70,34.52,23.72,15.29,44.96,53.55,55.09,51,41.14,12,16.81,31};
 
 float hipAngles[numAngles] = {-7.53,25,0,-33,-25,-10,10,15};
 float kneeAngles[numAngles] = {-36.76,-43,-45,-48,-30,-40,-35,-30};
 float ankleAngles[numAngles] = {31.44,48,60,66,45,40,30,38};
 int pulse;

 bool FL = false;
 bool FR = true;
 bool HL = false;
 bool HR = false;
 int FLi = 6;
 int FRi = 1;
 int HLi = 5;
 int HRi = 7;

 // FRONT LEFT FOOT
 float thetaH_FL = (hipAngles[0])*pi/180;
 float thetaK_FL = (kneeAngles[0])*pi/180;
 float thetaA_FL = (ankleAngles[0])*pi/180;
 float targetH_FL = (hipAngles[FLi])*pi/180;
 float targetK_FL = (kneeAngles[FLi])*pi/180;
 float targetA_FL = (ankleAngles[FLi]-5)*pi/180;

 // FRONT RIGHT FOOT
 float thetaH_FR = (hipAngles[0])*pi/180;
 float thetaK_FR = (kneeAngles[0])*pi/180;
 float thetaA_FR = (ankleAngles[0])*pi/180;
 float targetH_FR = (hipAngles[FRi])*pi/180;
 float targetK_FR = (kneeAngles[FRi])*pi/180;
 float targetA_FR = (ankleAngles[FRi])*pi/180;

 // HIND LEFT FOOT
 float thetaH_HL = (hipAngles[0])*pi/180;
 float thetaK_HL = (kneeAngles[0])*pi/180;
 float thetaA_HL = (ankleAngles[0])*pi/180;
 float targetH_HL = (hipAngles[HLi])*pi/180;
 float targetK_HL = (kneeAngles[HLi])*pi/180;
 float targetA_HL = (ankleAngles[HLi]+8)*pi/180;

 // HIND RIGHT FOOT
 float thetaH_HR = (hipAngles[0])*pi/180;
 float thetaK_HR = (kneeAngles[0])*pi/180;
 float thetaA_HR = (ankleAngles[0])*pi/180;
 float targetH_HR = (hipAngles[HRi])*pi/180;
 float targetK_HR = (kneeAngles[HRi])*pi/180;
 float targetA_HR = (ankleAngles[HRi])*pi/180;

 
 /* Returns the pulse length corresponding to the given angle between 0 and 130 degrees (input angle is in radians) */
 int radToPulse( int joint, float rad ) {
  // y = mx + b for DH angle to pulse
  pulse = int(float(servoMed-servoMin)*rad/(pi/2) + servoMed);
  // make sure pulse is within range
  if( pulse > servoMax ) {
    pulse = servoMax;
  }
  else if(pulse < servoMin) {
    pulse = servoMin;
  }
  return pulse;
 }

 /* Moves the given joint to the given angle */
 void moveJoint( int joint, float angle ) {
  driver.setPWM( joint, 0, radToPulse(joint,angle) );
 }
 
 int main() {
  initUSART();
  init(); // haven't yet figured out why this is necessary, but it is
  driver.begin(); // start motor driver object
  driver.setPWMFreq(60); // analog servos run at ~60 hz
  
  DDRC = 0b00110000; // configure OC1B pin PC[5:4] as output
  SREG |= 0b10000000; // enable global interrupts

  // move legs to starting position
  moveJoint( hipFR, thetaH_FR );
  moveJoint( kneeFR, thetaK_FR );
  moveJoint( ankleFR, thetaA_FR );

  moveJoint( hipHL, -thetaH_HL );
  moveJoint( kneeHL, -thetaK_HL);
  moveJoint( ankleHL, -thetaA_HL );
  
  moveJoint( hipFL, -thetaH_FL );
  moveJoint( kneeFL, -thetaK_FL );
  moveJoint( ankleFL, -thetaA_FL );
  
  moveJoint( hipHR, thetaH_HR );
  moveJoint( kneeHR, thetaK_HR );
  moveJoint( ankleHR, thetaA_HR );
      
  printString("start\n");
  _delay_ms(1000);
    
  while( true) {
    // FRONT RIGHT LEG
    thetaH_FR += (targetH_FR-thetaH_FR)/20.0;
    thetaK_FR += (targetK_FR-thetaK_FR)/20.0;
    thetaA_FR += (targetA_FR-thetaA_FR)/20.0;
    moveJoint( hipFR, thetaH_FR );
    moveJoint( kneeFR, thetaK_FR );
    moveJoint( ankleFR, thetaA_FR );

    // HIND LEFT LEG
    thetaH_HL += (targetH_HL-thetaH_HL)/20.0;
    thetaK_HL += (targetK_HL-thetaK_HL)/20.0;
    thetaA_HL += (targetA_HL-thetaA_HL)/20.0;
    moveJoint( hipHL, -thetaH_HL );
    moveJoint( kneeHL, -thetaK_HL);
    moveJoint( ankleHL, -thetaA_HL );

    // FRONT LEFT LEG
    thetaH_FL += (targetH_FL-thetaH_FL)/20.0;
    thetaK_FL += (targetK_FL-thetaK_FL)/20.0;
    thetaA_FL += (targetA_FL-thetaA_FL)/20.0;
    moveJoint( hipFL, -thetaH_FL );
    moveJoint( kneeFL, -thetaK_FL );
    moveJoint( ankleFL, -thetaA_FL );

    // HIND RIGHT LEG
    thetaH_HR += (targetH_HR-thetaH_HR)/20.0;
    thetaK_HR += (targetK_HR-thetaK_HR)/20.0;
    thetaA_HR += (targetA_HR-thetaA_HR)/20.0;
    moveJoint( hipHR, thetaH_HR );
    moveJoint( kneeHR, thetaK_HR );
    moveJoint( ankleHR, thetaA_HR );

    if( (abs(thetaH_FR-targetH_FR) < 0.08) and
        (abs(thetaH_HL-targetH_HL) < 0.08) and
        (abs(thetaH_FL-targetH_FL) < 0.08) and
        (abs(thetaH_HR-targetH_HR) < 0.08) ) {

      // FRONT RIGHT UP
      if( FR ) {
        FRi += 1;
        if(FRi >= 4) {
          FR = false;
          HL = true;
          HLi = 0;
          FLi += 1;
          HRi -= 1;
        }
      }
      // HIND LEFT UP
      else if( HL ) {
        HLi += 1;
        if(HLi >= 4) {
          HL = false;
          HLi = 7;
          FL = true;
          FLi = 0;
          FRi += 1;
          HRi -= 1;
        }
      }
      // FRONT LEFT UP
      else if( FL ) {
        FLi += 1;
        if(FLi >= 4) {
          FL = false;
          HR = true;
          HRi = 0;
          FRi += 1;
          HLi -= 1;
        }
      }
      // HIND RIGHT UP
      else if(HR) {
        HRi += 1;
        if(HRi >= 4) {
          HR = false;
          HRi = 7;
          FR = true;
          FRi = 0;
          FLi += 1;
          HLi -= 1;
        }
      }

      // FRONT LEFT FOOT
      targetH_FL = (hipAngles[FLi])*pi/180;
      targetK_FL = (kneeAngles[FLi])*pi/180;
      targetA_FL = (ankleAngles[FLi]-5)*pi/180;
    
      // FRONT RIGHT FOOT
      targetH_FR = (hipAngles[FRi])*pi/180;
      targetK_FR = (kneeAngles[FRi])*pi/180;
      targetA_FR = (ankleAngles[FRi])*pi/180;
    
      // HIND LEFT FOOT
      targetH_HL = (hipAngles[HLi])*pi/180;
      targetK_HL = (kneeAngles[HLi])*pi/180;
      targetA_HL = (ankleAngles[HLi]+8)*pi/180;
    
      // HIND RIGHT FOOT
      targetH_HR = (hipAngles[HRi])*pi/180;
      targetK_HR = (kneeAngles[HRi])*pi/180;
      targetA_HR = (ankleAngles[HRi])*pi/180;
    }
    
  }
  return 0;
 }
