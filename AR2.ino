/*
 *  Copyright (c) 2020, hugh maclaurin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

** c++ interpretation of Chris Annin's AR2 forward kinematic
** and inverse kinematic Python code ( AR2.py ). Also his DH 
** param table, limit and step parameters ( a bit modified to suit ).
** The code is designed to run on an Arduino Mega ( maybe ) and Teensy 3.5 and use a
** command/response application to pass position parameters, 
** x, y, z, yaw, pitch, roll or angle parameters J1-J6 and other commands.
** I've written this application using object Pascal ( Delphi )
** but you could do it yourself with any development environment
** that can talk to a COM port. The control program was developed on VMware using Windows 7
** and Delphi2010. It is a 32-bit application. I run it on my Windows 8.1 laptop. Please feel free
** to try it on any other version of Windows. Just remember it uses the MS Scripting Engine as an 
** inprocess server which means it talks to a registered MS dll.
** 
** TODO: create a AR2_Arm class with the forward and inverse kinematics and gotoPoint methods.
** 
** AR2 for Arduino by Hugo ( nactech@iinet.net.au )
** Motion profiles using AccelStepper libraray
** Matrix operations using MatrixMath library
** 15.March.2020
**
** MatrixMath attribution
** A GitHub repo for the MatrixMath Arduino library - http://playground.arduino.cc/Code/MatrixMath
** There is a nice Arduine library on the Arduino Playground for Matrix Mathematical operations, but it wasn't in the form of a library,
** and it required extra work to be installed and used, so we brought this into a good state and added it up on GitHub for anyone who 
** is interested in using it 
** Enjoy ;)
** ### License
** The original author didn't specify a License, but I'm going out on a limb and guessing that he implied GPL2. I would also be happy 
** if someone offered me a beer for my work, so you could also consider it BeerWare.
** But as I said, I don't know the original author's intentions, so think of it what you will, just don't sue me or anything
**
** Motor Homing code using AccelStepper and the Serial Monitor
** Created by Yvan / https://Brainy-Bits.com
** This code is in the public domain...
** You can: copy it, use it, modify it, share it or just plain ignore it!
** Thx!
**
** Split function
** https://stackoverflow.com/questions/9072320/split-string-into-string-array
**
**
############################################################################
## Version AR2.2.0 #########################################################
############################################################################
""" AR2 - Stepper motor robot control software
    Copyright (c) 2017, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
    * Selling robots, robot parts, or any versions of robots or software based on this 
      work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com
*/

//#include <IRremote.h>
#include <AccelStepper.h>
#include <Bounce2.h>
#include <Servo.h> 
 
#include <MatrixMath.h>
#include <math.h>

const float VERSION = 1.10;
bool verbose = false; // Set to true to print the intermediate matrices. Check against kinemetric model.

// Steppers
#define _MIN_PULSE_WIDTH 150
#define MAXSPEED 1000
#define ACCELERATION 500

// Change these to achieve to correct 'home' pose
#define base_StepLimit 15200
#define shoulder_StepLimit 7300
// changed from 7850
#define elbow_StepLimit 5100
// changed from 15200
#define wrist_yaw_StepLimit 14700
#define wrist_pitch_StepLimit 4575
// changed from 6625
#define wrist_roll_StepLimit 6800

#define J1stepPin   2
#define J1dirPin    3
#define J2stepPin   4
#define J2dirPin    5
#define J3stepPin   6
#define J3dirPin    7
#define J4stepPin   8
#define J4dirPin    9
#define J5stepPin   10
#define J5dirPin    11
#define J6stepPin   12
#define J6dirPin    13

#define J1calPin    14
#define J2calPin    15
#define J3calPin    16
#define J4calPin    17
#define J5calPin    18
#define J6calPin    19
// stepper drivers are enabled by default
#define EN          20
#define gripperPin  23

Servo Gripper; 

AccelStepper j1_stepper( AccelStepper::DRIVER, J1stepPin, J1dirPin );
AccelStepper j2_stepper( AccelStepper::DRIVER, J2stepPin, J2dirPin );
AccelStepper j3_stepper( AccelStepper::DRIVER, J3stepPin, J3dirPin );
AccelStepper j4_stepper( AccelStepper::DRIVER, J4stepPin, J4dirPin );
AccelStepper j5_stepper( AccelStepper::DRIVER, J5stepPin, J5dirPin );
AccelStepper j6_stepper( AccelStepper::DRIVER, J6stepPin, J6dirPin );

enum joint_type { BASE = 1, SHOULDER, ELBOW, WRIST_YAW, WRIST_PITCH, WRIST_ROLL };

struct StepperInfo {
  joint_type j_type;
  long CurrStep;
  float CurrAngle;
  float NegAngleLimit;
  float PosAngleLimit;
  long StepLimit;
  float StepsPerDeg;
  float DegPerStep;
};

StepperInfo _base;
StepperInfo _shoulder;
StepperInfo _elbow;
StepperInfo _wrist_yaw;
StepperInfo _wrist_pitch;
StepperInfo _wrist_roll;

bool wrist_config_f = true;

// Work frame input
float UFx  = 0;
float UFy  = 0;
float UFz  = 0;
float UFrx = 0;
float UFry = 0;
float UFrz = 0;
// Tool frame input
float TFx  = 0;
float TFy  = 0;
float TFz  = 0;
float TFrx = 0;
float TFry = 0;
float TFrz = 0;

// Denavit Hartenberg Parameters
// DH alpha (link twist)
const float DHr1 = -90.0;
const float DHr2 = 0.0;
const float DHr3 = 90.0;
const float DHr4 = -90.0;
const float DHr5 = 90.0;
const float DHr6 =0.0;
// DH a (link length)
const float DHa1 = 64.2;
const float DHa2 = 305.0;
const float DHa3 = 0.0;
const float DHa4 = 0.0;
const float DHa5 = 0.0;
const float DHa6 = 0.0;
//DH d (link offset)
const float DHd1 = 169.77;
const float DHd2 = 0.0;
const float DHd3 = 0.0;
const float DHd4 = -222.63;
const float DHd5 = 0.0;
const float DHd6 = -36.25;
// DH theta (joint angle)
const float DHt1 = 0.0;
const float DHt2 = 0.0;
const float DHt3 = -90.0;
const float DHt4 = 0.0;
const float DHt5 = 0.0;
const float DHt6 = 180.0;
// Denavit Hartenberg table
// DH table row x col 
float DHt[6][4];

float ToRadians( float deg ) { return ( deg*PI/180 ); }
float ToDegrees( float rad ) { return ( rad*180/PI ); }

// J1-6 in -> x, y, z, yaw, pitch and roll out
void CalcFwdKin(  float J1AngCur, float J2AngCur, float J3AngCur, // input angles
                  float J4AngCur, float J5AngCur, float J6AngCur,
                  float& x, float& y, float& z, 
                  float& yaw, float& pitch, float& roll )
{
  // work frame
  float Wf[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  }; 
  // tool frame
  float Tf[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };
  // Temporary frames
  float tmp1f[4][4];
  float tmp2f[4][4];
  
  // joint frames
  float J1f[4][4];
  float J2f[4][4];
  float J3f[4][4];
  float J4f[4][4];
  float J5f[4][4];
  float J6f[4][4];

  // Angles input
  // Save these for use in joint frames
  float C13 = ToRadians( float( J1AngCur ) + DHt1 );
  float C14 = ToRadians( float( J2AngCur ) + DHt2 );
  float C15 = ToRadians( float( J3AngCur ) + DHt3 );
  float C16 = ToRadians( float( J4AngCur ) + DHt4 );
  float C17 = ToRadians( float( J5AngCur ) + DHt5 );
  float C18 = ToRadians( float( J6AngCur ) + DHt6 );
  float D13 = ToRadians( DHr1 );
  float D14 = ToRadians( DHr2 );
  float D15 = ToRadians( DHr3 );
  float D16 = ToRadians( DHr4 );
  float D17 = ToRadians( DHr5 );
  float D18 = ToRadians( DHr6 );
  float E13 = DHd1;
  float E14 = DHd2;
  // unused float E15 = DHd3;
  float E16 = DHd4;
  float E17 = DHd5;
  float E18 = DHd6;
  float F13 = DHa1;
  float F14 = DHa2;
  float F15 = DHa3;
  float F16 = DHa4;
  float F17 = DHa5;
  float F18 = DHa6;

  // DHtable - not used as such, but looks good when printed
  // column C - 0 - theta
  DHt[0][0] = ToRadians( float( J1AngCur ) + DHt1 );
  DHt[1][0] = ToRadians( float( J2AngCur ) + DHt2 );
  DHt[2][0] = ToRadians( float( J3AngCur ) + DHt3 );
  DHt[3][0] = ToRadians( float( J4AngCur ) + DHt4 );
  DHt[4][0] = ToRadians( float( J5AngCur ) + DHt5 );
  DHt[5][0] = ToRadians( float( J6AngCur ) + DHt6 );
  // column D - 1 - alpha
  DHt[0][1] = ToRadians( DHr1 );
  DHt[1][1] = ToRadians( DHr2 );
  DHt[2][1] = ToRadians( DHr3 );
  DHt[3][1] = ToRadians( DHr4 );
  DHt[4][1] = ToRadians( DHr5 );
  DHt[5][1] = ToRadians( DHr6 );
  // column E - 2 - d
  DHt[0][2] = DHd1;
  DHt[1][2] = DHd2;
  DHt[2][2] = DHd3;
  DHt[3][2] = DHd4;
  DHt[4][2] = DHd5;
  DHt[5][2] = DHd6;
  // column F - 3 - a
  DHt[0][3] = DHa1;
  DHt[1][3] = DHa2;
  DHt[2][3] = DHa3;
  DHt[3][3] = DHa4;
  DHt[4][3] = DHa5;
  DHt[5][3] = DHa6;
  if ( verbose ) Matrix.Print( (float*)DHt, 6, 4, "DH table");
  
  // Work frame input
  // Work frame table
  Wf[0][0] = cos(ToRadians(UFrz))*cos(ToRadians(UFry));
  Wf[1][0] = sin(ToRadians(UFrz))*cos(ToRadians(UFry));
  Wf[2][0] = -sin(ToRadians(UFrz));
  Wf[3][0] = 0.0;
  
  Wf[0][1] = -sin(ToRadians(UFrz))*cos(ToRadians(UFrx))+cos(ToRadians(UFrz))*sin(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[1][1] = cos(ToRadians(UFrz))*cos(ToRadians(UFrx))+sin(ToRadians(UFrz))*sin(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[2][1] = cos(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[3][1] = 0.0;
  
  Wf[0][2] = sin(ToRadians(UFrz))*sin(ToRadians(UFrx))+cos(ToRadians(UFrz))*sin(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[1][2] = -cos(ToRadians(UFrz))*sin(ToRadians(UFrx))+sin(ToRadians(UFrz))*sin(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[2][2] = cos(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[3][2] = 0.0;
  
  Wf[0][3] = UFx;
  Wf[1][3] = UFy;
  Wf[2][3] = UFz;
  Wf[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)Wf, 4, 4, "Work frame");
  
  // J1 frame
  J1f[0][0] = cos( C13 );
  J1f[1][0] = sin( C13 );
  J1f[2][0] = 0.0;
  J1f[3][0] = 0.0;
  
  J1f[0][1] = -sin( C13 ) * cos( D13 );
  J1f[1][1] =  cos( C13 ) * cos( D13 );
  J1f[2][1] =  sin( D13 );
  J1f[3][1] = 0.0;

  J1f[0][2] =  sin( C13 ) * sin( D13 );
  J1f[1][2] = -cos( C13 ) * sin( D13 );
  J1f[2][2] =  cos( D13 );
  J1f[3][2] = 0.0;

  J1f[0][3] = F13 * cos( C13 );
  J1f[1][3] = F13 * sin( C13 );
  J1f[2][3] = E13;
  J1f[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)J1f, 4, 4, "J1f frame");
  // J2 frame
  J2f[0][0] = cos( C14 );
  J2f[1][0] = sin( C14 );
  J2f[2][0] = 0.0;
  J2f[3][0] = 0.0;

  J2f[0][1] = -sin( C14 ) * cos( D14 );
  J2f[1][1] =  cos( C14 ) * cos( D14 );
  J2f[2][1] =  sin( D14 );
  J2f[3][1] = 0.0;

  J2f[0][2] =  sin( C14 ) * sin( D14 );
  J2f[1][2] = -cos( C14 ) * sin( D14 );
  J2f[2][2] = cos( D14 );
  J2f[3][2] = 0.0;

  J2f[0][3] = F14 * cos( C14 );
  J2f[1][3] = F14 * sin( C14 );
  J2f[2][3] = E14;
  J2f[3][3] = 1.0;

  // J3 frame
  J3f[0][0] = cos( C15 );
  J3f[1][0] = sin( C15 );
  J3f[2][0] = 0.0;
  J3f[3][0] = 0.0;
  
  J3f[0][1] = -sin( C15 ) * cos( D15 );
  J3f[1][1] =  cos( C15 ) * cos (D15 );
  J3f[2][1] =  sin( D15 );
  J3f[3][1] = 0.0;
  
  J3f[0][2] =  sin( C15 ) * sin( D15 );
  J3f[1][2] = -cos( C15 ) * sin( D15 );
  J3f[2][2] =  cos( D15 );
  J3f[3][2] = 0.0;
  
  J3f[0][3] = F15 * cos( C15 );
  J3f[1][3] = F15 * sin( C15 );
  J3f[2][3] = 0.0;
  J3f[3][3] = 1.0;

  // J4 frame
  J4f[0][0] = cos( C16 );
  J4f[1][0] = sin( C16 );
  J4f[2][0] = 0.0; 
  J4f[3][0] = 0.0;
  
  J4f[0][1] = -sin( C16 ) * cos( D16 );
  J4f[1][1] =  cos( C16 ) * cos( D16 );
  J4f[2][1] = sin( D16 );
  J4f[3][1] = 0.0;
  
  J4f[0][2] =  sin( C16 ) * sin (D16 );
  J4f[1][2] = -cos( C16 ) * sin( D16 );
  J4f[2][2] = cos( D16 );
  J4f[3][2] = 0.0;
  
  J4f[0][3] = F16 * cos( C16 );
  J4f[1][3] = F16 * sin( C16 );
  J4f[2][3] = E16;
  J4f[3][3] = 1.0;

  // J5 frame
  J5f[0][0] = cos( C17 );
  J5f[1][0] = sin( C17 );
  J5f[2][0] = 0.0;
  J5f[3][0] = 0.0;

  J5f[0][1] = -sin( C17 ) * cos( D17 );
  J5f[1][1] =  cos( C17 ) * cos( D17 );
  J5f[2][1] = sin( D17 );
  J5f[3][1] = 0.0;

  J5f[0][2] =  sin( C17 ) * sin( D17 );
  J5f[1][2] = -cos( C17 ) * sin( D17 ); 
  J5f[2][2] = cos( D17 );
  J5f[3][2] = 0.0;

  J5f[0][3] = F17 * cos( C17 );
  J5f[1][3] = F17 * sin( C17 );
  J5f[2][3] = E17;
  J5f[3][3] = 1.0;

  //J6 frame
  J6f[0][0] = cos( C18 );
  J6f[1][0] = sin( C18 );
  J6f[2][0] = 0.0;
  J6f[3][0] = 0.0;

  J6f[0][1] = -sin( C18 ) * cos( D18 );
  J6f[1][1] =  cos( C18 ) * cos( D18 );
  J6f[2][1] = sin( D18 );
  J6f[3][1] = 0.0;

  J6f[0][2] =  sin( C18 ) * sin( D18 );
  J6f[1][2] = -cos( C18 ) * sin( D18 );
  J6f[2][2] = cos( D18 );
  J6f[3][2] = 0.0;

  J6f[0][3] = F18 * cos( C18 );
  J6f[1][3] = F18 * sin( C18 );
  J6f[2][3] = E18;
  J6f[3][3] = 1.0;

  // Tool frame
  Tf[0][0] = cos(radians(TFrz))*cos(radians(TFry));
  Tf[1][0] = sin(radians(TFrz))*cos(radians(TFry));
  Tf[2][0] = -sin(radians(TFrz));
  Tf[3][0] = 0.0;

  Tf[0][1] = -sin(ToRadians(TFrz))*cos(ToRadians(TFrx))+cos(ToRadians(TFrz))*sin(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[1][1] = cos(ToRadians(TFrz))*cos(ToRadians(TFrx))+sin(ToRadians(TFrz))*sin(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[2][1] = cos(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[3][1] = 0.0;

  Tf[0][2] = sin(ToRadians(TFrz))*sin(ToRadians(TFrx))+cos(ToRadians(TFrz))*sin(ToRadians(TFry))*cos(ToRadians(TFrx));
  Tf[1][2] = -cos(ToRadians(TFrz))*sin(ToRadians(TFrx))+sin(ToRadians(TFrz))*sin(ToRadians(TFry))*cos(ToRadians(TFrx));
  Tf[2][2] = cos(ToRadians(TFry))*cos(ToRadians(TFrx));
  Tf[3][2] = 0.0;

  Tf[0][3] = TFx;
  Tf[1][3] = TFy;
  Tf[2][3] = TFz;
  Tf[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)Tf, 4, 4, "Tool frame");
  
  // Wf * J1f
  Matrix.Multiply( (float*)Wf, (float*)J1f, 4, 4, 4, (float*)tmp1f );
  if ( verbose ) Matrix.Print( (float*)tmp1f, 4, 4, "Wf * J1f");
  // (Wf * J1f) * J2f
  Matrix.Multiply( (float*)tmp1f, (float*)J2f, 4, 4, 4, (float*)tmp2f );
  if ( verbose ) Matrix.Print( (float*)tmp2f, 4, 4, "(Wf * J1f) * J2f");
  // (Wf * J1f * J2f ) * J3f
  Matrix.Multiply( (float*)tmp2f, (float*)J3f, 4, 4, 4, (float*)tmp1f );
  if ( verbose ) Matrix.Print( (float*)tmp1f, 4, 4, "(Wf * J1f * J2f ) * J3f");
  // (Wf * J1f * J2f * J3f) * J4f
  Matrix.Multiply( (float*)tmp1f, (float*)J4f, 4, 4, 4, (float*)tmp2f );
  if ( verbose ) Matrix.Print( (float*)tmp2f, 4, 4, "(Wf * J1f * J2f * J3f) * J4f");
  // (Wf * J1f * J2f * J3f * J4f) * J5f
  Matrix.Multiply( (float*)tmp2f, (float*)J5f, 4, 4, 4, (float*)tmp1f );
  if ( verbose ) Matrix.Print( (float*)tmp1f, 4, 4, "(Wf * J1f * J2f * J3f * J4f) * J5f");
  // (Wf * J1f * J2f * J3f * J4f * J5f) * J6f 
  Matrix.Multiply( (float*)tmp1f, (float*)J6f, 4, 4, 4, (float*)tmp2f );
  if ( verbose ) Matrix.Print( (float*)tmp2f, 4, 4, "(Wf * J1f * J2f * J3f * J4f * J5f) * J6f");
  // (Wf * J1 * J2 * J3 * J4 * J5 * J6) * Tf
  Matrix.Multiply( (float*)tmp2f, (float*)Tf, 4, 4, 4, (float*)tmp1f );
  if ( verbose ) Matrix.Print( (float*)tmp1f, 4, 4, "Final frame");
  
/*  Final frame
 *          G     H     I     J
 *   60   0,0   0,1   0,2   0,3
 *   61   1,0   1,1   1,2   1,3
 *   62   2,0   2,1   2,2   2,3
 *   63   3,0   3,1   3,2   3,3
 *   
 */
  float I60 = tmp1f[0][2];
  float I61 = tmp1f[1][2];
  float I62 = tmp1f[2][2];
  float G62 = tmp1f[2][0];
  float H62 = tmp1f[2][1];

  float J60 = tmp1f[0][3]; // x
  float J61 = tmp1f[1][3]; // y
  float J62 = tmp1f[2][3]; // z

  // Output x, y, z, yaw, pitch and roll.....
  
  // yaw (I7), pitch (I8), roll (I9) in radians
  float I8 = atan2( sqrt( ( I60 * I60 ) + ( I61 * I61 ) ), -I62 );  // pitch
  float I7 = atan2( ( G62 / I8 ), ( H62 / I8 ) );                   // yaw
  float I9 = atan2( ( I60 / I8 ), ( I61 / I8 ) );                   // roll
  // x (J60), y (J61), z (J62)
  float H4 = J60; // x
  float H5 = J61; // y
  float H6 = J62; // z
  float H7 = ToDegrees( I7 ); // yaw
  float H8 = ToDegrees( I8 ); // pitch
  float H9 = ToDegrees( I9 ); // roll
  // return values 
  x = H4;
  y = H5;
  z = H6;
  yaw = H7;
  pitch = H8;
  roll = H9;
//  Serial.print( "x=" ); Serial.print( H4 );
//  Serial.print( ", y=" ); Serial.print( H5 );
//  Serial.print( ", z=" ); Serial.print( double( H6 ) );
//  Serial.print( ", yaw=" ); Serial.print( H7 );
//  Serial.print( ", pitch=" ); Serial.print( H8 );
//  Serial.print( ", roll=" ); Serial.println( H9 );
}

// x, y, z, yaw, pitch and roll in -> J1-6 out
void CalcInvKin(  float CX, float CY, float CZ,     // x, y, z input
                  float CRx, float CRy, float CRz,  // yaw, pitch and roll
                  // Joint output angles
                  float& J1out, float& J2out, float& J3out, float& J4out, float& J5out, float& J6out )



{
  // work frame
  float Wf[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  }; 
  // tool frame
  float Tf[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };
  // Temporary frames
  float tmp1f[4][4];
  float tmp2f[4][4];

  // Intermediate results tables ( rotations )
  float R0t[4][4];
  float R06t[4][4];
  float R05t[4][4];
  float R36t[3][3];

  float R01t[4][4];
  float R02t[4][4];
  float R03t[4][4];
  float R03trans[3][3];

  // joint frames
  float J1f[4][4];
  float J2f[4][4];
  float J3f[4][4];


  int V9 = 0;
  // determine quadrant
  if ( ( CX > 0 ) && ( CY > 0 ) ) {
    V9 = 1;
  } else {
    if ( ( CX > 0 ) && ( CY < 0 ) ) {
      V9 = 2;
    } else {
      if ( (CX < 0 ) && ( CY < 0 ) ) {  
        V9 = 3;
      } else {
        if ( ( CX < 0 ) && ( CY > 0 ) ) {
          V9 = 4;
        }
      }
    }
  }
  // DH table
  // column D - 1 - alpha
  DHt[0][1] = ToRadians( DHr1 );
  DHt[1][1] = ToRadians( DHr2 );
  DHt[2][1] = ToRadians( DHr3 );
  DHt[3][1] = ToRadians( DHr4 );
  DHt[4][1] = ToRadians( DHr5 );
  DHt[5][1] = ToRadians( DHr6 );
  // column E - 2 - d
  DHt[0][2] = DHd1;
  DHt[1][2] = DHd2;
  DHt[2][2] = DHd3;
  DHt[3][2] = DHd4;
  DHt[4][2] = DHd5;
  DHt[5][2] = DHd6;
  // column F - 3 - a
  DHt[0][3] = DHa1;
  DHt[1][3] = DHa2;
  DHt[2][3] = DHa3;
  DHt[3][3] = DHa4;
  DHt[4][3] = DHa5;
  DHt[5][3] = DHa6;
  if ( verbose ) Matrix.Print( (float*)DHt, 6, 4, "DH table");
  
  // Work frame input
  Wf[0][0] = cos(ToRadians(UFrz))*cos(ToRadians(UFry));
  Wf[0][1] = -sin(ToRadians(UFrz))*cos(ToRadians(UFrx))+cos(ToRadians(UFrz))*sin(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[0][2] = sin(ToRadians(UFrz))*sin(ToRadians(UFrx))+cos(ToRadians(UFrz))*sin(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[0][3] = UFx;

  Wf[1][0] = sin(ToRadians(UFrz))*cos(ToRadians(UFry));
  Wf[1][1] = cos(ToRadians(UFrz))*cos(ToRadians(UFrx))+sin(ToRadians(UFrz))*sin(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[1][2] = -cos(ToRadians(UFrz))*sin(ToRadians(UFrx))+sin(ToRadians(UFrz))*sin(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[1][3] = UFy;

  Wf[2][0] = -sin(ToRadians(UFrz));
  Wf[2][1] = cos(ToRadians(UFry))*sin(ToRadians(UFrx));
  Wf[2][2] = cos(ToRadians(UFry))*cos(ToRadians(UFrx));
  Wf[2][3] = UFz;

  Wf[3][0] = 0.0;
  Wf[3][1] = 0.0;
  Wf[3][2] = 0.0;
  Wf[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)Wf, 4, 4, "Work frame");

  // R 0-T
  R0t[0][0] = cos(ToRadians(CRx))*cos(ToRadians(CRz))-cos(ToRadians(CRy))*sin(ToRadians(CRx))*sin(ToRadians(CRz)); 
  R0t[0][1] = cos(ToRadians(CRz))*sin(ToRadians(CRx))+cos(ToRadians(CRx))*cos(ToRadians(CRy))*sin(ToRadians(CRz));
  R0t[0][2] = sin(ToRadians(CRy))*sin(ToRadians(CRz));
  R0t[0][3] = CX;
  
  R0t[1][0] = cos(ToRadians(CRy))*cos(ToRadians(CRz))*sin(ToRadians(CRx))+cos(ToRadians(CRx))*sin(ToRadians(CRz));
  R0t[1][1] = cos(ToRadians(CRx))*cos(ToRadians(CRy))*cos(ToRadians(CRz))-sin(ToRadians(CRx))*sin(ToRadians(CRz));
  R0t[1][2] = cos(ToRadians(CRz))*sin(ToRadians(CRy));
  R0t[1][3] = CY;

  R0t[2][0] = sin(ToRadians(CRx))*sin(ToRadians(CRy));
  R0t[2][1] = cos(ToRadians(CRx))*sin(ToRadians(CRy));
  R0t[2][2] = -cos(ToRadians(CRy));
  R0t[2][3] = CZ;
  
  R0t[3][0] = 0.0;
  R0t[3][1] = 0.0;
  R0t[3][2] = 0.0;
  R0t[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)R0t, 4, 4, "R0-T");
  
  Matrix.Multiply( (float*)Wf, (float*)R0t, 4, 4, 4, (float*)tmp1f );
  tmp1f[0][0] = tmp1f[0][0] * -1; // not sure why the cell needs to be negated. Talk to Chris for explanation
  if ( verbose ) Matrix.Print( (float*)tmp1f, 4, 4, "R0-T offset by work frame");

  // Tool frame
  Tf[0][0] = cos(ToRadians(TFrz))*cos(ToRadians(TFry));
  Tf[0][1] = -sin(ToRadians(TFrz))*cos(ToRadians(TFrx))+cos(ToRadians(TFrz))*sin(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[0][2] = sin(ToRadians(TFrz))*sin(ToRadians(TFrx))+cos(ToRadians(TFrz))*sin(ToRadians(TFry))*cos(ToRadians(TFrx));
  Tf[0][3] = TFx;
  
  Tf[1][0] = sin(ToRadians(TFrz))*cos(ToRadians(TFry));
  Tf[1][1] = cos(ToRadians(TFrz))*cos(ToRadians(TFrx))+sin(ToRadians(TFrz))*sin(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[1][2] = -cos(ToRadians(TFrz))*sin(ToRadians(TFrx))+sin(ToRadians(TFrz))*sin(ToRadians(TFry))*cos(ToRadians(TFrx)); 
  Tf[1][3] = TFy;

  Tf[2][0] = -sin(ToRadians(TFrz));
  Tf[2][1] = cos(ToRadians(TFry))*sin(ToRadians(TFrx));
  Tf[2][2] = cos(ToRadians(TFry))*cos(ToRadians(TFrx)); 
  Tf[2][3] = TFz;

  Tf[3][0] = 0.0;
  Tf[3][1] = 0.0;
  Tf[3][2] = 0.0;
  Tf[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)Tf, 4, 4, "Tool frame");

  // Invert tool frame
  Matrix.Invert((float*)Tf, 4);
  if ( verbose ) Matrix.Print( (float*)Tf, 4, 4, "Tool frame inverted");

  // R0-T offset by work frame * inverted tool frame -> R 0-6
  Matrix.Multiply( (float*)tmp1f, (float*)Tf, 4, 4, 4, (float*)R06t );
  if ( verbose ) Matrix.Print( (float*)R06t, 4, 4, "R0-6");

  // REMOVE R 0-6 table
  tmp2f[0][0] = cos(ToRadians(180));
  tmp2f[0][1] = sin(ToRadians(180));
  tmp2f[0][2] = 0.0;
  tmp2f[0][3] = 0.0;

  tmp2f[1][0] = -sin(ToRadians(180))*cos(ToRadians(DHr6));
  tmp2f[1][1] = cos(ToRadians(180))*cos(ToRadians(DHr6));
  tmp2f[1][2] = sin(ToRadians(DHr6));
  tmp2f[1][3] = 0.0;

  tmp2f[2][0] = sin(ToRadians(180))*sin(ToRadians(DHr6));
  tmp2f[2][1] = -cos(ToRadians(180))*sin(ToRadians(DHr6));
  tmp2f[2][2] = cos(ToRadians(DHr6));
  tmp2f[2][3] = -DHd6;

  tmp2f[3][0] = 0.0;
  tmp2f[3][1] = 0.0;
  tmp2f[3][2] = 0.0;
  tmp2f[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)tmp2f, 4, 4, "Remove R0-5");
  
  // Apply remove R 0-6 -> R 0-5 (centre spherical wrist)
  Matrix.Multiply( (float*)R06t, (float*)tmp2f, 4, 4, 4, (float*)R05t );
  if ( verbose ) Matrix.Print( (float*)R05t, 4, 4, "R0-5");

  // CALCULATE J1 ANGLE
  J1out = atan( R05t[1][3] / R05t[0][3] );
  switch ( V9 ) {
    case 0 :
    case 1 : 
    case 2 : J1out = ToDegrees( J1out );
    break;
    case 3 : J1out = -180 + ToDegrees( J1out ); 
    break;
    case 4 : J1out = 180 + ToDegrees( J1out );
    break;
  }
/*
  float E13 = DHd1;
  float E14 = DHd2;
  float E15 = DHd3;
  float E16 = DHd4;
  float E17 = DHd5;
  float E18 = DHd6;
  float F13 = DHa1;
  float F14 = DHa2;
  float F15 = DHa3;
 */
  // CALCULATE J2 ANGLE  FWD
//  Serial.println( "Calculate J2 angle forward" );
  float O18 = sqrt( ( R05t[1][3] * R05t[1][3] ) + ( R05t[0][3] * R05t[0][3] ) );
//  Serial.print( "O18=" ); Serial.println( O18 ); 
  float O19 = R05t[2][3] - DHd1;
//  Serial.print( "O19=" ); Serial.println( O19 ); 
  float O20 = O18 - DHa1;
//  Serial.print( "O20=" ); Serial.println( O20 ); 
  float O21 = sqrt( ( O19 * O19 ) + ( O20 * O20 ) );
//  Serial.print( "O21=" ); Serial.println( O21 ); 
  float O22 = ToDegrees( atan( O19 / O20 ) );
//  Serial.print( "O22=" ); Serial.println( O22 ); 
  // Arduino float overflows with these numbers
  double x1 = double( DHa2 * DHa2 );
  double x2 = double( O21 * O21 );
  double x3 = double( DHd4 * DHd4 );
  double x4 = x1 + x2 - x3;
  double x5 = double( 2 * DHa2 * O21 );
  float O23 = ToDegrees( acos( ( x4 ) / x5 ) );
//  Serial.print( "O23=" ); Serial.println( O23 ); 
//  float O24 = 180 - ToDegrees( acos( ( ( DHd4 * DHd4 ) + ( DHa2 * DHa2 ) - ( O21 * O21 ) ) / ( 2 * abs( DHd4 ) * DHa2 ) ) );
  x4 =  x3 + x1 - x2;
  float O24 = 180 - ToDegrees( acos( x4 / ( 2 * abs( DHd4 ) * DHa2 ) ) );
//  Serial.print( "O24=" ); Serial.println( O24 ); 
  // unused float O25 = ToDegrees( atan( abs( DHd4 ) / DHa3 ) );
//  Serial.print( "O25=" ); Serial.println( O25 ); 
  float O26 = -( O22 + O23 );
//  Serial.print( "O26=" ); Serial.println( O26 ); 
  float O27 = O24;
//  Serial.print( "O27=" ); Serial.println( O27 ); 

  // CALCULATE J2 ANGLE MID
//  Serial.println( "Calculate J2 angle mid" );
  float P20 = -O20;
//  Serial.print( "P20=" ); Serial.println( P20 ); 
  float P21 = sqrt( ( O19 * O19 ) + ( P20 * P20 ) );
//  Serial.print( "P21=" ); Serial.println( P21 ); 
  float P22 = ToDegrees( acos( ( ( DHa2 * DHa2 ) + ( P21 * P21 ) - ( DHd4 * DHd4 ) ) / ( 2 * DHa2 * P21 ) ) );
//  Serial.print( "P22=" ); Serial.println( P22 ); 
  float P23 = ToDegrees( atan( P20 / O19 ) );
//  Serial.print( "P23=" ); Serial.println( P23 ); 
  float P24 = 180 - ToDegrees( acos( ( ( DHd4 * DHd4 ) + ( DHa2 * DHa2 ) - ( P21 * P21 ) ) / ( 2 * abs( DHd4 ) * DHa2 ) ) );
//  Serial.print( "P24=" ); Serial.println( P24 ); 
  float P25 = 90 - ( P22 + P23 );
//  Serial.print( "P25=" ); Serial.println( P25 ); 
  float P26 = -180 + P25;
//  Serial.print( "P26=" ); Serial.println( P26 ); 
  float P27 = P24;
//  Serial.print( "P27=" ); Serial.println( P27 ); 
  // J2,J3
//  Serial.println( ( O20 < 0 ) ? "less than" : "not less than" );
  ( O20 < 0 ) ? J2out = P26 : J2out = O26;
  ( O20 < 0 ) ? J3out = P27 : J3out = O27;
  // return J1-3
  
  // --------------------------------------------------------------------------------
  // 

  // Save these for use in joint frames
  float D13 = ToRadians( DHr1 );
  float D14 = ToRadians( DHr2 );
  float D15 = ToRadians( DHr3 );
  float E13 = DHd1;
  float E14 = DHd2;
  float E15 = DHd3;
  float F13 = DHa1;
  float F14 = DHa2;
  float F15 = DHa3;

  float Q4 = ToRadians( J1out );
  float Q5 = ToRadians( J2out );
  float Q6 = ToRadians( J3out - 90 );
  // J1 frame
  J1f[0][0] = cos( Q4 );
  J1f[0][1] = -sin( Q4 ) * cos( D13 );
  J1f[0][2] =  sin( Q4 ) * sin( D13 );
  J1f[0][3] = F13 * cos( Q4 );
  
  J1f[1][0] = sin( Q4 );
  J1f[1][1] =  cos( Q4 ) * cos( D13 );
  J1f[1][2] = -cos( Q4 ) * sin( D13 );
  J1f[1][3] = F13 * sin( Q4 );

  J1f[2][0] = 0.0;
  J1f[2][1] =  sin( D13 );
  J1f[2][2] =  cos( D13 );
  J1f[2][3] = E13;

  J1f[3][0] = 0.0;
  J1f[3][1] = 0.0;
  J1f[3][2] = 0.0;
  J1f[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)J1f, 4, 4, "J1f frame");
  // J2 frame
  J2f[0][0] = cos( Q5 );
  J2f[0][1] = -sin( Q5 ) * cos( D14 );
  J2f[0][2] =  sin( Q5 ) * sin( D14 );
  J2f[0][3] = F14 * cos( Q5 );

  J2f[1][0] = sin( Q5 );
  J2f[1][1] =  cos( Q5 ) * cos( D14 );
  J2f[1][2] = -cos( Q5 ) * sin( D14 );
  J2f[1][3] = F14 * sin( Q5 );

  J2f[2][0] = 0.0;
  J2f[2][1] =  sin( D14 );
  J2f[2][2] = cos( D14 );
  J2f[2][3] = E14;

  J2f[3][0] = 0.0;
  J2f[3][1] = 0.0;
  J2f[3][2] = 0.0;
  J2f[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)J2f, 4, 4, "J2f frame");

  // J3 frame
  J3f[0][0] = cos( Q6 );
  J3f[0][1] = -sin( Q6 ) * cos( D15 );
  J3f[0][2] =  sin( Q6 ) * sin( D15 );
  J3f[0][3] = F15 * cos( Q6 );
  
  J3f[1][0] = sin( Q6 );
  J3f[1][1] =  cos( Q6 ) * cos (D15 );
  J3f[1][2] = -cos( Q6 ) * sin( D15 );
  J3f[1][3] = F15 * sin( Q6 );
  
  J3f[2][0] = 0.0;
  J3f[2][1] =  sin( D15 );
  J3f[2][2] =  cos( D15 );
  J3f[2][3] = E15;
  
  J3f[3][0] = 0.0;
  J3f[3][2] = 0.0;
  J3f[3][1] = 0.0;
  J3f[3][3] = 1.0;
  if ( verbose ) Matrix.Print( (float*)J3f, 4, 4, "J3f frame");

  if ( verbose ) Matrix.Print( (float*)Wf, 4, 4, "Work frame");
  // R 0-1
  Matrix.Multiply( (float*)Wf, (float*)J1f, 4, 4, 4, (float*)R01t );
  if ( verbose ) Matrix.Print( (float*)R01t, 4, 4, "R01t table");
  // R 0-2 
  Matrix.Multiply( (float*)R01t, (float*)J2f, 4, 4, 4, (float*)R02t ); 
  if ( verbose ) Matrix.Print( (float*)R02t, 4, 4, "R02t table");
  // R 0-3 
  Matrix.Multiply( (float*)R02t, (float*)J3f, 4, 4, 4, (float*)R03t ); 
  if ( verbose ) Matrix.Print( (float*)R03t, 4, 4, "R03t table");
  // Extract 3x3 table
  float tmpR03t[3][3];
  tmpR03t[0][0] = R03t[0][0];
  tmpR03t[0][1] = R03t[0][1];
  tmpR03t[0][2] = R03t[0][2];
  tmpR03t[1][0] = R03t[1][0];
  tmpR03t[1][1] = R03t[1][1];
  tmpR03t[1][2] = R03t[1][2];
  tmpR03t[2][0] = R03t[2][0];
  tmpR03t[2][1] = R03t[2][1];
  tmpR03t[2][2] = R03t[2][2];
  if ( verbose ) Matrix.Print( (float*)tmpR03t, 3, 3, "tmpR03t" );
  // R 0-3 transposed
  Matrix.Transpose( (float*)tmpR03t, 3, 3, (float*)R03trans );
  if ( verbose ) Matrix.Print( (float*)R03trans, 3, 3, "R03trans table" );
  // Extract 3 x 3 table from R05t
  float tmpR05t[3][3];
  tmpR05t[0][0] = R05t[0][0];
  tmpR05t[0][1] = R05t[0][1];
  tmpR05t[0][2] = R05t[0][2];
  tmpR05t[1][0] = R05t[1][0];
  tmpR05t[1][1] = R05t[1][1];
  tmpR05t[1][2] = R05t[1][2];
  tmpR05t[2][0] = R05t[2][0];
  tmpR05t[2][1] = R05t[2][1];
  tmpR05t[2][2] = R05t[2][2];
  if ( verbose ) Matrix.Print( (float*)tmpR05t, 3, 3, "tmpR05t table" );
  
  // R 3-6 (spherical wrist  orientation)
  Matrix.Multiply( (float*)R03trans, (float*)tmpR05t, 3, 3, 3, (float*)R36t );
  if ( verbose ) Matrix.Print( (float*)R36t, 3, 3, "R 3-6 (spherical wrist  orientation)");

  // Extract wrist orientation parameters
  float X74 = R36t[2][0];
  float Y74 = R36t[2][1];
  float Z72 = R36t[0][2];
  float Z73 = R36t[1][2];
  float Z74 = R36t[2][2];
  // wrist  orientation
  float R7 = ToDegrees( atan2( Z73, Z72 ) );
  float R8 = ToDegrees( atan2( sqrt( ( 1 - ( Z74 *Z74 ) ) ), Z74 ) );
  float R9;
  if ( Y74 < 0 ) 
    R9 = ToDegrees( atan2( -Y74, X74 ) )-180;
  else
    R9 = ToDegrees( atan2( -Y74, X74 ) )+180;    
  float S7 = ToDegrees( atan2( -Z73, -Z72 ) );
  float S8 = ToDegrees( atan2( -sqrt( ( 1 - ( Z74 * Z74 ) ) ), Z74 ) );
  float S9;
  if ( Y74 < 0 )
    S9 = ToDegrees( atan2( Y74, -X74 ) )+180;
  else
    S9 = ToDegrees( atan2( Y74, -X74 ) )-180;

//  Serial.print( "Y74=" ); Serial.print( Y74 );
//  Serial.print( ", R9=" ); Serial.print( R9 );
//  Serial.print( ", S9=" ); Serial.println( S9 );

  // return  J4-6
  wrist_config_f ? J5out = R8 : J5out = S8;
  ( J5out > 0 )  ? J4out = R7 : J4out = S7;
  ( J5out < 0 )  ? J6out = S9 : J6out = R9;

//  Serial.print( "R7=" ); Serial.print( R7 );
//  Serial.print( ", R8=" ); Serial.print( R8 );
//  Serial.print( ", R9=" ); Serial.print( R9 );
//  Serial.print( ", S7=" ); Serial.print( S7 );
//  Serial.print( ", S8=" ); Serial.println( S8 );
//  Serial.print( ", S9=" ); Serial.println( S9 );
  
//  Serial.print( "J1=" ); Serial.print( J1out );
//  Serial.print( ", J2=" ); Serial.print( J2out );
//  Serial.print( ", J3=" ); Serial.print( J3out );
//  Serial.print( ", J4=" ); Serial.print( J4out );
//  Serial.print( ", J5=" ); Serial.print( J5out );
//  Serial.print( ", J6=" ); Serial.println( J6out );
}

// floating point map 
float map_f( float x, float in_min, float in_max, float out_min, float out_max) 
{
  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}

// Travel smoothly from current point to another point
bool gotoPoint( float CX, float CY, float CZ, float CRx, float CRy, float CRz )
{
  bool IsError = false;
  // Joint output angles
  float J1;
  float J2;
  float J3;
  float J4;
  float J5;
  float J6;

  Serial.println( "Input parameters..." );
  Serial.print( "x=" ); Serial.print( CX );
  Serial.print( ", y=" ); Serial.print( CY );
  Serial.print( ", z=" ); Serial.print( CZ );
  Serial.print( ", yaw=" ); Serial.print( CRx );
  Serial.print( ", pitch=" ); Serial.print( CRy );
  Serial.print( ", roll=" ); Serial.println( CRz );

  // Inverse Kinematics
  // Input world coordinates
  CalcInvKin( CX, CY, CZ, CRx, CRy, CRz, J1, J2, J3, J4, J5, J6 );
//  Serial.println( "Inverse kinematics, output joint angles" );
//  Serial.print( "J1=" ); Serial.print( J1 );
//  Serial.print( ", J2=" ); Serial.print( J2 );
//  Serial.print( ", J3=" ); Serial.print( J3 );
//  Serial.print( ", J4=" ); Serial.print( J4 );
//  Serial.print( ", J5=" ); Serial.print( J5 );
//  Serial.print( ", J6=" ); Serial.println( J6 );

  // Check angle limits
  if ( ( J1 < _base.NegAngleLimit ) || ( J1 > _base.PosAngleLimit ) ) {
    Serial.print( "J1 bounds error " ); Serial.print( J1 ); Serial.print( ", " ); Serial.print( _base.NegAngleLimit ); Serial.print( ", " ); Serial.println( _base.PosAngleLimit );
    IsError = true;
    ( J1 < _base.NegAngleLimit ) ? J1 = _base.NegAngleLimit : J1 = _base.PosAngleLimit;
  }
  if ( ( J2 < _shoulder.NegAngleLimit ) || ( J2 > _shoulder.PosAngleLimit ) ) {
    Serial.print( "J2 bounds error - angle limit enforced " ); Serial.print( J2 ); Serial.print( ", " ); Serial.print( _shoulder.NegAngleLimit ); Serial.print( ", " ); Serial.println( _shoulder.PosAngleLimit );
    IsError = true;
    ( J2 < _shoulder.NegAngleLimit ) ? J2 = _shoulder.NegAngleLimit : J2 = _shoulder.PosAngleLimit;
  }
  if ( ( J3 < _elbow.NegAngleLimit ) || ( J3 > _elbow.PosAngleLimit ) ) {
    Serial.print( "J3 bounds error - angle limit enforced " ); Serial.print( J3 ); Serial.print( ", " ); Serial.print( _elbow.NegAngleLimit ); Serial.print( ", " ); Serial.println( _elbow.PosAngleLimit );
    IsError = true;
    ( J3 < _elbow.NegAngleLimit ) ? J3 = _elbow.NegAngleLimit : J3 = _elbow.PosAngleLimit;
  }
  if ( ( J4 < _wrist_yaw.NegAngleLimit ) || ( J4 > _wrist_yaw.PosAngleLimit ) ) {
    Serial.print( "J4 bounds error - angle limit enforced " ); Serial.print( J4 ); Serial.print( ", " ); Serial.print( _wrist_yaw.NegAngleLimit ); Serial.print( ", " ); Serial.println( _wrist_yaw.PosAngleLimit );
    IsError = true;
    ( J4 < _wrist_yaw.NegAngleLimit ) ? J4 = _wrist_yaw.NegAngleLimit : J4 = _wrist_yaw.PosAngleLimit;
  }
  if ( ( J5 < _wrist_pitch.NegAngleLimit ) || ( J5 > _wrist_pitch.PosAngleLimit ) ) {
    Serial.print( "J5 bounds error - angle limit enforced " ); Serial.print( J5 ); Serial.print( ", " ); Serial.print( _wrist_pitch.NegAngleLimit ); Serial.print( ", " ); Serial.println( _wrist_pitch.PosAngleLimit );
    IsError = true;
    ( J5 < _wrist_pitch.NegAngleLimit ) ? J5 = _wrist_pitch.NegAngleLimit : J5 = _wrist_pitch.PosAngleLimit;
  }
  if ( ( J6 < _wrist_roll.NegAngleLimit ) || ( J6 > _wrist_roll.PosAngleLimit ) ) {
    Serial.print( "J6 bounds error - angle limit enforced " ); Serial.print( J6 ); Serial.print( ", " ); Serial.print( _wrist_roll.NegAngleLimit ); Serial.print( ", " ); Serial.println( _wrist_roll.PosAngleLimit );
    IsError = true;
    ( J6 < _wrist_roll.NegAngleLimit ) ? J6 = _wrist_roll.NegAngleLimit : J6 = _wrist_roll.PosAngleLimit;
  }
  //if ( IsError ) return ( not IsError );
  
  Serial.print( "Inverse kinematics, output joint angles" );
  ( IsError == true ) ? Serial.print( " (corrected)\n" ) : Serial.print( "\n" );
  Serial.print( "J1=" ); Serial.print( J1 );
  Serial.print( ", J2=" ); Serial.print( J2 );
  Serial.print( ", J3=" ); Serial.print( J3 );
  Serial.print( ", J4=" ); Serial.print( J4 );
  Serial.print( ", J5=" ); Serial.print( J5 );
  Serial.print( ", J6=" ); Serial.println( J6 );

  // Calculate absolute step position for each joint,
  // and constrain to 0 - StepLimit 
  _base.CurrAngle = J1;
  _base.CurrStep = int32_t( floor( map_f( J1, _base.NegAngleLimit, _base.PosAngleLimit, 0, _base.StepLimit ) ) );
  _base.CurrStep = constrain( _base.CurrStep, 0, _base.StepLimit );
  j1_stepper.moveTo( _base.CurrStep );

  _shoulder.CurrAngle = J2;
  _shoulder.CurrStep = int32_t( floor( map_f( J2, _shoulder.NegAngleLimit, _shoulder.PosAngleLimit, 0, _shoulder.StepLimit ) ) );
  _shoulder.CurrStep = constrain( _shoulder.CurrStep, 0, _shoulder.StepLimit );
  j2_stepper.moveTo( _shoulder.CurrStep );

  _elbow.CurrAngle = J3;
  _elbow.CurrStep = int32_t( floor( map_f( J3, _elbow.NegAngleLimit, _elbow.PosAngleLimit, 0, _elbow.StepLimit ) ) );
  _elbow.CurrStep = constrain( _elbow.CurrStep, 0, _elbow.StepLimit );
  j3_stepper.moveTo( _elbow.CurrStep );

  _wrist_yaw.CurrAngle = J4;
  _wrist_yaw.CurrStep = int32_t( floor( map_f( J4, _wrist_yaw.NegAngleLimit, _wrist_yaw.PosAngleLimit, 0, _wrist_yaw.StepLimit ) ) );
  _wrist_yaw.CurrStep = constrain( _wrist_yaw.CurrStep, 0, _wrist_yaw.StepLimit );
  j4_stepper.moveTo( _wrist_yaw.CurrStep );

  _wrist_pitch.CurrAngle = J5;
  _wrist_pitch.CurrStep = int32_t( floor( map_f( J5, _wrist_pitch.NegAngleLimit, _wrist_pitch.PosAngleLimit, 0, _wrist_pitch.StepLimit ) ) );
  _wrist_pitch.CurrStep = constrain( _wrist_pitch.CurrStep, 0, _wrist_pitch.StepLimit );
  j5_stepper.moveTo( _wrist_pitch.CurrStep );

  _wrist_roll.CurrAngle = J6;
  _wrist_roll.CurrStep = long( floor( map_f( J6, _wrist_roll.NegAngleLimit, _wrist_roll.PosAngleLimit, 0, _wrist_roll.StepLimit ) ) );
  _wrist_roll.CurrStep = constrain( _wrist_roll.CurrStep, 0, _wrist_roll.StepLimit );
  j6_stepper.moveTo( _wrist_roll.CurrStep );

  Serial.println( "Absolute step positions" );
  Serial.print( "J1 steps=" );   Serial.print(  _base.CurrStep );
  Serial.print( ", J2 steps=" ); Serial.print(  _shoulder.CurrStep );
  Serial.print( ", J3 steps=" ); Serial.print(  _elbow.CurrStep );
  Serial.print( ", J4 steps=" ); Serial.print(  _wrist_yaw.CurrStep );
  Serial.print( ", J5 steps=" ); Serial.print(  _wrist_pitch.CurrStep );
  Serial.print( ", J6 steps=" ); Serial.println(  _wrist_roll.CurrStep );

  // Move to absolute position 
  while ( true ) { // Thank you AccelStepper library!!
    if (  ( j1_stepper.distanceToGo() == 0 ) && ( j2_stepper.distanceToGo() == 0 ) && 
          ( j3_stepper.distanceToGo() == 0 ) && ( j4_stepper.distanceToGo() == 0 ) && 
          ( j5_stepper.distanceToGo() == 0 ) && ( j6_stepper.distanceToGo() == 0 ) ) break;
    j1_stepper.run();
    j2_stepper.run();
    j3_stepper.run();
    j4_stepper.run();
    j5_stepper.run();
    j6_stepper.run();
  }

  return ( not IsError );
}


void setup() {
  Serial.begin( 9600 );

  Gripper.attach( gripperPin );
  
  pinMode( J1stepPin, OUTPUT );
  pinMode( J1dirPin, OUTPUT );
  pinMode( J2stepPin, OUTPUT );
  pinMode( J2dirPin, OUTPUT );
  pinMode( J3stepPin, OUTPUT );
  pinMode( J3dirPin, OUTPUT );
  pinMode( J4stepPin, OUTPUT );
  pinMode( J4dirPin, OUTPUT );
  pinMode( J5stepPin, OUTPUT );
  pinMode( J5dirPin, OUTPUT );
  pinMode( J6stepPin, OUTPUT );
  pinMode( J6dirPin, OUTPUT );

  pinMode( J1calPin, INPUT_PULLUP ); 
  pinMode( J2calPin, INPUT_PULLUP ); 
  pinMode( J3calPin, INPUT_PULLUP ); 
  pinMode( J4calPin, INPUT_PULLUP ); 
  pinMode( J5calPin, INPUT_PULLUP ); 
  pinMode( J6calPin, INPUT_PULLUP ); 

  j1_stepper.setEnablePin( EN );
  j1_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j1_stepper.setPinsInverted( true, false, true ); // ccw
  // Change these to suit your stepper
  j1_stepper.setMaxSpeed( MAXSPEED );
  j1_stepper.setAcceleration( ACCELERATION );
//  j1_stepper.enableOutputs();
  j1_stepper.setCurrentPosition( 0 );

  j2_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j2_stepper.setPinsInverted( true, false, true ); // ccw
  j2_stepper.setMaxSpeed( MAXSPEED );
  j2_stepper.setAcceleration( ACCELERATION );
  j2_stepper.setCurrentPosition( 0 );

  j3_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j3_stepper.setPinsInverted( false, false, true ); // cw
  j3_stepper.setMaxSpeed( MAXSPEED );
  j3_stepper.setAcceleration( ACCELERATION );
  j3_stepper.setCurrentPosition( 0 );

  j4_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j4_stepper.setPinsInverted( false, false, true ); // cw
  j4_stepper.setMaxSpeed( MAXSPEED );
  j4_stepper.setAcceleration( ACCELERATION );
  j4_stepper.setCurrentPosition( 0 );

  j5_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j5_stepper.setPinsInverted( true, false, true ); // ccw
  j5_stepper.setMaxSpeed( MAXSPEED );
  j5_stepper.setAcceleration( ACCELERATION );
  j5_stepper.setCurrentPosition( 0 );

  j6_stepper.setMinPulseWidth( _MIN_PULSE_WIDTH );
  j6_stepper.setPinsInverted( false, false, true ); // cw
  j6_stepper.setMaxSpeed( MAXSPEED );
  j6_stepper.setAcceleration( ACCELERATION );
  j6_stepper.setCurrentPosition( 0 );

  // Initialise 
  // base J1
  _base.j_type = BASE;
  _base.CurrStep = 0;
  _base.CurrAngle = -170.00;
  _base.NegAngleLimit = -170.00;
  _base.PosAngleLimit = 170.00;
  _base.StepLimit = base_StepLimit;
  _base.StepsPerDeg = float( float( _base.StepLimit ) / float( _base.PosAngleLimit - _base.NegAngleLimit ) );
  _base.DegPerStep = float( ( _base.PosAngleLimit - _base.NegAngleLimit ) / float( _base.StepLimit ) );
  // shoulder J2
  _shoulder.j_type = SHOULDER;
  _shoulder.CurrStep = 0;
  _shoulder.CurrAngle = -132.00;
  _shoulder.NegAngleLimit = -132.00;
  _shoulder.PosAngleLimit = 0.00;
  _shoulder.StepLimit = shoulder_StepLimit;
  _shoulder.StepsPerDeg = float( float( _shoulder.StepLimit ) / float( _shoulder.PosAngleLimit - _shoulder.NegAngleLimit ) );
  _shoulder.DegPerStep = float( ( _shoulder.PosAngleLimit - _shoulder.NegAngleLimit ) / float( _shoulder.StepLimit ) );
  // elbow J3 
  _elbow.j_type = ELBOW;
  _elbow.CurrStep = 0;
  _elbow.CurrAngle = 1.00;
  _elbow.NegAngleLimit = 1.00;
  _elbow.PosAngleLimit = 141.00;
  _elbow.StepLimit = elbow_StepLimit; // 7850;
  _elbow.StepsPerDeg = float( float( _elbow.StepLimit ) / float( _elbow.PosAngleLimit - _elbow.NegAngleLimit ) );
  _elbow.DegPerStep = float( ( _elbow.PosAngleLimit - _elbow.NegAngleLimit ) / float( _elbow.StepLimit ) );
  // wrist yaw J4
  _wrist_yaw.j_type = WRIST_YAW;
  _wrist_yaw.CurrStep = 0;
  _wrist_yaw.CurrAngle = -165.00;
  _wrist_yaw.NegAngleLimit = -165.00;
  _wrist_yaw.PosAngleLimit = 165.00;
  _wrist_yaw.StepLimit = wrist_yaw_StepLimit; // 15200;
  _wrist_yaw.StepsPerDeg = float( float( _wrist_yaw.StepLimit ) / float( _wrist_yaw.PosAngleLimit - _wrist_yaw.NegAngleLimit ) );
  _wrist_yaw.DegPerStep = float( ( _wrist_yaw.PosAngleLimit - _wrist_yaw.NegAngleLimit ) / float( _wrist_yaw.StepLimit ) );
  // wrist pitch J5
  _wrist_pitch.j_type = WRIST_PITCH;
  _wrist_pitch.CurrStep = 0;
  _wrist_pitch.CurrAngle = -105.00;
  _wrist_pitch.NegAngleLimit = -105.00;
  _wrist_pitch.PosAngleLimit = 105.00;
  _wrist_pitch.StepLimit = wrist_pitch_StepLimit;
  _wrist_pitch.StepsPerDeg = float( float( _wrist_pitch.StepLimit ) / float( _wrist_pitch.PosAngleLimit - _wrist_pitch.NegAngleLimit ) );
  _wrist_pitch.DegPerStep = float( ( _wrist_pitch.PosAngleLimit - _wrist_pitch.NegAngleLimit ) / float( _wrist_pitch.StepLimit ) );
  // wrist roll J6
  _wrist_roll.j_type = WRIST_ROLL;
  _wrist_roll.CurrStep = 0;
  _wrist_roll.CurrAngle = -155.00;
  _wrist_roll.NegAngleLimit = -155.00;
  _wrist_roll.PosAngleLimit = 155.00;
  _wrist_roll.StepLimit = wrist_roll_StepLimit; // 6625;
  _wrist_roll.StepsPerDeg = float( float( _wrist_roll.StepLimit ) / float( _wrist_roll.PosAngleLimit - _wrist_roll.NegAngleLimit ) );
  _wrist_roll.DegPerStep = float( (  _wrist_roll.PosAngleLimit -  _wrist_roll.NegAngleLimit ) / float(  _wrist_roll.StepLimit ) );

  Serial.print( "AR2 firmware V" ); Serial.println( VERSION );
  Serial.print( "Pronto\n" );
}

String comdata = "";
String str ="";

float CX;
float CY; 
float CZ;
float CRx; // yaw
float CRy; // pitch
float CRz; // roll
float _j1, j2, j3, j4, j5, j6;
int _speed, _accel;

// split
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1  };
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Execute command
// a + list of 6 angles
// p + list of 6 global positions
// d + milliseconds
// c + 0 - 6 ( configure )
// k park position
// h home position, all square
// m + speed, acceleration ( motion )
// s + 0 - 180 ( servo )
// b ( toggle verbose sw )
// v ( version )
void loop() {
  char cmd;
  int i;

  // command/response loop
  // read string from serial monitor
  if ( Serial.available() ) {
    comdata = Serial.readStringUntil( '\n' );
    //Serial.println("x, y, z=" + comdata );
    // first char is command
    cmd = comdata.charAt( 0 );
    comdata.remove( 0, 1 );
    printfreeMemory();
    switch( cmd ) { // command
      case 'a' : ; // expect list of angles
        str = getValue( comdata, ',', 0 );
        _j1 = str.toFloat();
        str = getValue( comdata, ',', 1 );
        j2 = str.toFloat();
        str = getValue( comdata, ',', 2 );
        j3 = str.toFloat();
        str = getValue( comdata, ',', 3 );
        j4 = str.toFloat();
        str = getValue( comdata, ',', 4 );
        j5 = str.toFloat();
        str = getValue( comdata, ',', 5 );
        j6 = str.toFloat();
        // Forward kinematics
        // Input joint angles
        CalcFwdKin( _j1, j2, j3, j4, j5, j6, CX, CY, CZ, CRx, CRy, CRz );
        if ( not( gotoPoint( CX, CY, CZ, CRx, CRy, CRz ) ) )
          Serial.println( "gotoPoint error..." );
      break;
      case 'p' : ; // expect list of positions
        str = getValue( comdata, ',', 0 );
        CX = str.toFloat();
        str = getValue( comdata, ',', 1 );
        CY = str.toFloat();
        str = getValue( comdata, ',', 2 );
        CZ = str.toFloat();
        str = getValue( comdata, ',', 3 );
        CRx = str.toFloat();
        str = getValue( comdata, ',', 4 );
        CRy = str.toFloat();
        str = getValue( comdata, ',', 5 );
        CRz = str.toFloat();
        if ( not( gotoPoint( CX, CY, CZ, CRx, CRy, CRz ) ) )
          Serial.println( "gotoPoint error..." );
      break;
      case 'd' :
        i = comdata.toInt(); 
        Serial.print( "Delay=" ); Serial.println( i ); // delay im millisec
        delay( i );
      break;
      case 'c' : // configuration
        i = comdata.toInt(); 
        Serial.print( "Configuration" ); Serial.println( i ); // config type
        configure_arm( i );
      break;
      case 'k' : // park
        Serial.println( "Park" );
        park_arm();
        break;
      case 'h' : // home
        home_arm();
        break;
      case 'm' : // motion - speed, acceleration
        str = getValue( comdata, ',', 0 );
        _speed = str.toInt();
        str = getValue( comdata, ',', 1 );
        _accel = str.toInt();
        set_motion( _speed, _accel );
        break;
      case 's' : // servo
        i = comdata.toInt();
        set_servo( i );
      case 'v' : Serial.print( "AR2 firmware V" ); Serial.println( VERSION );
      break;
      case 'b' : verbose = ( not verbose );
    }
    printfreeMemory(); // keep an eye on free memory
    Serial.print( "Pronto\n" ); // response
  }
 }

// Set gripper position 0 - 180
void set_servo( int p )
{
  Serial.print( "Servo=" ); Serial.println( p );
  p = constrain( p, 0, 180 );
  Gripper.write( p ); 
}
// Set stepper motion parameters speed and acceleration
void set_motion( int s, int a )
{
  j1_stepper.setMaxSpeed( s );
  j1_stepper.setAcceleration( a );
  j2_stepper.setMaxSpeed( s );
  j2_stepper.setAcceleration( a );
  j3_stepper.setMaxSpeed( s );
  j3_stepper.setAcceleration( a );
  j4_stepper.setMaxSpeed( s );
  j4_stepper.setAcceleration( a );
  j5_stepper.setMaxSpeed( s );
  j5_stepper.setAcceleration( a );
  j6_stepper.setMaxSpeed( s );
  j6_stepper.setAcceleration( a );
  Serial.print( "Motion parameters: Speed=" ); Serial.print( s ); 
  Serial.print( ", Acceleration=" ); Serial.println( a );
}

// move to home position
void home_arm( void )
{
  Serial.println( "Moving to home position" );
  // Forward kinematics
  // X) 286.84   Y) -0.1   Z) 438.52   W) -89.923   P) 179.9   R) -89.923
  // j1=0, j2=-90, j3=90, j4=0, j5=90, j6=0
  Serial.println( "Calibration position: j1=0, j2=-90, j3=90, j4=0, j5=90, j6=0" );
  if ( not( gotoPoint( 286, 0, 440, -90, 180, -90 ) ) )
      Serial.println( "gotoPoint error..." );
}
// move to park position
// must recalibrate before use
void park_arm( void )
{
  j1_stepper.moveTo( 7600 );
  j2_stepper.moveTo( 2322 );
  j3_stepper.moveTo( 7850 );
  j4_stepper.moveTo( 7600 );
  j5_stepper.moveTo( 2500 );
  j6_stepper.moveTo( 3312 );
  // Move to absolute position 
  while ( true ) { 
    if (  ( j1_stepper.distanceToGo() == 0 ) && ( j2_stepper.distanceToGo() == 0 ) && 
          ( j3_stepper.distanceToGo() == 0 ) && ( j4_stepper.distanceToGo() == 0 ) && 
          ( j5_stepper.distanceToGo() == 0 ) && ( j6_stepper.distanceToGo() == 0 ) ) break;
    j1_stepper.run();
    j2_stepper.run();
    j3_stepper.run();
    j4_stepper.run();
    j5_stepper.run();
    j6_stepper.run();
  }
  Serial.println( "Warning: Arm must be re-calibrated before use" );
}

// Run stepper slowly backwards until limit switch goes high
void configure_stepper( AccelStepper& a_stepper, int limit_pin )
{
  Bounce *debouncer = new Bounce; 
  // Limit switch debouncer
  debouncer->attach( limit_pin );
  debouncer->interval( 20 ); // interval in ms

  long initial_homing = -1;  // Used to Home Stepper at startup

  // Set Max Speed and Acceleration of each Steppers at startup for homing
  a_stepper.setMaxSpeed( 500.0 );      // Set Max Speed of Stepper 100 (Slower to get better accuracy)
  a_stepper.setAcceleration( 100.0 );  // Set Acceleration of Stepper

  // Start Homing procedure of Stepper Motor at startup
  Serial.print("Stepper is Homing . . . . . . . . . . . ");
  a_stepper.setCurrentPosition( 0 );

  // Update the Bounce instances :
  debouncer->update();
  // Get the updated value :
  int limit_reached = debouncer->read();
  
  while ( limit_reached == 0 ) {     // Make the Stepper move CCW until the switch is activated   
    a_stepper.moveTo( initial_homing );   // Set the position to move to
    initial_homing--;                     // Decrease by 1 for next move if needed
    a_stepper.run();                      // Start moving the stepper
    delay( 5 );
    debouncer->update();
    limit_reached = debouncer->read();
  }
  a_stepper.setCurrentPosition( 0 );
  delete debouncer;
  // reset speed and acceleration
  a_stepper.setMaxSpeed( MAXSPEED );      
  a_stepper.setAcceleration( ACCELERATION );  
}

void configure_all( void )
{
  float a;
  
  Serial.println( "Configure all..." );
  // Check all limit switches are off
  // If there's any problem with the limit switch wiring, it will be on and the
  // limit will be reached without moving the stepper. Visual check all limits are low.
  Serial.print( "J1 limit switch is " ); Serial.println( J1calPin==1 ? "on" : "off" );
  Serial.print( "J2 limit switch is " ); Serial.println( J2calPin==1 ? "on" : "off" );
  Serial.print( "J3 limit switch is " ); Serial.println( J3calPin==1 ? "on" : "off" );
  Serial.print( "J4 limit switch is " ); Serial.println( J4calPin==1 ? "on" : "off" );
  Serial.print( "J5 limit switch is " ); Serial.println( J5calPin==1 ? "on" : "off" );
  Serial.print( "J6 limit switch is " ); Serial.println( J6calPin==1 ? "on" : "off" );
  
  Serial.println( "J1 stepper" );
  configure_stepper( j1_stepper, J1calPin );
  Serial.println( "J1 limit reached" );
  // base J1
  _base.j_type = BASE;
  _base.CurrStep = 0;
  _base.CurrAngle = -170.00;
  _base.NegAngleLimit = -170.00;
  _base.PosAngleLimit = 170.00;
  _base.StepLimit = base_StepLimit;
  _base.StepsPerDeg = float( float( _base.StepLimit ) / float( _base.PosAngleLimit - _base.NegAngleLimit ) );
  _base.DegPerStep = float( ( _base.PosAngleLimit - _base.NegAngleLimit ) / float( _base.StepLimit ) );
  // Move to 0
  a = 0;
  _base.CurrAngle = a;
  _base.CurrStep = int32_t( floor( map_f( a, _base.NegAngleLimit, _base.PosAngleLimit, 0, _base.StepLimit ) ) );
  _base.CurrStep = constrain( _base.CurrStep, 0, _base.StepLimit );
  j1_stepper.moveTo( _base.CurrStep );
  while ( true ) { 
    if ( j1_stepper.distanceToGo() == 0 )  break;
    j1_stepper.run();
  }
  
  Serial.println( "J2 stepper" );
  configure_stepper( j2_stepper, J2calPin );
  Serial.println( "J2 limit reached" );
  // shoulder J2
  _shoulder.j_type = SHOULDER;
  _shoulder.CurrStep = 0;
  _shoulder.CurrAngle = -132.00;
  _shoulder.NegAngleLimit = -132.00;
  _shoulder.PosAngleLimit = 0.00;
  _shoulder.StepLimit = shoulder_StepLimit;
  _shoulder.StepsPerDeg = float( float( _shoulder.StepLimit ) / float( _shoulder.PosAngleLimit - _shoulder.NegAngleLimit ) );
  _shoulder.DegPerStep = float( ( _shoulder.PosAngleLimit - _shoulder.NegAngleLimit ) / float( _shoulder.StepLimit ) );
  // Move to -90
  a = -90;
  _shoulder.CurrAngle = a;
  _shoulder.CurrStep = int32_t( floor( map_f( a, _shoulder.NegAngleLimit, _shoulder.PosAngleLimit, 0, _shoulder.StepLimit ) ) );
  _shoulder.CurrStep = constrain( _shoulder.CurrStep, 0, _shoulder.StepLimit );
  j2_stepper.moveTo( _shoulder.CurrStep );
  while ( true ) { 
    if ( j2_stepper.distanceToGo() == 0 )  break;
    j2_stepper.run();
  }
  
  Serial.println( "J3 stepper" );
  configure_stepper( j3_stepper, J3calPin );
  Serial.println( "J3 limit reached" );
  // elbow J3 
  _elbow.j_type = ELBOW;
  _elbow.CurrStep = 0;
  _elbow.CurrAngle = 1.00;
  _elbow.NegAngleLimit = 1.00;
  _elbow.PosAngleLimit = 141.00;
  _elbow.StepLimit = elbow_StepLimit; // 7850;
  _elbow.StepsPerDeg = float( float( _elbow.StepLimit ) / float( _elbow.PosAngleLimit - _elbow.NegAngleLimit ) );
  _elbow.DegPerStep = float( ( _elbow.PosAngleLimit - _elbow.NegAngleLimit ) / float( _elbow.StepLimit ) );
  // Move to 90
  a = 90;
  _elbow.CurrAngle = a;
  _elbow.CurrStep = int32_t( floor( map_f( a, _elbow.NegAngleLimit, _elbow.PosAngleLimit, 0, _elbow.StepLimit ) ) );
  _elbow.CurrStep = constrain( _elbow.CurrStep, 0, _elbow.StepLimit );
  j3_stepper.moveTo( _elbow.CurrStep );
  while ( true ) { 
    if ( j3_stepper.distanceToGo() == 0 )  break;
    j3_stepper.run();
  }

  Serial.println( "J4 stepper" );
  configure_stepper( j4_stepper, J4calPin );
  Serial.println( "J4 limit reached" );
  // wrist yaw J4
  _wrist_yaw.j_type = WRIST_YAW;
  _wrist_yaw.CurrStep = 0;
  _wrist_yaw.CurrAngle = -165.00;
  _wrist_yaw.NegAngleLimit = -165.00;
  _wrist_yaw.PosAngleLimit = 165.00;
  _wrist_yaw.StepLimit = wrist_yaw_StepLimit; // 15200;
  _wrist_yaw.StepsPerDeg = float( float( _wrist_yaw.StepLimit ) / float( _wrist_yaw.PosAngleLimit - _wrist_yaw.NegAngleLimit ) );
  _wrist_yaw.DegPerStep = float( ( _wrist_yaw.PosAngleLimit - _wrist_yaw.NegAngleLimit ) / float( _wrist_yaw.StepLimit ) );
  // Move to 0
  a = 0;
  _wrist_yaw.CurrAngle = a;
  _wrist_yaw.CurrStep = int32_t( floor( map_f( a, _wrist_yaw.NegAngleLimit, _wrist_yaw.PosAngleLimit, 0, _wrist_yaw.StepLimit ) ) );
  _wrist_yaw.CurrStep = constrain( _wrist_yaw.CurrStep, 0, _wrist_yaw.StepLimit );
  j4_stepper.moveTo( _wrist_yaw.CurrStep );
  while ( true ) { 
    if ( j4_stepper.distanceToGo() == 0 )  break;
    j4_stepper.run();
  }
    
  Serial.println( "J5 stepper" );
  configure_stepper( j5_stepper, J5calPin );
  Serial.println( "J5 limit reached" );
  // wrist pitch J5
  _wrist_pitch.j_type = WRIST_PITCH;
  _wrist_pitch.CurrStep = 0;
  _wrist_pitch.CurrAngle = -105.00;
  _wrist_pitch.NegAngleLimit = -105.00;
  _wrist_pitch.PosAngleLimit = 105.00;
  _wrist_pitch.StepLimit = wrist_pitch_StepLimit;
  _wrist_pitch.StepsPerDeg = float( float( _wrist_pitch.StepLimit ) / float( _wrist_pitch.PosAngleLimit - _wrist_pitch.NegAngleLimit ) );
  _wrist_pitch.DegPerStep = float( ( _wrist_pitch.PosAngleLimit - _wrist_pitch.NegAngleLimit ) / float( _wrist_pitch.StepLimit ) );
  // Move to 90
  a = 90;
  _wrist_pitch.CurrAngle = a;
  _wrist_pitch.CurrStep = int32_t( floor( map_f( a, _wrist_pitch.NegAngleLimit, _wrist_pitch.PosAngleLimit, 0, _wrist_pitch.StepLimit ) ) );
  _wrist_pitch.CurrStep = constrain( _wrist_pitch.CurrStep, 0, _wrist_pitch.StepLimit );
  j5_stepper.moveTo( _wrist_pitch.CurrStep );
  while ( true ) { 
    if ( j5_stepper.distanceToGo() == 0 )  break;
    j5_stepper.run();
  }
    
  Serial.println( "J6 stepper" );
  configure_stepper( j6_stepper, J6calPin );
  Serial.println( "J6 limit reached" );
  // wrist roll J6
  _wrist_roll.j_type = WRIST_ROLL;
  _wrist_roll.CurrStep = 0;
  _wrist_roll.CurrAngle = -155.00;
  _wrist_roll.NegAngleLimit = -155.00;
  _wrist_roll.PosAngleLimit = 155.00;
  _wrist_roll.StepLimit = wrist_roll_StepLimit; // 6625;
  _wrist_roll.StepsPerDeg = float( float( _wrist_roll.StepLimit ) / float( _wrist_roll.PosAngleLimit - _wrist_roll.NegAngleLimit ) );
  _wrist_roll.DegPerStep = float( (  _wrist_roll.PosAngleLimit -  _wrist_roll.NegAngleLimit ) / float(  _wrist_roll.StepLimit ) );
  // Move to 0
  a = 0;
  _wrist_roll.CurrAngle = a;
  _wrist_roll.CurrStep = long( floor( map_f( a, _wrist_roll.NegAngleLimit, _wrist_roll.PosAngleLimit, 0, _wrist_roll.StepLimit ) ) );
  _wrist_roll.CurrStep = constrain( _wrist_roll.CurrStep, 0, _wrist_roll.StepLimit );
  j6_stepper.moveTo( _wrist_roll.CurrStep );
  while ( true ) { 
    if ( j6_stepper.distanceToGo() == 0 )  break;
    j6_stepper.run();
  }
  // Will be at home position but do it again
  // Move to home position. All angles should be square
  home_arm();
}

void configure_arm( int c )
{
  switch( c ) {
    case 0 : // all
      configure_all();
    break;
    case 1 : 
      configure_stepper( j1_stepper, J1calPin ); // j1
      Serial.println( "J1 limit reached" );
      _base.CurrStep = 0;
      _base.CurrAngle = -170.00;
    break;
    case 2 : configure_stepper( j2_stepper, J2calPin ); // j2
      Serial.println( "J2 limit reached" );
      _shoulder.CurrStep = 0;
      _shoulder.CurrAngle = -132.00;
    break;
    case 3 : configure_stepper( j3_stepper, J3calPin ); // j3
      Serial.println( "J3 limit reached" );
      _elbow.CurrStep = 0;
      _elbow.CurrAngle = 1.00;
    break;
    case 4 : configure_stepper( j4_stepper, J4calPin ); // j4
      Serial.println( "J4 limit reached" );
      _wrist_yaw.CurrStep = 0;
      _wrist_yaw.CurrAngle = -165.00;
    break;
    case 5 : configure_stepper( j5_stepper, J5calPin ); // j5
      Serial.println( "J5 limit reached" );
      _wrist_pitch.CurrStep = 0;
      _wrist_pitch.CurrAngle = -105.00;
    break;
    case 6 : configure_stepper( j6_stepper, J6calPin ); // j6
      Serial.println( "J6 limit reached" );
      _wrist_roll.CurrStep = 0;
      _wrist_roll.CurrAngle = -155.00;
    break;
  };
};

// Keep track of dynamic memory
void printfreeMemory( void )
{
  Serial.print( "Free memory=" ); Serial.println( freeMemory() );  
}

 #ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory( void ) {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
