#include <PID_v1.h>
#include <PS2X_lib.h>
#include <thruster.h>
#include <Stepper.h>
#include <Wire.h>
#define DIR1fl   39
#define DIR2fl   43
#define DIR1fr  8
#define DIR2fr  9
#define DIR1bl 5
#define DIR2bl 6
#define DIR1br  49
#define DIR2br  51
#define DIR1ml 2
#define DIR2ml 14
#define DIR1mr  12
#define DIR2mr 11
#define pwmfl  45
#define pwmfr 10
#define pwmbl 7
#define pwmbr 46
#define pwmml 4
#define pwmmr  13
#define CLOCK  A3
#define COMMAND A1
#define ATTENTION A2
#define DATA A0
#define FLOAT_LIMIT 1
#define CORDINATE_CONST 136
#define OUTPUT_BAUD_RATE 57600
#define OUTPUT_DATA_INTERVAL 20  // in milliseconds
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)
#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)
#define ALT_SEA_LEVEL_PRESSURE 102133
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)
int pwmarr[]={
  0,0,0,0,0,0};
// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
// RAW sensor data
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//float accel_min[3];
//float accel_max[3];

float magnetom[3];
//float magnetom_min[3];
//float magnetom_max[3];

float gyro[3];
float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3]= {
  0, 0, 0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3]= {
  0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {
  0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {
  0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {
  0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {
  0, 0, 0}; // Omega Integrator
float Omega[3]= {
  0, 0, 0};
float errorRollPitch[3] = {
  0, 0, 0};
float errorYaw[3] = {
  0, 0, 0};
float DCM_Matrix[3][3] = {
  {
    1, 0, 0                            }
  , {
    0, 1, 0                            }
  , {
    0, 0, 1                            }
};
float Update_Matrix[3][3] = {
  {
    0, 1, 2                            }
  , {
    3, 4, 5                            }
  , {
    6, 7, 8                            }
};
float Temporary_Matrix[3][3] = {
  {
    0, 0, 0                            }
  , {
    0, 0, 0                            }
  , {
    0, 0, 0                            }
};

// Euler angles
double yaw, pitch, roll;
double fpitch;
double fyaw;
double froll;
// DCM timing in the main loops
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;
/*thruster frthruster;
 thruster flthruster;
 thruster brthruster;
 thruster blthruster;
 thruster mlthruster;
 thruster mrthruster;
 */
Thruster flthruster;
Thruster frthruster;
Thruster blthruster;
Thruster brthruster;
Thruster mlthruster;
Thruster mrthruster;
int type;
int vibrate=0;
double pitchsp;
double yawsp;
double rollsp;
PS2X control;
void ReadSensors() {
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();  
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {
    1.0f, 0.0f, 0.0f                            };

  ReadSensors();
  timestamp = millis();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));

  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
  // Magnetometer axis mapping
  Magn_Vector[1] = -magnetom[0];
  Magn_Vector[0] = -magnetom[1];
  Magn_Vector[2] = -magnetom[2];

  // Magnetometer values mapping
  Magn_Vector[0] -= MAGN_X_OFFSET;
  Magn_Vector[0] *= MAGN_X_SCALE;
  Magn_Vector[1] -= MAGN_Y_OFFSET;
  Magn_Vector[1] *= MAGN_Y_SCALE;
  Magn_Vector[2] -= MAGN_Z_OFFSET;
  Magn_Vector[2] *= MAGN_Z_SCALE;

  // Accelerometer axis mapping
  Accel_Vector[1] = accel[0];
  Accel_Vector[0] = accel[1];
  Accel_Vector[2] = accel[2];

  // Accelerometer values mapping
  Accel_Vector[0] -= ACCEL_X_OFFSET;
  Accel_Vector[0] *= ACCEL_X_SCALE;
  Accel_Vector[1] -= ACCEL_Y_OFFSET;
  Accel_Vector[1] *= ACCEL_Y_SCALE;
  Accel_Vector[2] -= ACCEL_Z_OFFSET;
  Accel_Vector[2] *= ACCEL_Z_SCALE;

  // Gyroscope axis mapping
  Gyro_Vector[1] = -gyro[0];
  Gyro_Vector[0] = -gyro[1];
  Gyro_Vector[2] = -gyro[2];

  // Gyroscope values mapping
  Gyro_Vector[0] -= GYRO_X_OFFSET;
  Gyro_Vector[0] *= GYRO_X_SCALE;
  Gyro_Vector[1] -= GYRO_Y_OFFSET;
  Gyro_Vector[1] *= GYRO_Y_SCALE;
  Gyro_Vector[2] -= GYRO_Z_OFFSET;
  Gyro_Vector[2] *= GYRO_Z_SCALE;
}
int pwmpitch[]={
  0,0,0,0,0,0};
int pwmroll[]={
  0,0,0,0,0,0};
int pwmyaw[]={
  0,0,0,0,0,0};
int pwmmove[]={
  0,0,0,0,0,0};
double fppid;
double bppid;
double cypid;
double aypid;
double rrpid;
double lrpid;
PID pitchfpid(&pitch,&fppid,&pitchsp,70,0,0,DIRECT);
PID pitchbpid(&pitch,&bppid,&pitchsp,70,0,0,DIRECT);
PID yawcpid(&yaw,&cypid,&yawsp,80,6,1,DIRECT);
PID yawapid(&yaw,&aypid,&yawsp,80,6,1,DIRECT);
PID rollrpid(&roll,&rrpid,&rollsp,150,5,1,DIRECT);
PID rolllpid(&roll,&lrpid,&rollsp,150,5,1,DIRECT);
void setup()
{
  // Init serial output
  Serial.begin(OUTPUT_BAUD_RATE);
  flthruster.attachThruster(DIR1fl,DIR2fl,pwmfl);
  frthruster.attachThruster(DIR1fr,DIR2fr,pwmfr);
  blthruster.attachThruster(DIR1bl,DIR2bl,pwmbl);
  brthruster.attachThruster(DIR1br,DIR2br,pwmbr);
  mlthruster.attachThruster(DIR1ml,DIR2ml,pwmml);
  mrthruster.attachThruster(DIR1mr,DIR2mr,pwmmr);

  Serial.println("Thrusters have been attached");
  attachPS2(CLOCK,COMMAND,ATTENTION,DATA);
//  Serial.println("Exited the PS2 function.");
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();
  Serial.println("IMU has been initialized.");
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  Serial.println("PID set to be initialized");
  pitchfpid.SetMode(AUTOMATIC);
  pitchbpid.SetMode(AUTOMATIC);
  yawcpid.SetMode(AUTOMATIC);
  yawapid.SetMode(AUTOMATIC);
  rollrpid.SetMode(AUTOMATIC);
  rolllpid.SetMode(AUTOMATIC);
//  pitchfpid.SetSampleTime(100);
//  pitchbpid.SetSampleTime(100);
  Serial.println("All PID has been initialized.");
  pitchfpid.SetOutputLimits(0,100);
  pitchbpid.SetOutputLimits(0,100);
  yawcpid.SetOutputLimits(0,50);
  yawapid.SetOutputLimits(0,50);
  rollrpid.SetOutputLimits(0,100);
  rolllpid.SetOutputLimits(0,100);
  yawcpid.SetSampleTime(500);
  yawapid.SetSampleTime(500);
  Serial.println("PID Output limits has been set to 55");
  int i =1;
  while(i<100)
  {
    getAngle();
    pitchsp=(pitchsp+pitch);
    yawsp=(yawsp+yaw);
    rollsp=(rollsp+roll);
    i++;
  }
  pitchsp/=i;
  yawsp/=i;
  rollsp/=i;
  Serial.println(pitchsp);
  Serial.println(yawsp);
  Serial.println(rollsp);
  fpitch=pitchsp;
  froll=rollsp;
  fyaw=rollsp;
}
void loop()
{
  int forw=1,bac=1,dow=1,up=1,left=1,right=1;
  Serial.println("Entered loop");
//  control.read_gamepad(false,0);
  //    Serial.println("Happy");
  
//    //Serial.println("hello");
//  blthruster.moveThruster(-25);//back left
//  frthruster.moveThruster(-25);//fro`  1`````nt left negative
//  flthruster.moveThruster(-25);//fucker
//  mrthruster.moveThruster(25);//back right
////
//  brthruster.moveThruster(25);//front right
//  mlthruster.moveThruster(200);
    getAngle();
//    // Serial.println("Loop");
//    //Serial.println("Entered controlled loop");
//    //    setPoint(1,1,1);//Put fix switch here
    pitchfpid.Compute();
    pitchbpid.Compute();
    yawcpid.Compute();
    yawapid.Compute();
    //    rollrpid.Compute();
    //    rolllpid.Compute();
    setPitchBearing();
    //    setRollBearing();
    setYawBearing();

    //using flags and switch to include reset in rest case
//    if(control.Button(PSB_R1))
//    {
//      resetPwm(0,1);
//      thrusterUpdate();
//      delay(50);
//      Serial.println("Forward");
//      moveForward(1);
//      forw=0;
//    }
//    else
//    {
//      forw=1;
//    }
//    if(control.Button(PSB_R2))
//    {
//
//      resetPwm(0,1);
//      thrusterUpdate();
//      delay(50);
//      Serial.println("back here is the big black dug cup of tug tuir eiu");
//      moveBackward(1);
//      bac=0;
//    }
//    else
//    {
//      bac=1;
//    }
//    if(control.Button(PSB_L1))
//    {
//
//      resetPwm(1,0);
//      thrusterUpdate();
//      delay(50);
//      Serial.println("Ascend");
//      ascend(1);
//      up=0;
//    }
//    else
//    {
//      up=1;
//    }
//    if(control.Button(PSB_L2))
//    {
//      dow=0;
//      resetPwm(1,0);
//      thrusterUpdate();
//      delay(50);
//      Serial.println("Descend");
//      descend(1);
//    }
//    else
//    {
//      dow=1;
//    }
//    if(control.Button(PSB_PAD_UP))
//    {
//      pwmmove[4]=200;
//      pwmcumulate();
//      thrusterUpdate();
//      forw=0;
//      setstaticsetpoint();
//    }
//    else
//    {
//      right=1;
//    }
//    if(control.Button(PSB_PAD_DOWN))
//    {
//      pwmmove[5]=200;
//      pwmcumulate();
//      thrusterUpdate();
//      left=0;
//      setstaticsetpoint();
//    }
//    else
//    {
//      left=1;
//    }
//    // ascend(0.1);
//    //back two are opposite
//
//    //  moveForward(1);
//    if(up&&dow)
//      resetPwm(1,0);
//    if(forw&&bac)
//      resetPwm(0,1);
    pwmcumulate();
    thrusterUpdate();

}
void attachPS2(uint8_t clk , uint8_t cmd, uint8_t att, uint8_t dat)
{
  int error;
  error = control.config_gamepad(clk,cmd,att,dat,true,true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  if(error == 0)
  {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }

  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  //Serial.print(ps2x.Analog(1), HEX);

  //type = ps2x.readType(); 
  Serial.println("About to read Controllwe type.");
  type = control.readType();
  switch(type) 
  {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  case 2:
    Serial.println("GuitarHero Controller Found");
    break;
  }
}
// Main loop
void pwmcumulate()
{
  int i =0;
  for(i=0;i<6;i++)
  {
    Serial.print (pwmpitch[i]);
    Serial.print("      ");
    Serial.print(pwmyaw[i]);
    Serial.print("      ");
    Serial.print(pwmroll[i]);
    Serial.print("      ");
    Serial.print(pwmmove[i]);
    Serial.println("      ");
    pwmarr[i]=pwmpitch[i]+pwmyaw[i]+pwmroll[i]+pwmmove[i];
  }
}
void thrusterUpdate(int fdl,int fdr,int bdl,int bdr,int mdl,int mdr)
{
  flthruster.moveThruster(fdl,1-fdl,pwmarr[0]);
  frthruster.moveThruster(fdr,1-fdr,pwmarr[1]);
  blthruster.moveThruster(bdl,1-bdl,pwmarr[2]);
  brthruster.moveThruster(bdr,1-bdr,pwmarr[3]);
  mrthruster.moveThruster(mdr,1-mdr,pwmarr[5]);
  mlthruster.moveThruster(mdl,1-mdl,pwmarr[4]);
}
void thrusterUpdate()
{
  int j=0;
   for(j=0;j<6;j++)
   {
   Serial.print(pwmarr[j]);
//   pwmarr[j]=00;
   Serial.print("    ");
   }
  Serial.print("pwm after fucker interferes:-");
//  Serial.println(pwmarr[4]);
  flthruster.moveThruster(1,0,pwmarr[0]);
  frthruster.moveThruster(1,0,pwmarr[1]);
  blthruster.moveThruster(1,0,pwmarr[2]);
  brthruster.moveThruster(1,0,pwmarr[3]);
  mrthruster.moveThruster(1,0,pwmarr[5]);
  mlthruster.moveThruster(1,0,pwmarr[4]);

}
void setPoint(float p,float y, float r)
{

  if(type==1)
  {
    control.read_gamepad(false, vibrate);
    float analog_x=calc(control.Analog(PSS_LX));
    float analog_y=calc(control.Analog(PSS_LY));
    float analog_z=calc(control.Analog(PSS_RY));
    if(abs(analog_x)<FLOAT_LIMIT)
      analog_x=0;
    if(abs(analog_y)<FLOAT_LIMIT)
      analog_y=0;
    if(abs(analog_z)<FLOAT_LIMIT)
      analog_z=0;

    if(abs(analog_x)>=FLOAT_LIMIT  | abs(analog_y)>=FLOAT_LIMIT)
    {
      rollsp=map(analog_x,0,255,-1.5,1.5)*r + rollsp*(1-r);
      pitchsp=map(analog_y,0,255,-1.5,1.5)*p+ pitchsp*(1-p);
      yawsp=map(analog_z,0,255,-1.5,1.5)*y + yawsp*(1-y);
    }
  }
  rollsp+=froll;
  pitchsp+=fpitch;
  yawsp+=fyaw;
  //  Serial.print("rollsp ->");
  //  Serial.println(rollsp);
  Serial.print("yawsp ->");
  Serial.println(yawsp);
  //  Serial.print("pitchsp ->");
  //  Serial.println(pitchsp);

}
/*double calc(float x_y)
{
  return x_y-CORDINATE_CONST;
}
*/
void getAngle()
{
  if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    ReadSensors();
    Compass_Heading(); // Calculate magnetic heading

      Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

      Serial.print(yaw);    
     Serial.println("-yaw");
     
    Serial.print(pitch);  
    Serial.println("-pitch");
//    Serial.print("pitchsp");
//    Serial.println(pitchsp);
     Serial.println(roll);   
     Serial.println("-roll");
     /*
     Serial.print(temperature);    
     Serial.println("-temperature");
     Serial.print(pressure);       
     Serial.println("-pressure");
     Serial.print(altitude);       
     Serial.println("-altitude ");
     Serial.println("----------------------");
     */
  }

}
void arrmult(double *ar1,double *ar2,double *resarr,int l,double constant)
{
  for(l=l-1;l>=0;l--)
  {
    *(resarr+l)+=*(ar1 +l) * *(ar2+l)*constant;
  }
}

void setYawBearing()
{
  Serial.print( "yawsp -> ....................................");
  Serial.println(yawsp);
  Serial.println("yaw-> .........................................");
  Serial.println(yaw);
//  pwmyaw[4]=cypid;
//  pwmyaw[5]=aypid*(-1);
  if( yawsp <yaw)
  {
    Serial.println("We are clockwise in yaw");
    Serial.print("cypid -> ");
    Serial.println(cypid);
    Serial.print("aypid ->");
    Serial.println((-1*aypid));

    pwmyaw[4]=cypid;
    pwmyaw[5]=aypid*(-1);
  }
  else if(yawsp>yaw)
  {
    Serial.println("We are anti-clockwise in yaw");
    Serial.print("cypid -> ");
    Serial.println((-1*cypid));
    Serial.print("aypid ->");
    Serial.println(aypid);
    pwmyaw[4]=cypid*(-1);
    pwmyaw[5]=aypid*(1);
  }


}
void setPitchBearing()
{
  if( pitchsp<pitch)
  {
    Serial.println("We are above in pitch");
    Serial.print("fppid -> ");
    Serial.println(fppid);
    Serial.print("bppid ->");
    Serial.println(-bppid);
    pwmpitch[0]=fppid;
    pwmpitch[1]=fppid;
    pwmpitch[2]=bppid*(-1);
    pwmpitch[3]=bppid*(-1);
  }
  else if(pitchsp>pitch)
  {

    Serial.println("We are below in pitch");
    Serial.print("fppid -> ");
    Serial.println(-fppid);
    Serial.print("bppid ->");
    Serial.println(bppid);
    pwmpitch[0]=fppid*(-1);
    pwmpitch[1]=fppid*(-1);
    pwmpitch[2]=bppid;
    pwmpitch[3]=bppid;
  }
}
void setRollBearing()
{
  if(rollsp<roll)
  {
    Serial.println("We are right in roll");
    Serial.print("rrpid -> ");
    Serial.println(rrpid);
    Serial.print("lrpid ->");
    Serial.println(-lrpid);
    pwmroll[1]=rrpid;
    pwmroll[3]=rrpid;
    pwmroll[0]=lrpid*(-1);
    pwmroll[2]=lrpid*(-1);
  }
  else if(rollsp>roll)
  {
    Serial.println("We are left in roll");
    Serial.print("rrpid -> ");
    Serial.println(-rrpid);
    Serial.print("lrpid ->");
    Serial.println(lrpid);
    pwmroll[1]=rrpid*(-1);
    pwmroll[3]=rrpid*(-1);
    pwmroll[0]=lrpid;
    pwmroll[2]=lrpid;
  }
}
/*
void ascend(double veloX)//velox is the fraction of maxVelocity
{
  int j;
  for(j=0;j<4;j++)
  {
    pwmmove[j]=veloX*255;
  }
}
void descend(double veloX)
{
  int j;
  for(j=0;j<4;j++)
  {
    pwmmove[j]=veloX*255*-1;
  } 
}
void  moveForward(double veloX)
{
  pwmmove[4]=veloX*200;
  pwmmove[5]=veloX*200;


}
void moveBackward(double veloX)
{
  pwmmove[4]=veloX*200*-1;
  pwmmove[5]=veloX*200*-1;

}*/
void resetPwm(int a, int m)
{
  int j=0;
  if(a)
  {
    pwmmove[0]=pwmmove[1]=pwmmove[2]=pwmmove[3]=0;
  }
  if(m)
  {
    pwmmove[4]=pwmmove[5]=0;
  }
  thrusterUpdate();
}
void setstaticsetpoint()
{
  int i=0;
  while(i<1000)
  {
    getAngle();
    pitchsp=(pitchsp+pitch);
    yawsp=(yawsp+yaw);
    rollsp=(rollsp+roll);
    i++;
  }
  pitchsp/=i;
  yawsp/=i;
  rollsp/=i;
  Serial.println(pitchsp);
  Serial.println(yawsp);
  Serial.println(rollsp);
  }












