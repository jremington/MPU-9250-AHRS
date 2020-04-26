//
// Tilt compensated compass  S.J. Remington 3/2020
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
//
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for magneto, use the companion program MPU9250_cal
//
// Below I use the diagonal element of matrix A and ignore the off diagonal components. 
// If those off diagonal terms are large, (most likely only for the magnetometer) 
// add those terms in to the correction routines getMag_cal() and getAcc_cal()
// 
// This version must be compiled with library routines in subfolder "libs"

#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "libs/I2Cdev.cpp"
#include "libs/MPU9250.cpp"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69

MPU9250 accelgyro;
I2Cdev   I2C_M;

// VERY VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The scale constants should *approximately* normalize the vector magnitude to 1.0
//The compass will NOT work well or at all if these are not correct

float A_cal[6] = {515.0, 279.0, 751.0, 5.96e-5, 6.26e-5, 6.06e-5}; // 0..2 offset xyz, 3..5 scale xyz
float M_cal[6] = {18.0, 28.3, -39.6, 0.01403, 0.01414, 0.01387}; // can make both 3x3 to handle off-diagonal corrections

uint8_t buffer_m[6];  //six points magnetometer

//raw data and scaled as vector
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


/*
  This tilt-compensated code assumes that the sensor is oriented with Mag X pointing
  to the North, Y pointing East, and Z pointing down (toward the ground).
  With the MPU9250, the accelerometer is aligned differently, so the accelerometer axes are swapped.

  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/
float p[] = {1, 0, 0};
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(9600);

  // initialize device
  accelgyro.initialize();
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
}
void loop()
{
  getAccel_cal(); //latest accelerometer reading, offset and scale corrected
  getMag_cal(); //ditto for magnetometer
  // correct for accelerometer and magnetometer alignment
  float t=Axyz[1];
  Axyz[1]=Axyz[0]; //swap x and y
  Axyz[0]=t;
  Axyz[2]=-Axyz[2]; //invert z axis
  Serial.println(get_heading(Axyz,Mxyz,p));

  // The MPU-9250 is noisy, so average a few headings for better results
  
  delay(1000);
}
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}


// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3])
{
  float E[3], N[3]; //direction vectors

  // cross "down" (acceleration vector) with magnetic vector (magnetic north + inclination) with  to produce "east"
  vector_cross(acc,mag, E);
  vector_normalize(E);

  // cross "east" with "down" to produce "north" (parallel to the ground)
  vector_cross(E, acc, N);
  vector_normalize(N);

  // compute heading
  int heading = round(atan2(vector_dot(E, p), vector_dot(N, p)) * 180 / M_PI);
  heading=-heading; //conventional nav, heading increases North to East
  if (heading < 0)
    heading += 360;
  return heading; 
}
void getAccel_raw(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
}
void getAccel_cal(void) {
  getAccel_raw();
  for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];
}
void getGyro_scaled(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Gxyz[0] = (float) gx * 250. / 32768; //250 LSB(d/s)
  Gxyz[1] = (float) gy * 250. / 32768;
  Gxyz[2] = (float) gz * 250. / 32768;
}

void getMag_raw(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;
  Mxyz[0] = (float) mx;
  Mxyz[1] = (float) my;
  Mxyz[2] = (float) mz;
}
void getMag_cal(void) {
  getMag_raw();
  for (int i = 0; i < 3; i++) Mxyz[i] = (Mxyz[i] - M_cal[i]) * M_cal[i + 3];
}
