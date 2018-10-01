#include <ioCC2541.h>
#include "OSAL.h"
#include "epl_hal_uart.h"
#include "epl_IMU.h"
#include <math.h>

float Q[4];
//float SamplePeriod;
float twoKi, twoKp;
char str_test[100];
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
float acc_x_angle, acc_y_angle;
volatile float beta = 0.1f;
long count=0;

union {
    float f;
    uint8 b[4];
}u1, u2;

void IMU_init()
{       
        Q[0] = 1.0f;
	Q[1] = 0.0f;
        Q[2] = 0.0f;
        Q[3] = 0.0f;
        //SamplePeriod = SAMPLE_PEROID;
	twoKp = 2.0f * 1.75f; //Kp = 0.5
	twoKi = 2.0f * 0.1f; //Ki = 0.1
}

void UpdateIMU(float gx, float  gy, float  gz, float ax, float ay, float az, float SamplePeriod)
{
	
	//float q[4], norm_acc_vector[3], v[3], e[3], qDot[4];
        float recipNorm;
        float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
        float qa, qb, qc;

        q0q0 = Q[0] * Q[0];
        q0q1 = Q[0] * Q[1];
        q0q2 = Q[0] * Q[2];
        q0q3 = Q[0] * Q[3];
        q1q1 = Q[1] * Q[1];
        q1q2 = Q[1] * Q[2];
        q1q3 = Q[1] * Q[3];
        q2q2 = Q[2] * Q[2];
        q2q3 = Q[2] * Q[3];
        q3q3 = Q[3] * Q[3];      
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
          float halfvx, halfvy, halfvz;
          
          // Normalise accelerometer measurement
          recipNorm = invSqrt(ax * ax + ay * ay + az * az);
          ax *= recipNorm;
          ay *= recipNorm;
          az *= recipNorm;
          
          // Estimated direction of gravity
          halfvx = q1q3 - q0q2;
          halfvy = q0q1 + q2q3;
          //halfvz = q0q0 - 0.5f + q3q3;
          halfvz = 0.5f * (q0q0 - q1q1 - q2q2 + q3q3);
          
          // Error is sum of cross product between estimated direction and measured direction of field vectors
          halfex += (ay * halfvz - az * halfvy);
          halfey += (az * halfvx - ax * halfvz);
          halfez += (ax * halfvy - ay * halfvx); 
        }
        
        // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
        if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
          // Compute and apply integral feedback if enabled
          if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * SamplePeriod;  // integral error scaled by Ki
            integralFBy += twoKi * halfey * SamplePeriod;
            integralFBz += twoKi * halfez * SamplePeriod;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
          }
          else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
          }

          // Apply proportional feedback
          gx += twoKp * halfex;
          gy += twoKp * halfey;
          gz += twoKp * halfez;
        }
        
        // Integrate rate of change of quaternion
        gx *= (0.5f * SamplePeriod);   // pre-multiply common factors
        gy *= (0.5f * SamplePeriod);
        gz *= (0.5f * SamplePeriod);
               
        qa = Q[0];
        qb = Q[1];
        qc = Q[2];
        Q[0] += (-qb * gx - qc * gy - Q[3] * gz);
        Q[1] += (qa * gx + qc * gz - Q[3] * gy);
        Q[2] += (qa * gy - qb * gz + Q[3] * gx);
        Q[3] += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
        Q[0] *= recipNorm;
        Q[1] *= recipNorm;
        Q[2] *= recipNorm;
        Q[3] *= recipNorm;
        
        q0q0 = Q[0] * Q[0];
        q0q1 = Q[0] * Q[1];
        q0q2 = Q[0] * Q[2];
        q0q3 = Q[0] * Q[3];
        q1q1 = Q[1] * Q[1];
        q1q2 = Q[1] * Q[2];
        q1q3 = Q[1] * Q[3];
        q2q2 = Q[2] * Q[2];
        q2q3 = Q[2] * Q[3];
        q3q3 = Q[3] * Q[3]; 
        
        //sprintf(str_test,"%f, %f, %f,  ",gyro[0],gyro[1],gyro[2]);
        //uartWriteString(str_test);
        
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float SamplePeriod) {
	float recipNorm;
     // float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	/*if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		UpdateIMU(gx, gy, gz, ax, ay, az,SamplePeriod);
		return;
	}
*/
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = Q[0] * Q[0];
        q0q1 = Q[0] * Q[1];
        q0q2 = Q[0] * Q[2];
        q0q3 = Q[0] * Q[3];
        q1q1 = Q[1] * Q[1];
        q1q2 = Q[1] * Q[2];
        q1q3 = Q[1] * Q[3];
        q2q2 = Q[2] * Q[2];
        q2q3 = Q[2] * Q[3];
        q3q3 = Q[3] * Q[3];       

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex *  SamplePeriod;	// integral error scaled by Ki
			integralFBy += twoKi * halfey *  SamplePeriod;  //1.0 delete
			integralFBz += twoKi * halfez * SamplePeriod;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * SamplePeriod);		// pre-multiply common factors
	gy *= (0.5f * SamplePeriod);   //1.0 delete
	gz *= (0.5f * SamplePeriod);
	qa = Q[0];
        qb = Q[1];
        qc = Q[2];

        Q[0] += (-qb * gx - qc * gy - Q[3] * gz);
        Q[1] += (qa * gx + qc * gz - Q[3] * gy);
        Q[2] += (qa * gy - qb * gz + Q[3] * gx);
        Q[3] += (qa * gz + qb * gy - qc * gx);
	// Normalise quaternion
	recipNorm = invSqrt( Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= recipNorm;
        Q[1] *= recipNorm;
        Q[2] *= recipNorm;
        Q[3] *= recipNorm;
        
        q0q0 = Q[0] * Q[0];
        q0q1 = Q[0] * Q[1];
        q0q2 = Q[0] * Q[2];
        q0q3 = Q[0] * Q[3];
        q1q1 = Q[1] * Q[1];
        q1q2 = Q[1] * Q[2];
        q1q3 = Q[1] * Q[3];
        q2q2 = Q[2] * Q[2];
        q2q3 = Q[2] * Q[3];
        q3q3 = Q[3] * Q[3]; 
}





void MadgwickAHRSupdateIMU(float gx, float  gy, float  gz, float ax, float ay, float az, float SamplePeriod) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-Q[1] * gx - Q[2] * gy - Q[3] * gz);
	qDot2 = 0.5f * (Q[0] * gx + Q[2] * gz - Q[3] * gy);
	qDot3 = 0.5f * (Q[0] * gy - Q[1] * gz + Q[3] * gx);
	qDot4 = 0.5f * (Q[0] * gz + Q[1] * gy - Q[2] * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * Q[0];
		_2q1 = 2.0f * Q[1];
		_2q2 = 2.0f * Q[2];
		_2q3 = 2.0f * Q[3];
		_4q0 = 4.0f * Q[0];
		_4q1 = 4.0f * Q[1];
		_4q2 = 4.0f * Q[2];
		_8q1 = 8.0f * Q[1];
		_8q2 = 8.0f * Q[2];
		q0q0 = Q[0] * Q[0];
		q1q1 = Q[1] * Q[1];
		q2q2 = Q[2] * Q[2];
		q3q3 = Q[3] * Q[3];

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * Q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * Q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * Q[3] - _2q1 * ax + 4.0f * q2q2 * Q[3] - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	Q[0] += qDot1 * SamplePeriod;
	Q[1] += qDot2 * SamplePeriod;
	Q[2] += qDot3 * SamplePeriod;
	Q[3] += qDot4 * SamplePeriod;

	// Normalise quaternion
	recipNorm = invSqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= recipNorm;
	Q[1] *= recipNorm;
	Q[2] *= recipNorm;
	Q[3] *= recipNorm;
        
        q0q0 = Q[0] * Q[0];
        q0q1 = Q[0] * Q[1];
        q0q2 = Q[0] * Q[2];
        q0q3 = Q[0] * Q[3];
        q1q1 = Q[1] * Q[1];
        q1q2 = Q[1] * Q[2];
        q1q3 = Q[1] * Q[3];
        q2q2 = Q[2] * Q[2];
        q2q3 = Q[2] * Q[3];
        q3q3 = Q[3] * Q[3]; 
}

void quatern2ReverseRotMat(float rotmat[][3])
{
//    rotmat[0][0] = 2*( Q[0]*Q[0] + Q[1]*Q[1] ) - 1;
//    rotmat[0][1] = 2*( Q[1]*Q[2] + Q[0]*Q[3] );
//    rotmat[0][2] = 2*( Q[1]*Q[3] - Q[0]*Q[2] );
//    rotmat[1][0] = 2*( Q[1]*Q[2] - Q[0]*Q[3]);
//    rotmat[1][1] = 2*( Q[0]*Q[0] + Q[2]*Q[2] ) - 1;
//    rotmat[1][2] = 2*( Q[2]*Q[3] + Q[0]*Q[1] );
//    rotmat[2][0] = 2*( Q[1]*Q[3] + Q[0]*Q[2] );
//    rotmat[2][1] = 2*( Q[2]*Q[3] - Q[0]*Q[1]);
//    rotmat[2][2] = 2*( Q[0]*Q[0] + Q[3]*Q[3] ) - 1;
     
    rotmat[0][0] = 2*( q0q0 + q1q1 ) - 1;
    rotmat[0][1] = 2*( q1q2 - q0q3 );
    rotmat[0][2] = 2*( q1q3 + q0q2 );
    rotmat[1][0] = 2*( q1q2 + q0q3);
    rotmat[1][1] = 2*( q0q0 + q2q2 ) - 1;
    rotmat[1][2] = 2*( q2q3 - q0q1 );
    rotmat[2][0] = 2*( q1q3 - q0q2 );
    rotmat[2][1] = 2*( q2q3 + q0q1);
    rotmat[2][2] = 2*( q0q0 + q3q3 ) - 1;
//  
//    rotmat[0][0] = 1 - 2*( q2q2 + q3q3 );
//    rotmat[0][1] = 2*( q1q2 + q0q3 );
//    rotmat[0][2] = 2*( q1q3 - q0q2 );
//    rotmat[1][0] = 2*( q1q2 - q0q3);
//    rotmat[1][1] = 1 - 2*( q0q0 + q3q3 );
//    rotmat[1][2] = 2*( q2q3 + q0q1 );
//    rotmat[2][0] = 2*( q1q3 + q0q2 );
//    rotmat[2][1] = 2*( q2q3 - q0q1);
//    rotmat[2][2] = 1 - 2*( q1q1 + q2q2 ); 

}


void quatern2euler(float * angles)
{
  
  /*angles[0] = atan2(2 * q1q2 - 2 * q0q3, 2 * q0q0 + 2 * q1q1 - 1); // psi roll
  angles[1] = -asin(2 * q1q3 + 2 * q0q2); // theta pitch
  angles[2] = atan2(2 * q2q3 - 2 * q0q1, 2 * q0q0 + 2 * q3q3 - 1); // phi yaw*/
  angles[0] = atan2(2 * q1q2 + 2 * q0q3, -2 * q2q2 - 2 * q3q3 + 1); // yaw
  angles[1] = -asin(2 * q1q3 - 2 * q0q2);// pitch
  angles[2] = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1);
 
  angles[0] *= RAD2DEG;
  angles[1] *= RAD2DEG;
  angles[2] *= RAD2DEG;

}

void quatern2euler360(float * angles) {
  float m11, m12, m21, m31, m32;
  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q1q3 - q0q2);
  gy = 2 * (q0q1 + q2q3);
  gz = q0q0 - q1q1 - q2q2 + q3q3;
    
//  m11 = 2.*(q1q2 + q0q3);
//  m12 = q0q0 + q1q1 - q2q2 - q3q3;
//  m21 = -2*(q1q3 - q0q2);               
//  m31 = 2*(q2q3 + q0q1);              
//  m32 = q0q0 - q1q1 - q2q2 + q3q3;

  angles[0] = atan2(2 * q1q2 - 2 * q0q3, 2 * q0q0 + 2 * q1q1 - 1); // psi roll
  angles[1] = -asin(2 * q1q3 + 2 * q0q2); // theta pitch
  angles[2] = atan2(2 * q2q3 - 2 * q0q1, 2 * q0q0 + 2 * q3q3 - 1); // phi yaw
 
  angles[0] *= RAD2DEG;
  angles[1] *= RAD2DEG;
  angles[2] *= RAD2DEG;
  
  // find angles for rotations about X, Y, and Z axes
  angles[2] = -atan2( m11, m12 ) * 57.2957795;
  angles[1] = -asin( m21 ) * 57.2957795;
  angles[0] = -atan2( m31, m32 ) * 57.2957795;
    
  //  	Gx	gy	gz
  //0-90	"+"		"+"
  //90-180	"+"		"-"
  //180-270	"-"		"-"
  //270-360	"-"		"+"
    
  if(gx >= 0 && gz < 0)
      angles[1] = 180 - angles[1];
     else if(gx < 0 && gz < 0)
       angles[1] = 180 - angles[1];
      else if(gx < 0 && gz >=0)
        angles[1] = 360 + angles[1];
        
  if(angles[0] < 0) angles[0] = 360 + angles[0];
  if(angles[2] < 0) angles[2] = 360 + angles[2];
  
  angles[2] = 360 - angles[2];
  
}

void rotMat2euler(float R[][3], float *angles)
{
  angles[0] = atan2(R[2][1],R[2][2]);
  angles[1] = -atan( R[2][0] / invSqrt(1-R[2][0]*R[2][0]) );
  angles[2] = atan2(R[1][0],R[0][0]);
  
  angles[0] *= 180/PI;
  angles[1] *= 180/PI;
  angles[2] *= 180/PI;
}

void kalman_init(float k_state[], float q, float r, float p, float init_v)
{
     k_state[_Q] = q;
     k_state[_R] = r;
     k_state[_P] = p;
     k_state[_VALUE] = init_v;
}

void kalman_update(float k_state[], float measurement)
{
   k_state[_P] = k_state[_P] + k_state[_Q];
   
   k_state[_K] = k_state[_P] / (k_state[_P] + k_state[_R]);
   k_state[_VALUE] = k_state[_VALUE] + k_state[_K] * (measurement - k_state[_VALUE]);
   k_state[_P] = (1 - k_state[_K]) * k_state[_P];
}

uint16 floatTouint16(float value) {
       
       uint16 result=0;
       uint8 temp, r_H=0, r_L=0;
       
       u1.f = value;
       r_H |= u1.b[3]&0x80; //sign
       temp = u1.b[3]<<1 | (u1.b[2]&0x80)-0x7F; //exp
       r_H |= temp<<4 | ( u1.b[2]>>3 & 0x0F ); //exp & fraction
       r_L |= u1.b[2]<<5 | u1.b[1]>>3;
       
      
       return ((uint16)r_H<<8) | r_L;
       
}

float uint16Tofloat(uint16 value) {

      uint8 v_H, v_L, exp;
  
      v_H = (uint8)(value>>8);
      v_L = (uint8)value & 0xFF;
      
      exp = ((v_H&0x70)>>4) + 0x7F; //calculate exp
      u2.b[3] = v_H&0x80 | exp>>1; //sign & exp
      u2.b[2] = exp<<7 | (v_H&0x0F)<<3 | v_L>>5;
      u2.b[1] = v_L<<3;
      u2.b[0] = 0;

      return u2.f;     
}


float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

