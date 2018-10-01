#include <hal_types.h>

#define PI            3.1415926
#define RAD2DEG       57.2957795
#define DEG2RAD       0.01745329251
#define SAMPLE_PEROID 0.01f

enum {_Q, _R, _P, _VALUE, _K};

void IMU_init();
void UpdateIMU(float gx, float  gy, float  gz, float ax, float ay, float az, float SamplePeriod);
void MadgwickAHRSupdateIMU(float gx, float  gy, float  gz, float ax, float ay, float az, float SamplePeriod);
void quatern2ReverseRotMat(float rotmat[][3]);
void quatern2euler(float * angles);
void quatern2euler360(float * angles);
void rotMat2euler(float rotmat[][3], float *angles);
void kalman_init(float k_state[], float q, float r, float p, float init_v);
void kalman_update(float k_state[], float measurement);
uint16 floatTouint16(float value);
float uint16Tofloat(uint16 value);
float invSqrt(float number);
