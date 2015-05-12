
#ifndef INCLUDE_PID_H_
#define INCLUDE_PID_H_

#define FREQ (250)
#define ZERO (0.000000001)

typedef struct _PID
{
  float Kp;
  float Ki;
  float Kd;
  float dt;

  float iMax;
  float iMin;

  float outMax;
  float outMin;

  float desired;

  float error;
  float lastError;

  float proportional;
  float intergal;
  float derivative;
  float sumError;

  float output;
}PID, *pPID;

extern void pidInit(pPID pid);
extern float pidUpdate(pPID pid, float feedback);
extern float pidReset(pPID pid);

#endif /* INCLUDE_PID_H_ */