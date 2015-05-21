
#include "header.h"

void pidInit(pPID pid)
{
  memset(pid, 0, sizeof(PID));
  pid->Kp = 1;
  pid->Ki = 0;
  pid->Kd = 0;

  pid->iMax =  128000;
  pid->iMin = -128000;

  pid->outMax =  64000;
  pid->outMin = -64000;

  pid->dt = (float)1/FREQ;
}

float pidUpdate(pPID pid, float feedback)
{
  pid->error = pid->desired - feedback;
  pid->sumError += pid->error;

  pid->proportional = pid->Kp * pid->error;
  pid->intergal     = pid->Ki * pid->sumError * pid->dt;

  if(pid->dt > ZERO)
  {
    pid->derivative   = pid->Kd * (pid->error - pid->lastError) / pid->dt;
  }

  if(pid->intergal > pid->iMax)  { pid->intergal = pid->iMax; }
  if(pid->intergal < pid->iMin)  { pid->intergal = pid->iMin; }

  pid->output = pid->proportional + pid->intergal + pid->derivative;

  if(pid->output > pid->outMax)  { pid->output = pid->outMax; }
  if(pid->output < pid->outMin)  { pid->output = pid->outMin; }

  pid->lastError = pid->error;
  return pid->output;
}

float pidReset(pPID pid)
{
  pid->error = 0;
  pid->lastError = 0;
  pid->sumError = 0;

  pid->proportional = 0;
  pid->intergal = 0;
  pid->derivative = 0;
}

float pidSetKp(pPID pid, float Kp)
{
  pid->Kp = Kp;
}

float pidSetKi(pPID pid, float Ki)
{
  pid->Ki = Ki;
}

float pidSetKd(pPID pid, float Kd)
{
  pid->Kd = Kd;
}
