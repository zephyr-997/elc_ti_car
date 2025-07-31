#ifndef __PID_H
#define __PID_H

#include "zf_common_typedef.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float error;
	float lasterror;
	float preverror;
	float interror;
	
	float p_out;
	float i_out;
	float d_out;
	float output;
	
	float i_limit;//积分限幅
	float o_limit;//输出限幅
} PID_t;

typedef struct
{
	float kp_s;
	float ki_s;
	float kd_s;
	float kp_t;
	float ki_t;
	float kd_t;
} pid_para;

extern PID_t SpeedPID, TurnPID;
extern volatile pid_para pid_param[2];

void pid_init(PID_t* pid, float kp, float ki, float kd, float i_limit, float o_limit);
void pid_set(PID_t* pid, float kp, float ki, float kd);
void pid_clean(PID_t* pid);

float pid_poisitional(PID_t* pid, float real, float target);
float pid_increment(PID_t* pid, float real, float target);
float pid_poisitional_turnning(PID_t* pid, float position, float GyroZ);

#endif