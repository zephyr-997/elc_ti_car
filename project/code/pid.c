#include "pid.h"

PID_t SpeedPID; //速度PID
PID_t TurnPID;  //位置PID
volatile pid_para pid_param[2]; //串口控制参数 0-左轮 1-右轮


void pid_init(PID_t* pid, float kp, float ki, float kd, float i_limit, float o_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->i_limit = i_limit;
	pid->o_limit = o_limit;	
	
	pid->error = 0.0f;
	pid->lasterror = 0.0f;
	pid->preverror = 0.0f;
	pid->interror = 0.0f;
	
	pid->p_out = 0.0f;
	pid->i_out = 0.0f;
	pid->d_out = 0.0f;
	pid->output = 0.0f;
}

//位置式PID
float pid_poisitional(PID_t* pid, float real, float target)
{
	pid->error = target - real;
	pid->interror += pid->error;
	
	//积分限幅
	if (pid->interror > pid->i_limit)
	{
		pid->interror = pid->i_limit;
	}
	else if (pid->interror < -pid->i_limit)
	{
		pid->interror = -pid->i_limit;
	}
	
	//线性、积分、微分、前馈共同作用
	pid->p_out = pid->kp * pid->error;
	pid->i_out = pid->ki * pid->interror;
	pid->d_out = pid->kd * (pid->error - pid->lasterror);
	
	pid->output = pid->p_out + pid->i_out + pid->d_out;
	
	pid->lasterror = pid->error;
	
	//输出限幅
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output; 
}

//增量式PID
float pid_increment(PID_t* pid, float real, float target)
{
	pid->error = target - real;
	
	pid->p_out = pid->kp * (pid->error - pid->lasterror);
	pid->i_out = pid->ki * pid->error;
	pid->d_out = pid->kd * (pid->error - 2 * pid->lasterror + pid->preverror);

	pid->output += pid->p_out + pid->i_out + pid->d_out;
	
	pid->preverror = pid->lasterror;
	pid->lasterror = pid->error;
	
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output;
}

//转向环pid
float pid_poisitional_turnning(PID_t* pid, float position, float GyroZ)
{
	pid->p_out = pid->kp * position;
	pid->d_out = pid->kd * GyroZ;

	pid->output = pid->p_out + pid->d_out;
	pid->lasterror = position;
	
	//输出限幅
	if (pid->output > pid->o_limit)
	{
		pid->output = pid->o_limit;
	}
	else if (pid->output < -pid->o_limit)
	{
		pid->output = -pid->o_limit;
	}
	
	return pid->output;
}

void pid_set(PID_t* pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void pid_clean(PID_t* pid)
{
	pid->lasterror = 0;
	pid->interror = 0;
	pid->preverror = 0;
	pid->output = 0;
}

