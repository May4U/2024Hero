#include "fire_Task.h"
#include "bsp_dr16.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"


#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "gimbal_config.h"
#include "bsp_referee.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "PC_VisionPro.h"

Fire_t Fire;
//引用来自gimbal_task任务变量
extern uint8_t pitch_motor_is_online;
extern uint8_t yaw_motor_is_online;

static void fire_task_init(void);
static void fire_pid_calculate(void);
static void fire_behaviour_choose(void);
static void Shoot_Check(void);
int16_t left_speed = 0;
int16_t right_speed = 0;
void FIRE_TASK(void const *argument)
{
	fire_task_init();
	while (1)
	{
        Gimbal_Fire_State_Set();
		fire_behaviour_choose();
		fire_pid_calculate();
        left_speed = Fire.left_motor->Motor_Information.speed;
        right_speed = -Fire.right_motor->Motor_Information.speed;
        Shoot_Check();
		DJIMotor_Send(Fire.left_motor);
		vTaskDelay(1);
	}
}

void fire_task_init(void)
{
	memset(&Fire, 0, sizeof(Fire_t));
	// 获得拨弹指针
    Fire.left_motor  = DJIMotor_Init(1,1,true,M3508,30);
    Fire.right_motor = DJIMotor_Init(1,2,false,M3508,30);

    Fire.left_motor->Using_PID = Speed_PID;
    Fire.right_motor->Using_PID = Speed_PID;
    
    PidInit(&Fire.left_motor->Speed_PID,3,0,0,Output_Limit);
 	PidInitMode(&Fire.left_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.left_motor->Speed_PID,StepIn,2,0);//过大的加减速会导致发射机构超电流断电
    PidInit(&Fire.right_motor->Speed_PID,3,0,0,Output_Limit);
 	PidInitMode(&Fire.right_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.right_motor->Speed_PID,StepIn,2,0);//逐渐减速，实测发现最大速度减速会导致发射机构超电流断电
	Fire.Gimbal_CMD = Get_Gimbal_CMD_point();
}
//4900
//3500
int16_t FIRE_SPEED_10_ = 5800;//5000;//5688;
int16_t FIRE_SPEED_16_ = 5800;//5000;//5688;

int16_t speed_left;
int16_t speed_righ;
uint8_t speed = 3;
void fire_behaviour_choose(void)
{
    static int16_t low_speed = 0;
    
    if (Fire.Gimbal_CMD->Fire_Ready == 0)
    {
        //if (low_speed > 0)
        //{
        //    low_speed-=10;
            DJIMotor_Set_val(Fire.left_motor, low_speed);
            DJIMotor_Set_val(Fire.right_motor, low_speed);
        //}

        return;
    }
   // 裁判系统弹速设置
	switch (Fire.Gimbal_CMD->shooter_id1_42mm_speed_limit)
	{
	case 10:
        low_speed = FIRE_SPEED_10_;
		DJIMotor_Set_val(Fire.left_motor, FIRE_SPEED_10_);
		DJIMotor_Set_val(Fire.right_motor, FIRE_SPEED_10_);
		break;
	case 16:
        low_speed = FIRE_SPEED_16_;
		DJIMotor_Set_val(Fire.left_motor, FIRE_SPEED_16_);
		DJIMotor_Set_val(Fire.right_motor, FIRE_SPEED_16_);
		break;
	default:
        low_speed = FIRE_SPEED_16_;
		DJIMotor_Set_val(Fire.left_motor, FIRE_SPEED_10_);
		DJIMotor_Set_val(Fire.right_motor, FIRE_SPEED_10_);
		break;
	}

}

void fire_pid_calculate(void)
{
    if (Fire.Gimbal_CMD->Fire_Ready == 0)
    {
        Fire.left_motor->set_speed = 0;
        Fire.right_motor->set_speed = 0;
    }
    speed_left = Fire.left_motor->Motor_Information.speed;
    speed_righ = -Fire.right_motor->Motor_Information.speed;
    Fire.left_motor->current_input = motor_speed_control(&Fire.left_motor->Speed_PID,
                                                          Fire.left_motor->set_speed,
                                                          Fire.left_motor->Motor_Information.speed);
    Fire.right_motor->current_input = motor_speed_control(&Fire.right_motor->Speed_PID,
                                                          Fire.right_motor->set_speed,
                                                          Fire.right_motor->Motor_Information.speed);
}
uint16_t fla_speed = 100;
void Shoot_Check(void)
{
    if (Fire.right_motor->set_speed != FIRE_SPEED_16_)   return;
    if (Fire.right_motor->Motor_Information.speed < FIRE_SPEED_16_ - fla_speed)
    {
        Flash_fire_bias_time(1);
    }
}