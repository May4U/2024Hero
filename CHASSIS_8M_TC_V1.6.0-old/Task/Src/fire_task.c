#include "fire_task.h"
/* ************************freertos******************** */

#include "CAN2.h"
fire_task_t fire_task;

void fire_init(void)
{  
    fire_task.Chassis = get_chassis_point();
    fire_task.set_position = 0;
    fire_task.last_set_position = 0;
    fire_task.increat_position = 0;
//    PidInit(&fire_task.Fire_Motor->Speed_PID, 15.0f, 0.0f, 0.5f, Output_Limit);
//	PidInitMode(&fire_task.Fire_Motor->Speed_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
//	
//	PidInit(&fire_task.Fire_Motor->Position_PID, 0.8f, 0.6f, 100.0f, Integral_Limit | Output_Limit);
//	PidInitMode(&fire_task.Fire_Motor->Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
//	PidInitMode(&fire_task.Fire_Motor->Position_PID,Integral_Limit, 150 , 0);

    
//    PidInit(&fire_task.Fire_Motor->Speed_PID, 2.5f, 0.0f, 0.0f, Integral_Limit | Output_Limit | Separated_Integral);
//	PidInitMode(&fire_task.Fire_Motor->Speed_PID,Output_Limit, 16000, 0);	
//	PidInitMode(&fire_task.Fire_Motor->Speed_PID,Integral_Limit, 1500, 0);	
//    PidInitMode(&fire_task.Fire_Motor->Speed_PID,Separated_Integral, 100, -100);	
//    
//	PidInit(&fire_task.Fire_Motor->Position_PID, 0.0599999987f, 0.0f, 0.00400000019f, Integral_Limit | Output_Limit);
//	PidInitMode(&fire_task.Fire_Motor->Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
//	PidInitMode(&fire_task.Fire_Motor->Position_PID,Integral_Limit, 150 , 0);
//    EncoderValZero(&fire_task.Fire_Motor->Motor_encoder);
}

static Fire_State_e ON_OR_OFF = NO_Fire_FORCE;//Ħ���ֿ���״̬
int16_t current_return = 500;
uint8_t duo_num = 0;
uint16_t duode = 0;
int16_t speed = 100;
void fire_work(void)
{
    static uint8_t Fist_set = 1;//�״�����Ŀ��ֵ
    static uint8_t Block = 0;//��ת��־
    static uint16_t On_Fire_Block_time = 0;//������תʱ��
    static uint16_t Re_turn = 0;
    
    if (fire_task.Chassis->Robot_cmd->increat_time > 0)
    {
        duo_num++;
        fire_task.Chassis->Robot_cmd->increat_time--;
        if (fire_task.increat_position != 0)
        {
            fire_task.set_position = fire_task.increat_position;
            fire_task.increat_position = 0;
        }
        else
            fire_task.set_position = fire_task.set_position - (69632) - duode; //����6�ŵ�����һ��ת60�㣬8192*19�����ٱȣ�/6 ��5000�������̳��ȥ������i����г�Ч��������
        
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
    } 
        
   
    //�������г�
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Fire)
    {
        if(fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val < fire_task.set_position + duo_num*duode)//�������г�
        {
            fire_task.set_position = fire_task.set_position+duo_num*duode;//ȡ�������г�
            fire_task.Chassis->Robot_cmd->Fire_State = Ready_Fire;
            fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
            duo_num = 0;
        }
    }
         //λ�û� -current  -speed
    //fire_task.Fire_Motor->current_input = motor_position_speed_control(&fire_task.Fire_Motor->Speed_PID, &fire_task.Fire_Motor->Position_PID, fire_task.set_position, fire_task.Fire_Motor->Motor_encoder.Encode_Record_Val ,fire_task.Fire_Motor->Motor_Information.speed);	
//     fire_task.Fire_lqr.k[0] = k_fire_lqr[0];   
//     fire_task.Fire_lqr.k[1] = k_fire_lqr[1];
//     float Pitch_motortarget = fire_task.set_angle - fire_task.Fire_Motor->Motor_encoder.Record_Angle;
//     double Pitch_system_state[2] = {(-Pitch_motortarget), fire_task.Fire_Motor->Motor_Information.speed};
//     LQR_Data_Update(&fire_task.Fire_lqr, Pitch_system_state);
//     LQR_Calculate(&fire_task.Fire_lqr);
//     fire_task.Fire_Motor->current_input = -fire_task.Fire_lqr.Output[0];
//     value_limit(fire_task.Fire_Motor->current_input,-curent_limit,curent_limit)

    if (fire_task.Chassis->Robot_cmd->Fire_State == NO_Fire_FORCE)
    {//����
        duo_num = 0;
        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;
        fire_task.Chassis->FIRE_MOTOR->current_input = 0;
        EncoderValZero(&fire_task.Chassis->FIRE_MOTOR->Motor_encoder);
        fire_task.set_position = 0;
        fire_task.increat_position = 0;
        fire_task.Chassis->FIRE_MOTOR->set_position = 0;
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Speed_PID);
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Position_PID);
    }else
        fire_task.Chassis->FIRE_MOTOR->Using_PID = Position_Speed_PID;
    
    static uint8_t first_on_empty = 1;
    
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Empty)
    {//�˵�������
        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;
        fire_task.set_position = fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val;
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
        if (first_on_empty == 1)//����δ��ɵ�Ŀ��λ��
        {       
            first_on_empty = 0;            
            fire_task.increat_position = fire_task.set_position;
        }
    }
    else
    {
        first_on_empty = 1;
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
    }
            //fire_task.Fire_Motor->current_input = PidCalculate(&fire_task.Fire_Motor->Speed_PID, speed, fire_task.Fire_Motor->Motor_Information.speed);
}

void Fire_Task(void const *argument)
{
	fire_init();
	while(1)
	{
		taskENTER_CRITICAL(); //�����ٽ���
        Fire_Set();
		fire_work();
		taskEXIT_CRITICAL(); //�˳��ٽ���
		vTaskDelay(1);
	}
}

