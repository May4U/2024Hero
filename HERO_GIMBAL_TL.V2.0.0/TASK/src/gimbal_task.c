/*--------------------- FIRMWARE --------------------*/
#include "string.h"
#include "usbd_cdc_if.h"
/*--------------------- TASK --------------------*/
#include "imu_task.h"
#include "gimbal_task.h"
#include "fire_Task.h"
#include "virtual_task.h"
#include "safe_task.h"
/*--------------------- COMMUINICATE --------------------*/
#include "bsp_can.h"
/*--------------------- CONTROL --------------------*/
#include "robot_cmd.h"
#include "DJI_Motor.h"
/*--------------------- ALGORITHM --------------------*/
#include "pid.h"
#include "maths.h"
#include "filter.h"
#include "lqr.h"
#define ABS(x) (((x)>0) ? (x) : (-x))
/*--------------------- BSP --------------------*/
typedef struct{
    const Gimbal_CMD_t* Gimbal_CMD;
    const INS_t         *ins;
    
    DJIMotor_object_t   *YAW_Motor;
    DJIMotor_object_t   *PITCH_Motor;
    //陀螺仪LQR
    LQR_t lqr_pitch;
    LQR_t lqr_yaw;
    
    LQR_int_t lqr_encoder_pitch;
    LQR_int_t lqr_encoder_yaw;
    pid_parameter_t     yaw_only_i_pid;
    //编码值PID
    pid_parameter_t      PITCH_Motor_Speed_PID;
    pid_parameter_t      PITCH_Motor_Position_PID;   
    pid_parameter_t      YAW_Motor_SPEED_PID;
    pid_parameter_t      YAW_Motor_Position_PID;
}Gimbal_t;
Gimbal_t Gimbal;

static void Gimbal_Work();
static void Gimbal_Init();
static void Gimbal_Not_Work();


//手瞄lqr参数
 float k_pitch_lqr[2] = {80.0f, 1.5f};//{160.0f, 2.0f};
 float k_yaw_lqr[2] = {-80.0f, -1.0f};
//自瞄lqr参数
 float k_virtual_pitch_lqr[2] = {60.0f, 1.2f};
 float k_virtual_yaw_lqr[2] = {-40.0f, -1.0f};
//狙击lqr参数
 float k_encoder_pitch_lqr[2] = {-800,-4};
 float k_encoder_yaw_lqr[2] = {-450,-2};

/*电机在线状态*/
uint8_t pitch_motor_is_online = 0;
void pitch_motor_online(void){pitch_motor_is_online = 1;}
void pitch_motor_disconnect(void){pitch_motor_is_online = 0;}
void pitch_can_rx_CallBack(void){Safe_Task_online_name_("Pitch_Motor_Safe");
    }

uint8_t yaw_motor_is_online = 0;
void yaw_motor_online(void){yaw_motor_is_online = 1;}
void yaw_motor_disconnect(void){yaw_motor_is_online = 0;}
void yaw_can_rx_CallBack(void){//Safe_Task_online_name_("Yaw_Motor_Safe");
    }


/**
 * @brief  云台主任务
 * @param
 * @retval void
 */
void GIMBAL_TASK(void const * argument)
{    
    while(Get_Virtual_task_init_falg() == 0)
        vTaskDelay(1);//等待视觉任务初始化

    Gimbal_Init();
    while (1)
    {
        taskENTER_CRITICAL();         // 进入临界区
        
        Gimbal_Deal_TLcontrol();  //处理图传遥控信息
        //if(pitch_motor_is_online)
            Gimbal_Work();      //云台正常工作
//        else
//            Gimbal_Not_Work();  //云台紧急状态
        
        DJMotor_Send_only_one(Gimbal.PITCH_Motor);
        DJMotor_Send_only_one(Gimbal.YAW_Motor);

		taskEXIT_CRITICAL();              // 退出临界区
        vTaskDelay(1); // 绝对延时//vTaskDelay(2);
        
    }
}


void Gimbal_Init()
{
    Gimbal.Gimbal_CMD = Get_Gimbal_CMD_point();
    Gimbal.ins = get_imu_control_point();
    Gimbal.YAW_Motor = DJIMotor_Init(2,1,false,GM6020,3600/2/PI);
    Gimbal.PITCH_Motor = DJIMotor_Init(1,5,false,GM6020,3600/2/PI);
    //YAW PID初始化
    PidInit(&Gimbal.PITCH_Motor_Speed_PID,150.f,0.f,12.0f,Integral_Limit | Output_Limit );
    PidInitMode(&Gimbal.PITCH_Motor_Speed_PID,Integral_Limit,10000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.PITCH_Motor_Speed_PID,Output_Limit,30000,0);//输出限幅模式设置
    
    PidInit(&Gimbal.PITCH_Motor_Position_PID,1.0f,0,0.25f,Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, Output_Limit, 200, 0);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, StepIn, 30, 0);
    
    PidInit(&Gimbal.YAW_Motor_SPEED_PID,200.0f,0.0f,15.0f,Output_Limit | Integral_Limit );//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.YAW_Motor_SPEED_PID,Integral_Limit,1000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.YAW_Motor_SPEED_PID,Output_Limit,25000,0);//输出限幅模式设置
    
    PidInit(&Gimbal.YAW_Motor_Position_PID,2.5f,0,0.25f,Output_Limit);//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.YAW_Motor_Position_PID,Output_Limit,200,0);//输出限幅模式设置
    
    PidInit(&Gimbal.yaw_only_i_pid,0.0f,100.0f,0.0f,Output_Limit | Integral_Limit | Separated_Integral);//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.yaw_only_i_pid,Integral_Limit,1000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.yaw_only_i_pid,Output_Limit,5000,0);//输出限幅模式设置
    PidInitMode(&Gimbal.yaw_only_i_pid,Separated_Integral,0.8,-0.8);
    //LQR初始化
    LQR_Init(&Gimbal.lqr_pitch, 2, 1, k_pitch_lqr);
    LQR_Init(&Gimbal.lqr_yaw, 2, 1, k_yaw_lqr);
    
    LQR_Int_Init(&Gimbal.lqr_encoder_pitch, 2, 1, k_virtual_pitch_lqr);
    LQR_Int_Init(&Gimbal.lqr_encoder_yaw, 2, 1, k_virtual_yaw_lqr);
    //创建安全任务
    Safe_task_add("Pitch_Motor_Safe", 30, pitch_motor_disconnect, pitch_motor_online);
//    Safe_task_add("Yaw_Motor_Safe", 30, yaw_motor_disconnect, yaw_motor_online);
    
    DJIMotor_CanRx_Callback(Gimbal.PITCH_Motor, pitch_can_rx_CallBack);//接收到Can信号时刷新安全任务
    DJIMotor_CanRx_Callback(Gimbal.YAW_Motor, yaw_can_rx_CallBack);
}

/**
  *@brief 根据电机绝对编码值判断IMU的Pitch目标角度可以增加减少多少
  */
fp32 UP_ANGLE = 0;
fp32 DOWN_ANGLE = 0;
fp32 UP_jia = 0.0f;
fp32 DOWN_jian = 3.0f;//6;
void IMU_PITCH_Angle_Set_Limit(fp32 *IMU_Pitch_Up_Limit, fp32 *IMU_Pitch_Down_Limit)
{
    //计算电机绝对角度可以还能增加减少多少
     fp32 Motor_Can_increased_Pitch_Angle = Gimbal.PITCH_Motor->Motor_encoder.Actual_Angle - PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE ;
     fp32 Motor_Can_reduced_Pitch_Angle   = PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE - Gimbal.PITCH_Motor->Motor_encoder.Actual_Angle ;
    *IMU_Pitch_Up_Limit   = Gimbal.ins->Pitch + Motor_Can_increased_Pitch_Angle;
    *IMU_Pitch_Down_Limit = Gimbal.ins->Pitch - Motor_Can_reduced_Pitch_Angle;
    //编码值有时候会发癫，整整少了8192编码值
    if (*IMU_Pitch_Up_Limit < -200)        *IMU_Pitch_Up_Limit+=360;
    if (*IMU_Pitch_Down_Limit < -200)      *IMU_Pitch_Down_Limit+=360;
    //Pitch电机数据未更新，按照原先限幅会出错，强制写角度
    if (pitch_motor_is_online == 0)
    {
        *IMU_Pitch_Up_Limit = 10;
        *IMU_Pitch_Down_Limit = -10;
    }        
}
float Pitch_output_torque;
int16_t pitch_current;
float Yaw_system_state[2];
int16_t yaw_speed_set;

int64_t pitch_motor_up_min_actual_encoder ;
int64_t pitch_motor_down_max_actual_encoder;
fp32    pitch_motor_up_min_actual_angle;
fp32    pitch_motor_down_max_actual_angle;
void Gimbal_Work()
{
    /*等待IMU数据*/
    static uint8_t imu_ok_falg = 0;
    if (Gimbal.ins->imu_ok_falg > 2000) imu_ok_falg = 1;
    if (imu_ok_falg == 0)   return;//IMU任务先运行会，不然PITCH没数据

    Gimbal_Deal_TLcontrol();
    IMU_PITCH_Angle_Set_Limit(&UP_ANGLE,&DOWN_ANGLE);
    UP_ANGLE += UP_jia;
    DOWN_ANGLE -= DOWN_jian;
    /*PITCH轴 锁IMU || 锁ENCODE  状态选择*/
    if (Gimbal.Gimbal_CMD->Gimbal_Work_State == OUT_POST_AUTO_ATTACK)//前哨站模式，改为锁编码值
    {
        if (ABS(Gimbal.ins->Pitch - Gimbal.Gimbal_CMD->Pitch_Set_IMU) < 8.0f && Gimbal.Gimbal_CMD->Pitch_Lock_type == IMU)
        {//达到目标IMU PITCH
            Gimbal_Pitch_Lock(ENCODER);
            Gimbal_Pitch_Set_Encoder(Gimbal.PITCH_Motor->Motor_encoder.Encode_Record_Val);
        }
        if (ABS(Gimbal.ins->Yaw - Gimbal.Gimbal_CMD->Yaw_Set_IMU) < 8.0f && Gimbal.Gimbal_CMD->Yaw_Lock_type == IMU)
        {//达到目标IMU PITCH
            Gimbal_Yaw_Lock(ENCODER);
            Gimbal_Yaw_Set_Encoder(Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val);
        }
    }
    else//改为锁IMU
    {
        if (Gimbal.Gimbal_CMD->Pitch_Lock_type == ENCODER)
        {
            Gimbal_Pitch_Lock(IMU);
            Gimbal_Pitch_Set_IMU(Gimbal.ins->Pitch);
        }
        if (Gimbal.Gimbal_CMD->Yaw_Lock_type == ENCODER)
        {
            Gimbal_Yaw_Lock(IMU);
            Gimbal_Yaw_Set_IMU(Gimbal.ins->Yaw);
        }    
    }
    
    /*根据锁的类型*/
    /*PITCH*/
//    pitch_motor_up_min_actual_encoder = PITCH_MOTOR_UP_MIN_ACTUAL_ENCODER + 200;
//    pitch_motor_down_max_actual_encoder = PITCH_MOTOR_DOWN_MAX_ACTUAL_ENCODER + 100;
    
    pitch_motor_up_min_actual_encoder = PITCH_MOTOR_UP_MIN_ACTUAL_ENCODER;
    pitch_motor_down_max_actual_encoder = PITCH_MOTOR_DOWN_MAX_ACTUAL_ENCODER;
    pitch_motor_up_min_actual_angle = PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE;
    pitch_motor_down_max_actual_angle = PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE;
    
    if (Gimbal.PITCH_Motor->Motor_encoder.Encode_Actual_Val<4000)
    {
        pitch_motor_up_min_actual_encoder -= 8192;
        pitch_motor_down_max_actual_encoder -= 8192;
    }
    
    if (Gimbal.Gimbal_CMD->Pitch_Limit_Max_Flag == 1)
    {
        pitch_motor_up_min_actual_angle += 10;
        UP_ANGLE += 10;
    }
        
    if (Gimbal.Gimbal_CMD->Pitch_Lock_type == ENCODER)//锁电机
    {
        
        Gimbal_CMD_Set(pitch_motor_up_min_actual_encoder,pitch_motor_down_max_actual_encoder);
        
//        int16_t pitch_current;
//        pitch_current = motor_position_speed_control(&Gimbal.PITCH_Motor_Speed_PID, 
//                                             &Gimbal.PITCH_Motor_Position_PID, 
//                                             Gimbal.Gimbal_CMD->Pitch_Set_Encoder,
//                                             Gimbal.PITCH_Motor->Motor_encoder.Encode_Record_Val,
//                                             Gimbal.PITCH_Motor->Motor_Information.speed);	
//        Gimbal.PITCH_Motor->voltage_input = pitch_current;
        //角度限幅
        Gimbal.lqr_encoder_pitch.k[0] = k_encoder_pitch_lqr[0];
        Gimbal.lqr_encoder_pitch.k[1] = k_encoder_pitch_lqr[1];

        int64_t Pitch_motortarget = Gimbal.Gimbal_CMD->Pitch_Set_Encoder - Gimbal.PITCH_Motor->Motor_encoder.Encode_Record_Val;
        int64_t Pitch_system_state[2] = {(-Pitch_motortarget), Gimbal.PITCH_Motor->Motor_Information.speed};
        LQR_Int_Data_Update(&Gimbal.lqr_encoder_pitch, Pitch_system_state);
        LQR_Int_Calculate(&Gimbal.lqr_encoder_pitch);
        Gimbal.PITCH_Motor->voltage_input = Gimbal.lqr_encoder_pitch.Output[0];

        value_limit(Gimbal.PITCH_Motor->voltage_input,25000,-25000)

//        if((Gimbal.PITCH_Motor->Motor_encoder.Encode_Actual_Val  > (pitch_motor_down_max_actual_encoder+100) && Gimbal.Gimbal_CMD->Pitch_Set_Encoder > Gimbal.PITCH_Motor->Motor_encoder.Encode_Actual_Val) 
//         ||(Gimbal.PITCH_Motor->Motor_encoder.Encode_Actual_Val  < (pitch_motor_up_min_actual_encoder-100) && Gimbal.Gimbal_CMD->Pitch_Set_Encoder < Gimbal.PITCH_Motor->Motor_encoder.Encode_Actual_Val) )
//        {
//            Gimbal.PITCH_Motor->voltage_input = 0;
//        }
    }

    else//锁imu
    {
        Gimbal_CMD_Set(UP_ANGLE,DOWN_ANGLE);//陀螺仪上下极限

        float Pitch_motortarget = Gimbal.Gimbal_CMD->Pitch_Set_IMU - Gimbal.ins->Pitch;
        float Pitch_system_state[2] = {((-Pitch_motortarget) / 57.295779513f), Gimbal.ins->Gyro[0]};
        LQR_Data_Update(&Gimbal.lqr_pitch, Pitch_system_state);
        LQR_Calculate(&Gimbal.lqr_pitch);
        
        Pitch_output_torque = Gimbal.lqr_pitch.Output[0];
        Gimbal.PITCH_Motor->voltage_input = torque_to_voltage_6020(Pitch_output_torque);  
        
        value_limit(Gimbal.PITCH_Motor->voltage_input,-25000,25000);
        
//        if (Gimbal.PITCH_Motor->Motor_encoder.Actual_Angle < (pitch_motor_up_min_actual_angle-5.0f) && Gimbal.Gimbal_CMD->Pitch_Set_IMU > Gimbal.ins->Pitch)
//            Gimbal.PITCH_Motor->voltage_input = 0;
//        if (Gimbal.PITCH_Motor->Motor_encoder.Actual_Angle > (pitch_motor_down_max_actual_angle+5.0f) && Gimbal.Gimbal_CMD->Pitch_Set_IMU < Gimbal.ins->Pitch)
//            Gimbal.PITCH_Motor->voltage_input = 0;
    }

    /*YAW*/
    if (Gimbal.Gimbal_CMD->Yaw_Lock_type == ENCODER)//锁电机
    {   
         yaw_speed_set = PidCalculate(&Gimbal.YAW_Motor_Position_PID, Gimbal.Gimbal_CMD->Yaw_Set_Encoder, Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val);
        
        Gimbal.YAW_Motor->voltage_input = PidCalculate(&Gimbal.YAW_Motor_SPEED_PID, yaw_speed_set, Gimbal.YAW_Motor->Motor_Information.speed);
//        Gimbal.lqr_encoder_yaw.k[0] = k_encoder_yaw_lqr[0];
//        Gimbal.lqr_encoder_yaw.k[1] = k_encoder_yaw_lqr[1];

//        int64_t Yaw_motortarget = Gimbal.Gimbal_CMD->Yaw_Set_Encoder - Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val;
//        int64_t Yaw_system_state[2] = {(-Yaw_motortarget), Gimbal.YAW_Motor->Motor_Information.speed};
//        LQR_Int_Data_Update(&Gimbal.lqr_encoder_yaw, Yaw_system_state);
//        LQR_Int_Calculate(&Gimbal.lqr_encoder_yaw);
//        
//        Gimbal.YAW_Motor->voltage_input = Gimbal.lqr_encoder_yaw.Output[0];
        
        value_limit(Gimbal.PITCH_Motor->voltage_input,-25000,25000);
    }
    else//锁IMU
    {
        //@note 此处可能有fp32越界的可能性
//        Yaw_Angle_Close = Gimbal.Gimbal_CMD->Yaw_Set_IMU - Gimbal.ins->Yaw;//计算目标YAW差值
//        if (Yaw_Angle_Close > 180)  Yaw_Angle_Close = -(360.f - Yaw_Angle_Close);
//        if (Yaw_Angle_Close <-180)  Yaw_Angle_Close = (360.f + Yaw_Angle_Close);


            Gimbal.lqr_yaw.k[0] = k_yaw_lqr[0];
            Gimbal.lqr_yaw.k[1] = k_yaw_lqr[1];
            
            float Yaw_motortarget = float_min_distance(Gimbal.Gimbal_CMD->Yaw_Set_IMU,Gimbal.ins->Yaw, -180, 180);
            float Yaw_system_state[2] = {((-Yaw_motortarget) / 57.295779513f), Gimbal.ins->Gyro[2]};
            LQR_Data_Update(&Gimbal.lqr_yaw, Yaw_system_state);
            LQR_Calculate(&Gimbal.lqr_yaw);
        
            float Yaw_output_torque =  Gimbal.lqr_yaw.Output[0];
            Gimbal.YAW_Motor->voltage_input = torque_to_voltage_6020(Yaw_output_torque);
//            Gimbal.YAW_Motor->voltage_input += PidCalculate(&Gimbal.yaw_only_i_pid, Gimbal.Gimbal_CMD->Yaw_Set_IMU, Gimbal.ins->Yaw);
            value_limit(Gimbal.PITCH_Motor->voltage_input,-25000,25000);
        
    }
}

void Gimbal_Not_Work()
{
    Gimbal.YAW_Motor->voltage_input = 0;
    Gimbal.PITCH_Motor->voltage_input = 0;
}

