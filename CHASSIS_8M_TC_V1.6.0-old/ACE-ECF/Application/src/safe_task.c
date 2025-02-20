/*************************** Dongguan-University of Technology -ACE**************************
 * @file    safe_task.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/11/5
 * @brief
 ******************************************************************************
 * @verbatim
 *  安全任务函数，支持用户自定义名称、失联检测时间、回调函数
 *  使用方法：
 *      申请一个安全任务，丢入名字及自定义回调函数
 *      类似与喂狗机制，在收到消息时刷新在线状态
 *      当达到失联阈值时执行自定义任务
 *  demo：
 *       在Init_task.c中的init_Task()函数里添加
 *       Safe_Init(1);//任务周期1ms
 *       //创建安全任务
 *       osThreadDef(SAFE_TASK, Safe_Task, osPriorityHigh, 0, 128);
 *		 Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);
 *    
 *       //申请安全任务
 *       safe_task_t* demo_safe_task_ptr = Safe_task_add(demo,10,void);
 *       //接收中断函数中喂狗，以下二选一
 *       Safe_Task_online_ptr_(demo_safe_task_ptr);//基于指针
 *       Safe_Task_online_name_(demo);             //基于名字
 * @attention
 *      如果要使用基于名字的喂狗，请确保名字不重复，因为只会喂一个
 *      是基于指针则无所谓
 *      删除任务也是如此
 * @version
 * v1.0   基础版本
 ************************** Dongguan-University of Technology -ACE***************************/
#include "safe_task.h"
#include <string.h>
/*任务间采用链表*/
uint32_t RUN_cycle_MS;
uint32_t DELAY_TIME;
safe_task_t* Head = NULL;
/**
  * @brief  Safe_Init     安全任务初始化
  * @param  RUN_cycle_ms  每次运行周期（ms）
  * @retval null
  */
void Safe_Init(uint32_t RUN_cycle_ms)
{
    RUN_cycle_MS = RUN_cycle_ms;
}
/**
  * @brief  Safe_task_add  添加一个安全任务
  * @param  name[20]       名称
  * @param  Discon_ms      检测失联时长 
  * @param  callback       回调函数
  * @retval temp_task_ptr  安全任务指针
  */

safe_task_t* Safe_task_add(const char* name, uint32_t Discon_ms , Callback callback)
{
    safe_task_t* temp_task_ptr = (safe_task_t*)malloc(sizeof(safe_task_t));
    safe_task_t* last_task_ptr = Head;
    
    //写入相关参数
    strcpy(temp_task_ptr->name, name);
    temp_task_ptr->Disconnection_threshold = Discon_ms;
    temp_task_ptr->Disconnection_count = 0;
    temp_task_ptr->Disconnection_falg = 0;
    temp_task_ptr->UserCallBack = callback;
    temp_task_ptr->next_task = NULL;
    //链表尾插法
    while(last_task_ptr->next_task != NULL)     last_task_ptr = last_task_ptr->next_task;
    last_task_ptr->next_task = temp_task_ptr;
    
    return temp_task_ptr;
}

void Safe_Task(void const *argument)
{
	Safe_Init(1);
	while(1)
	{
		taskENTER_CRITICAL(); //进入临界区
        
        safe_task_t* temp_task_ptr = Head->next_task;
        while(temp_task_ptr != NULL)
        {
            temp_task_ptr->Disconnection_count += RUN_cycle_MS;//添加这一轮任务的失联时间
            if (temp_task_ptr->Disconnection_count >= temp_task_ptr->Disconnection_threshold)//达到失联阈值
            {
                temp_task_ptr->Disconnection_falg = 1;//失联标志
                temp_task_ptr->UserCallBack();//执行用户自定义回调函数
            }
            temp_task_ptr = temp_task_ptr->next_task;
        }
        
		taskEXIT_CRITICAL(); //退出临界区
		vTaskDelay(RUN_cycle_MS);
	}
}

/**
  * @brief  Safe_Task_online_name_  安全任务在线刷新
  * @param  temp_task_ptr           刷新的任务指针
  * @return uint8_t                 刷新结果
  * @retval 1：成功刷新；0：刷新失败
  */
uint8_t Safe_Task_online_ptr_(safe_task_t* temp_task_ptr)
{
    if(temp_task_ptr == NULL)   return 0;
    temp_task_ptr->Disconnection_count = 0;
    temp_task_ptr->Disconnection_falg = 0;
    temp_task_ptr->Disconnection_threshold = 0;
    return 1;
}

/**
  * @brief  Safe_Task_online_name_  安全任务在线刷新
  * @param  name                    刷新的任务名称
  * @return uint8_t                 刷新结果
  * @retval 1：成功刷新；0：刷新失败
  */
uint8_t Safe_Task_online_name_(char * name)
{
    safe_task_t* temp_task_ptr = Head->next_task;
    while(temp_task_ptr != NULL)
    {
        if (strcmp(temp_task_ptr->name,name) == 0)
        {
            temp_task_ptr->Disconnection_count = 0;
            temp_task_ptr->Disconnection_falg = 0;
            temp_task_ptr->Disconnection_threshold = 0;
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}

/**
  * @brief  Safe_task_delect_ptr_   安全任务删除
  * @param  name                    删除的任务指针
  * @return uint8_t                 删除结果
  * @retval 1：成功删除；0：删除失败
  */
uint8_t Safe_task_delect_ptr_(safe_task_t* delect_task_ptr)
{
    safe_task_t* temp_task_ptr = Head->next_task;
    
    if(temp_task_ptr == delect_task_ptr)
    {
        Head->next_task = NULL;
        free(delect_task_ptr);
        return 1;
    }
    
    while(temp_task_ptr->next_task != NULL)
    {
        if(temp_task_ptr->next_task == delect_task_ptr)
        {
            temp_task_ptr->next_task = delect_task_ptr->next_task;
            free(delect_task_ptr);
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}
/**
  * @brief  Safe_task_delect_name_  安全任务删除
  * @param  name                    删除的任务名称
  * @return uint8_t                 删除结果
  * @retval 1：成功删除；0：删除失败
  */
uint8_t Safe_Task_delect_name_(char * delect_task_name)
{
    safe_task_t* temp_task_ptr = Head->next_task;
    safe_task_t* delect_task_ptr;
    
    if(strcmp(temp_task_ptr->name,delect_task_name) == 0)
    {
        Head->next_task = NULL;
        free(delect_task_ptr);
        return 1;
    }
    
    while(temp_task_ptr->next_task != NULL)
    {
        if(strcmp(temp_task_ptr->next_task->name,delect_task_name) == 0)
        {
            delect_task_ptr = temp_task_ptr->next_task;
            temp_task_ptr->next_task = delect_task_ptr->next_task;
            free(delect_task_ptr);
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}