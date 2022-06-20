#include "system_time.h"
#include "app_timer.h"
#include "system_time.h"

#include "key.h"

#include "ble_adv.h"

#include "ble_comm.h"
#include "driver_init.h"
#include "beacon1_adv.h"
#include "user_flash.h"

extern bool adv_send_start;
extern STU_HIS StuHis;

/********************************************************************/ // variable variable define 
// timer 
APP_TIMER_DEF(system_timer_id);  
static uint8_t system_timer_flg=0;  

/*******************************************************/
static void system_timer_Hand(void * p_context)
{
	UNUSED_PARAMETER(p_context);
    system_timer_flg = 1;
}

void system_timer_init(void)
{
	uint32_t err_code;
	 // Create timers.
    err_code = app_timer_create(&system_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                system_timer_Hand);
	APP_ERROR_CHECK(err_code);
	
	
	system_timer_start();
}

void system_timer_start(void)
{
	ret_code_t err_code;
    err_code = app_timer_start(system_timer_id,APP_TIMER_TICKS(1000) , NULL);
    APP_ERROR_CHECK(err_code);
}

void system_timer_stop(void)
{
	ret_code_t err_code;
	err_code = app_timer_stop(system_timer_id);
	APP_ERROR_CHECK(err_code);
}

/**************************************
*system timer 1s
*
************************************/
void task_system_timer(void)
{	//static uint32_t systim = 0;
	if(system_timer_flg==0) return;	
	   system_timer_flg =0;
		//systim++;
	   //BLE_RTT("systime [%d]\r\n",systim);
	task_ble_tx_power();
	//task_ble_adv_start();
	Sterilize_handle();
	device_OnOff_manage();
}  

