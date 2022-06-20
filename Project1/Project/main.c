#include "nrf_dfu.h"
#include "ble_adv.h"
#include "ble_init.h"
#include "driver_init.h"
#include "system_time.h"
#include "wdt.h"

#include "led_manage.h"
#include "battery_check.h"
#include "user_flash.h"
#include "beacon2_adv.h"

uint8_t SOFTH_VERSION=1;
uint8_t SOFTL_VERSION=0;
uint8_t TEST_VERSION=1;

/**************************************************
* @function: assert_nrf_callback err 
********************************************/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/*******************************/
int main(void)
{
	BLE_RTT("-----current sys ver is %s ,last modify time is %s %s ------\r\n",SOFTH_VERSION,__DATE__,__TIME__);
	BLE_RTT("---chip ram size is %x\r\n",NRF_FICR->INFO.RAM*1024);
	
	//dfu boot para addr
	boot_to_new_appilacation(0,0);	
	
	Platform_driver_init();
	ble_Init();
	//dc-dc enable----
	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
	Cycletimers_increase();
	BLE_RTT("device start.........\r\n");
    for (;;)
    {
        Power_manager();
		//led_timer_manage();
		task_read_battery();
		//down dfu file 
	    task_dfu_upfile();	
		task_system_timer();
		task_save_flash();
		task_adv_update();
		Feed_WacthDog();
    }
}

