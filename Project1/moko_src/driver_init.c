#include "driver_init.h"

#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf_soc.h"
#include "key.h"
#include "system_time.h"
#include "wdt.h"

#include "led_manage.h"
#include "user_flash.h"
#include "ble_adv.h"


/***********************************************
*is_newPcb
*********************************************/
uint8_t is_new_pcb=0;
void is_newPcb(void)
{
	nrf_gpio_cfg_input(TEST_BOARD_PIN,NRF_GPIO_PIN_PULLDOWN);	
    if(nrf_gpio_pin_read(TEST_BOARD_PIN))
    {
       is_new_pcb=1;
    }
    nrf_gpio_cfg_default(TEST_BOARD_PIN);
}

/***********************************************
*function    : Platform_driver_init
*description :
************************************************/
void Platform_driver_init(void)
{
	uint32_t err_code;
	
	// Initialize.
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	
	key_Init();
	system_timer_init();	
	watchdog_init(30000);
	led_init();
	adv_timer_config();
	read_user_flash();
}
/*********************************/
void Power_manager(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}














