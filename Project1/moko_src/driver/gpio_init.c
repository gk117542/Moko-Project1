#include "gpio_init.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"

#include "ble_gap.h"
#include "nrf_delay.h"
#include "driver_init.h"
#include "led_manage.h"
#include "ble_adv.h"
#include "ble_comm.h"



/**************************************
*function:key_irq_hand
*description: hanlde pin irq function 
**********************/
uint8_t key_press_flg=0;
void pin_irq_handle(nrf_drv_gpiote_pin_t pin_no, 
	               nrf_gpiote_polarity_t action)
{
	if(nrf_gpio_pin_read(KEY_PIN)==0)
	{ 
		key_press_flg=1;
		BLE_RTT("key_irq_hand....\r\n");
	}

}

/**********************************************************************
*function:enable_pin_irq
*description:  enable pin irq 
**************************************************************/
void enable_pin_irq(uint32_t pin,bool enable_flg)
{
    uint32_t err_code;
		
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    config.pull = NRF_GPIO_PIN_PULLUP;

	// key---------init
    if(KEY_PIN == pin)
    {
        config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
        config.hi_accuracy =false;
    }
		
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    if(true == enable_flg)
    {
        err_code = nrf_drv_gpiote_in_init(pin, &config, pin_irq_handle);
        APP_ERROR_CHECK(err_code);
        nrf_drv_gpiote_in_event_enable(pin, true);
    }
    else
    {
        nrf_drv_gpiote_in_event_disable(pin);
        nrf_drv_gpiote_in_uninit(pin);
    }
}









