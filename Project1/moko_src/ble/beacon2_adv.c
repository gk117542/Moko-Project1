#include "beacon2_adv.h"
#include "battery_check.h"
#include "ble_adv.h"
#include "user_flash.h"

void Cycletimers_increase(void)
{
	StuHis.Cycle ++;
	if(StuHis.Cycle>63)
	{
		StuHis.Cycle = 0;
	}
	save_user_flash();
}

void set_cycle_times(uint8_t cycle)
{
	if(cycle<63)
	{
		StuHis.Cycle = cycle;
	}
	save_user_flash();
	return;
}

void g_Beacon2_adv(void)
{
	uint8_t Send_byte_No2 = 0;
	uint8_t Batter_valu = 0;
	//bit 0,iBeacon indicator bit
	Send_byte_No2 = 0x01;
	//bit 1,battery 75% level bit
	Batter_valu = get_battery_level();
	if(Batter_valu>75)
	{
		Send_byte_No2+=0x02;
	}
	if(StuHis.Cycle<64)
	{
		Send_byte_No2 += (StuHis.Cycle<<2);
	}
	adv_update_packet_byte(Send_byte_No2);
}

