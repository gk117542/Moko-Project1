#include "beacon1_adv.h"
#include "battery_check.h"
#include "ble_adv.h"
#include "driver_init.h"

uint8_t Sterilize_send_byte=0x00;
static uint32_t Sterilize_counter = 0;

//Update Sterilize counter value by cmd
void Sterilize_set_counter_value(uint8_t value)
{
	if(value>43){
		Sterilize_counter = ((value-42)*168+744)*3600;
	}
	else if((value>=13)&&(value<=43)){
		Sterilize_counter = (value-12)*24*3600;
	}
	else if(value < 13){
		Sterilize_counter = value*3600;
	}
	return;
}

void Sterilize_handle(void)
{	
	uint32_t Hours;
	Sterilize_counter++;
	Hours = Sterilize_counter/3600;
	if(Hours>=4272)
	{
		Sterilize_counter = 0;
		Sterilize_send_byte = 0;
		return;
	}
	if(Hours<12)
	{
		Sterilize_send_byte = Hours;
	}
	else if((Hours>=12)&&(Hours<24))
	{
		Sterilize_send_byte = 12;
	}
	else if((Hours>=24)&&(Hours<=744))
	{
		Sterilize_send_byte = 12+(Hours/24);
	}
	else if((Hours>744)&&(Hours<4272))
	{
		Sterilize_send_byte = 42+((Hours-744)/168);
	}
}

void g_Beacon1_adv(void)
{
	uint8_t Send_byte_No1 = 0;
	uint8_t Batter_valu = 0;
	//bit 0,iBeacon indicator bit
	Send_byte_No1=0x00;
	//bit 1,battery 25% level bit
	Batter_valu = get_battery_level();
	if(Batter_valu>25){
		Send_byte_No1+=0x02;
	}
	//bit 2-7,sterilize time
	if(Sterilize_send_byte<64)
	{
		Send_byte_No1 += (Sterilize_send_byte<<2);
	}
	adv_update_packet_byte(Send_byte_No1);
	set_ble_start();
}

uint8_t get_Sterilize_byte(void)
{
	return Sterilize_send_byte;
}

bool set_Sterilize_byte(uint8_t sterbyte)
{
	if(sterbyte>63){
		BLE_RTT("SET VALUE OUT OF RANGE [%d]!\r\n",sterbyte);
		return false;
	}else{
		Sterilize_set_counter_value(sterbyte);
		return true;
	}
}

