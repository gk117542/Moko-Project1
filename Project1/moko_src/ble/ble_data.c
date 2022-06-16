#include "ble_data.h"
#include "periph_service.h"
#include "user_flash.h"
#include "ble_adv.h"
#include "driver_init.h"
#include "beacon1_adv.h"
#include <string.h>

extern uint8_t beacon_infor[21];
#define READ_FLAG		0x00
#define WRITE_FLAG		0x01

enum{
	CMD_TXPOWER=0x20,
	CMD_ADVINTER=0x21,//advertise interval
	CMD_BEAUUID=0x22,//iBeacon UUID
	CMD_MAJOR=0x23,//iBeacon major bytes
	CMD_MINOR=0x24,//iBeacon minor byte
	CMD_RSSI=0x25,//iBeacon RSSI byte
	CMD_STERILIZE=0x26,//Sterilize time
	CMD_DEVCYCLE=0x27,//Device cycle count
	CMD_BLERST=0x28,//ibeacon reset
};
uint32_t nus_send_cmd(uint8_t cmd,uint8_t *p_data,uint8_t lenth)
{
    uint32_t err;
    uint8_t buf[50];
    uint16_t len=0,i;
	
	
    buf[len++]=0xeb;
    buf[len++]=cmd;
    buf[len++]=0;
    buf[len++]=lenth;
	
    if(lenth)
    {
        for(i=0; i<lenth; i++)
            buf[len++]=p_data[i];
    }
	
    Periph_service_send(FEX0_CHAR,buf,len);
//    BLE_RTT("result:%x",err);
    return err;
}
void g_send_cmd(uint8_t rwflg,uint8_t cmd,uint8_t *p_data,uint8_t lenth)
{
	uint8_t buf[50];
    uint16_t len=0,i;
    buf[len++]=0xeb;
    buf[len++]=rwflg;
    buf[len++]=cmd;
    buf[len++]=lenth;
    if(lenth)
    {
        for(i=0; i<lenth; i++)
            buf[len++]=p_data[i];
    }
    Periph_service_send(FEX0_CHAR,buf,len);
    return;
}

/*****************************************************************
*@brief ble_commd_analyze 
*[data in]:p_data :app data in   len :data len
*[head] [cmd]  [sn]  [data len]  data
*0xea     10    00      01       01
*********************************/

void  ble_commd_analyze(uint8_t *p_data,uint8_t len)
{
    uint8_t cmd,dataslen;
    uint8_t buf[20];
	uint8_t rwflg = READ_FLAG;
    if(p_data[0]!=0xea ||(p_data[3]+4)!=len){
        g_send_cmd(READ_FLAG,0x0d,0,0);
        return;
    }
	rwflg= p_data[1];
	cmd= p_data[2];
    dataslen = p_data[3];	
	switch(cmd)
	{
		case CMD_TXPOWER:
			if(rwflg==READ_FLAG){
				buf[0]=StuHis.tx_Power;
				g_send_cmd(rwflg,cmd,buf,1);
			}else if(rwflg==WRITE_FLAG){
				if(1==dataslen){
	                StuHis.tx_Power=p_data[4];
					buf[0]=0xaa;
					save_user_flash();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_ADVINTER:
			if(rwflg==READ_FLAG){
				buf[0]=StuHis.adv_interval;
				g_send_cmd(rwflg,cmd,buf,1);
			}else if(rwflg==WRITE_FLAG){
				if(1==dataslen){
	                StuHis.adv_interval=p_data[4];
					buf[0]=0xaa;
					save_user_flash();
					set_ble_start();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_BEAUUID:
			if(rwflg==READ_FLAG){
				for(int i = 0;i<16;i++){
					buf[i]=beacon_infor[i];
				}
				g_send_cmd(rwflg,cmd,buf,16);
			}else if(rwflg == WRITE_FLAG){
				if(0x10==dataslen){
					memcpy(StuHis.Beacon_uuid, &p_data[4], 0x10);
					buf[0]=0xaa;
					save_user_flash();
					set_ble_start();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_MAJOR:
			if(rwflg==READ_FLAG){
				buf[0] = beacon_infor[16];
				buf[1] = beacon_infor[17];
				g_send_cmd(rwflg,cmd,buf,2);
			}else if(rwflg==WRITE_FLAG){
				if(0x02==dataslen){
					memcpy(StuHis.Beacon_major, &p_data[4], 0x02);
					buf[0]=0xaa;
					save_user_flash();
					set_ble_start();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_MINOR:
			if(rwflg==READ_FLAG){
				buf[0] = beacon_infor[18];
				g_send_cmd(rwflg,cmd,buf,1);
			}else if(rwflg==WRITE_FLAG){
				if(0x01==dataslen){
					memcpy(&StuHis.Beacon_minor, &p_data[4], 0x01);
					buf[0]=0xaa;
					save_user_flash();
					set_ble_start();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_RSSI:
			if(rwflg==READ_FLAG){
				buf[0] = beacon_infor[20];
				g_send_cmd(rwflg,cmd,buf,1);
			}else if(rwflg==WRITE_FLAG){
				if(0x01==dataslen){
					memcpy(&StuHis.Beacon_rssi, &p_data[4], 0x01);
					buf[0]=0xaa;
					save_user_flash();
					set_ble_start();
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_STERILIZE:
			if(rwflg==READ_FLAG){
				buf[0] = get_Sterilize_byte();
				if(buf[0]<=63){
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}else if(rwflg==WRITE_FLAG){
				if(0x01==dataslen){
					if(set_Sterilize_byte(p_data[4])){
						buf[0]=0xaa;
						set_ble_start();
						g_send_cmd(rwflg,cmd,buf,1);
					}else{
						buf[0]=0x00;
						g_send_cmd(rwflg,cmd,buf,1);
					}
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_DEVCYCLE:
			if(rwflg==READ_FLAG){
				buf[0] = StuHis.Cycle;
				g_send_cmd(rwflg,cmd,buf,1);
				}else if(rwflg==WRITE_FLAG){
				if(0x01==dataslen){
					buf[0]=0xaa;
					memcpy(&StuHis.Cycle, &p_data[4], 0x01);
					g_send_cmd(rwflg,cmd,buf,1);
				}else{
					buf[0]=0x00;
					g_send_cmd(rwflg,cmd,buf,1);
				}
			}
		break;
		case CMD_BLERST:
			if(rwflg==WRITE_FLAG){
				buf[0]=0xaa;
				//set_device_rst();
				g_send_cmd(rwflg,cmd,buf,1);
			}
		break;
			default:break;
	}
}


