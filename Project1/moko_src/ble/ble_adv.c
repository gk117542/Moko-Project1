#include "ble_adv.h"
#include "ble_advdata.h"
#include "stdint.h"

#include "app_error.h"
#include "ble_comm.h"
#include "driver_init.h"
#include "user_flash.h"
#include "ble_init.h"
#include "beacon1_adv.h"
#include "beacon2_adv.h"
#include "nrf_delay.h"
#include "key.h"
#include "app_timer.h"

static  uint8_t m_adv_data[31]={0};
static  uint8_t adv_data_len;
extern STU_HIS StuHis;
bool adv_send_start = true;

/*******************************************************************/         
uint8_t key_code[2] = /**< Information advertised by the Beacon. */
{
  0x00,0x00
};
  
uint8_t beacon_infor[21] = /**< Information advertised by the Beacon. */
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  //0xE2,0xC5,0x6D,0xB5,0xDF,0xFB,0x48,0xD2,0xB0,0x60,0xD0,0xF5,0xA7,0x10,0x96,0xE0, // uuid
  0X00,0x00,// major
  0x00,// mainor
  0x00,//
  0xC5// rssi
};

/***********************************************************
*@function :iBeacon_adv1
*@describe : it is device strat state packe
*******************************************/
static void iBeacon_adv(void)
{
    adv_data_len=0;	
	m_adv_data[adv_data_len++]= 0x02;//Len
    m_adv_data[adv_data_len++]= 0x01;//type
    m_adv_data[adv_data_len++]= 0x06;
	
	// ibeacon data 
	m_adv_data[adv_data_len++]= 0x1A;//sizeof(key_code)+1+4;//len=sizeof(beacon_infor)+len(type)+4
    m_adv_data[adv_data_len++]= 0xFF; //type
	
    m_adv_data[adv_data_len++]= 0x4C;//device infor
    m_adv_data[adv_data_len++]= 0x00;
	
    m_adv_data[adv_data_len++]= 0x02;
    m_adv_data[adv_data_len++]= 0x15;

	
	#if 0
    m_adv_data[adv_data_len++]= 0x11;//key code
    m_adv_data[adv_data_len++]= 0x22;
	#else
	memcpy(&beacon_infor,StuHis.Beacon_uuid,16);
	memcpy(&beacon_infor[16],StuHis.Beacon_major,2);
	memcpy(&beacon_infor[18],&StuHis.Beacon_minor,1);
	memcpy(&beacon_infor[20],&StuHis.Beacon_rssi,1);
	
 	memcpy(m_adv_data+adv_data_len,beacon_infor,sizeof(beacon_infor));
	  adv_data_len+=sizeof(beacon_infor);
	#endif
}

void adv_update_packet_byte(uint8_t mbyte)
{
	beacon_infor[19] = mbyte;
	return;
}

/***********************************************************
*@function :iBeacon_adv_data_set
*@describe : set ibeacon response packet data  
              in ble adv = Raw data to be placed in scan response packet
*******************************************/
static  uint8_t m_rsp_data[31]={0};
static  uint8_t rsp_data_len;

static void iBeacon_rsp_pack_set(void)
{
	char  temp_mac[5];
	memset(temp_mac,0,5);
    sprintf(temp_mac,"%02x%02x",StuHis.mac_addr[1],StuHis.mac_addr[0]);	
	
    rsp_data_len=0;	
	m_rsp_data[rsp_data_len++]= 0x02;//Len
    m_rsp_data[rsp_data_len++]= 0x0A;//type	
    m_rsp_data[rsp_data_len++]= 0x00;//TX power vale
	
	//rsp_data
	m_rsp_data[rsp_data_len++]= strlen("BUTTON-")+1+4;
    m_rsp_data[rsp_data_len++]= 0x09; //type
	
	memcpy(m_rsp_data+rsp_data_len,"BUTTON-",strlen("BUTTON-"));
	  rsp_data_len+=strlen("BUTTON-");
	
	memcpy(m_rsp_data+rsp_data_len,temp_mac,4);
	  rsp_data_len+=4;
	
}

/***********************************************************
*@function :adv_data_padding
*@describe : Raw data  = adv packet + scan response packet
*******************************************/
static void adv_data_padding(void)
{
    uint32_t    err_code;
	
	
	iBeacon_adv();
	
	err_code = sd_ble_gap_adv_data_set(m_adv_data,adv_data_len,NULL,0);

	iBeacon_rsp_pack_set();
    err_code = sd_ble_gap_adv_data_set(m_adv_data,adv_data_len,m_rsp_data,rsp_data_len);

	if(err_code!=NRF_SUCCESS)
	{
		BLE_RTT("[adv_data_padding] err_code=%x \r\n",err_code);
//	    APP_ERROR_CHECK(err_code);
	}
}

/***********************************************************
*@function : adv_params_config
*@describe : set ble adv para
*******************************************/
static 	ble_gap_adv_params_t adv_params;
static void adv_params_config(void)
{
	memset(&adv_params, 0, sizeof(ble_gap_adv_params_t));
	
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;//BLE_GAP_ADV_TYPE_ADV_IND;   //BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;

    adv_params.p_peer_addr = NULL; 
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
	adv_params.interval    = BLE_GAP_ADV_INTERVAL_MAX;
    //adv_params.interval  = MSEC_TO_UNITS(StuHis.adv_interval, UNIT_0_625_MS);//BLE_GAP_ADV_INTERVAL_MIN
    adv_params.timeout     =  0;
}

static bool start_adv_flg = false;
/*********************************************
*  ble_adv_stop
***********************/
void ble_adv_stop(void)
{
	uint32_t    err_code;
	err_code = sd_ble_gap_adv_stop();
	start_adv_flg = false;
	if(err_code!=NRF_SUCCESS)
	{
	    //BLE_RTT("[ble_adv_stop]===>err=%d\r\n",err_code);
	}
//    APP_ERROR_CHECK(err_code);
	//BLE_RTT("=======BLE ADV stop=======\r\n");

}

/*********************************************
*  ble_adv_start
***********************/
static void ble_adv_start(void)
{
    uint32_t    err_code;

	adv_data_padding();	
	adv_params_config();
	
    err_code = sd_ble_gap_adv_start(&adv_params, BLE_CONN_CFG_TAG_DEFAULT);
	
	if(err_code!=NRF_SUCCESS)
	{
		BLE_RTT("[ble_adv_start]  err=0x%x\r\n",err_code);
		APP_ERROR_CHECK(err_code);	
	}
	start_adv_flg = true;
	//BLE_RTT("=======BLE ADV start=======\r\n");
}


/*********************************************************
*function:ble_adv_start_mode
*
*************************************/
static uint8_t ble_start_flg=0;
void set_ble_start(void)
{
	ble_start_flg=1;
}

void task_ble_adv_start(void)
{
	if(is_ble_connect()==true)
		return;
    if(ble_start_flg==0)
		return;
	ble_start_flg=0;
	ble_adv_stop();
    ble_adv_start();
}

APP_TIMER_DEF(g_adv_update_timer);
static void g_adv_update_timeout(void * p_context)
{
	adv_send_start = true;
}

void start_adv_timer(uint32_t msecond)
{
	adv_send_start = false;
	app_timer_start(g_adv_update_timer,APP_TIMER_TICKS(msecond),NULL);
}

void adv_update_timer_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&g_adv_update_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                g_adv_update_timeout);
    APP_ERROR_CHECK(err_code);
}

void adv_timer_config(void)
{
	adv_update_timer_init();
	app_timer_start(g_adv_update_timer,APP_TIMER_TICKS(100),NULL);
}

void task_adv_update(void)
{
	static uint8_t adv_state = 0;
	if(is_ble_connect()) return;
	if(!adv_send_start) return ;
	ble_adv_stop();
	switch(adv_state)
	{
		case 0x00://Send first advertising packet
			g_Beacon1_adv();
			start_adv_timer(100);
			//BLE_RTT("adv send first packet ---\r\n");
			adv_state = 0x01;
		break;
		case 0x01://Send second advertising packet
			g_Beacon2_adv();
			//BLE_RTT("adv send second packet ---\r\n");
			start_adv_timer(StuHis.adv_interval*1000);
			adv_state = 0x00;
		break;
		default:break;
	}
	
	ble_adv_start();
	task_ble_tx_power();
}


