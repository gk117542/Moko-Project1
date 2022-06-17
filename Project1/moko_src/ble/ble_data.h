#ifndef ble_data_h_
#define  ble_data_h_
#include "stdint.h"
enum
{
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

void  ble_commd_analyze(uint8_t *p_data,uint8_t len);
#endif



