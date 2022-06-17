#ifndef ble_adv_h_
#define ble_adv_h_




#include "stdint.h"



void ble_adv_stop(void);

void set_ble_start(void);
void  task_ble_adv_start(void);
void adv_update_packet_byte(uint8_t mbyte);
void task_adv_update(void);
void adv_timer_config(void);

#endif

