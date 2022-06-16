#ifndef BEACON1_ADV_H
#define BEACON1_ADV_H
#include "stdint.h"
#include "stdbool.h"

void g_Beacon1_adv(void);
void Sterilize_handle(void);
void Sterilize_set_counter_value(uint8_t value);
uint8_t get_Sterilize_byte(void);
bool set_Sterilize_byte(uint8_t sterbyte);
#endif
