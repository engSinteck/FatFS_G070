/*
 * screen_mpx.h
 *
 *  Created on: 5 de jul de 2019
 *      Author: rinaldo
 */

#ifndef EX15_XT_SRC_SCREEN_MPX_H_
#define EX15_XT_SRC_SCREEN_MPX_H_

void screen_reading_mpx(void);
void create_style_bar(void);
void create_vumeter_m(void);
void create_vumeter_r(void);
void create_vumeter_l(void);
void btn_next_audio(void);
void ButtonEventTelaMpx(uint8_t event, uint8_t tipo, uint8_t id);
uint32_t map_mpx(uint32_t value, uint32_t x_min, uint32_t x_max, uint32_t y_min, uint32_t y_max);
void bargraph_mpx_off(uint8_t value);
void update_vumeter_mpx2(uint32_t value);

#endif /* EX15_XT_SRC_SCREEN_MPX_H_ */
