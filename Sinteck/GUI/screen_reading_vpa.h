/*
 * screen_reading_vpa.h
 *
 *  Created on: 5 de jul de 2019
 *      Author: rinaldo
 */

#ifndef EX15_XT_SRC_SCREEN_READING_VPA_H_
#define EX15_XT_SRC_SCREEN_READING_VPA_H_

void screen_reading_vpa(void);
void btn_next_vpa(void);
void btn_prev_vpa(void);
void create_vumeter_vpa_1(void);
void create_vumeter_ipa_1(void);
void print_vpa_1(float value);
void print_ipa_1(float value);
void update_vumeter_vpa_1(float value);
void update_vumeter_ipa_1(float value);
void ButtonEventTelaReading_Vpa(uint8_t event, uint8_t tipo, uint8_t id);

#endif /* EX15_XT_SRC_SCREEN_READING_VPA_H_ */
