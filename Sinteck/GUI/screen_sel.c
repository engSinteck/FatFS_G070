/*
 * screen_sel.c
 *
 *  Created on: 4 de jul de 2019
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#include <GUI/GUI_EX15-XT.h>
#include "main.h"
#include "lvgl/lvgl.h"
#include "stdio.h"
#include "string.h"
#include "key.h"

extern uint32_t TelaAtiva;
extern uint8_t MenuSel;

static void btn_event_btn1(lv_obj_t * btn, lv_event_t event);
//static void btn_event_esc_sel(lv_obj_t * btn, lv_event_t event);
void sel_screen_send_button(uint32_t btn, lv_btn_state_t state);

static lv_obj_t * Tela_Sel;
static lv_obj_t * imgbtn1[4];
static lv_style_t style_fundo;
static lv_obj_t * img_fundo;


void screen_sel(void)
{
	MenuSel = 0;
	// Create a Screen
	Tela_Sel = lv_obj_create(NULL, NULL);
//	lv_style_copy(&style_fundo, &lv_style_plain_color);
//	style_fundo.body.main_color = LV_COLOR_BLACK;
//	style_fundo.body.grad_color = LV_COLOR_BLACK;
//	lv_obj_set_style(Tela_Sel, &style_fundo); 					// Configura o estilo criado

	// Imagem de Fundo
//	img_fundo = lv_img_create(Tela_Sel, NULL);
//	lv_img_set_src(img_fundo, "F:tela_sel.bin");
//	lv_obj_set_protect(img_fundo, LV_PROTECT_POS);
	// Cria Botoes
	create_buttons();

	lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
	lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
	lv_btn_set_state(imgbtn1[2], LV_BTN_STATE_TGL_REL);
	lv_btn_set_state(imgbtn1[3], LV_BTN_STATE_TGL_REL);

	lv_scr_load(Tela_Sel);
	TelaAtiva = TelaSelecao;
}

void create_buttons(void)
{
	// Create Button Frequency
	imgbtn1[0] = lv_imgbtn_create(Tela_Sel, NULL);
	lv_obj_set_user_data(imgbtn1[0], 0);
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_REL, "F:Btn_Frequency.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_REL, "F:Btn_Frequency.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_PR, "F:Btn_Frequency_am.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_PR, "F:Btn_Frequency.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_REL, "F:Btn_Frequency.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_INA, "F:Btn_Frequency.bin");
	lv_obj_set_event_cb(imgbtn1[0], btn_event_btn1);
	lv_obj_set_pos(imgbtn1[0], 1, 18);

	imgbtn1[1] = lv_imgbtn_create(Tela_Sel, imgbtn1[0]);
	lv_obj_set_user_data(imgbtn1[1], 1);
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_REL, "F:Btn_Audio.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_REL, "F:Btn_Audio.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_PR, "F:Btn_Audio_am.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_PR, "F:Btn_Audio.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_REL, "F:Btn_Audio.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_INA, "F:Btn_Audio.bin");
	lv_obj_set_pos(imgbtn1[1], 1, 45);

	imgbtn1[2] = lv_imgbtn_create(Tela_Sel, imgbtn1[0]);
	lv_obj_set_user_data(imgbtn1[2], 2);
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_REL, "F:Btn_RF.bin");
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_TGL_REL, "F:Btn_RF.bin");
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_TGL_PR, "F:Btn_RF_am.bin");
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_PR, "F:Btn_RF.bin");
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_REL, "F:Btn_RF.bin");
	lv_imgbtn_set_src(imgbtn1[2], LV_BTN_STATE_INA, "F:Btn_RF.bin");
	lv_obj_set_pos(imgbtn1[2], 1, 73);

	imgbtn1[3] = lv_imgbtn_create(Tela_Sel, imgbtn1[0]);
	lv_obj_set_user_data(imgbtn1[3], 3);
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_REL, "F:Btn_Readings.bin");
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_TGL_REL, "F:Btn_Readings.bin");
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_TGL_PR, "F:Btn_Readings_am.bin");
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_PR, "F:Btn_Readings.bin");
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_REL, "F:Btn_Readings.bin");
	lv_imgbtn_set_src(imgbtn1[3], LV_BTN_STATE_INA, "F:Btn_Readings.bin");
	lv_obj_set_pos(imgbtn1[3], 1, 100);
}

/*
static void btn_event_esc_sel(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		uint32_t id = lv_obj_get_user_data(btn);
		lv_obj_del(Tela_Sel);
		main_screen();
	}
}
*/

void sel_screen_send_button(uint32_t btn, lv_btn_state_t state)
{
	switch(btn) {
		case 0:
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[2], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[3], LV_BTN_STATE_TGL_REL);
			break;
		case 1:
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
			lv_btn_set_state(imgbtn1[2], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[3], LV_BTN_STATE_TGL_REL);
			break;
		case 2:
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[2], LV_BTN_STATE_TGL_PR);
			lv_btn_set_state(imgbtn1[3], LV_BTN_STATE_TGL_REL);
			break;
		case 3:
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[2], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[3], LV_BTN_STATE_TGL_PR);
			break;
	}
}

void ButtonEventTelaSelecao(uint8_t event, uint8_t tipo, uint8_t id)
{
	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			switch(id) {
				case KEY_DN:
					if(MenuSel >= 1) MenuSel--;
					sel_screen_send_button(MenuSel, LV_BTN_STATE_TGL_PR);
					//logI("Debug: Tela_Selecao KEY_Down: MenuSel: %ld\n", MenuSel);
					break;
				case KEY_UP:
					MenuSel++;
					if(MenuSel >= 3) MenuSel = 3;
					sel_screen_send_button(MenuSel, LV_BTN_STATE_TGL_PR);
					//logI("Debug: Tela_Selecao KEY_UP: MenuSel: %ld\n", MenuSel);
					break;
				case KEY_ENTER:
					lv_event_send(imgbtn1[MenuSel], LV_EVENT_APPLY, &MenuSel);
					//logI("Debug: Tela_Selecao KEY_ENTER: MenuSel: %ld\n", MenuSel);
					break;
				case KEY_ESC:
					//lv_event_send(img_fundo, LV_EVENT_APPLY, NULL);
					lv_obj_del(Tela_Sel);
					main_screen();
					break;
			}
		}
	}
}

static void btn_event_btn1(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		uint32_t id = lv_obj_get_user_data(btn);
		lv_obj_del(Tela_Sel);
		switch(id) {
			case 0:
				//screen_freq();
				break;
			case 1:
				//screen_audio();
				break;
			case 2:
				//screen_rf();
				break;
			case 3:
				//screen_readings();
				break;
		}
	}
}
