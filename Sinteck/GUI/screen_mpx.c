/*
 * screen_mpx.c
 *
 *  Created on: 5 de jul de 2019
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */
#include <GUI/GUI_EX15-XT.h>
#include "main.h"
#include "lvgl/lvgl.h"
#include "stdio.h"
#include "string.h"
#include "key.h"

static void btn_event_next_audio(lv_obj_t * btn, lv_event_t event);
static void btn_event_esc_audio(lv_obj_t * btn, lv_event_t event);
static void update_mpx_screen(lv_task_t * param);

extern uint32_t TelaAtiva, mpx, audio_l, audio_r;

static lv_obj_t * Tela_Reading_MPX;
static lv_obj_t * imgbtn_next;
static lv_obj_t * imgbtn_prev;
static lv_obj_t * bar_m[24];
static lv_obj_t * bar_l[24];
static lv_obj_t * bar_r[24];
static lv_style_t style_indic_bar;
static lv_style_t style_indic_bar_vd;
static lv_style_t style_indic_bar_am;
static lv_style_t style_indic_bar_cy;
static lv_style_t style_indic_bar_vm;
static lv_style_t style_fundo;
static lv_task_t * Task_Mpx;

#if LV_USE_BACKGROUND
	static lv_obj_t * img_fundo;
#endif

const int32_t m_pos_x[24] = {14, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80,
                             86, 92, 98, 104, 110, 116, 122, 128, 134, 140, 146, 152};


const int32_t r_pos_x[24] = {14, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80,
                             86, 92, 98, 104, 110, 116, 122, 128, 134, 140, 146, 152};

const int32_t l_pos_x[24] = {14, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80,
                             86, 92, 98, 104, 110, 116, 122, 128, 134, 140, 146, 152};

#if LV_USE_FILESYSTEM == 0
	LV_IMG_DECLARE(tela_reading_audio);
	LV_IMG_DECLARE(BtnESC);
	LV_IMG_DECLARE(Btn_next);
#endif

void screen_reading_mpx(void)
{
	// Create a Screen
	Tela_Reading_MPX = lv_obj_create(NULL, NULL);
	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_Reading_MPX, &style_fundo); 					// Configura o estilo criado

#if LV_USE_BACKGROUND
	// Imagem de Fundo
	img_fundo = lv_img_create(Tela_Reading_MPX, NULL);
#if	LV_USE_FILESYSTEM
    lv_img_set_src(img_fundo, "P:/EX15-XT/img/tela_reading_audio.bin");
#else
    lv_img_set_src(img_fundo, &tela_reading_audio);
#endif
    lv_obj_set_protect(img_fundo, LV_PROTECT_POS);
	lv_obj_set_event_cb(img_fundo, btn_event_esc_audio);
	lv_obj_set_click(img_fundo, 1);
#endif

	create_style_bar();
	create_vumeter_m();
	create_vumeter_r();
	create_vumeter_l();
	btn_next_audio();
	lv_scr_load(Tela_Reading_MPX);

	// Task Update Main Screen
	Task_Mpx = lv_task_create(update_mpx_screen, 500, LV_TASK_PRIO_MID, NULL);

	TelaAtiva = TelaMpx;
}

void create_style_bar(void)
{
	// Indicador OFF
	lv_style_copy(&style_indic_bar, &lv_style_pretty);
	style_indic_bar.body.radius = 0;
	style_indic_bar.body.grad_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.main_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.border.color = LV_COLOR_MAKE(105, 105, 105);
	// Indicador Verde
	lv_style_copy(&style_indic_bar_vd, &lv_style_pretty);
	style_indic_bar_vd.body.radius = 0;
	style_indic_bar_vd.body.grad_color = LV_COLOR_LIME;
	style_indic_bar_vd.body.main_color = LV_COLOR_LIME;
	style_indic_bar_vd.body.border.color = LV_COLOR_LIME;
	// Indicador CYAN
	lv_style_copy(&style_indic_bar_cy, &lv_style_pretty);
	style_indic_bar_cy.body.radius = 0;
	style_indic_bar_cy.body.grad_color = LV_COLOR_CYAN;
	style_indic_bar_cy.body.main_color = LV_COLOR_CYAN;
	style_indic_bar_cy.body.border.color = LV_COLOR_CYAN;
	// Indicador Amarelo
	lv_style_copy(&style_indic_bar_am, &lv_style_pretty);
	style_indic_bar_am.body.radius = 0;
	style_indic_bar_am.body.grad_color = LV_COLOR_YELLOW;
	style_indic_bar_am.body.main_color = LV_COLOR_YELLOW;
	style_indic_bar_am.body.border.color = LV_COLOR_YELLOW;
	// Indicador Vermelho
	lv_style_copy(&style_indic_bar_vm, &lv_style_pretty);
	style_indic_bar_vm.body.radius = 0;
	style_indic_bar_vm.body.grad_color = LV_COLOR_RED;
	style_indic_bar_vm.body.main_color = LV_COLOR_RED;
	style_indic_bar_vm.body.border.color = LV_COLOR_RED;
}

void create_vumeter_m(void)
{
	// Create a default bar
	for(uint8_t x = 0; x < 24; x++) {
		bar_m[x] = lv_bar_create(Tela_Reading_MPX, NULL);
		lv_obj_set_size(bar_m[x], 4, 11);
		lv_bar_set_style(bar_m[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_m[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_m[x], NULL, LV_ALIGN_IN_TOP_LEFT, m_pos_x[x], 48);
		lv_bar_set_value(bar_m[x], 100, 0);
	}
}

void create_vumeter_r(void)
{
	// Create a default bar
	for(uint8_t x = 0; x < 24; x++) {
		bar_r[x] = lv_bar_create(Tela_Reading_MPX, NULL);
		lv_obj_set_size(bar_r[x], 4, 21);
		lv_bar_set_style(bar_r[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_r[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_r[x], NULL, LV_ALIGN_IN_TOP_LEFT, r_pos_x[x], 71);
		lv_bar_set_value(bar_r[x], 100, 0);
	}
}

void create_vumeter_l(void)
{
	// Create a default bar
	for(uint8_t x = 0; x < 24; x++) {
		bar_l[x] = lv_bar_create(Tela_Reading_MPX, NULL);
		lv_obj_set_size(bar_l[x], 4, 21);
		lv_bar_set_style(bar_l[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_l[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_l[x], NULL, LV_ALIGN_IN_TOP_LEFT, l_pos_x[x], 103);
		lv_bar_set_value(bar_l[x], 100, 0);
	}
}

void btn_next_audio(void)
{
	// Create an Image button
	imgbtn_next = lv_imgbtn_create(Tela_Reading_MPX, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_next.bin");
#else
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_TGL_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_TGL_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next, LV_BTN_STATE_INA, &Btn_next);
#endif
	lv_obj_set_event_cb(imgbtn_next, btn_event_next_audio);
	lv_obj_set_pos(imgbtn_next, 112, 18);
}

static void btn_event_next_audio(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button Next Released SCREEN_READING_TEMP\n");
		lv_task_del(Task_Mpx);
		lv_obj_del(Tela_Reading_MPX);
		screen_reading_temp();
	}
}

void btn_esc_audio(void)
{
	// Create an Image button
	imgbtn_prev = lv_imgbtn_create(Tela_Reading_MPX, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_INA, "P:/EX15-XT/img/BtnESC.bin");
#else
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_INA, &BtnESC);
#endif
	lv_obj_set_event_cb(imgbtn_prev, btn_event_esc_audio);
	lv_obj_set_pos(imgbtn_prev, 2, 18);
}


uint32_t map_mpx(uint32_t value, uint32_t x_min, uint32_t x_max, uint32_t y_min, uint32_t y_max)
{
    return (value - x_min) * (y_max - y_min) / (x_max - x_min) + x_min;
}

void bargraph_mpx_off(uint8_t value)
{
	for(uint8_t x = value; x < 20; x++) {
		lv_bar_set_style(bar_m[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_m[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
	}
}

void update_vumeter_mpx2(uint32_t value)
{

}

static void update_mpx_screen(lv_task_t * param)
{
//	update_vumeter_mpx2(bar_m[0], map_mpx(mpx, 0, 4095, 0, 24));
//	update_vumeter_mpx2(bar_l[0], map_mpx(audio_l, 0, 4095, 0, 24));
//	update_vumeter_mpx2(bar_r[0], map_mpx(audio_l, 0, 4095, 0, 24));
}

static void btn_event_esc_audio(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button ESC Released\n");
		lv_task_del(Task_Mpx);
		lv_obj_del(Tela_Reading_MPX);
		screen_readings();
	}
}

void ButtonEventTelaMpx(uint8_t event, uint8_t tipo, uint8_t id)
{
	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			switch(id) {
				case KEY_DN:
					//lv_event_send(img_fundo, LV_EVENT_APPLY, NULL);
					lv_obj_del(Tela_Reading_MPX);
					lv_task_del(Task_Mpx);
					screen_readings();
					break;
				case KEY_UP:
					lv_event_send(imgbtn_next, LV_EVENT_APPLY, NULL);
					break;
				case KEY_ENTER:
					break;
				case KEY_ESC:
					break;
			}
		}
	}
}
