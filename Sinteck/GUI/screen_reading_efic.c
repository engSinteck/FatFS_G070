/*
 * screen_reading_efic.c
 *
 *  Created on: 5 de jul de 2019
 *      Author: rinaldo
 */
#include <GUI/GUI_EX15-XT.h>
#include "main.h"
#include "lvgl/lvgl.h"
#include "stdio.h"
#include "string.h"
#include "key.h"

extern char buffer[];
extern uint32_t TelaAtiva;
extern float f_vpa, f_ipa, f_vin, forward, reflected;

static void btn_event_next_efic(lv_obj_t * btn, lv_event_t event);
static void btn_event_prev_efic(lv_obj_t * btn, lv_event_t event);
static void update_screen_efic(lv_task_t * param);

static lv_obj_t * Tela_Reading_EFIC;
static lv_task_t * Task_Reading_EFIC;
static lv_obj_t * imgbtn_next;
static lv_obj_t * imgbtn_prev;
static lv_obj_t * bar_e[16];
static lv_obj_t * txt_efic;
static lv_style_t style_indic_bar;
static lv_style_t style_indic_bar_vd;
static lv_style_t style_fundo;
#if LV_USE_BACKGROUND
	static lv_obj_t * img_fundo;
#endif

const int32_t e_pos_x[16] = {10, 19, 28, 37, 47, 56, 65, 74, 83,
                             92, 102, 111, 120, 129, 138, 147};

#if LV_USE_FILESYSTEM == 0
	LV_IMG_DECLARE(tela_reading_efic);
	LV_IMG_DECLARE(Btn_prev);
	LV_IMG_DECLARE(Btn_next);
#endif

void screen_reading_efic(void)
{
	// Create a Screen
	Tela_Reading_EFIC = lv_obj_create(NULL, NULL);
	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_Reading_EFIC, &style_fundo); 					// Configura o estilo criado

#if LV_USE_BACKGROUND
	// Imagem de Fundo
	img_fundo = lv_img_create(Tela_Reading_EFIC, NULL);
#if	LV_USE_FILESYSTEM
	lv_img_set_src(img_fundo, "P:/EX15-XT/img/tela_reading_efic.bin");
#else
	lv_img_set_src(img_fundo, &tela_reading_efic);
#endif
	lv_obj_set_protect(img_fundo, LV_PROTECT_POS);
#endif

	btn_next_efic();
	btn_prev_efic();
	create_vumeter_eficiencia();
	print_eficiencia();
	update_vumeter_eficiencia();

    lv_scr_load(Tela_Reading_EFIC);

	// Task Update Main Screen
    Task_Reading_EFIC = lv_task_create(update_screen_efic, 500, LV_TASK_PRIO_MID, NULL);
    TelaAtiva = TelaReading_Efic;
}

void btn_next_efic(void)
{
	// Create an Image button
	imgbtn_next = lv_imgbtn_create(Tela_Reading_EFIC, NULL);
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
	lv_obj_set_event_cb(imgbtn_next, btn_event_next_efic);
	lv_obj_set_pos(imgbtn_next, 112, 18);
}

static void btn_event_next_efic(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button Next Released SCREEN_READING_STATUS\n");
		lv_task_del(Task_Reading_EFIC);
		lv_obj_del(Tela_Reading_EFIC);
		screen_reading_status();
	}
}

void btn_prev_efic(void)
{
	// Create an Image button
	imgbtn_prev = lv_imgbtn_create(Tela_Reading_EFIC, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_prev.bin");
#else
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_REL, &Btn_prev);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_REL, &Btn_prev);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_TGL_PR, &Btn_prev);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_PR, &Btn_prev);
	lv_imgbtn_set_src(imgbtn_prev, LV_BTN_STATE_INA, &Btn_prev);
#endif
	lv_obj_set_event_cb(imgbtn_prev, btn_event_prev_efic);
	lv_obj_set_pos(imgbtn_prev, 32, 18);
}

static void btn_event_prev_efic(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button ESC Released\n");
		lv_task_del(Task_Reading_EFIC);
		lv_obj_del(Tela_Reading_EFIC);
		screen_reading_vpa();
	}
}

void create_vumeter_eficiencia(void)
{
	// Area do Barguraph Frequency
	// Indicador OFF
	lv_style_copy(&style_indic_bar, &lv_style_pretty_color);
	style_indic_bar.body.radius = 0;
	style_indic_bar.body.grad_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.main_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.border.color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.shadow.color = LV_COLOR_MAKE(105, 105, 105);


	lv_style_copy(&style_indic_bar_vd, &lv_style_pretty_color);
	style_indic_bar_vd.body.radius = 0;
	style_indic_bar_vd.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.main_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.border.color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.shadow.color = LV_COLOR_MAKE(0, 255, 0);

	// Create a default bar
	for(uint8_t x = 0; x < 16; x++) {
		bar_e[x] = lv_bar_create(Tela_Reading_EFIC, NULL);
		lv_obj_set_size(bar_e[x], 5, 19);
		lv_bar_set_style(bar_e[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_e[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_e[x], NULL, LV_ALIGN_IN_TOP_LEFT, e_pos_x[x], 76);
		lv_bar_set_value(bar_e[x], 100, 1);
	}
}

void update_vumeter_eficiencia(void)
{
	uint32_t x;
	float resul;

	if((float)forward != 0 || (float)(f_vpa * f_ipa) != 0) {
		resul = (((forward / (f_vpa * f_ipa)) * 100) / 5) - 1;
	}
	else {
		resul = 0;
	}

	if(resul > 15) resul = 15;

	for(x = 0; x <= resul; x++) {
		lv_bar_set_style(bar_e[x], LV_BAR_STYLE_BG, &style_indic_bar_vd);
		lv_bar_set_style(bar_e[x], LV_BAR_STYLE_INDIC, &style_indic_bar_vd);
	}
}

void print_eficiencia(void)
{
	float resul ;

	if((float)forward != 0 || (float)(f_vpa * f_ipa) != 0) {
		resul = (forward / (f_vpa * f_ipa)) * 100;
	}
	else {
		resul = 0;
	}

	sprintf(buffer, "%0.1f", resul);

	// Area de Refletida
	static lv_style_t style_txt1;
	lv_style_copy(&style_txt1, &lv_style_plain);
	style_txt1.text.font = &lv_font_eurostile_24;					// &lv_font_eurostile_28;
	style_txt1.text.letter_space = 1;
	style_txt1.text.line_space = 1;
	style_txt1.text.color = LV_COLOR_CYAN;

	// Cria um novo rotulo
	txt_efic = lv_label_create(Tela_Reading_EFIC, NULL);
	lv_obj_set_style(txt_efic, &style_txt1); 						// Configura o estilo criado
	lv_label_set_long_mode(txt_efic, LV_LABEL_LONG_EXPAND); 		// Quebra as linhas longas
	lv_label_set_recolor(txt_efic, true); 							// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_efic, LV_ALIGN_IN_TOP_RIGHT); 			// Centraliza linhas alinhadas
	lv_label_set_text(txt_efic, buffer);
	lv_obj_set_width(txt_efic, 300); 								// Configuura o comprimento
	lv_obj_align(txt_efic, NULL, LV_ALIGN_IN_TOP_RIGHT, -70, 45); 	// Alinha ao centro
}


static void update_screen_efic(lv_task_t * param)
{
	float resul ;

	if((float)forward != 0 || (float)(f_vpa * f_ipa) != 0) {
		resul = (forward / (f_vpa * f_ipa)) * 100;
	}
	else {
		resul = 0;
	}

	sprintf(buffer, "%0.1f", resul);
	lv_label_set_text(txt_efic, buffer);
	update_vumeter_eficiencia();

}

void ButtonEventTelaReading_Efic(uint8_t event, uint8_t tipo, uint8_t id)
{
	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			switch(id) {
				case KEY_DN:
					lv_event_send(imgbtn_prev, LV_EVENT_APPLY, NULL);
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
