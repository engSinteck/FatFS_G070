/*
 * screen_rf.c
 *
 *  Created on: 4 de jul de 2019
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */

#include <GUI/GUI_EX15-XT.h>
#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "key.h"
#include "math.h"

static void event_handler_target(lv_obj_t * obj, lv_event_t event);
static void event_handler_swr(lv_obj_t * obj, lv_event_t event);
//static void btn_event_prev_rf2(lv_obj_t * btn, lv_event_t event);
static void btn_event_next_rf1(lv_obj_t * btn, lv_event_t event);
static void btn_event_next_rf(lv_obj_t * btn, lv_event_t event);
//static void btn_event_esc_rf2(lv_obj_t * btn, lv_event_t event);
static void btn_power(lv_obj_t * btn, lv_event_t event);
//static void btn_event_esc_rf1(lv_obj_t * btn, lv_event_t event);
static void btn_event_esc_rf(lv_obj_t * btn, lv_event_t event);
static void update_rf(lv_task_t * param);
static void update_rf_1(lv_task_t * param);


extern char buffer[];
extern uint8_t RFEnable;
extern float forward, max_rfl, target;
extern uint32_t TelaAtiva, Max_Reflected, Target_Int;

uint32_t TelaProgRF = 0, IndiceRF = 0, ProgRF = 0;
uint16_t roller_swr[2];
uint32_t TelaProgRF_1 = 0, IndiceRF_1 = 0, ProgRF_1 = 0;
uint16_t roller_target[2];
uint32_t TelaProgRF_2 = 0, IndiceRF_2 = 0;

static lv_obj_t * Tela_RF;
static lv_obj_t * Tela_RF_1;
static lv_obj_t * Tela_RF_2;
static lv_task_t * Task_RF;
static lv_task_t * Task_RF_1;
static lv_obj_t * bar_swr[20];
static lv_obj_t * bar_fwd[20];
static lv_obj_t * bar_pwr[10];
static lv_obj_t * imgbtn1[2];
static lv_obj_t * imgbtn_next[2];
static lv_obj_t *rollerswr[2];
static lv_obj_t *rollertarget[2];
static lv_style_t style_indic_bar;
static lv_style_t style_indic_bar_vd;
static lv_style_t style_roller_anim;
static lv_style_t style_roller_b;
static lv_style_t style_roller_bg;
static lv_style_t style_roller_save;
static lv_style_t style_fundo;
static lv_anim_t sa;

#if LV_USE_BACKGROUND
	static lv_obj_t * img_fundo;
	static lv_obj_t * img_fundo_1;
	static lv_obj_t * img_fundo_2;
#endif

const int32_t swr_pos_x[20] = {8, 15, 23, 30, 37, 44, 51, 59, 66, 73,
                               81, 88, 95, 102, 110, 117, 124, 132, 139, 146};

const int32_t fwd_pos_x[20] = {8, 16, 23, 30, 38, 45, 52, 60, 67, 74,
                               82, 89, 96, 103, 111, 118, 125, 133, 140, 147};

const int32_t pwr_pos_x[20] = {6, 14, 22, 30, 37, 45, 53, 61, 68, 76};

#if LV_USE_FILESYSTEM == 0
	LV_IMG_DECLARE(tela_rf);
	LV_IMG_DECLARE(tela_rf_1);
	LV_IMG_DECLARE(tela_rf_2);
	LV_IMG_DECLARE(Btn_poweron);
	LV_IMG_DECLARE(Btn_poweron_vd);
	LV_IMG_DECLARE(Btn_poweron_am);
	LV_IMG_DECLARE(Btn_poweroff);
	LV_IMG_DECLARE(Btn_poweroff_vm);
	LV_IMG_DECLARE(Btn_poweroff_am);
	LV_IMG_DECLARE(BtnESC);
	LV_IMG_DECLARE(Btn_next);
	LV_IMG_DECLARE(Btn_prev);
#endif

void screen_rf(void)
{
	// Create a Screen
	Tela_RF = lv_obj_create(NULL, NULL);
	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_RF, &style_fundo); 					// Configura o estilo criado

#if LV_USE_BACKGROUND
	// Imagem de Fundo
	img_fundo = lv_img_create(Tela_RF, NULL);
#if	LV_USE_FILESYSTEM
    lv_img_set_src(img_fundo, "P:/EX15-XT/img/tela_rf.bin");
#else
    lv_img_set_src(img_fundo, &tela_rf);
#endif
	lv_obj_set_protect(img_fundo, LV_PROTECT_POS);
	lv_obj_set_event_cb(img_fundo, btn_event_esc_rf);
	lv_obj_set_click(img_fundo, 1);
#endif

	btn_next_rf();
	create_vumeter_swr();
	prog_swr(Max_Reflected);
	update_vumeter_swr(Max_Reflected);
	lv_scr_load(Tela_RF);

	// Task Update Vu-Meter
	Task_RF = lv_task_create(update_rf, 500, LV_TASK_PRIO_MID, NULL);
	TelaAtiva = TelaRF;
}

void screen_RF_1(void)
{
	// Create a Screen
	Tela_RF_1 = lv_obj_create(NULL, NULL);
	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_RF_1, &style_fundo); 					// Configura o estilo criado

#if LV_USE_BACKGROUND
	// Imagem de Fundo
	img_fundo_1 = lv_img_create(Tela_RF_1, NULL);
#if	LV_USE_FILESYSTEM
	lv_img_set_src(img_fundo_1, "P:/EX15-XT/img/tela_rf_1.bin");
#else
	lv_img_set_src(img_fundo_1, &tela_rf_1);
#endif
	lv_obj_set_protect(img_fundo_1, LV_PROTECT_POS);
	lv_obj_set_event_cb(img_fundo_1, btn_event_esc_rf1);
	lv_obj_set_click(img_fundo_1, 1);
#endif

	btn_next_rf1();
	create_vumeter_fwd();
	prog_target(target);
	update_vumeter_fwd(target);
	lv_scr_load(Tela_RF_1);

	// Task Update Vu-Meter
	Task_RF_1 = lv_task_create(update_rf_1, 500, LV_TASK_PRIO_MID, NULL);
	TelaAtiva = TelaRF_1;
}

void screen_RF_2(void)
{
	// Create a Screen
	Tela_RF_2 = lv_obj_create(NULL, NULL);
	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_RF_2, &style_fundo); 					// Configura o estilo criado

#if LV_USE_BACKGROUND
	// Imagem de Fundo
	img_fundo_2 = lv_img_create(Tela_RF_2, NULL);
#if	LV_USE_FILESYSTEM
	lv_img_set_src(img_fundo_2, "P:/EX15-XT/img/tela_rf_2.bin");
#else
	lv_img_set_src(img_fundo_2, &tela_rf_2);
#endif
	lv_obj_set_protect(img_fundo_2, LV_PROTECT_POS);
	lv_obj_set_event_cb(img_fundo_2, btn_event_esc_rf2);
	lv_obj_set_click(img_fundo_2, 1);
#endif

	// POWER ON
	imgbtn1[0] = lv_imgbtn_create(Tela_RF_2, NULL);
	lv_obj_set_user_data(imgbtn1[0], 0);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_poweron.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_poweron_vd.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_poweron.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_poweron.bin");
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_poweron.bin_am");
#else
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_REL, &Btn_poweron);
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_REL, &Btn_poweron_vd);
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_TGL_PR, &Btn_poweron);
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_PR, &Btn_poweron);
	lv_imgbtn_set_src(imgbtn1[0], LV_BTN_STATE_INA, &Btn_poweron_am);
#endif

	lv_obj_set_event_cb(imgbtn1[0], btn_power);
	lv_obj_set_pos(imgbtn1[0], 1, 35);
	// POWER OFF
	imgbtn1[1] = lv_imgbtn_create(Tela_RF_2, imgbtn1[0]);
	lv_obj_set_user_data(imgbtn1[1], 1);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_poweroff.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_poweroff_vm.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_poweroff.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_poweroff.bin");
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_poweroff_am.bin");
#else
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_REL, &Btn_poweroff);
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_REL, &Btn_poweroff_vm);
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_TGL_PR, &Btn_poweroff);
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_PR, &Btn_poweroff);
	lv_imgbtn_set_src(imgbtn1[1], LV_BTN_STATE_INA, &Btn_poweroff_am);
#endif
	lv_obj_set_pos(imgbtn1[1], 1, 63);

	if(RFEnable) {
		lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
		lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
	}
	else {
		lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
		lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
	}

	//btn_prev_rf2();
	create_vumeter_pwr();
	update_vumeter_pwr(forward);
	print_pwr(forward);
	lv_scr_load(Tela_RF_2);
	TelaAtiva = TelaRF_2;
}

void create_vumeter_swr(void)
{
	// Area do Bargraph Frequency
	// Indicador OFF
	lv_style_copy(&style_indic_bar, &lv_style_pretty);
	style_indic_bar.body.radius = 0;
	style_indic_bar.body.grad_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.main_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.border.color = LV_COLOR_MAKE(105, 105, 105);

	lv_style_copy(&style_indic_bar_vd, &lv_style_pretty);
	style_indic_bar_vd.body.radius = 0;
	style_indic_bar_vd.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.main_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.border.color = LV_COLOR_MAKE(0, 255, 0);

	// Create a default bar
	for(uint8_t x = 0; x < 20; x++) {
		bar_swr[x] = lv_bar_create(Tela_RF, NULL);
		lv_obj_set_size(bar_swr[x], 4, 18);
		lv_bar_set_style(bar_swr[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_swr[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_swr[x], NULL, LV_ALIGN_IN_TOP_LEFT, swr_pos_x[x], 86);
		lv_bar_set_value(bar_swr[x], 100, 0);
	}
}

void create_vumeter_pwr(void)
{
	// Area do Bargraph
	// Indicador OFF
	lv_style_copy(&style_indic_bar, &lv_style_pretty);
	style_indic_bar.body.radius = 0;
	style_indic_bar.body.grad_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.main_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.border.color = LV_COLOR_MAKE(105, 105, 105);

	lv_style_copy(&style_indic_bar_vd, &lv_style_pretty);
	style_indic_bar_vd.body.radius = 0;
	style_indic_bar_vd.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.main_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.border.color = LV_COLOR_MAKE(0, 255, 0);

	// Create a default bar
	for(uint8_t x = 0; x < 10; x++) {
		bar_pwr[x] = lv_bar_create(Tela_RF_2, NULL);
		lv_obj_set_size(bar_pwr[x], 4, 20);
		lv_bar_set_style(bar_pwr[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_pwr[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_pwr[x], NULL, LV_ALIGN_IN_TOP_LEFT, pwr_pos_x[x], 93);
		lv_bar_set_value(bar_pwr[x], 100, 0);
	}
}

void update_vumeter_swr(uint32_t swr)
{
	uint32_t y;
	uint32_t resul;

	// Create a default bar
	for(uint8_t x = 0; x < 20; x++) {
		lv_bar_set_style(bar_swr[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_swr[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
	}

	if(swr != 0) {
		resul = (uint32_t) (((((swr / 10) / target) * 100) / 2.50f)) ;
		//logI("Debug: Task_SWR Target: %0.1f SWR: %ld Resul: %ld\n", target, swr, resul);
		if(resul >= 20) resul = 19;

		for(y = 0; y <= resul; y++) {
			lv_bar_set_style(bar_swr[y], LV_BAR_STYLE_BG, &style_indic_bar_vd);
			lv_bar_set_style(bar_swr[y], LV_BAR_STYLE_INDIC, &style_indic_bar_vd);
		}
	}
}

void update_vumeter_fwd(float fwd)
{
	uint32_t y;
	uint32_t resul;

	// Create a default bar
	for(uint8_t x = 0; x < 20; x++) {
		lv_bar_set_style(bar_fwd[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_fwd[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
	}

	if(fwd != 0) {
		resul = (uint32_t) ((((((float)fwd) / 15.0f) * 100) / 5.0f) - 1) ;
		if(resul >= 20) resul = 19;
		//printf("Debug: Task_FWD FWD: %0.1f Resul: %ld\n", fwd, resul);

		for(y = 0; y <= resul; y++) {
			lv_bar_set_style(bar_fwd[y], LV_BAR_STYLE_BG, &style_indic_bar_vd);
			lv_bar_set_style(bar_fwd[y], LV_BAR_STYLE_INDIC, &style_indic_bar_vd);
		}
	}
}

void update_vumeter_pwr(float fwd)
{
	uint32_t x;
	uint32_t resul = (uint32_t)( (((fwd / 15.0f) * 100) / 10) - 1);
	if(resul > 10) resul = 9;

	for(x = 0; x <= resul; x++) {
		lv_bar_set_style(bar_pwr[x], LV_BAR_STYLE_BG, &style_indic_bar_vd);
		lv_bar_set_style(bar_pwr[x], LV_BAR_STYLE_INDIC, &style_indic_bar_vd);
	}
}

void print_pwr(float pwr)
{
	// Area de Potencia
	static lv_style_t style_txt_fwd;
	lv_style_copy(&style_txt_fwd, &lv_style_plain);
	style_txt_fwd.text.font = &lv_font_eurostile_24;					// &lv_font_eurostile_22;
	style_txt_fwd.text.letter_space = 1;
	style_txt_fwd.text.line_space = 1;
	style_txt_fwd.text.color = LV_COLOR_CYAN;

	lv_obj_t *txt_fwd = lv_label_create(Tela_RF_2, NULL);
	lv_obj_set_style(txt_fwd, &style_txt_fwd); 							// Configura o estilo criado
	lv_label_set_long_mode(txt_fwd, LV_LABEL_LONG_BREAK); 				// Quebra as linhas longas
	lv_label_set_recolor(txt_fwd, true); 								// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_fwd, LV_ALIGN_IN_BOTTOM_RIGHT); 				// Centraliza linhas alinhadas
	sprintf(buffer, "%0.1f", pwr);
	lv_label_set_text(txt_fwd, buffer);
	lv_obj_set_width(txt_fwd, 200); 									// Configuura o comprimento
	lv_obj_align(txt_fwd, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -27, -12); 	// Alinha ao centro
}

void print_reflected(float swr)
{
	sprintf(buffer, "%0.1f", swr);

	// Area de Refletida
	static lv_style_t style_txt1;
	lv_style_copy(&style_txt1, &lv_style_plain);
	style_txt1.text.font = &lv_font_eurostile_24; 					// &lv_font_eurostile_28;
	style_txt1.text.letter_space = 1;
	style_txt1.text.line_space = 1;
	style_txt1.text.color = LV_COLOR_WHITE;

	// Cria um novo rotulo
	lv_obj_t * txt_swr = lv_label_create(Tela_RF, NULL);
	lv_obj_set_style(txt_swr, &style_txt1); 						// Configura o estilo criado
	lv_label_set_long_mode(txt_swr, LV_LABEL_LONG_EXPAND); 			// Quebra as linhas longas
	lv_label_set_recolor(txt_swr, true); 							// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_swr, LV_ALIGN_IN_TOP_LEFT); 				// Centraliza linhas alinhadas
	lv_label_set_text(txt_swr, "1");
	lv_obj_set_width(txt_swr, 300); 								// Configuura o comprimento
	lv_obj_align(txt_swr, NULL, LV_ALIGN_IN_TOP_LEFT, 46, 41); 		// Alinha ao centro

	// Cria um novo rotulo
	lv_obj_t * txt_swr1 = lv_label_create(Tela_RF, NULL);
	lv_obj_set_style(txt_swr1, &style_txt1); 						// Configura o estilo criado
	lv_label_set_long_mode(txt_swr1, LV_LABEL_LONG_EXPAND); 		// Quebra as linhas longas
	lv_label_set_recolor(txt_swr1, true); 							// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_swr1, LV_ALIGN_IN_TOP_LEFT); 			// Centraliza linhas alinhadas
	lv_label_set_text(txt_swr1, "2");
	lv_obj_set_width(txt_swr1, 300); 								// Configuura o comprimento
	lv_obj_align(txt_swr1, NULL, LV_ALIGN_IN_TOP_LEFT, 93, 41); 	// Alinha ao centro
}


void print_forward(float fwd)
{
	sprintf(buffer, "%0.1f", fwd);

	// Area de Refletida
	static lv_style_t style_txt1;
	lv_style_copy(&style_txt1, &lv_style_plain);
	style_txt1.text.font = &lv_font_eurostile_24;					// &lv_font_eurostile_28
	style_txt1.text.letter_space = 1;
	style_txt1.text.line_space = 1;
	style_txt1.text.color = LV_COLOR_WHITE;

	// Cria um novo rotulo
	lv_obj_t * txt_fwd = lv_label_create(Tela_RF, NULL);
	lv_obj_set_style(txt_fwd, &style_txt1); 						// Configura o estilo criado
	lv_label_set_long_mode(txt_fwd, LV_LABEL_LONG_EXPAND); 			// Quebra as linhas longas
	lv_label_set_recolor(txt_fwd, true); 							// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_fwd, LV_ALIGN_IN_TOP_LEFT); 				// Centraliza linhas alinhadas
	lv_label_set_text(txt_fwd, "1");
	lv_obj_set_width(txt_fwd, 300); 								// Configuura o comprimento
	lv_obj_align(txt_fwd, NULL, LV_ALIGN_IN_TOP_LEFT, 46, 41); 		// Alinha ao centro

	// Cria um novo rotulo
	lv_obj_t * txt_fwd1 = lv_label_create(Tela_RF, NULL);
	lv_obj_set_style(txt_fwd1, &style_txt1); 						// Configura o estilo criado
	lv_label_set_long_mode(txt_fwd1, LV_LABEL_LONG_EXPAND); 		// Quebra as linhas longas
	lv_label_set_recolor(txt_fwd1, true); 							// Ativa recolorizar por comandos no texto
	lv_label_set_align(txt_fwd1, LV_ALIGN_IN_TOP_LEFT); 			// Centraliza linhas alinhadas
	lv_label_set_text(txt_fwd1, "5");
	lv_obj_set_width(txt_fwd1, 300); 								// Configuura o comprimento
	lv_obj_align(txt_fwd1, NULL, LV_ALIGN_IN_TOP_LEFT, 93, 41); 	// Alinha ao centro
}

void create_vumeter_fwd(void)
{
	// Area do Bargraph
	// Indicador OFF
	lv_style_copy(&style_indic_bar, &lv_style_pretty);
	style_indic_bar.body.radius = 0;
	style_indic_bar.body.grad_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.main_color = LV_COLOR_MAKE(105, 105, 105);
	style_indic_bar.body.border.color = LV_COLOR_MAKE(105, 105, 105);

	lv_style_copy(&style_indic_bar_vd, &lv_style_pretty);
	style_indic_bar_vd.body.radius = 0;
	style_indic_bar_vd.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.main_color = LV_COLOR_MAKE(0, 255, 0);
	style_indic_bar_vd.body.border.color = LV_COLOR_MAKE(0, 255, 0);

	// Create a default bar
	for(uint8_t x = 0; x < 20; x++) {
		bar_fwd[x] = lv_bar_create(Tela_RF, NULL);
		lv_obj_set_size(bar_fwd[x], 4, 19);
		lv_bar_set_style(bar_fwd[x], LV_BAR_STYLE_BG, &style_indic_bar);
		lv_bar_set_style(bar_fwd[x], LV_BAR_STYLE_INDIC, &style_indic_bar);
		lv_obj_align(bar_fwd[x], NULL, LV_ALIGN_IN_TOP_LEFT, fwd_pos_x[x], 86);
		lv_bar_set_value(bar_fwd[x], 100, 0);
	}
}

void btn_esc_rf(void)
{
	// Create an Image button
	lv_obj_t * imgbtn1 = lv_imgbtn_create(Tela_RF, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, "P:/EX15-XT/img/BtnESC.bin");
#else
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, &BtnESC);
#endif
	lv_obj_set_event_cb(imgbtn1, btn_event_esc_rf);
	lv_obj_set_pos(imgbtn1, 143, 2);
}

static void btn_event_esc_rf(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("TELA RF - Button ESC Released\n");
		lv_task_del(Task_RF);
		lv_obj_del(Tela_RF);
		screen_sel();
	}
}

/*
void btn_esc_rf1(void)
{
	// Create an Image button
	lv_obj_t * imgbtn1 = lv_imgbtn_create(Tela_RF, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, "P:/EX15-XT/img/BtnESC.bin");
#else
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, &BtnESC);
#endif
	lv_obj_set_event_cb(imgbtn1, btn_event_esc_rf1);
	lv_obj_set_pos(imgbtn1, 143, 2);
}
*/
/*
static void btn_event_esc_rf1(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button ESC  1 Released\n");
		lv_task_del(Task_RF_1);
		lv_obj_del(Tela_RF_1);
		screen_rf();
	}
}
*/

static void btn_power(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_RELEASED) {
		uint32_t id = lv_obj_get_user_data(btn);
		//printf("Button Power %d Released\n" , id);
		if(id == 0) {
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
			RFEnable = 1;
			//HAL_GPIO_WritePin(RF_ENB_GPIO_Port, RF_ENB_Pin, GPIO_PIN_SET);
		}
		else if(id == 1) {
			lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
			lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
			RFEnable = 0;
			//HAL_GPIO_WritePin(RF_ENB_GPIO_Port, RF_ENB_Pin, GPIO_PIN_RESET);
		}
		//
		//printf("RFEnable: %d\n", RFEnable);
	}
}

/*
void btn_esc_rf2(void)
{
	// Create an Image button
	lv_obj_t * imgbtn1 = lv_imgbtn_create(Tela_RF, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, "P:/EX15-XT/img/BtnESC.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, "P:/EX15-XT/img/BtnESC.bin");
#else
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, &BtnESC);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, &BtnESC);
#endif
	lv_obj_set_event_cb(imgbtn1, btn_event_esc_rf2);
	lv_obj_set_pos(imgbtn1, 143, 2);
}
*/
/*
static void btn_event_esc_rf2(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button ESC  2 Released\n");
		lv_obj_del(Tela_RF_2);
		screen_RF_1();
	}
}
*/

void btn_next_rf(void)
{
	// Create an Image button
	imgbtn_next[0] = lv_imgbtn_create(Tela_RF, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_next.bin");
#else
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_TGL_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_TGL_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[0], LV_BTN_STATE_INA, &Btn_next);
#endif
	lv_obj_set_event_cb(imgbtn_next[0], btn_event_next_rf);
	lv_obj_set_pos(imgbtn_next[0], 112, 18);
}

static void btn_event_next_rf(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("TELA_RF - Button Next Released\n");
		lv_task_del(Task_RF);
		lv_obj_del(Tela_RF);
		screen_RF_1();
	}
}

void btn_next_rf1(void)
{
	// Create an Image button
	imgbtn_next[1] = lv_imgbtn_create(Tela_RF_1, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_next.bin");
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_next.bin");
#else
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_TGL_REL, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_TGL_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_PR, &Btn_next);
	lv_imgbtn_set_src(imgbtn_next[1], LV_BTN_STATE_INA, &Btn_next);
#endif
	lv_obj_set_event_cb(imgbtn_next[1], btn_event_next_rf1);
	lv_obj_set_pos(imgbtn_next[1], 112, 18);
}

static void btn_event_next_rf1(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button Next  1 Released\n");
		lv_task_del(Task_RF_1);
		lv_obj_del(Tela_RF_1);
		screen_RF_2();
	}
}

/*
void btn_prev_rf2(void)
{
	// Create an Image button
	lv_obj_t * imgbtn1 = lv_imgbtn_create(Tela_RF_1, NULL);
#if	LV_USE_FILESYSTEM
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, "P:/EX15-XT/img/Btn_prev.bin");
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, "P:/EX15-XT/img/Btn_prev.bin");
#else
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_REL, &Btn_prev);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_REL, &Btn_prev);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_TGL_PR, &Btn_prev);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_PR, &Btn_prev);
	lv_imgbtn_set_src(imgbtn1, LV_BTN_STATE_INA, &Btn_prev);
#endif
	lv_obj_set_event_cb(imgbtn1, btn_event_prev_rf2);
	lv_obj_set_pos(imgbtn1, 32, 18);
}


static void btn_event_prev_rf2(lv_obj_t * btn, lv_event_t event)
{
	if(event == LV_EVENT_APPLY) {
		//printf("Button Prev 2 Released\n");
		lv_obj_del(Tela_RF_2);
		screen_RF_1();
	}
}
*/

void prog_swr(uint32_t swr)
{
	uint32_t un, ml;

	if( swr > 0) {
		un = swr / 10;
		ml = (swr % 10);
		//logI("Debug: Prog_SWR SWR: %ld  Unidade: %d, Mantissa: %d\n", swr, un, ml);
	}
	else {
		un = 0;
		ml = 0;
	}
	lv_style_copy(&style_roller_anim, &lv_style_plain_color);
	style_roller_anim.body.main_color = LV_COLOR_GRAY;
	style_roller_anim.body.grad_color = LV_COLOR_BLACK;
	style_roller_anim.text.font = &lv_font_eurostile_24;
	style_roller_anim.text.letter_space = 2;
	style_roller_anim.text.line_space = 24;
	style_roller_anim.text.color = LV_COLOR_WHITE;

	lv_style_copy(&style_roller_b, &lv_style_plain_color);
	style_roller_b.body.main_color = LV_COLOR_GRAY;
	style_roller_b.body.grad_color = LV_COLOR_BLACK;
	style_roller_b.text.font = &lv_font_eurostile_24;
	style_roller_b.text.letter_space = 2;
	style_roller_b.text.line_space = 24;
	style_roller_b.text.color = LV_COLOR_WHITE;

	lv_style_copy(&style_roller_bg, &lv_style_plain_color);
	style_roller_bg.body.main_color = LV_COLOR_YELLOW;
	style_roller_bg.body.grad_color = LV_COLOR_YELLOW;
	style_roller_bg.text.font = &lv_font_eurostile_24;
	style_roller_bg.text.letter_space = 2;
	style_roller_bg.text.line_space = 24;
	style_roller_bg.text.color = LV_COLOR_BLACK;

	lv_style_copy(&style_roller_save, &lv_style_plain_color);
	style_roller_save.body.main_color = LV_COLOR_LIME;
	style_roller_save.body.grad_color = LV_COLOR_LIME;
	style_roller_save.text.font = &lv_font_eurostile_24;
	style_roller_save.text.letter_space = 2;
	style_roller_save.text.line_space = 24;
	style_roller_save.text.color = LV_COLOR_WHITE;

	// Animate the new style
	lv_style_anim_init(&sa);
	lv_style_anim_set_styles(&sa, &style_roller_anim, &style_roller_b, &style_roller_bg);
	lv_style_anim_set_time(&sa, 500, 500);
	lv_style_anim_set_playback(&sa, 500);
	lv_style_anim_set_repeat(&sa, 500);
	lv_style_anim_create(&sa);

	// Unidade
	rollerswr[0] = lv_roller_create(Tela_RF, NULL);
	lv_obj_set_user_data(rollerswr[0], 0);
    lv_roller_set_options(rollerswr[0], "0\n1\n2\n3\n4\n5\n6\n7\n8\n9", LV_ROLLER_MODE_INIFINITE);
    lv_roller_set_visible_row_count(rollerswr[0], 2);
    lv_roller_set_selected(rollerswr[0], un, true);
    lv_roller_set_fix_width(rollerswr[0], 34);
    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_BG, &style_roller_b);
    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
    lv_obj_align(rollerswr[0], NULL, LV_ALIGN_IN_TOP_LEFT, 39, 40);
    lv_obj_set_event_cb(rollerswr[0], event_handler_swr);
    // Mantis
    rollerswr[1] = lv_roller_create(Tela_RF, NULL);
    lv_obj_set_user_data(rollerswr[1], 1);
    lv_roller_set_options(rollerswr[1], "0\n1\n2\n3\n4\n5\n6\n7\n8\n9", LV_ROLLER_MODE_INIFINITE);
    lv_roller_set_visible_row_count(rollerswr[1], 2);
    lv_roller_set_selected(rollerswr[1], ml, true);
    lv_roller_set_fix_width(rollerswr[1], 35);
    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_BG, &style_roller_b);
    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
    lv_obj_align(rollerswr[1], NULL, LV_ALIGN_IN_TOP_LEFT, 84, 40);
    lv_obj_set_event_cb(rollerswr[1], event_handler_swr);
}


static void event_handler_swr(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        char buf[32];
        char out[6];

//        uint32_t id = lv_obj_get_user_data(obj);

        lv_roller_get_selected_str(obj, buf, sizeof(buf));

        lv_roller_get_selected_str(rollerswr[0], buf, sizeof(buf));
        out[0] = buf[0];
        lv_roller_get_selected_str(rollerswr[1], buf, sizeof(buf));
        out[1] = buf[0];
        out[3] = 0;

        max_rfl = (float)atoi(out);
        //logI("Selected Roller: %d Digit: %s  MAX_RFL: %0.1fW B0: %d B1: %d\n", id, buf, max_rfl, buf[0], buf[1]);
    }
}


void prog_target(float fwd)
{
	uint32_t teste, un, ml;

	if( fwd > 0) {
		teste = (uint32_t) fwd * 10;
		un = teste / 100;
		ml = (teste % 100) / 10;
		//printf("Debug: Prog_TARGET Target: %0.1f Teste: %ld  Unidade: %d, Mantissa: %d\n", fwd, teste, un, ml);
	}
	else {
		un = 0;
		ml = 0;
	}

	lv_style_copy(&style_roller_anim, &lv_style_plain_color);
	style_roller_anim.body.main_color = LV_COLOR_GRAY;
	style_roller_anim.body.grad_color = LV_COLOR_BLACK;
	style_roller_anim.text.font = &lv_font_eurostile_24;
	style_roller_anim.text.letter_space = 2;
	style_roller_anim.text.line_space = 24;
	style_roller_anim.text.color = LV_COLOR_WHITE;

	lv_style_copy(&style_roller_b, &lv_style_plain_color);
	style_roller_b.body.main_color = LV_COLOR_GRAY;
	style_roller_b.body.grad_color = LV_COLOR_BLACK;
	style_roller_b.text.font = &lv_font_eurostile_24;
	style_roller_b.text.letter_space = 2;
	style_roller_b.text.line_space = 24;
	style_roller_b.text.color = LV_COLOR_WHITE;

	lv_style_copy(&style_roller_bg, &lv_style_plain_color);
	style_roller_bg.body.main_color = LV_COLOR_YELLOW;
	style_roller_bg.body.grad_color = LV_COLOR_YELLOW;
	style_roller_bg.text.font = &lv_font_eurostile_24;
	style_roller_bg.text.letter_space = 2;
	style_roller_bg.text.line_space = 24;
	style_roller_bg.text.color = LV_COLOR_BLACK;

	lv_style_copy(&style_roller_save, &lv_style_plain_color);
	style_roller_save.body.main_color = LV_COLOR_LIME;
	style_roller_save.body.grad_color = LV_COLOR_LIME;
	style_roller_save.text.font = &lv_font_eurostile_24;
	style_roller_save.text.letter_space = 2;
	style_roller_save.text.line_space = 24;
	style_roller_save.text.color = LV_COLOR_WHITE;

	// Animate the new style
	lv_style_anim_init(&sa);
	lv_style_anim_set_styles(&sa, &style_roller_anim, &style_roller_b, &style_roller_bg);
	lv_style_anim_set_time(&sa, 500, 500);
	lv_style_anim_set_playback(&sa, 500);
	lv_style_anim_set_repeat(&sa, 500);
	lv_style_anim_create(&sa);

	// Milhar
	rollertarget[0] = lv_roller_create(Tela_RF, NULL);
	lv_obj_set_user_data(rollertarget[0], 5);
    lv_roller_set_options(rollertarget[0], "0\n1", LV_ROLLER_MODE_INIFINITE);
    lv_roller_set_visible_row_count(rollertarget[0], 2);
    lv_roller_set_selected(rollertarget[0], un, true);
    lv_roller_set_fix_width(rollertarget[0], 34);
    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_BG, &style_roller_b);
    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
    lv_obj_align(rollertarget[0], NULL, LV_ALIGN_IN_TOP_LEFT, 39, 40);
    lv_obj_set_event_cb(rollertarget[0], event_handler_target);
    // Centena
    rollertarget[1] = lv_roller_create(Tela_RF, NULL);
    lv_obj_set_user_data(rollertarget[1], 4);
    lv_roller_set_options(rollertarget[1], "0\n1\n2\n3\n4\n5\n6\n7\n8\n9", LV_ROLLER_MODE_INIFINITE);
    lv_roller_set_visible_row_count(rollertarget[1], 2);
    lv_roller_set_selected(rollertarget[1], ml, true);
    lv_roller_set_fix_width(rollertarget[1], 35);
    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_BG, &style_roller_b);
    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
    lv_obj_align(rollertarget[1], NULL, LV_ALIGN_IN_TOP_LEFT, 84, 40);
    lv_obj_set_event_cb(rollertarget[1], event_handler_target);
}


static void event_handler_target(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        char buf[32];
        char out[6];

//        uint32_t id = lv_obj_get_user_data(obj);

        lv_roller_get_selected_str(obj, buf, sizeof(buf));


        lv_roller_get_selected_str(rollerswr[0], buf, sizeof(buf));
        out[0] = buf[0];
        lv_roller_get_selected_str(rollerswr[1], buf, sizeof(buf));
        out[1] = buf[0];
        out[5] = 0;

        target = (float)atoi(out);
        //logI("Selected Roller: %d Digit: %s  TARGET: %0.1fW\n", id, buf, target);
    }
}

static void update_rf(lv_task_t * param)
{
	update_vumeter_swr(Max_Reflected);
}

static void update_rf_1(lv_task_t * param)
{
	update_vumeter_fwd(target);
}

void update_style_roller_rf(uint32_t idx)
{
	switch(idx) {
		case 0:
			lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
			break;
		case 1:
			lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_BG, &style_roller_bg);
		    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_SEL, &style_roller_bg);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
			break;
		case 2:
			lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_BG, &style_roller_bg);
		    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_SEL, &style_roller_bg);
			break;
	}
}

void update_style_roller_rf_1(uint32_t idx)
{
	switch(idx) {
		case 0:
			lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
			break;
		case 1:
			lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_BG, &style_roller_bg);
		    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_SEL, &style_roller_bg);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_SEL, &style_roller_b);
			break;
		case 2:
			lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_BG, &style_roller_b);
		    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_SEL, &style_roller_b);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_BG, &style_roller_bg);
		    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_SEL, &style_roller_bg);
			break;
	}
}

void ButtonEventTelaRF(uint8_t event, uint8_t tipo, uint8_t id)
{
    char buf[32];
    char out[6];

	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			//logI("TelaRF - TelaProgRF: %ld IndiceRF: %ld\n", TelaProgRF, IndiceRF);
			switch(id) {
				case KEY_DN:
					if(TelaProgRF == 0) {
						//lv_event_send(img_fundo, LV_EVENT_APPLY, NULL);
						lv_anim_del (sa.var, NULL);
						lv_task_del(Task_RF);
						lv_obj_del(Tela_RF);
						screen_sel();
					}
					else if(TelaProgRF == 1) {
						if(IndiceRF >= 1) IndiceRF--;
						//logI("TelaRF - KEY_DN - IndiceRF: %ld\n", IndiceRF);
						update_style_roller_rf(IndiceRF + 1);
					}
					else if(TelaProgRF == 2) {
						if(IndiceRF == 0) {
							if(roller_swr[0] > 0) roller_swr[0]--;
							lv_roller_set_selected(rollerswr[IndiceRF], roller_swr[0], false);
						}
						else {
							if(roller_swr[1] > 0) roller_swr[1]--;
							lv_roller_set_selected(rollerswr[IndiceRF], roller_swr[1], false);
						}
						//logI("Debug: KEY_DN Roller_SWR: %ld, Dado: %ld %d\n", IndiceRF, roller_swr[0], roller_swr[1]);
					}
					break;
				case KEY_UP:
					if(TelaProgRF == 0) {
						lv_event_send(imgbtn_next[0], LV_EVENT_APPLY, NULL);
					}
					else if(TelaProgRF == 1){
						IndiceRF++;
						if(IndiceRF > 1) IndiceRF = 1;
						//logI("TelaRF - KEY_UP - IndiceRF: %ld\n", IndiceRF);
						update_style_roller_rf(IndiceRF + 1);
					}
					else if(TelaProgRF == 2) {
						if(IndiceRF == 0) {
							roller_swr[0]++;
							if(roller_swr[0] > 2) roller_swr[0] = 0;
							lv_roller_set_selected(rollerswr[IndiceRF], roller_swr[0], false);
						}
						else {
							roller_swr[1]++;
							if(roller_swr[1] > 9) roller_swr[1] = 0;
							lv_roller_set_selected(rollerswr[IndiceRF], roller_swr[1], false);
						}
						//logI("Debug: KEY_UP Roller_SWR: %ld, Dado: %d %d\n", IndiceRF, roller_swr[0], roller_swr[1]);
					}
					break;
				case KEY_ENTER:
					if(TelaProgRF == 0) {
						TelaProgRF = 1;
						IndiceRF = 0;
						update_style_roller_rf(IndiceRF + 1);
					}
					else if(TelaProgRF == 1) {
						TelaProgRF = 2;
						roller_swr[0] = Max_Reflected / 10;
						roller_swr[1] = Max_Reflected % 10;
					    lv_roller_set_style(rollerswr[IndiceRF], LV_ROLLER_STYLE_BG, &style_roller_anim);
					    lv_roller_set_style(rollerswr[IndiceRF], LV_ROLLER_STYLE_SEL, &style_roller_anim);
						//logI("Debug: Roller[0] = %d, Roller[1] = %d IndiceRF: %d\n", roller_swr[0], roller_swr[1], IndiceRF);
					}
					else if(TelaProgRF == 2) {
						TelaProgRF = 3;
						lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_BG, &style_roller_save);
					    lv_roller_set_style(rollerswr[0], LV_ROLLER_STYLE_SEL, &style_roller_save);
					    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_BG, &style_roller_save);
					    lv_roller_set_style(rollerswr[1], LV_ROLLER_STYLE_SEL, &style_roller_save);
				        // Get Value Roller
					    lv_roller_get_selected_str(rollerswr[0], buf, sizeof(buf));
				        out[0] = buf[0];
				        lv_roller_get_selected_str(rollerswr[1], buf, sizeof(buf));
				        out[1] = buf[0];
				        out[3] = 0;

				        Max_Reflected = atoi(out);
				        max_rfl = (float)(Max_Reflected);
				        max_rfl /= 10;
				        //logI("Save MAX_RFL: Digit: %s  Max_reflected: %ld   max_rfl: %0.1fW  B1: %s\n", buf, Max_Reflected, max_rfl, out);
					}
					break;
				case KEY_ESC:
					TelaProgRF = 0;
					IndiceRF = 0;
					update_style_roller_rf(0);
					break;
			}
		}
	}
}

void ButtonEventTelaRF_1(uint8_t event, uint8_t tipo, uint8_t id)
{
    char buf[32];
    char out[6];

	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			switch(id) {
				case KEY_DN:
					if(TelaProgRF_1 == 0) {
						//lv_event_send(img_fundo, LV_EVENT_APPLY, NULL);
						lv_anim_del (sa.var, NULL);
						lv_task_del(Task_RF_1);
						lv_obj_del(Tela_RF_1);
						screen_rf();
					}
					else if(TelaProgRF_1 == 1) {
						if(IndiceRF_1 >= 1) IndiceRF_1--;
						//logI("TelaRF_1 - KEY_DN - IndiceRF_1: %ld\n", IndiceRF_1);
						update_style_roller_rf_1(IndiceRF_1 + 1);
					}
					else if(TelaProgRF_1 == 2) {
						if(IndiceRF_1 == 0) {
							if(roller_target[0] > 0) roller_target[0]--;
							lv_roller_set_selected(rollertarget[IndiceRF_1], roller_target[0], false);
						}
						else {
							if(roller_target[1] > 0) roller_target[1]--;
							lv_roller_set_selected(rollertarget[IndiceRF_1], roller_target[1], false);
						}
						//logI("Debug: KEY_DN Roller_Target: %ld, Dado: %ld %d\n", IndiceRF_1, roller_target[0], roller_target[1]);
					}
					break;
				case KEY_UP:
					if(TelaProgRF_1 == 0) {
						lv_event_send(imgbtn_next[1], LV_EVENT_APPLY, NULL);
					}
					else if(TelaProgRF_1 == 1){
						IndiceRF_1++;
						if(IndiceRF_1 > 1) IndiceRF_1 = 1;
						//logI("TelaRF - KEY_UP - IndiceRF: %ld\n", IndiceRF_1);
						update_style_roller_rf_1(IndiceRF_1 + 1);
					}
					else if(TelaProgRF_1 == 2) {
						if(IndiceRF_1 == 0) {
							roller_target[0]++;
							if(roller_target[0] > 2) roller_target[0] = 0;
							lv_roller_set_selected(rollertarget[IndiceRF_1], roller_target[0], false);
						}
						else {
							roller_target[1]++;
							if(roller_target[1] > 9) roller_target[1] = 0;
							lv_roller_set_selected(rollertarget[IndiceRF_1], roller_target[1], false);
						}
						//logI("Debug: KEY_UP Roller_SWR: %ld, Dado: %d %d\n", IndiceRF, roller_target[0], roller_target[1]);
					}
					break;
				case KEY_ENTER:
					if(TelaProgRF_1 == 0) {
						TelaProgRF_1 = 1;
						IndiceRF_1 = 0;
						update_style_roller_rf_1(IndiceRF_1 + 1);
					}
					else if(TelaProgRF_1 == 1) {
						TelaProgRF_1 = 2;
						roller_target[0] = Target_Int / 10;
						roller_target[1] = Target_Int % 10;
					    lv_roller_set_style(rollertarget[IndiceRF_1], LV_ROLLER_STYLE_BG, &style_roller_anim);
					    lv_roller_set_style(rollertarget[IndiceRF_1], LV_ROLLER_STYLE_SEL, &style_roller_anim);
						//logI("Debug Target: Roller[0] = %d, Roller[1] = %d IndiceRF_1: %d\n", roller_target[0], roller_target[1], IndiceRF_1);
					}
					else if(TelaProgRF_1 == 2) {
						TelaProgRF_1 = 3;
						lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_BG, &style_roller_save);
					    lv_roller_set_style(rollertarget[0], LV_ROLLER_STYLE_SEL, &style_roller_save);
					    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_BG, &style_roller_save);
					    lv_roller_set_style(rollertarget[1], LV_ROLLER_STYLE_SEL, &style_roller_save);
				        // Get Value Roller
					    lv_roller_get_selected_str(rollertarget[0], buf, sizeof(buf));
				        out[0] = buf[0];
				        lv_roller_get_selected_str(rollertarget[1], buf, sizeof(buf));
				        out[1] = buf[0];
				        out[3] = 0;

				        Target_Int = atoi(out);
				        target = (float)(Target_Int);
				        //logI("Save TARGET: Digit: %s  Target_Int: %ld target: %0.1fW  B1: %s\n", buf, Target_Int, target, out);
					}
					break;
				case KEY_ESC:
					TelaProgRF_1 = 0;
					IndiceRF_1 = 0;
					update_style_roller_rf_1(0);
					break;
			}
		}
	}
}

void ButtonEventTelaRF_2(uint8_t event, uint8_t tipo, uint8_t id)
{
	static uint8_t var = 0;

	if(event == EVT_PBTN_INPUT) {
		if(tipo == PBTN_SCLK) {	// Single Click
			switch(id) {
				case KEY_DN:
					if(TelaProgRF_2 == 0) {
						//lv_event_send(img_fundo_2, LV_EVENT_APPLY, NULL);
						lv_obj_del(Tela_RF_2);
						screen_RF_1();
					}
					else if(TelaProgRF_2 == 1) {
						var = !var;
						if(var) {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_INA);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
						}
						else {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_INA);
						}
					}
					break;
				case KEY_UP:
					if(TelaProgRF_2 == 1) {
						var = !var;
						if(var) {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_INA);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
						}
						else {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_INA);
						}
					}
					break;
				case KEY_ENTER:
					if(TelaProgRF_2 == 0) {
						TelaProgRF_2 = 1;
						var = RFEnable;
						if(var) {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_INA);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
						}
						else {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_INA);
						}
					}
					else if(TelaProgRF_2 == 1) {
						TelaProgRF_2 = 2;
						//logI("Salva RFENABLE %d\n", var);
						RFEnable = var;
						if(RFEnable) {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
						}
						else {
							lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
							lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
						}
					}
					break;
				case KEY_ESC:
					TelaProgRF_2 = 0;
					IndiceRF_2 = 0;
					if(RFEnable) {
						lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_REL);
						lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_PR);
					}
					else {
						lv_btn_set_state(imgbtn1[0], LV_BTN_STATE_TGL_PR);
						lv_btn_set_state(imgbtn1[1], LV_BTN_STATE_TGL_REL);
					}
					break;
			}
		}
	}
}
