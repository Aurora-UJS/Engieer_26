//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_default.h"

#define TOTAL_FIGURE 1
#define TOTAL_STRING 4

ui_interface_figure_t ui_default_now_figures[TOTAL_FIGURE];
uint8_t ui_default_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_default_now_strings[TOTAL_STRING];
uint8_t ui_default_dirty_string[TOTAL_STRING];

uint8_t ui_default_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    3,
    3,
    3,
    3,
    3,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_default_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_default_last_strings[TOTAL_STRING];
#endif

static uint8_t ui_default_init_pending = 0U;

static void ui_default_set_name(uint8_t name[3], char a, char b, char c)
{
    name[0] = (uint8_t)a;
    name[1] = (uint8_t)b;
    name[2] = (uint8_t)c;
}

static uint8_t ui_default_has_pending_init_frames(void)
{
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (ui_default_dirty_figure[i] > 0U) {
            return 1U;
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (ui_default_dirty_string[i] > 0U) {
            return 1U;
        }
    }
    return 0U;
}

static void ui_default_finish_init(void)
{
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_default_now_figures[i].operate_type = 2U;
        ui_default_dirty_figure[i] = ui_default_max_send_count[i];
#ifndef MANUAL_DIRTY
        ui_default_last_figures[i] = ui_default_now_figures[i];
#endif
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_default_now_strings[i].operate_type = 2U;
        ui_default_dirty_string[i] = ui_default_max_send_count[TOTAL_FIGURE + i];
#ifndef MANUAL_DIRTY
        ui_default_last_strings[i] = ui_default_now_strings[i];
#endif
    }
    ui_default_init_pending = 0U;
}

#define SCAN_AND_SEND() ui_scan_and_send(ui_default_now_figures, ui_default_dirty_figure, ui_default_now_strings, ui_default_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_default() {
    ui_default_Ungroup_store00->figure_type = 3;
    ui_default_Ungroup_store00->operate_type = 1;
    ui_default_Ungroup_store00->layer = 0;
    ui_default_Ungroup_store00->color = 0;
    ui_default_Ungroup_store00->start_x = 1600;
    ui_default_Ungroup_store00->start_y = 700;
    ui_default_Ungroup_store00->width = 20;
    ui_default_Ungroup_store00->rx = 40;
    ui_default_Ungroup_store00->ry = 40;

    ui_default_Ungroup_Chas_mode->figure_type = 7;
    ui_default_Ungroup_Chas_mode->operate_type = 1;
    ui_default_Ungroup_Chas_mode->layer = 0;
    ui_default_Ungroup_Chas_mode->color = 3;
    ui_default_Ungroup_Chas_mode->start_x = 80;
    ui_default_Ungroup_Chas_mode->start_y = 800;
    ui_default_Ungroup_Chas_mode->width = 3;
    ui_default_Ungroup_Chas_mode->font_size = 25;
    ui_default_Ungroup_Chas_mode->str_length = 9;
    strcpy(ui_default_Ungroup_Chas_mode->string, "CHas_mode");

    ui_default_Ungroup_Arm_mode->figure_type = 7;
    ui_default_Ungroup_Arm_mode->operate_type = 1;
    ui_default_Ungroup_Arm_mode->layer = 0;
    ui_default_Ungroup_Arm_mode->color = 3;
    ui_default_Ungroup_Arm_mode->start_x = 80;
    ui_default_Ungroup_Arm_mode->start_y = 750;
    ui_default_Ungroup_Arm_mode->width = 3;
    ui_default_Ungroup_Arm_mode->font_size = 25;
    ui_default_Ungroup_Arm_mode->str_length = 8;
    strcpy(ui_default_Ungroup_Arm_mode->string, "Arm_Mode");

    ui_default_Ungroup_Rising_Mode->figure_type = 7;
    ui_default_Ungroup_Rising_Mode->operate_type = 1;
    ui_default_Ungroup_Rising_Mode->layer = 0;
    ui_default_Ungroup_Rising_Mode->color = 3;
    ui_default_Ungroup_Rising_Mode->start_x = 80;
    ui_default_Ungroup_Rising_Mode->start_y = 700;
    ui_default_Ungroup_Rising_Mode->width = 3;
    ui_default_Ungroup_Rising_Mode->font_size = 25;
    ui_default_Ungroup_Rising_Mode->str_length = 9;
    strcpy(ui_default_Ungroup_Rising_Mode->string, "Risg_mode");

    ui_default_Ungroup_Ctrl_orig->figure_type = 7;
    ui_default_Ungroup_Ctrl_orig->operate_type = 1;
    ui_default_Ungroup_Ctrl_orig->layer = 0;
    ui_default_Ungroup_Ctrl_orig->color = 3;
    ui_default_Ungroup_Ctrl_orig->start_x = 80;
    ui_default_Ungroup_Ctrl_orig->start_y = 650;
    ui_default_Ungroup_Ctrl_orig->width = 3;
    ui_default_Ungroup_Ctrl_orig->font_size = 25;
    ui_default_Ungroup_Ctrl_orig->str_length = 9;
    strcpy(ui_default_Ungroup_Ctrl_orig->string, "Ctrl_orig");

    ui_default_set_name(ui_default_now_figures[0].figure_name, 'S', '0', '0');
    ui_default_set_name(ui_default_now_strings[0].figure_name, 'C', 'M', 'D');
    ui_default_set_name(ui_default_now_strings[1].figure_name, 'A', 'M', 'D');
    ui_default_set_name(ui_default_now_strings[2].figure_name, 'R', 'M', 'D');
    ui_default_set_name(ui_default_now_strings[3].figure_name, 'C', 'O', 'R');

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_default_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_default_last_figures[i] = ui_default_now_figures[i];
#endif
        ui_default_dirty_figure[i] = ui_default_max_send_count[i];
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_default_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_default_last_strings[i] = ui_default_now_strings[i];
#endif
        ui_default_dirty_string[i] = ui_default_max_send_count[TOTAL_FIGURE + i];
    }

    ui_default_init_pending = 1U;
}

void ui_update_default() {
    if (ui_default_init_pending != 0U) {
        SCAN_AND_SEND();
        if (ui_default_has_pending_init_frames() == 0U) {
            ui_default_finish_init();
        }
        return;
    }
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_default_now_figures[i], &ui_default_last_figures[i], sizeof(ui_default_now_figures[i])) != 0) {
            ui_default_dirty_figure[i] = ui_default_max_send_count[i];
            ui_default_last_figures[i] = ui_default_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_default_now_strings[i], &ui_default_last_strings[i], sizeof(ui_default_now_strings[i])) != 0) {
            ui_default_dirty_string[i] = ui_default_max_send_count[TOTAL_FIGURE + i];
            ui_default_last_strings[i] = ui_default_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
