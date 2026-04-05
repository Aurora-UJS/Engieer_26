//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_default_H
#define UI_default_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_default_now_figures[1];
extern uint8_t ui_default_dirty_figure[1];
extern ui_interface_string_t ui_default_now_strings[4];
extern uint8_t ui_default_dirty_string[4];

extern uint8_t ui_default_max_send_count[5];

#define ui_default_Ungroup_store00 ((ui_interface_ellipse_t*)&(ui_default_now_figures[0]))

#define ui_default_Ungroup_Chas_mode (&(ui_default_now_strings[0]))
#define ui_default_Ungroup_Arm_mode (&(ui_default_now_strings[1]))
#define ui_default_Ungroup_Rising_Mode (&(ui_default_now_strings[2]))
#define ui_default_Ungroup_Ctrl_orig (&(ui_default_now_strings[3]))

#define ui_default_Ungroup_store00_max_send_count (ui_default_max_send_count[0])

#define ui_default_Ungroup_Chas_mode_max_send_count (ui_default_max_send_count[1])
#define ui_default_Ungroup_Arm_mode_max_send_count (ui_default_max_send_count[2])
#define ui_default_Ungroup_Rising_Mode_max_send_count (ui_default_max_send_count[3])
#define ui_default_Ungroup_Ctrl_orig_max_send_count (ui_default_max_send_count[4])

#ifdef MANUAL_DIRTY
#define ui_default_Ungroup_store00_dirty (ui_default_dirty_figure[0])

#define ui_default_Ungroup_Chas_mode_dirty (ui_default_dirty_string[0])
#define ui_default_Ungroup_Arm_mode_dirty (ui_default_dirty_string[1])
#define ui_default_Ungroup_Rising_Mode_dirty (ui_default_dirty_string[2])
#define ui_default_Ungroup_Ctrl_orig_dirty (ui_default_dirty_string[3])
#endif

void ui_init_default();
void ui_update_default();

#endif // UI_default_H
