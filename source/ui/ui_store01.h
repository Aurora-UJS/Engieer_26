//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_store01_H
#define UI_store01_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_store01_now_figures[4];
extern uint8_t ui_store01_dirty_figure[4];
extern ui_interface_string_t ui_store01_now_strings[8];
extern uint8_t ui_store01_dirty_string[8];

extern uint8_t ui_store01_max_send_count[12];

#define ui_store01_Ungroup_store00 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[0]))
#define ui_store01_Ungroup_store03 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[1]))
#define ui_store01_Ungroup_store01 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[2]))
#define ui_store01_Ungroup_store02 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[3]))

#define ui_store01_Ungroup_Chas_mode (&(ui_store01_now_strings[0]))
#define ui_store01_Ungroup_Arm_mode (&(ui_store01_now_strings[1]))
#define ui_store01_Ungroup_Chas_disp (&(ui_store01_now_strings[2]))
#define ui_store01_Ungroup_arm_disp (&(ui_store01_now_strings[3]))
#define ui_store01_Ungroup_risg_disp (&(ui_store01_now_strings[4]))
#define ui_store01_Ungroup_ctrl_disp (&(ui_store01_now_strings[5]))
#define ui_store01_Ungroup_Rising_Mode (&(ui_store01_now_strings[6]))
#define ui_store01_Ungroup_Ctrl_orig (&(ui_store01_now_strings[7]))

#define ui_store01_Ungroup_store00_max_send_count (ui_store01_max_send_count[0])
#define ui_store01_Ungroup_store03_max_send_count (ui_store01_max_send_count[1])
#define ui_store01_Ungroup_store01_max_send_count (ui_store01_max_send_count[2])
#define ui_store01_Ungroup_store02_max_send_count (ui_store01_max_send_count[3])

#define ui_store01_Ungroup_Chas_mode_max_send_count (ui_store01_max_send_count[4])
#define ui_store01_Ungroup_Arm_mode_max_send_count (ui_store01_max_send_count[5])
#define ui_store01_Ungroup_Chas_disp_max_send_count (ui_store01_max_send_count[6])
#define ui_store01_Ungroup_arm_disp_max_send_count (ui_store01_max_send_count[7])
#define ui_store01_Ungroup_risg_disp_max_send_count (ui_store01_max_send_count[8])
#define ui_store01_Ungroup_ctrl_disp_max_send_count (ui_store01_max_send_count[9])
#define ui_store01_Ungroup_Rising_Mode_max_send_count (ui_store01_max_send_count[10])
#define ui_store01_Ungroup_Ctrl_orig_max_send_count (ui_store01_max_send_count[11])

#ifdef MANUAL_DIRTY
#define ui_store01_Ungroup_store00_dirty (ui_store01_dirty_figure[0])
#define ui_store01_Ungroup_store03_dirty (ui_store01_dirty_figure[1])
#define ui_store01_Ungroup_store01_dirty (ui_store01_dirty_figure[2])
#define ui_store01_Ungroup_store02_dirty (ui_store01_dirty_figure[3])

#define ui_store01_Ungroup_Chas_mode_dirty (ui_store01_dirty_string[0])
#define ui_store01_Ungroup_Arm_mode_dirty (ui_store01_dirty_string[1])
#define ui_store01_Ungroup_Chas_disp_dirty (ui_store01_dirty_string[2])
#define ui_store01_Ungroup_arm_disp_dirty (ui_store01_dirty_string[3])
#define ui_store01_Ungroup_risg_disp_dirty (ui_store01_dirty_string[4])
#define ui_store01_Ungroup_ctrl_disp_dirty (ui_store01_dirty_string[5])
#define ui_store01_Ungroup_Rising_Mode_dirty (ui_store01_dirty_string[6])
#define ui_store01_Ungroup_Ctrl_orig_dirty (ui_store01_dirty_string[7])
#endif

void ui_init_store01();
void ui_update_store01();

#endif // UI_store01_H
