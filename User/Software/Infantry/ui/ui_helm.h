#ifndef UI_HELM_H
#define UI_HELM_H

#include "ui_interface.h"
extern ui_interface_figure_t ui_helm_now_figure[9];
extern uint8_t ui_helm_dirty_figure[9];
extern ui_interface_string_t ui_helm_now_string[2];
extern uint8_t ui_helm_dirty_string[2];
#define SCAN_AND_SEND() ui_scan_and_send(ui_helm_now_figure, ui_helm_dirty_figure, ui_helm_now_string, ui_helm_dirty_string, 9, 2)

#define ui_helm_shoot_auto_rect ((ui_interface_rect_t*)&(ui_helm_now_figure[0]))
#define ui_helm_shoot_round ((ui_interface_round_t*)&(ui_helm_now_figure[1]))
#define ui_helm_chassis_arc ((ui_interface_arc_t*)&(ui_helm_now_figure[2]))
#define ui_helm_supercap_rect ((ui_interface_rect_t*)&(ui_helm_now_figure[3]))
#define ui_helm_supercap_line ((ui_interface_line_t*)&(ui_helm_now_figure[4]))
#define ui_helm_shoot_line ((ui_interface_line_t*)&(ui_helm_now_figure[5]))
#define ui_helm_shootend_round ((ui_interface_round_t*)&(ui_helm_now_figure[6]))
#define ui_helm_shootend_round_long ((ui_interface_round_t*)&(ui_helm_now_figure[7]))

#define ui_helm_mode_text (&ui_helm_now_string[0])
#define ui_helm_pitch_number ((ui_interface_number_t*)&(ui_helm_now_figure[8]))

#ifndef MANUAL_DIRTY
#define ui_helm_shoot_auto_rect_dirty (ui_helm_dirty_figure[0])
#define ui_helm_shoot_round_dirty (ui_helm_dirty_figure[1])
#define ui_helm_chassis_arc_dirty (ui_helm_dirty_figure[2])
#define ui_helm_supercap_rect_dirty (ui_helm_dirty_figure[3])
#define ui_helm_supercap_line_dirty (ui_helm_dirty_figure[4])
#define ui_helm_shoot_line_dirty (ui_helm_dirty_figure[5])
#define ui_helm_shootend_round_dirty (ui_helm_dirty_figure[6])
#define ui_helm_shootend_round_long_dirty (ui_helm_dirty_figure[7])
#endif


void ui_init_helm();
void ui_update_helm();

#endif