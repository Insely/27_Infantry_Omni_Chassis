//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_rect_t *ui_g_auto_autoRect;

void ui_init_g_auto();
void ui_update_g_auto();
void ui_remove_g_auto();

extern ui_interface_arc_t *ui_g_cap_capArc;
extern ui_interface_arc_t *ui_g_cap_engArc;

void ui_init_g_cap();
void ui_update_g_cap();
void ui_remove_g_cap();

extern ui_interface_line_t *ui_g_shoot_shootLine;
extern ui_interface_line_t *ui_g_shoot_1mLine;
extern ui_interface_line_t *ui_g_shoot_3mLine;
extern ui_interface_line_t *ui_g_shoot_5mLine;

void ui_init_g_shoot();
void ui_update_g_shoot();
void ui_remove_g_shoot();

extern ui_interface_number_t *ui_g_statu_shootFloat;
extern ui_interface_arc_t *ui_g_statu_chassisArc;
extern ui_interface_ellipse_t *ui_g_statu_statuEllipse;
extern ui_interface_string_t *ui_g_statu_modeText;

void ui_init_g_statu();
void ui_update_g_statu();
void ui_remove_g_statu();


#endif // UI_g_H
