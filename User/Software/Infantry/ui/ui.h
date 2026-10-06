// //
// // Created by RM UI Designer
// // Static Edition
// //

// #ifndef UI_H
// #define UI_H
// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "ui_interface.h"

// #include "ui_g.h"

// void ui_init_g_auto();
// void ui_update_g_auto();
// void ui_remove_g_auto();
// void ui_init_g_cap();
// void ui_update_g_cap();
// void ui_remove_g_cap();
// void ui_init_g_shoot();
// void ui_update_g_shoot();
// void ui_remove_g_shoot();
// void ui_init_g_statu();
// void ui_update_g_statu();
// void ui_remove_g_statu();

// void ui_init();
// void ui_updata();
// void Chassis_ui_change(float angle);
// void chassis_mode_change();
// void Auto_ui_change(void);
// void shoot_ui_updata();
// void Supercap_ui_change();

// #ifdef __cplusplus
// }
// #endif

// #endif // UI_H
#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_helm.h"
#include "ui_default.h"

void ui_init();
void ui_updata();
void Supercapui_change(float votage);
void Shootui_change();
void Chassisui_change(float angle);
void Autoui_change(void);

#ifdef __cplusplus
}
#endif

#endif // UI_H
