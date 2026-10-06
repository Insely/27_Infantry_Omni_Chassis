#include "string.h"
#include "ui_interface.h"
#include "ui_helm.h"
#include "Global_status.h"

ui_interface_figure_t ui_helm_now_figure[9];
uint8_t ui_helm_dirty_figure[9];

ui_interface_string_t ui_helm_now_string[2];
uint8_t ui_helm_dirty_string[2];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_helm_last_figures[9];
ui_interface_string_t ui_helm_last_strings[2];
#endif



void ui_init_helm()
{
    ui_helm_shoot_auto_rect->figure_type = 1;
    ui_helm_shoot_auto_rect->operate_type = 1;
    ui_helm_shoot_auto_rect->layer = 0;
    ui_helm_shoot_auto_rect->color = 8;
    ui_helm_shoot_auto_rect->start_x = 685;
    ui_helm_shoot_auto_rect->start_y = 246;
    ui_helm_shoot_auto_rect->width = 5;
    ui_helm_shoot_auto_rect->end_x = 1247;
    ui_helm_shoot_auto_rect->end_y = 808;

    ui_helm_shoot_round->figure_type = 2;
    ui_helm_shoot_round->operate_type = 1;
    ui_helm_shoot_round->layer = 0;
    ui_helm_shoot_round->color = 4;
    ui_helm_shoot_round->start_x = 960;
    ui_helm_shoot_round->start_y = 540;
    ui_helm_shoot_round->width = 2;
    ui_helm_shoot_round->r = 13;

    ui_helm_chassis_arc->figure_type = 4;
    ui_helm_chassis_arc->operate_type = 1;
    ui_helm_chassis_arc->layer = 0;
    ui_helm_chassis_arc->color = 1;
    ui_helm_chassis_arc->start_x = 960;
    ui_helm_chassis_arc->start_y = 540;
    ui_helm_chassis_arc->width = 5;
    ui_helm_chassis_arc->start_angle = 150;
    ui_helm_chassis_arc->end_angle = 210;
    ui_helm_chassis_arc->rx = 330;
    ui_helm_chassis_arc->ry = 330;

    ui_helm_supercap_rect->figure_type = 1;
    ui_helm_supercap_rect->operate_type = 1;
    ui_helm_supercap_rect->layer = 0;
    ui_helm_supercap_rect->color = 6;
    ui_helm_supercap_rect->start_x = 445;
    ui_helm_supercap_rect->start_y = 159;
    ui_helm_supercap_rect->width = 3;
    ui_helm_supercap_rect->end_x = 1436;
    ui_helm_supercap_rect->end_y = 201;

    ui_helm_supercap_line->figure_type = 0;
    ui_helm_supercap_line->operate_type = 1;
    ui_helm_supercap_line->layer = 0;
    ui_helm_supercap_line->color = 2;
    ui_helm_supercap_line->start_x = 450;
    ui_helm_supercap_line->start_y = 166;
    ui_helm_supercap_line->width = 30;
    ui_helm_supercap_line->end_x = 1429;
    ui_helm_supercap_line->end_y = 166;

    ui_helm_shoot_line->figure_type = 0;
    ui_helm_shoot_line->operate_type = 1;
    ui_helm_shoot_line->layer = 0;
    ui_helm_shoot_line->color = 0;
    ui_helm_shoot_line->start_x = 960;
    ui_helm_shoot_line->start_y = 540;
    ui_helm_shoot_line->width = 2;
    ui_helm_shoot_line->end_x = 960;
    ui_helm_shoot_line->end_y = 400;

    ui_helm_shootend_round->figure_type = 2;
    ui_helm_shootend_round->operate_type = 1;
    ui_helm_shootend_round->layer = 0;
    ui_helm_shootend_round->color = 0;
    ui_helm_shootend_round->start_x = 956;
    ui_helm_shootend_round->start_y = 494;
    ui_helm_shootend_round->width = 3;
    ui_helm_shootend_round->r = 12;

    // 底盘模式文字
    ui_helm_mode_text->figure_type = 7;
    ui_helm_mode_text->operate_type = 1;
    ui_helm_mode_text->layer = 0;
    ui_helm_mode_text->color = 8;
    ui_helm_mode_text->start_x = 950 ;
    ui_helm_mode_text->start_y = 400;
    ui_helm_mode_text->width = 3;
    ui_helm_mode_text->font_size = 30;
    if (Global.Chassis.mode == FLOW) {
        ui_helm_mode_text->str_length = 3;
        ui_helm_mode_text->color = 2;
        strcpy(ui_helm_mode_text->string, "YES");
    } else {
        ui_helm_mode_text->str_length = 2;
        ui_helm_mode_text->color = 4;
        strcpy(ui_helm_mode_text->string, "NO");
    }

    // pitch角度数字
    ui_helm_pitch_number->figure_type = 6;
    ui_helm_pitch_number->operate_type = 1;
    ui_helm_pitch_number->layer = 0;
    ui_helm_pitch_number->color = 2;
    ui_helm_pitch_number->start_x = 1500;
    ui_helm_pitch_number->start_y = 540;
    ui_helm_pitch_number->width = 3;
    ui_helm_pitch_number->font_size = 30;
    ui_helm_pitch_number->_b = 1;
    ui_helm_pitch_number->number = 0;

    uint32_t idx = 0;
    for(int i=0;i<9;i++)
    {
        ui_helm_now_figure[i].figure_name[2] = idx & 0xFF;
        ui_helm_now_figure[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_helm_now_figure[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_helm_now_figure[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_helm_last_figures[i] = ui_helm_now_figure[i];
#endif
        ui_helm_dirty_figure[i] = 1;
        idx++;
    }

    // string name
    for(int i=0;i<2;i++)
    {
        ui_helm_now_string[i].figure_name[2] = idx & 0xFF;
        ui_helm_now_string[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_helm_now_string[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_helm_now_string[i].operate_type = 1;
        ui_helm_dirty_string[i] = 1;
#ifndef MANUAL_DIRTY
        ui_helm_last_strings[i] = ui_helm_now_string[i];
#endif
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < 9; i++) {
        ui_helm_now_figure[i].operate_type = 2;
    }
    for (int i = 0; i < 2; i++) {
        ui_helm_now_string[i].operate_type = 2;
    }
}

void ui_update_helm() 
{
    #ifndef MANUAL_DIRTY
    for (int i = 0; i < 9; i++) {
        if (memcmp(&ui_helm_now_figure[i], &ui_helm_last_figures[i], sizeof(ui_helm_now_figure[i])) != 0) {
            ui_helm_dirty_figure[i] = 1;
            ui_helm_last_figures[i] = ui_helm_now_figure[i];
        }
    }
    for (int i = 0; i < 2; i++) {
        if (memcmp(&ui_helm_now_string[i], &ui_helm_last_strings[i], sizeof(ui_helm_now_string[i])) != 0) {
            ui_helm_dirty_string[i] = 1;
            ui_helm_last_strings[i] = ui_helm_now_string[i];
        }
    }
#endif
    SCAN_AND_SEND();
}