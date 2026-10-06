#include <string.h>

#include "ui_interface.h"

ui_1_frame_t ui_g_auto_0;

ui_interface_rect_t *ui_g_auto_autoRect = (ui_interface_rect_t*)&(ui_g_auto_0.data[0]);

void _ui_init_g_auto_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_auto_0.data[i].figure_name[0] = 0;
        ui_g_auto_0.data[i].figure_name[1] = 0;
        ui_g_auto_0.data[i].figure_name[2] = i + 0;
        ui_g_auto_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_auto_0.data[i].operate_type = 0;
    }

    ui_g_auto_autoRect->figure_type = 1;
    ui_g_auto_autoRect->operate_type = 1;
    ui_g_auto_autoRect->layer = 4;
    ui_g_auto_autoRect->color = 5;
    ui_g_auto_autoRect->start_x = 737;
    ui_g_auto_autoRect->start_y = 426;
    ui_g_auto_autoRect->width = 3;
    ui_g_auto_autoRect->end_x = 1187;
    ui_g_auto_autoRect->end_y = 726;


    ui_proc_1_frame(&ui_g_auto_0);
    SEND_MESSAGE((uint8_t *) &ui_g_auto_0, sizeof(ui_g_auto_0));
}

void _ui_update_g_auto_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_auto_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_auto_0);
    SEND_MESSAGE((uint8_t *) &ui_g_auto_0, sizeof(ui_g_auto_0));
}

void _ui_remove_g_auto_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_auto_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_auto_0);
    SEND_MESSAGE((uint8_t *) &ui_g_auto_0, sizeof(ui_g_auto_0));
}


void ui_init_g_auto() {
    _ui_init_g_auto_0();
}

void ui_update_g_auto() {
    _ui_update_g_auto_0();
}

void ui_remove_g_auto() {
    _ui_remove_g_auto_0();
}

ui_2_frame_t ui_g_cap_0;

ui_interface_arc_t *ui_g_cap_capArc = (ui_interface_arc_t*)&(ui_g_cap_0.data[0]);
ui_interface_arc_t *ui_g_cap_engArc = (ui_interface_arc_t*)&(ui_g_cap_0.data[1]);

void _ui_init_g_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_cap_0.data[i].figure_name[0] = 0;
        ui_g_cap_0.data[i].figure_name[1] = 1;
        ui_g_cap_0.data[i].figure_name[2] = i + 0;
        ui_g_cap_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_cap_0.data[i].operate_type = 0;
    }

    ui_g_cap_capArc->figure_type = 4;
    ui_g_cap_capArc->operate_type = 1;
    ui_g_cap_capArc->layer = 0;
    ui_g_cap_capArc->color = 2;
    ui_g_cap_capArc->start_x = 1260;
    ui_g_cap_capArc->start_y = 545;
    ui_g_cap_capArc->width = 15;
    ui_g_cap_capArc->start_angle = 45;
    ui_g_cap_capArc->end_angle = 135;
    ui_g_cap_capArc->rx = 350;
    ui_g_cap_capArc->ry = 400;

    ui_g_cap_engArc->figure_type = 4;
    ui_g_cap_engArc->operate_type = 1;
    ui_g_cap_engArc->layer = 0;
    ui_g_cap_engArc->color = 3;
    ui_g_cap_engArc->start_x = 660;
    ui_g_cap_engArc->start_y = 545;
    ui_g_cap_engArc->width = 15;
    ui_g_cap_engArc->start_angle = 225;
    ui_g_cap_engArc->end_angle = 315;
    ui_g_cap_engArc->rx = 350;
    ui_g_cap_engArc->ry = 400;


    ui_proc_2_frame(&ui_g_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_g_cap_0, sizeof(ui_g_cap_0));
}

void _ui_update_g_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_cap_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_g_cap_0, sizeof(ui_g_cap_0));
}

void _ui_remove_g_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_cap_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_g_cap_0, sizeof(ui_g_cap_0));
}


void ui_init_g_cap() {
    _ui_init_g_cap_0();
}

void ui_update_g_cap() {
    _ui_update_g_cap_0();
}

void ui_remove_g_cap() {
    _ui_remove_g_cap_0();
}

ui_5_frame_t ui_g_shoot_0;

ui_interface_line_t *ui_g_shoot_shootLine = (ui_interface_line_t*)&(ui_g_shoot_0.data[0]);
ui_interface_line_t *ui_g_shoot_1mLine = (ui_interface_line_t*)&(ui_g_shoot_0.data[1]);
ui_interface_line_t *ui_g_shoot_3mLine = (ui_interface_line_t*)&(ui_g_shoot_0.data[2]);
ui_interface_line_t *ui_g_shoot_5mLine = (ui_interface_line_t*)&(ui_g_shoot_0.data[3]);

void _ui_init_g_shoot_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_shoot_0.data[i].figure_name[0] = 0;
        ui_g_shoot_0.data[i].figure_name[1] = 2;
        ui_g_shoot_0.data[i].figure_name[2] = i + 0;
        ui_g_shoot_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_shoot_0.data[i].operate_type = 0;
    }

    ui_g_shoot_shootLine->figure_type = 0;
    ui_g_shoot_shootLine->operate_type = 1;
    ui_g_shoot_shootLine->layer = 1;
    ui_g_shoot_shootLine->color = 8;
    ui_g_shoot_shootLine->start_x = 960;
    ui_g_shoot_shootLine->start_y = 87;
    ui_g_shoot_shootLine->width = 2;
    ui_g_shoot_shootLine->end_x = 960;
    ui_g_shoot_shootLine->end_y = 837;

    ui_g_shoot_1mLine->figure_type = 0;
    ui_g_shoot_1mLine->operate_type = 1;
    ui_g_shoot_1mLine->layer = 1;
    ui_g_shoot_1mLine->color = 8;
    ui_g_shoot_1mLine->start_x = 890;
    ui_g_shoot_1mLine->start_y = 550;
    ui_g_shoot_1mLine->width = 2;
    ui_g_shoot_1mLine->end_x = 1030;
    ui_g_shoot_1mLine->end_y = 550;

    ui_g_shoot_3mLine->figure_type = 0;
    ui_g_shoot_3mLine->operate_type = 1;
    ui_g_shoot_3mLine->layer = 1;
    ui_g_shoot_3mLine->color = 8;
    ui_g_shoot_3mLine->start_x = 930;
    ui_g_shoot_3mLine->start_y = 525;
    ui_g_shoot_3mLine->width = 2;
    ui_g_shoot_3mLine->end_x = 989;
    ui_g_shoot_3mLine->end_y = 525;

    ui_g_shoot_5mLine->figure_type = 0;
    ui_g_shoot_5mLine->operate_type = 1;
    ui_g_shoot_5mLine->layer = 1;
    ui_g_shoot_5mLine->color = 8;
    ui_g_shoot_5mLine->start_x = 980;
    ui_g_shoot_5mLine->start_y = 485;
    ui_g_shoot_5mLine->width = 2;
    ui_g_shoot_5mLine->end_x = 940;
    ui_g_shoot_5mLine->end_y = 485;


    ui_proc_5_frame(&ui_g_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shoot_0, sizeof(ui_g_shoot_0));
}

void _ui_update_g_shoot_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_shoot_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shoot_0, sizeof(ui_g_shoot_0));
}

void _ui_remove_g_shoot_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_shoot_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_shoot_0, sizeof(ui_g_shoot_0));
}


void ui_init_g_shoot() {
    _ui_init_g_shoot_0();
}

void ui_update_g_shoot() {
    _ui_update_g_shoot_0();
}

void ui_remove_g_shoot() {
    _ui_remove_g_shoot_0();
}

ui_5_frame_t ui_g_statu_0;

ui_interface_number_t *ui_g_statu_shootFloat = (ui_interface_number_t*)&(ui_g_statu_0.data[0]);
ui_interface_arc_t *ui_g_statu_chassisArc = (ui_interface_arc_t*)&(ui_g_statu_0.data[1]);
ui_interface_ellipse_t *ui_g_statu_statuEllipse = (ui_interface_ellipse_t*)&(ui_g_statu_0.data[2]);

void _ui_init_g_statu_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_statu_0.data[i].figure_name[0] = 0;
        ui_g_statu_0.data[i].figure_name[1] = 3;
        ui_g_statu_0.data[i].figure_name[2] = i + 0;
        ui_g_statu_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_statu_0.data[i].operate_type = 0;
    }

    ui_g_statu_shootFloat->figure_type = 5;
    ui_g_statu_shootFloat->operate_type = 1;
    ui_g_statu_shootFloat->layer = 2;
    ui_g_statu_shootFloat->color = 7;
    ui_g_statu_shootFloat->start_x = 1032;
    ui_g_statu_shootFloat->start_y = 661;
    ui_g_statu_shootFloat->width = 2;
    ui_g_statu_shootFloat->font_size = 20;
    ui_g_statu_shootFloat->number = 12345;

    ui_g_statu_chassisArc->figure_type = 4;
    ui_g_statu_chassisArc->operate_type = 1;
    ui_g_statu_chassisArc->layer = 3;
    ui_g_statu_chassisArc->color = 0;
    ui_g_statu_chassisArc->start_x = 960;
    ui_g_statu_chassisArc->start_y = 160;
    ui_g_statu_chassisArc->width = 10;
    ui_g_statu_chassisArc->start_angle = 315;
    ui_g_statu_chassisArc->end_angle = 45;
    ui_g_statu_chassisArc->rx = 100;
    ui_g_statu_chassisArc->ry = 100;

    ui_g_statu_statuEllipse->figure_type = 3;
    ui_g_statu_statuEllipse->operate_type = 1;
    ui_g_statu_statuEllipse->layer = 4;
    ui_g_statu_statuEllipse->color = 6;
    ui_g_statu_statuEllipse->start_x = 157;
    ui_g_statu_statuEllipse->start_y = 771;
    ui_g_statu_statuEllipse->width = 10;
    ui_g_statu_statuEllipse->rx = 50;
    ui_g_statu_statuEllipse->ry = 50;


    ui_proc_5_frame(&ui_g_statu_0);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_0, sizeof(ui_g_statu_0));
}

void _ui_update_g_statu_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_statu_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_statu_0);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_0, sizeof(ui_g_statu_0));
}

void _ui_remove_g_statu_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_statu_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_statu_0);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_0, sizeof(ui_g_statu_0));
}

ui_string_frame_t ui_g_statu_1;
ui_interface_string_t* ui_g_statu_modeText = &(ui_g_statu_1.option);

void _ui_init_g_statu_1() {
    ui_g_statu_1.option.figure_name[0] = 0;
    ui_g_statu_1.option.figure_name[1] = 3;
    ui_g_statu_1.option.figure_name[2] = 3;
    ui_g_statu_1.option.operate_type = 1;

    ui_g_statu_modeText->figure_type = 7;
    ui_g_statu_modeText->operate_type = 1;
    ui_g_statu_modeText->layer = 3;
    ui_g_statu_modeText->color = 4;
    ui_g_statu_modeText->start_x = 860;
    ui_g_statu_modeText->start_y = 185;
    ui_g_statu_modeText->width = 5;
    ui_g_statu_modeText->font_size = 50;
    ui_g_statu_modeText->str_length = 4;
    strcpy(ui_g_statu_modeText->string, "FLOW");


    ui_proc_string_frame(&ui_g_statu_1);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_1, sizeof(ui_g_statu_1));
}

void _ui_update_g_statu_1() {
    ui_g_statu_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_statu_1);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_1, sizeof(ui_g_statu_1));
}

void _ui_remove_g_statu_1() {
    ui_g_statu_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_statu_1);
    SEND_MESSAGE((uint8_t *) &ui_g_statu_1, sizeof(ui_g_statu_1));
}

void ui_init_g_statu() {
    _ui_init_g_statu_0();
    _ui_init_g_statu_1();
}

void ui_update_g_statu() {
    _ui_update_g_statu_0();
    _ui_update_g_statu_1();
}

void ui_remove_g_statu() {
    _ui_remove_g_statu_0();
    _ui_remove_g_statu_1();
}

