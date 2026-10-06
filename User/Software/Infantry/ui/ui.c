#include "ui.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Shoot.h"
#include "supercup.h"
#include <math.h>

#include "referee_system.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ui_helm.h"
#include "Gimbal.h"

// void ui_init()
// {
//     ui_self_id = Referee_data.robot_id;

//     ui_init_g_auto();
//     osDelay(30);
//     ui_init_g_cap();
//     osDelay(30);
//     ui_init_g_shoot();
//     osDelay(30);
//     ui_init_g_statu();
//     osDelay(50);
// }
// int flag_ui = 0;
// void ui_updata()
// {
//     flag_ui++;
//     if (flag_ui >= 20)
//     {
//         flag_ui = 0;

//         if (ui_g_statu_statuEllipse->color < 7)
//             ui_g_statu_statuEllipse->color += 1;
//         else
//             ui_g_statu_statuEllipse->color = 0;
//     }

//     chassis_mode_change();
//     Chassis_ui_change(chassis.relative_angle);
//     Auto_ui_change();
//     shoot_ui_updata();
//     Supercap_ui_change();

//     ui_update_g_auto();
//     osDelay(1);
//     ui_update_g_cap();
//     osDelay(1);
//     ui_update_g_shoot();
//     osDelay(1);
//     ui_update_g_statu();
//     osDelay(1);
// }
// void Chassis_ui_change(float angle)
// {

//     float angle_start, angle_end;
//     if (angle > 360)
//         angle -= 360;
//     angle_start = angle - 20.0f;
//     angle_end = angle + 20.0f;
//     if (angle_start <= 0.0f)
//         angle_start += 360.0f;
//     if (angle_end <= 0.0f)
//         angle_end += 360.0f;
//     if (angle_end >= 360.0f)
//         angle_end -= 360.0f;
//     if (angle_start >= 360.0f)
//         angle_start -= 360.0f;
//     ui_g_statu_chassisArc->start_angle = angle_start;
//     ui_g_statu_chassisArc->end_angle = angle_end;
// }
// void chassis_mode_change()
// {

//     if (Global.Chassis.mode == FLOW)
//         strcpy(ui_g_statu_modeText->string, "FLOW"); //
//     else if (Global.Chassis.mode == SPIN_P)
//         strcpy(ui_g_statu_modeText->string, "SPIN"); //

//     if (Global.Control.mode == LOCK)
//         strcpy(ui_g_statu_modeText->string, "LOCK"); //
// }
// void Auto_ui_change(void)
// {
//     switch (Global.Auto.input.fire) 
//     {
//     case 0:
//         ui_g_auto_autoRect->color = 4; 
//         break;
//     case 1:
//         ui_g_auto_autoRect->color = 2; 
//         break;
//     default:
//         ui_g_auto_autoRect->color = 4; 
//         break;
//     }

//     if (Global.Auto.input.Auto_control_online <= 0)
//     {
//         ui_g_auto_autoRect->color = 7; 
//     }
// }
// void shoot_ui_updata()
// {
//     if (Global.Shoot.shoot_mode == READY)
//         ui_g_statu_shootFloat->color = 2;
//     else if (Global.Shoot.shoot_mode == CLOSE)
//         ui_g_statu_shootFloat->color = 4;
//     else
//         ui_g_statu_shootFloat->color = 7;

//     ui_g_statu_shootFloat->number = (int32_t)(Shoot.shoot_speed_set * 1000);
// }
// void Supercap_ui_change()
// {
//     float cnt = cap.remain_vol * cap.remain_vol;
//     float percent = (cnt - 0.0f) / 576.0f;
//     // float percent = cap.remain_vol / 24;
//     if (percent >= 1)
//         percent = 1;
//     if (percent <= 0)
//         percent = 0;
//     if (percent <= 0.1)
//         percent = 0.1;
//     if (percent >= 0.8)
//         ui_g_cap_capArc->color = 2; 
//     if (percent <= 0.8)
//         ui_g_cap_capArc->color = 3; 
//     if (percent <= 0.3)
//         ui_g_cap_capArc->color = 4; 

//     ui_g_cap_capArc->end_angle = ui_g_cap_capArc->start_angle + percent * 90;
//     if (ui_g_cap_capArc->end_angle > ui_g_cap_capArc->start_angle + 90)
//         ui_g_cap_capArc->end_angle = ui_g_cap_capArc->start_angle + 90;
// }
void ui_init()
{
  // ui_init_Omni_Shoot_Low();
  // osDelay(20);
  // ui_init_Omni_Chassis_high();
  // osDelay(20);
  // ui_remove_Omni_Shoot_static();
  // osDelay(20);
  ui_init_helm();
}

void chassis_mode_change()
{
    if (Global.Chassis.mode == FLOW)
    {
        strcpy(ui_helm_mode_text->string, "YES");
        ui_helm_mode_text->str_length = 3;
        ui_helm_mode_text->color = 2; // 绿
    }
    else
    {
        strcpy(ui_helm_mode_text->string, "NO");
        ui_helm_mode_text->str_length = 2;
        ui_helm_mode_text->color = 4; // 紫红
    }
}

void pitch_ui_change()
{
    ui_helm_pitch_number->number = -(int32_t)(Gimbal.pitch_location_now);
}



void ui_updata()
{
  chassis_mode_change();
  pitch_ui_change();
  ui_helm_dirty_string[0] = 1;
  ui_helm_dirty_string[1] = 1;
  ui_update_helm();
  ui_self_id = Referee_data.robot_id;
}

void Supercapui_change(float votage)
{
  float cnt = votage * votage;
  float percent = (cnt - 49.0f) / 480.0f;
  if (percent >= 1)
    percent = 1;
  if (percent <= 0)
    percent = 0;
  if (percent >= 0.6)
    ui_helm_supercap_line->color = 2; // 绿色
  if (percent <= 0.6)
    ui_helm_supercap_line->color = 3; // 橙色
  if (percent <= 0.3)
    ui_helm_supercap_line->color = 4; // 紫红色
  ui_helm_supercap_line->end_x = ui_helm_supercap_line->start_x + 980.0f * percent;

  if (Global.Cap.mode != FULL)
  {
    ui_helm_supercap_rect->color = 6;
    ui_helm_supercap_rect->width = 1;
  }
  else
  {
    ui_helm_supercap_rect->color = 1;
    ui_helm_supercap_rect->width = 8;
  }
}

void Shootui_change()
{
  // 摩擦轮
  if (Global.Shoot.shoot_mode != CLOSE)
  {
    if (Global.Shoot.shoot_status == OK)
      ui_helm_shoot_round->color = 2; // 达速绿色
    else
      ui_helm_shoot_round->color = 3; // 启动中橙色
    ui_helm_shootend_round->color = 0;
  }
  else
  {
    ui_helm_shoot_round->color = 4; // 关闭紫红色
    ui_helm_shootend_round->color = 0;
  }
}

void Chassisui_change(float angle)
{
  float angle_start, angle_end;
  // 弧度转度数，并映射到0~360
  float angle_deg = angle * 57.2957795f; // 180/PI
  angle_deg = fmodf(angle_deg, 360.0f);
  if (angle_deg < 0)
    angle_deg += 360.0f;
  angle_start = angle_deg - 20.0f;
  angle_end = angle_deg + 20.0f;
  if (angle_start < 0.0f)
    angle_start += 360.0f;
  if (angle_end >= 360.0f)
    angle_end -= 360.0f;
  ui_helm_chassis_arc->start_angle = (uint32_t)angle_start;
  ui_helm_chassis_arc->end_angle = (uint32_t)angle_end;
  if (Global.Chassis.mode == FLOW)
    ui_helm_chassis_arc->color = 8; // 非小陀螺白色
  else if (Global.Chassis.mode == SPIN_P)
    ui_helm_chassis_arc->color = 3; // 小陀螺橙色
  else
    ui_helm_chassis_arc->color = 8; // 默认白色
}

void Autoui_change(void)
{
  switch (Global.Auto.input.control_mode) // 自瞄圈
  {
  case 1:
    ui_helm_shoot_auto_rect->color = 4; // 紫红色
    break;
  case 2:
    ui_helm_shoot_auto_rect->color = 2; // 绿色
    break;
  default:
    ui_helm_shoot_auto_rect->color = 4; // 紫红色
    break;
  }
  if (Global.Auto.input.Auto_control_online <= 0)
  {
    ui_helm_shoot_auto_rect->color = 7; // 掉线黑色
  }
}
