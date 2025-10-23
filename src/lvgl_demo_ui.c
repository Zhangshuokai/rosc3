/**
 * @file lvgl_demo_ui.c
 * @brief LVGL UI演示代码
 */

#include "lvgl.h"
#include "lvgl_demo_ui.h"

/**
 * @brief 创建演示UI界面
 * @param disp LVGL显示对象指针
 */
void lvgl_demo_ui(lv_disp_t *disp)
{
    /* 获取当前活动屏幕 */
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    
    /* 创建标签对象 */
    lv_obj_t *label = lv_label_create(scr);
    
    /* 设置长文本模式为循环滚动 */
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    
    /* 设置文本内容 */
    lv_label_set_text(label, "Hello ESP32-C3! Hello LVGL!");
    
    /* 设置标签宽度为屏幕宽度 */
    lv_obj_set_width(label, disp->driver->hor_res);
    
    /* 对齐到屏幕顶部中心 */
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}