/**
 * @file lv_conf.h
 * LVGL配置文件
 * 针对ESP32-C3单色OLED优化
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   颜色设置
 *====================*/

/* 颜色深度：1位单色显示 */
#define LV_COLOR_DEPTH 1

/* 交换RGB565颜色的2个字节（16位色才需要） */
#define LV_COLOR_16_SWAP 0

/* 1位和8位模式的屏幕透明度 */
#define LV_COLOR_SCREEN_TRANSP 0

/*=========================
   内存设置
 *=========================*/

/* 1: 使用自定义malloc/free，0: 使用内置的lv_mem_alloc()/lv_mem_free() */
#define LV_MEM_CUSTOM 0

/* LVGL内部内存池大小（单位：字节） */
/* 对于128x64单色屏幕，可以设置较小值 */
#define LV_MEM_SIZE (16U * 1024U)  /* 16KB，可根据实际调整 */

/* 内存池缓冲区的地址（仅LV_MEM_CUSTOM=0时使用） */
#define LV_MEM_BUF_MAX_NUM 16

/* 使用memcpy和memset代替自定义的mem函数 */
#define LV_MEMCPY_MEMSET_STD 1

/*====================
   HAL设置
 *====================*/

/* 默认显示刷新周期（毫秒） */
#define LV_DISP_DEF_REFR_PERIOD 30

/* 输入设备读取周期（毫秒） */
#define LV_INDEV_DEF_READ_PERIOD 30

/* 使用自定义tick源 */
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "freertos/FreeRTOS.h"
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (xTaskGetTickCount() * portTICK_PERIOD_MS)
#endif

/* DPI（每英寸点数） - 用于字体缩放 */
#define LV_DPI_DEF 130

/*=================
   操作系统设置
 *=================*/

/* 1: 使用操作系统的malloc/free */
#define LV_USE_OS LV_OS_FREERTOS

/*=======================
   功能配置
 *=======================*/

/* 日志配置 */
#define LV_USE_LOG 1
#if LV_USE_LOG
    /* 日志级别：TRACE, INFO, WARN, ERROR, USER, NONE */
    #define LV_LOG_LEVEL LV_LOG_LEVEL_INFO
    
    /* 使用printf打印日志 */
    #define LV_LOG_PRINTF 1
    
    #if LV_LOG_PRINTF
        #include <stdio.h>
        #define LV_LOG_PRINTF_INCLUDE <stdio.h>
    #endif
#endif

/* 1: 启用断言（开发阶段建议启用） */
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0

/* 1: 使用用户数据 */
#define LV_USE_USER_DATA 1

/*==================
   字体配置
 *==================*/

/* Montserrat字体系列（LVGL内置） */
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1  /* 12号字体，适合小屏幕 */
#define LV_FONT_MONTSERRAT_14 1  /* 14号字体 */
#define LV_FONT_MONTSERRAT_16 0
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0

/* 默认字体 */
#define LV_FONT_DEFAULT &lv_font_montserrat_12

/* 启用内置字体的子像素渲染（仅对16位以上色深有效） */
#define LV_FONT_SUBPX_BGR 0

/*===================
   组件配置
 *===================*/

/* 基础组件（必需） */
#define LV_USE_OBJ 1
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_IMG 1

/* 其他组件（按需启用以节省内存） */
#define LV_USE_ARC 0
#define LV_USE_BAR 0
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS 0
#define LV_USE_CHECKBOX 0
#define LV_USE_DROPDOWN 0
#define LV_USE_LINE 0
#define LV_USE_ROLLER 0
#define LV_USE_SLIDER 0
#define LV_USE_SWITCH 0
#define LV_USE_TEXTAREA 0
#define LV_USE_TABLE 0

/* 扩展绘图（根据需要启用） */
#define LV_USE_DRAW_MASKS 0
#define LV_USE_DRAW_TRANSFORM 0

/*==================
   主题配置
 *==================*/

/* 单色主题最适合1位显示 */
#define LV_USE_THEME_DEFAULT 0
#define LV_USE_THEME_BASIC 0
#define LV_USE_THEME_MONO 1  /* 单色主题 */

/*==================
   示例和演示
 *==================*/

/* 禁用内置示例以节省闪存 */
#define LV_BUILD_EXAMPLES 0

/*==================
   其他设置
 *==================*/

/* 快照功能（截图） */
#define LV_USE_SNAPSHOT 0

/* 文件系统支持 */
#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0

/* PNG解码器 */
#define LV_USE_PNG 0

/* BMP解码器 */
#define LV_USE_BMP 0

/* JPG解码器 */
#define LV_USE_SJPG 0

/* GIF解码器 */
#define LV_USE_GIF 0

/* QR码生成器 */
#define LV_USE_QRCODE 0

/* FreeType字体渲染 */
#define LV_USE_FREETYPE 0

/* RLOTTIE动画 */
#define LV_USE_RLOTTIE 0

/* FFmpeg视频支持 */
#define LV_USE_FFMPEG 0

#endif /* LV_CONF_H */