/**
 * @file lv_conf.h
 * Configuration file for v8.3.x
 * Tailored for RP2040 + GC9A01 (240x240)
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/*Color depth: 1 (1 byte per pixel), 8 (RGB332), 16 (RGB565), 32 (ARGB8888)*/
#define LV_COLOR_DEPTH 16

/*Swap the 2 bytes of RGB565 color. Useful if the display has an 8-bit interface (e.g. SPI)*/
#define LV_COLOR_16_SWAP 1

/*=========================
   MEMORY SETTINGS
 *=========================*/

/*1: use custom malloc/free, 0: use the built-in `lv_mem_alloc` and `lv_mem_free`*/
#define LV_MEM_CUSTOM 0

/*Size of the memory used by `lv_mem_alloc` in bytes (32kB)*/
#define LV_MEM_SIZE (32 * 1024U)

/*Set an address for the memory pool instead of allocating it as a normal array. Can be in external SRAM too.*/
#define LV_MEM_ADR 0

/*====================
   HAL SETTINGS
 *====================*/

/*1: use a custom tick source. (We will feed this from the RP2040 clock)*/
#define LV_TICK_CUSTOM 0
#define LV_TICK_CUSTOM_INCLUDE "pico/time.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (to_ms_since_boot(get_absolute_time()))

/*====================
   FEATURE CONFIGURATION
 *====================*/

/*Enable complex draw engine (Shadows, gradients, etc)*/
#define LV_DRAW_COMPLEX 1

/*Default display refresh period. LVGL will redraw changed areas at this rate (33ms = 30FPS)*/
#define LV_DISP_DEF_REFR_PERIOD 33

/*Input device read period in milliseconds*/
#define LV_INDEV_DEF_READ_PERIOD 33

/*====================
   FONT USAGE
 *====================*/

/*Montserrat fonts with ASCII range and some symbols using bpp = 4
 *https://fonts.google.com/specimen/Montserrat*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 0
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/* Set the default font */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#endif /*LV_CONF_H*/