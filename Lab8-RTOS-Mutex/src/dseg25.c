/*******************************************************************************
 * Size: 25 px
 * Bpp: 1
 * Opts:
 ******************************************************************************/
#define LV_LVGL_H_INCLUDE_SIMPLE
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef DSEG25
#define DSEG25 1
#endif

#if DSEG25

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+002D "-" */
    0x7f, 0xff, 0xfe,

    /* U+002E "." */
    0xff, 0x80,

    /* U+0030 "0" */
    0xff, 0xff, 0x7f, 0xf7, 0x0, 0x1e, 0x0, 0x3c,
    0x0, 0x78, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0,
    0x7, 0x80, 0xf, 0x0, 0x1e, 0x0, 0x18, 0x0,
    0x30, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0, 0x7,
    0x80, 0xf, 0x0, 0x1e, 0x0, 0x3c, 0x0, 0x78,
    0x0, 0xf0, 0x1, 0xff, 0xfd, 0xbf, 0xfe,

    /* U+0031 "1" */
    0x5f, 0xff, 0xfd, 0x7f, 0xff, 0xfd,

    /* U+0032 "2" */
    0xff, 0xfe, 0x7f, 0xf4, 0x0, 0x18, 0x0, 0x30,
    0x0, 0x60, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0,
    0x6, 0x0, 0xc, 0x0, 0x19, 0xff, 0xc3, 0xff,
    0x98, 0x0, 0x30, 0x0, 0x60, 0x0, 0xc0, 0x1,
    0x80, 0x3, 0x0, 0x6, 0x0, 0xc, 0x0, 0x18,
    0x0, 0x30, 0x0, 0x7f, 0xfc, 0xbf, 0xfe,

    /* U+0033 "3" */
    0xff, 0xfe, 0x7f, 0xf4, 0x0, 0x18, 0x0, 0x30,
    0x0, 0x60, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0,
    0x6, 0x0, 0xc, 0x0, 0x19, 0xff, 0xc3, 0xff,
    0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x6,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x60,
    0x0, 0xc0, 0x1, 0x9f, 0xfd, 0x3f, 0xfe,

    /* U+0034 "4" */
    0x0, 0x3, 0x0, 0x7, 0x0, 0x1e, 0x0, 0x3c,
    0x0, 0x78, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0,
    0x7, 0x80, 0xf, 0x0, 0x1f, 0xff, 0xc3, 0xff,
    0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x6,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x60,
    0x0, 0xc0, 0x0, 0x80,

    /* U+0035 "5" */
    0xff, 0xfd, 0x7f, 0xf3, 0x0, 0x6, 0x0, 0xc,
    0x0, 0x18, 0x0, 0x30, 0x0, 0x60, 0x0, 0xc0,
    0x1, 0x80, 0x3, 0x0, 0x7, 0xff, 0xc3, 0xff,
    0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x6,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x60,
    0x0, 0xc0, 0x1, 0x9f, 0xfd, 0x3f, 0xfe,

    /* U+0036 "6" */
    0xff, 0xfd, 0x7f, 0xf3, 0x0, 0x6, 0x0, 0xc,
    0x0, 0x18, 0x0, 0x30, 0x0, 0x60, 0x0, 0xc0,
    0x1, 0x80, 0x3, 0x0, 0x7, 0xff, 0xc3, 0xff,
    0xf8, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0, 0x7,
    0x80, 0xf, 0x0, 0x1e, 0x0, 0x3c, 0x0, 0x78,
    0x0, 0xf0, 0x1, 0xff, 0xfd, 0xbf, 0xfe,

    /* U+0037 "7" */
    0xff, 0xff, 0x7f, 0xf7, 0x0, 0x1e, 0x0, 0x3c,
    0x0, 0x78, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0,
    0x7, 0x80, 0xf, 0x0, 0x1e, 0x0, 0x0, 0x0,
    0x60, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x6,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x60,
    0x0, 0xc0, 0x1, 0x80, 0x0,

    /* U+0038 "8" */
    0xff, 0xff, 0x7f, 0xf7, 0x0, 0x1e, 0x0, 0x3c,
    0x0, 0x78, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0,
    0x7, 0x80, 0xf, 0x0, 0x1f, 0xff, 0xc3, 0xff,
    0xf8, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0, 0x7,
    0x80, 0xf, 0x0, 0x1e, 0x0, 0x3c, 0x0, 0x78,
    0x0, 0xf0, 0x1, 0xff, 0xfd, 0xbf, 0xfe,

    /* U+0039 "9" */
    0xff, 0xff, 0x7f, 0xf7, 0x0, 0x1e, 0x0, 0x3c,
    0x0, 0x78, 0x0, 0xf0, 0x1, 0xe0, 0x3, 0xc0,
    0x7, 0x80, 0xf, 0x0, 0x1f, 0xff, 0xc3, 0xff,
    0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x6,
    0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x60,
    0x0, 0xc0, 0x1, 0x9f, 0xfd, 0x3f, 0xfe,

    /* U+003A ":" */
    0xff, 0x80, 0x0, 0x3, 0xfe};

/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 80, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 326, .box_w = 12, .box_h = 2, .ofs_x = 4, .ofs_y = 11},
    {.bitmap_index = 4, .adv_w = 0, .box_w = 3, .box_h = 3, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 6, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 53, .adv_w = 326, .box_w = 2, .box_h = 24, .ofs_x = 16, .ofs_y = 1},
    {.bitmap_index = 59, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 106, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 153, .adv_w = 326, .box_w = 15, .box_h = 23, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 197, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 244, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 291, .adv_w = 326, .box_w = 15, .box_h = 24, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 336, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 383, .adv_w = 326, .box_w = 15, .box_h = 25, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 430, .adv_w = 80, .box_w = 3, .box_h = 13, .ofs_x = 1, .ofs_y = 6}};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0xd, 0xe};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
    {
        {.range_start = 32, .range_length = 15, .glyph_id_start = 1, .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 3, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY},
        {.range_start = 48, .range_length = 11, .glyph_id_start = 4, .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY}};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};

/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t dseg25 = {
#else
lv_font_t dseg25 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt, /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt, /*Function pointer to get glyph's bitmap*/
    .line_height = 25,                              /*The maximum line height required by the font*/
    .base_line = 0,                                 /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -3,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};

#endif /*#if DSEG25*/
