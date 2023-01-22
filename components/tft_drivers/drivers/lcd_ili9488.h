/*
 * SPDX-FileCopyrightText: 2022 atanisoft (github.com/atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "esp_lcd_panel_vendor.h"

#define LCD_BITS_PER_PX     18

#ifdef __cplusplus
extern "C" {
#endif

static const ledc_mode_t BACKLIGHT_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t BACKLIGHT_LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_t BACKLIGHT_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t BACKLIGHT_LEDC_TIMER_RESOLUTION = LEDC_TIMER_10_BIT;
static const uint32_t BACKLIGHT_LEDC_FRQUENCY = 5000;

/**
 * @brief Create LCD panel for model ILI9488
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[in] buffer_size size of buffer to allocate for color conversions.
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 * 
 * NOTE: If you are using the SPI interface you *MUST* 18-bit color mode
 * in @param panel_dev_config field bits_per_pixel and @param buffer_size
 * must be provided.
 * 
 * NOTE: For parallel IO (Intel 8080) interface 16-bit color mode should
 * be used and @param buffer_size will be ignored.

 */
esp_err_t esp_lcd_new_panel_ili9488(const esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    const size_t buffer_size,
                                    esp_lcd_panel_handle_t *ret_panel);

#define esp_lcd_new_panel_ili9341 esp_lcd_new_panel

#ifdef __cplusplus
}
#endif