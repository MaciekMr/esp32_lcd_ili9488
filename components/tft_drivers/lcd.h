/**
  ******************************************************************************
  * @file    lcd.h
  * @author  MCD Application Team
  * @version V4.0.1
  * @date    21-July-2015
  * @brief   This file contains all the functions prototypes for the LCD driver.   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "hal/ledc_types.h"
#include "driver/ledc.h"
#include "lvgl.h"
#include "../spi_controller/spi_interface.h"
/* The 16 bits values (color codes, bitmap) byte order
 * - 0: ne reverse order
 * - 1: reverse order
 *   note: Using reverse order is only recommended in 8 bit fsmc dma mode so that dma can be turned on. 
           In all other cases it is disadvantageous.
 */
#define  LCD_REVERSE16   0

#define LCD_TAG "LCD"

typedef struct
{
  void     (*Init)(void);
  uint16_t (*ReadID)(void);
  void     (*DisplayOn)(void);
  void     (*DisplayOff)(void);
  void     (*SetCursor)(uint16_t, uint16_t);
  void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
  uint16_t (*ReadPixel)(uint16_t, uint16_t);
  
  void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  
  uint16_t (*GetLcdPixelWidth)(void);
  uint16_t (*GetLcdPixelHeight)(void);
  void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
  void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t*);
  void     (*FillRect)(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*ReadRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t*);
  void     (*Scroll)(int16_t, uint16_t, uint16_t);
}LCD_DrvTypeDef;    


#ifdef __cplusplus
}
#endif

#define LCD_DC_PIN CONFIG_SPI_DC_PIN
#define LCD_BL_PIN CONFIG_SPI_BL_PIN
#define LCD_CS_PIN CONFIG_SPI_CS_PIN
#define LCD_RS_PIN CONFIG_SPI_RST_PIN

class LCD_COMMON
{

private:
    ESP32_SPI spi_handler;
    gpio_config_t pin_conf; //config for pin_dc, pin_bl, pin_cs, pin_rst;
    ledc_channel_config_t LCD_backlight_channel;
    ledc_timer_config_t LCD_backlight_timer;
    esp_lcd_panel_dev_config_t panel_config;
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_io_handle_t lcd_io_handle = NULL; //lcdpannel handler
    esp_lcd_panel_io_spi_config_t lcd_io_config;
    lv_disp_drv_t display_driver;

public:
    LCD_COMMON();
    ~LCD_COMMON();
    void init_lcd(); //init pins for D/C CS BL RS for output and set the initial values based on setting 
    void set_bl(uint8_t); //set backlight on/off via dedicated pin
    void lcd_reset(); //reset lcd using pin
    void set_transmission_mode(bool); //set using D/C true->Data , false->command
    void set_lcd_selected(bool);  //set CS to on if true
};




#endif /* __LCD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/