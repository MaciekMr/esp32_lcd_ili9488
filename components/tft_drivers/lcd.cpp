#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "lcd.h"
#include "sdkconfig.h"

/*****************************************************
 * Based on configuration select the proper driver
 * and map the function to pointer defiend in lcd.h
*/

#ifdef CONFIG_ILI9488
    //#include "drivers/ili9488.h"
    #include "drivers/lcd_ili9488.h"

#endif


bool notify_lvgl_flush_ready(esp_lcd_panel_io_t*, esp_lcd_panel_io_event_data_t*, void*)
{

    return (true);
}

LCD_COMMON::LCD_COMMON()
{

}

LCD_COMMON::~LCD_COMMON()
{

}

void LCD_COMMON::init_lcd()
{
    //Set the pins D/C BL and CS to output

    pin_conf.pin_bit_mask = ((1<<LCD_DC_PIN) | (1<<LCD_CS_PIN) |(1<<LCD_RS_PIN));
    ESP_LOGI(LCD_TAG, "Pin mask is set to %ju", pin_conf.pin_bit_mask);
    pin_conf.mode = GPIO_MODE_OUTPUT;
    pin_conf.pull_up_en = (gpio_pullup_t) 0;  
    pin_conf.pull_down_en = (gpio_pulldown_t) 0;
    pin_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pin_conf);

    //Init BL pin to controll brigthness via value 0-100
    

    LCD_backlight_channel.gpio_num = (gpio_num_t)LCD_BL_PIN,
    LCD_backlight_channel.speed_mode = BACKLIGHT_LEDC_MODE,
    LCD_backlight_channel.channel = BACKLIGHT_LEDC_CHANNEL,
    LCD_backlight_channel.intr_type = LEDC_INTR_DISABLE,
    LCD_backlight_channel.timer_sel = BACKLIGHT_LEDC_TIMER,
    LCD_backlight_channel.duty = 0,
    LCD_backlight_channel.hpoint = 0,
    LCD_backlight_channel.flags.output_invert = 0; 

    LCD_backlight_timer.speed_mode = BACKLIGHT_LEDC_MODE;
    LCD_backlight_timer.duty_resolution = BACKLIGHT_LEDC_TIMER_RESOLUTION;
    LCD_backlight_timer.timer_num = BACKLIGHT_LEDC_TIMER;
    LCD_backlight_timer.freq_hz = BACKLIGHT_LEDC_FRQUENCY;
    LCD_backlight_timer.clk_cfg = LEDC_AUTO_CLK;

    ESP_LOGI(LCD_TAG, "Initializing LEDC for backlight pin: %d", LCD_BL_PIN);

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));

    //Typical LCD settings
    lcd_io_config.dc_gpio_num = DC;
    lcd_io_config.cs_gpio_num = CS;
    lcd_io_config.pclk_hz = CLOCK;
    lcd_io_config.lcd_cmd_bits = CMD_BITS;
    lcd_io_config.lcd_param_bits = DATA_BITS;
    lcd_io_config.spi_mode = 0;
    lcd_io_config.trans_queue_depth = 10;
    lcd_io_config.on_color_trans_done = notify_lvgl_flush_ready;
    lcd_io_config.user_ctx = &display_driver;
    lcd_io_config.flags.dc_as_cmd_phase = 0;
    lcd_io_config.flags.dc_low_on_data = 0;
    lcd_io_config.flags.octal_mode = 0;
    lcd_io_config.flags.lsb_first = 0;

    //Sett the io inerface of the display
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI_LCD_HOST, &lcd_io_config, &lcd_io_handle));
    
    panel_config.reset_gpio_num = LCD_RS_PIN;
    panel_config.flags.reset_active_high = 0;
    panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
    panel_config.bits_per_pixel = LCD_BITS_PER_PX;
    panel_config.color_space = CONFIG_DISPLAY_COLOR_MODE;
    panel_config.flags.reset_active_high = 0;
    panel_config.vendor_config = NULL;
    
    /*
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &lcd_io_handle)); 

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel(io_handle, &panel_config, &panel_handle));
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    //Now set the BL on and initialise lvgl lib
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    */

}

void LCD_COMMON::set_bl(uint8_t mode) //set backlight on/off via dedicated pin
{
    ESP_LOGI(LCD_TAG, "BL is requested %d", mode?1:0);
    
    if (brightness_percentage > 100)
    {
        brightness_percentage = 100;
    }    
    else if (brightness_percentage < 0)
    {
        brightness_percentage = 0;
    }
    ESP_LOGI(TAG, "Setting backlight to %d%%", brightness_percentage);

    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percentage) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL));
    
    /*
    if(mode)
        gpio_set_level(LCD_BL_PIN, 1);
    else
        gpio_set_level(LCD_BL_PIN, 0);
    */
}

/**********************
 * 
 * Switch reset on for 20ms
*/
void LCD_COMMON::lcd_reset() //reset lcd using pin
{
    gpio_set_level(LCD_RS_PIN,1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(LCD_RS_PIN,0);
}

void LCD_COMMON::set_transmission_mode(bool mode)
{
    if(mode) //set to transfer data
        gpio_set_level(LCD_DC_PIN,1);
    else
        gpio_set_level(LCD_DC_PIN,0);
}

void LCD_COMMON::set_lcd_selected(bool mode) //true start transmission 
{
    if(mode) //set to transfer data
        gpio_set_level(LCD_CS_PIN,1);
    else
        gpio_set_level(LCD_CS_PIN,0);
}