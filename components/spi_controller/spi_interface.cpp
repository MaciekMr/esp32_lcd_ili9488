

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"
#include "spi_interface.h"

/*****************************************
 * All commands set to SPI will be transfered via queue
 * SPI interface will take these commands/data and trnsfer using DMA to LCD
 * SPI write part will be run as task
 * https://github.com/espressif/esp-idf/blob/49551cc48cb3cdd5563059028749616de313f0ec/examples/peripherals/lcd/spi_lcd_touch/main/spi_lcd_touch_example_main.c
*/

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    //lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    //lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(void *)
{

}

/*
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
*/
void spi_callback()
{

}

void task_spi_handler()
{

}

void cs_high(spi_transaction_t*)
{

}

void cs_low(spi_transaction_t*)
{
    
}



ESP32_SPI::ESP32_SPI()
{

}

ESP32_SPI::~ESP32_SPI()
{

}

void ESP32_SPI::init_spi()
{
   esp_err_t err = ESP_OK;

    ESP_LOGI(SPI_TAG, "Initializing bus SPI%d...", SPI_LCD_HOST+1);
    
    buscfg.miso_io_num = MISO;
    buscfg.mosi_io_num = MOSI;
    buscfg.sclk_io_num = SCK;
    buscfg.quadwp_io_num = -1;  //write protect 
    buscfg.quadhd_io_num = -1;  //hold state
    buscfg.data4_io_num = GPIO_NUM_NC;
    buscfg.data5_io_num = GPIO_NUM_NC;
    buscfg.data6_io_num = GPIO_NUM_NC;
    buscfg.data7_io_num = GPIO_NUM_NC;
    //buscfg.max_transfer_sz = 0;  // DMA enabled 32,
    buscfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);
    buscfg.flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER;
    buscfg.intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM;
    //Initialize the SPI bus
    err = spi_bus_initialize(SPI_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(SPI_TAG, "Attach to main flash bus...");

    
    
    /*
    devicecfg.cs_io  = (gpio_num_t)CS;
    devicecfg.host = SPI_LCD_HOST;
    devicecfg.miso_io = (gpio_num_t)MISO;


    deviceinterfacecfg.command_bits = 8;
    deviceinterfacecfg.clock_speed_hz = CLOCK;
    deviceinterfacecfg.mode = 0;
    deviceinterfacecfg.spics_io_num = -1;
    deviceinterfacecfg.queue_size = 1;
    deviceinterfacecfg.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS;
    deviceinterfacecfg.pre_cb = cs_high;
    deviceinterfacecfg.post_cb = cs_low;
    deviceinterfacecfg.input_delay_ns = INPUT_DELAY_NS;

    err = spi_bus_add_device(SPI_LCD_HOST, &deviceinterfacecfg, &devicehandle);
    */


    if  (err != ESP_OK) {
        ESP_LOGE(SPI_TAG, "Add device has failed!");
    }
}