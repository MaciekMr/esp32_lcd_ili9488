#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lcd.h"


extern "C" void app_main(void);


void app_main(void)
{
    LCD_COMMON lcd_common;
    lcd_common.init_lcd();
    lcd_common.set_bl(true);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    lcd_common.set_bl(false);
}