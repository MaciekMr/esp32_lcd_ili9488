set(tft_drv "lcd.cpp")

idf_component_register(SRCS "${tft_drv}"
         INCLUDE_DIRS "."
         REQUIRES esp_common freertos hal log nvs_flash driver esp_timer esp_lcd)

target_compile_options(${COMPONENT_LIB} PUBLIC "-Wno-unused-variable" "-Wno-missing-field-initializers" "-fpermissive" -std=gnu++2b)