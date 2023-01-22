
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"


#define SPI_TAG "SPI_IF"


#define MOSI           CONFIG_SPI_MOSI_PIN
#define MISO           CONFIG_SPI_MISO_PIN
#define SCK            CONFIG_SPI_SCK_PIN
#define CS             CONFIG_SPI_CS_PIN
#define RST            CONFIG_SPI_RST_PIN
#define BL             CONFIG_SPI_BL_PIN
#define DC             CONFIG_SPI_DC_PIN
#define CLOCK          CONFIG_SPI_SPEED
#define CMD_BITS       8
#define DATA_BITS      8

#define LCD_H_RES      480
#define INPUT_DELAY_NS 20

#ifdef CONFIG_SPI_HOST2
    #define SPI_LCD_HOST  SPI2_HOST
#endif
#ifdef CONFIG_SPI_HOST1
    #define SPI_LCD_HOST  SPI1_HOST
#endif

extern "C"  void spi_callback();

void cs_high(spi_transaction_t*);
void cs_low(spi_transaction_t*);


struct device_config_t{
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;         ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_eeprom_init()`.
};

class ESP32_SPI
{

friend void spi_callback();

protected:
    spi_bus_config_t buscfg;
    device_config_t  devicecfg;
    spi_device_interface_config_t deviceinterfacecfg;
    spi_device_handle_t  devicehandle;
    esp_lcd_panel_io_handle_t lcd_io_handle = NULL; //lcdpannel handler
    esp_lcd_panel_io_spi_config_t lcd_io_config;

public:
    ESP32_SPI();
    ~ESP32_SPI();
    void init_spi(); //Initialise spi channel with pins
    void deinit_spi();
    void set_spi_speed(); //set spi speed in Hz
    void set_dma_ch();  //Set DMA channel
    void add_device();
    void write_data();
    void read_data();
    void write_command();

};