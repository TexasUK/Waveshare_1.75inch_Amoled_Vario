#pragma once

/***********************config*************************/

#define SPI_FREQUENCY         40000000 //
#define TFT_SPI_MODE          SPI_MODE0
#define TFT_SPI_HOST          SPI2_HOST

#define EXAMPLE_LCD_H_RES     466 // sls project horizontal resolution
#define EXAMPLE_LCD_V_RES     466 // sls project vertical resolution
#define LVGL_LCD_BUF_SIZE     (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES)

#define LCD_WIDTH             466 //physical display horizontal resolution
#define LCD_HEIGHT            466 //physical display vertical resolution
#define SEND_BUF_SIZE         (0x4000)//(LCD_WIDTH * LCD_HEIGHT + 8) / 10

// H0175Y003AM - 1.75 inches

#define TFT_QSPI_CS           12
#define TFT_QSPI_SCK          38
#define TFT_QSPI_D0           4
#define TFT_QSPI_D1           5
#define TFT_QSPI_D2           6
#define TFT_QSPI_D3           7
#define TFT_QSPI_RST          39
#define LCD_VCI_EN            -1
#define TFT_TE                -1

// IIC
#define IIC_SDA 15
#define IIC_SCL 14

// TOUCH
#define TOUCH_INT 11
#define TOUCH_RST 40

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA -1

// SD
#define SD_CS -1
#define SD_MOSI -1
#define SD_MISO -1
#define SD_SCLK -1

// PCF8563
#define PCF8563_INT -1

// ===================================================================
// =================== Audio Pin Definitions =========================
// ===================================================================

// I2S audio pins
#define BCLKPIN         9
#define WSPIN          45  
#define DOPIN           8
#define MCLK_OUT_PIN   42
#define PAPIN          46

// Touch configuration
#define TOUCH_SWAP_XY   false
#define TOUCH_MIRROR_X  true
#define TOUCH_MIRROR_Y  true