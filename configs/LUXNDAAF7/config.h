/*
 * Lumenier LUX F765 (LUXNDAAF7), STM32F765VIT6, 8 MHz HSE.
 * Pin/timer/DMA defaults from the aircraft's Betaflight 4.5.4 dump (2026-09-22).
 * Sensor and oscillator identification cross-checked against the manufacturer
 * manual and ArduPilot's LumenierLUXF765-NDAA/hwdef.dat.
 * Aircraft tuning and port functions live in the SDK's config.txt.
 */
#pragma once

#define FC_TARGET_MCU STM32F765
#define BOARD_NAME LUXNDAAF7
#define MANUFACTURER_ID LMNR
#define SYSTEM_HSE_MHZ 8

#define USE_ACC
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_SDCARD
#define USE_MAX7456
#define USE_MAG
#define USE_GPS

#define BEEPER_PIN               PC8
#define MOTOR1_PIN               PA0
#define MOTOR2_PIN               PA1
#define MOTOR3_PIN               PA2
#define MOTOR4_PIN               PA3
#define MOTOR5_PIN               PD12
#define MOTOR6_PIN               PD13
#define MOTOR7_PIN               PD14
#define MOTOR8_PIN               PD15
#define RX_PPM_PIN               PE13
#define LED_STRIP_PIN            PB5
#define UART1_TX_PIN             PA9
#define UART2_TX_PIN             PD5
#define UART3_TX_PIN             PD8
#define UART4_TX_PIN             PD1
#define UART5_TX_PIN             PB6
#define UART6_TX_PIN             PC6
#define UART7_TX_PIN             PE8
#define UART8_TX_PIN             PE1
#define UART1_RX_PIN             PA10
#define UART2_RX_PIN             PD6
#define UART3_RX_PIN             PD9
#define UART4_RX_PIN             PD0
#define UART5_RX_PIN             PD2
#define UART6_RX_PIN             PC7
#define UART7_RX_PIN             PE7
#define UART8_RX_PIN             PE0
#define I2C2_SCL_PIN             PB10
#define I2C3_SCL_PIN             PA8
#define I2C2_SDA_PIN             PB11
#define I2C3_SDA_PIN             PC9
#define LED0_PIN                 PE4
#define SPI1_SCK_PIN             PA5
#define SPI2_SCK_PIN             PB13
#define SPI3_SCK_PIN             PC10
#define SPI4_SCK_PIN             PE2
#define SPI1_SDI_PIN             PA6
#define SPI2_SDI_PIN             PB14
#define SPI3_SDI_PIN             PC11
#define SPI4_SDI_PIN             PE5
#define SPI1_SDO_PIN             PA7
#define SPI2_SDO_PIN             PB15
#define SPI3_SDO_PIN             PC12
#define SPI4_SDO_PIN             PE6
#define CAMERA_CONTROL_PIN       PE10
#define ADC_VBAT_PIN             PC3
#define ADC_RSSI_PIN             PC1
#define ADC_CURR_PIN             PC2
#define SDCARD_SPI_CS_PIN        PE12
#define SDCARD_DETECT_PIN        PE3
#define PINIO1_PIN               PC5
#define FLASH_CS_PIN             PB12
#define MAX7456_SPI_CS_PIN       PA15
#define GYRO_1_EXTI_PIN          PE14
#define GYRO_1_CS_PIN            PA4
#define USB_DETECT_PIN           PC4

#define BEEPER_INVERTED
#define ADC1_DMA_OPT 1
#define SPI4_TX_DMA_OPT 0
#define SPI4_RX_DMA_OPT -1
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG
#define DEFAULT_GYRO_TO_USE GYRO_CONFIG_USE_GYRO_1
#define MAG_I2C_INSTANCE I2CDEV_3
#define BARO_I2C_INSTANCE I2CDEV_3
#define DEFAULT_BARO_I2C_ADDRESS 118
#define FLASH_SPI_INSTANCE SPI2
#define MAX7456_SPI_INSTANCE SPI3
#define SDCARD_SPI_INSTANCE SPI4
#define SDCARD_DETECT_INVERTED
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 179
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define PINIO1_CONFIG 129
#define PINIO1_BOX 0
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER SERIALRX_CRSF
#define SERIALRX_UART SERIAL_PORT_USART6

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PE13, 1, 1) \
    TIMER_PIN_MAP( 1, PB5 , 1, 0) \
    TIMER_PIN_MAP( 2, PB0 , 2, 0) \
    TIMER_PIN_MAP( 3, PB1 , 2, 0) \
    TIMER_PIN_MAP( 4, PE9 , 1, 2) \
    TIMER_PIN_MAP( 5, PE11, 1, 1) \
    TIMER_PIN_MAP( 6, PA0 , 1, 0) \
    TIMER_PIN_MAP( 7, PA1 , 1, 0) \
    TIMER_PIN_MAP( 8, PA2 , 1, 0) \
    TIMER_PIN_MAP( 9, PA3 , 1, 0) \
    TIMER_PIN_MAP(10, PD12, 1, 0) \
    TIMER_PIN_MAP(11, PD13, 1, 0) \
    TIMER_PIN_MAP(12, PD14, 1, 0) \
    TIMER_PIN_MAP(13, PD15, 1, -1)
