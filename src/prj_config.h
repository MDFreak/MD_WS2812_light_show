#ifndef _PRJ_CONFIG_H_
  #define _PRJ_CONFIG_H_

  #include <Arduino.h>
  #include <md_defines.h>
  #include <project.h>

  // ******************************************
  // --- board management
    #if !(BOARD ^ MC_ESP32_Node)
      // --- system
        #define SER_BAUDRATE ESP_SER_BAUD
        // --- network
        // --- user output
          // --- LEDs
          // --- display
          // --- acustic output
        // --- user input
        // --- sensors
        // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK_OUT > OFF)
              #define PIN_BOARD_LED 2
            #endif
        // --- user input
          #if (USE_CTRL_POTI_ADC > OFF)
              #define PIN_INP_POTI_1 35   // ADC 1-5
            #endif

          #if (USE_CTRL_SW_INP > OFF)
              #define PIN_INP_SW_1  32   // INPUT_PULLUP
            #endif

          #if (USE_FAN_CNT_INP > OFF)
              #define PIN_CNT_FAN_1 36
              #define PIN_CNT_FAN_2 34
            #endif
        #if (USE_PWM_INP > OFF)
              #define PIN_PWM_INP_1 PIN_CNT_FAN_2
          #endif
        // --- user output
          #if (USE_TRAFFIC_LED_OUT > OFF)
              #define PIN_TL_RED    26   // RGB red
              #define PIN_TL_YELLOW 25   // RGB green
              #define PIN_TL_GREEN  33   // RGB blue
            #endif

          #if (USE_RGBLED_PWM > OFF)
              #define PIN_RGB_RED   33 //26   // RGB red
              #define PIN_RGB_GREEN 26   // RGB blue
              #define PIN_RGB_BLUE  14 //33   // RGB blue
            #endif

          #if (USE_FAN_PWM > OFF)
              #define PIN_PWM_FAN_1 0
              #define PIN_PWM_FAN_2 4
            #endif

          #if (USE_OUT_FREQ_PWM > OFF)
              #define PIN_FREQ_1    26
            #endif

          #if (USE_WS2812_MATRIX_OUT > OFF)
              #define PIN_WS2812_MD1  5
                //#define PIN_WS2812_MD2  x
                //#define PIN_WS2812_MD3  x
                //#define PIN_WS2812_MD4  x
            #endif

          #if (USE_WS2812_LINE_OUT > OFF)
              #define PIN_WS2812_LD1     12
                //#define PIN_WS2812_D2  x
                //#define PIN_WS2812_D3  x
                //#define PIN_WS2812_D4  x
            #endif

          #if (USE_TFT > OFF)
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif

          #if (USE_BUZZER_PWM > OFF)
              #define PIN_BUZZ      32
            #endif
        // --- sensors
          #if (USE_DS18B20_1W_IO > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_TYPE_K_SPI > OFF)
              #define TYPEK_DATA_PIN 19   // SPI MISO
              #define TYPEK_CLK_PIN  18   // SPI CLK
              #define TYPEK1_CS_PIN  16
              #define TYPEK2_CS_PIN  17
            #endif
          #if (USE_MQ135_GAS_ADC > OFF)
              #define PIN_MQ135     36
            #endif

        // --- PWM channels   0..15
          #if (USE_PWM_OUT > OFF)
              #if (USE_AOUT > OFF)
                  #if (USE_BUZZER_PWM > OFF)
                      #define PWM_BUZZ  0
                    #endif
                #endif

              #if (USE_TRAFFIC_LED_OUT > OFF)
                  #define PWM_TL_GREEN  1
                  #define PWM_TL_YELLOW 2
                  #define PWM_TL_RED    3
                #endif

              #if (USE_RGBLED_PWM > OFF)
                  #define PWM_RGB_RED   1
                  #define PWM_RGB_GREEN 2
                  #define PWM_RGB_BLUE  3
                #endif

              #if (USE_FAN_PWM > OFF)
                  #define PWM_FAN_1     4
                  #define PWM_FAN_2     5
                #endif

              #if (USE_OUT_FREQ_PWM > OFF)
                  #define PWM_FREQ_1    6
                #endif
            #endif

        // --- ADC channels
          #if (USE_ADC1 > 0)
              #if (USE_MQ135_GAS_ADC > OFF)
                  #define ADC_MQ135       3   // ADC 1-3
                #endif

              #if (USE_CTRL_POTI_ADC > OFF)
                  #define ADC_INP_POTI_1  5   // ADC 1-5
                #endif

            #endif

        // --- counter channels  0..7
          #if (USE_CNT_INP > OFF)
              #if (USE_FAN_CNT_INP > OFF)
                  #define CNT_FAN_1     0
                  #define CNT_FAN_2     1
                #endif
            #endif

        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif

      #endif

    #if !(BOARD ^ MC_ESP32_D1_MINI)
      // --- system
        #define SER_BAUDRATE ESP_SER_BAUD
        // --- network
        // --- user output
          // --- LEDs
          // --- display
          // --- acustic output
        // --- user input
        // --- sensors
        // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK_OUT > OFF)
              #define PIN_BOARD_LED 2
            #endif
        // --- user input
          #if (USE_CTRL_POTI_ADC > OFF)
              #define PIN_INP_POTI_1 35   // ADC 1-5
            #endif

          #if (USE_CTRL_SW_INP > OFF)
              #define PIN_INP_SW_1  32   // INPUT_PULLUP
            #endif

          #if (USE_FAN_CNT_INP > OFF)
              #define PIN_CNT_FAN_1 36
              #define PIN_CNT_FAN_2 34
            #endif
        #if (USE_PWM_INP > OFF)
              #define PIN_PWM_INP_1 PIN_CNT_FAN_2
          #endif
        // --- user output
          #if (USE_TRAFFIC_LED_OUT > OFF)
              #define PIN_TL_RED    26   // RGB red
              #define PIN_TL_YELLOW 25   // RGB green
              #define PIN_TL_GREEN  33   // RGB blue
            #endif

          #if (USE_RGBLED_PWM > OFF)
              #define PIN_RGB_RED   33 //26   // RGB red
              #define PIN_RGB_GREEN 26   // RGB blue
              #define PIN_RGB_BLUE  14 //33   // RGB blue
            #endif

          #if (USE_FAN_PWM > OFF)
              #define PIN_PWM_FAN_1 0
              #define PIN_PWM_FAN_2 4
            #endif

          #if (USE_OUT_FREQ_PWM > OFF)
              #define PIN_FREQ_1    26
            #endif

          #if (USE_WS2812_MATRIX_OUT > OFF)
              #define PIN_WS2812_M1      16
              #if (USE_WS2812_MATRIX_OUT > 1)
                  #define PIN_WS2812_M2  17
                #endif
                //#define PIN_WS2812_M3  x
                //#define PIN_WS2812_M4  x
            #endif

          #if (USE_WS2812_LINE_OUT > OFF)
              #define PIN_WS2812_L1      16
              #define PIN_WS2812_L2      17
                //#define PIN_WS2812_L3  x
                //#define PIN_WS2812_L4  x
            #endif

          #if (USE_TFT > OFF)
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif

          #if (USE_BUZZER_PWM > OFF)
              #define PIN_BUZZ      32
            #endif
        // --- sensors
          #if (USE_DS18B20_1W_IO > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_TYPE_K_SPI > OFF)
              #define TYPEK_DATA_PIN 19   // SPI MISO
              #define TYPEK_CLK_PIN  18   // SPI CLK
              #define TYPEK1_CS_PIN  16
              #define TYPEK2_CS_PIN  17
            #endif
          #if (USE_MQ135_GAS_ADC > OFF)
              #define PIN_MQ135     36
            #endif

        // --- PWM channels   0..15
          #if (USE_PWM_OUT > OFF)
              #if (USE_AOUT > OFF)
                  #if (USE_BUZZER_PWM > OFF)
                      #define PWM_BUZZ  0
                    #endif
                #endif

              #if (USE_TRAFFIC_LED_OUT > OFF)
                  #define PWM_TL_GREEN  1
                  #define PWM_TL_YELLOW 2
                  #define PWM_TL_RED    3
                #endif

              #if (USE_RGBLED_PWM > OFF)
                  #define PWM_RGB_RED   1
                  #define PWM_RGB_GREEN 2
                  #define PWM_RGB_BLUE  3
                #endif

              #if (USE_FAN_PWM > OFF)
                  #define PWM_FAN_1     4
                  #define PWM_FAN_2     5
                #endif

              #if (USE_OUT_FREQ_PWM > OFF)
                  #define PWM_FREQ_1    6
                #endif
            #endif

        // --- ADC channels
          #if (USE_ADC1 > 0)
              #if (USE_MQ135_GAS_ADC > OFF)
                  #define ADC_MQ135       3   // ADC 1-3
                #endif

              #if (USE_CTRL_POTI_ADC > OFF)
                  #define ADC_INP_POTI_1  5   // ADC 1-5
                #endif

            #endif

        // --- counter channels  0..7
          #if (USE_CNT_INP > OFF)
              #if (USE_FAN_CNT_INP > OFF)
                  #define CNT_FAN_1     0
                  #define CNT_FAN_2     1
                #endif
            #endif

        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif

      #endif

    #if !(BOARD ^ MC_ESP32_D1_R32)
      // --- system
      // --- network
      // --- user output
        // --- LEDs
        // --- display
        // --- acustic output
      // --- user input
      // --- sensors
      // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK_OUT > 0)
              #define PIN_BOARD_LED   2
            #endif

        // --- user output
          #if (USE_TRAFFIC_LED_OUT > 0)
              // traffic lamp
              #define PIN_TL_RED    17
              #define PIN_TL_YELLOW 25
              #define PIN_TL_GREEN  26
            #endif
          #ifdef USE_TFT
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif
          #if defined(USE_BUZZER_PWM)
              #define PIN_BUZZ      27
            #endif
        // --- sensors
          #if (USE_DS18B20_1W_IO > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_MQ135_GAS_ADC > OFF)
              #define PIN_MQ135     34
              #define PIN_CO2_THOLD 35     // analog threshold for evaluation
            #endif

        // --- PWM
          // --- channels
            #define PWM_BUZZ          0
            #define PWM_TL_GREEN      1
            #define PWM_TL_YELLOW     2
            #define PWM_TL_RED        3

        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif
      #endif

    #if !(BOARD ^ MC_AV_NANO_V3)
      #endif

    #if !(BOARD ^ MC_AV_UNO_V3)
      #endif // BOARD

  // ******************************************
  // --- specification
    // --- system
      // --- error status bits
        #define ERRBIT_TOUCH     0x00000001     // touchscreen
        #define ERRBIT_SERVER    0x00000002     // webserver
        #define ERRBIT_WIFI      0x00000004     // WIFI connection
        #define ERRBIT_NTPTIME   0x00000008     // NTP timeserver connection
      // --- generic
        #define SCAN_I2C   128
        #define CHECK_I2C_DEVICES
        //#define UTC_SEASONTIME UTC_WINTERTIME
        #define UTC_SEASONTIME UTC_SUMMERTIME
        #if (USE_TASKING > 0)
            #define USE_SONGTASK
            #define TASK_SLICE_T  5000ul   // task beat [us]
          #endif // USE_TASKING
        #if (USE_LED_BLINK_OUT > 0)
            #define BLINKTIME_MS  1200ul
            #define BLINKPWM_FREQ 400
            #define BLINKPWM_RES  8
          #endif

    // --- I2C interface
      // --- address configuration
        #if (USE_OLED_I2C > OFF)
            #define I2C_ADDR_OLED1      I2C_OLED_3C
            #define I2C_OLED1_USE_I2C1
            #if defined( I2C_OLED1_USE_I2C1 )
                #define I2C_OLED1       I2C1
                #define I2C_SCL_OLED1   PIN_I2C1_SCL
                #define I2C_SDA_OLED1   PIN_I2C1_SDA
              #else
                  #define I2C_OLED1     I2C
                  #define I2C_SCL_OLED1 PIN_I2C2_SCL
                  #define I2C_SDA_OLED1 PIN_I2C2_SDA
              #endif
            #if (( USE_I2C > 1 ) && ( USE_OLED_I2C > 1 ))
                #define I2C_ADDR_OLED2      I2C_OLED
                #define I2C_OLED2_USE_I2C2
                  #if defined( I2C_OLED2_USE_I2C1 )
                    #define I2C_OLED2       I2C1
                    #define I2C_SCL_OLED2   PIN_I2C1_SCL
                    #define I2C_SDA_OLED2   PIN_I2C1_SDA
                  #else
                      #define I2C_OLED2     I2C2
                      #define I2C_SCL_OLED2 PIN_I2C2_SCL
                      #define I2C_SDA_OLED2 PIN_I2C2_SDA
                    #endif
              #endif
          #endif

        #if (USE_FRAM_I2C > OFF)
            #define I2C_ADDR_FRAM1          I2C_FRAM_50
            #define I2C_FRAM1_USE_I2C1
              #if defined( I2C_FRAM1_USE_I2C1 )
                #define I2C_FRAM1           I2C1
                #define I2C_SCL_FRAM1       PIN_I2C1_SCL
                #define I2C_SDA_FRAM1       PIN_I2C1_SDA
              #else
                  #define I2C_FRAM1         I2C2
                  #define I2C_SCL_FRAM1     PIN_I2C2_SCL
                  #define I2C_SDA_FRAM1     PIN_I2C2_SDA
                #endif
            #if (( USE_I2C > 1 ) && ( USE_FRAM_I2C > 1 ))
              #define I2C_ADDR_FRAM2        I2C_FRAM_50
                #define I2C_FRAM2_USE_I2C2
                  #if defined( I2C_FRAM2_USE_I2C1 )
                    #define I2C_FRAM2       I2C1
                    #define I2C_SCL_FRAM2   I2C1_SCL
                    #define I2C_SDA_FRAM2   I2C1_SDA
                  #else
                      #define I2C_FRAM2     I2C2
                      #define I2C_SCL_FRAM2 I2C2_SCL
                      #define I2C_SDA_FRAM2 I2C2_SDA
                    #endif
              #endif
          #endif

        #if ( USE_BME280_I2C > OFF )
            #define I2C_ADDR_BME2801        I2C_BME280
            #define I2C_BME2801_USE_I2C1
              #if defined( I2C_BME2801_USE_I2C1 )
                #define I2C_BME2801         I2C1
                #define I2C_SCL_BME2801     PIN_I2C1_SCL
                #define I2C_SDA_BME2801     PIN_I2C1_SDA
              #else
                  #define I2C_BME2801       I2C2
                  #define I2C_SCL_BME2801   PIN_I2C2_SCL
                  #define I2C_SDA_BME2801   PIN_I2C2_SDA
                #endif
            #if (( USE_I2C > 1 ) && ( USE_BME280_I2C > 1 ))
                #define I2C_ADDR_BME2802      I2C_BME280
                #define I2C_BME2802_USE_I2C2
                  #if defined( I2C_BME2802_USE_I2C1 )
                    #define I2C_BME2802       I2C1
                    #define I2C_SCL_BME2802   I2C1_SCL
                    #define I2C_SDA_BME2802   I2C1_SDA
                  #else
                      #define I2C_BME2802     I2C2
                      #define I2C_SCL_BME2802 I2C2_SCL
                      #define I2C_SDA_BME2802 I2C2_SDA
                    #endif
              #endif
          #endif

    // --- user interface
      // --- display output
        #define DISP_CYCLE       1000ul   // Intervallzeit [us]
        // output status line
        #define STAT_TIMEDEF     5000u    // default time to clear status

      // WS2812 LEDs
        #if (USE_WS2812_MATRIX_OUT > OFF)
            #ifndef USE_OUTPUT_CYCLE
                #define USE_OUTPUT_CYCLE
              #endif
            #define COLCHAR_2812   6
            #define COLBMP_2812    8
            #define ROWBMP_2812    8
            #define UPD_2812_M1_MS 8
            #define BRIGHT_2812_M1 5

            #define ROW1_2812_M1   NEO_MATRIX_TOP
            #define COL1_2812_M1   NEO_MATRIX_LEFT
            #define DIR_2812_M1    NEO_MATRIX_COLUMNS
            #define ORI_2812_M1    NEO_MATRIX_ZIGZAG

            #define TYPE_2812_M1   WS2812B
            #define COLORD_2812_M1 NEO_GRB
            #define COLPIX_2812_M1 64
            #define ROWPIX_2812_M1 8
            #define LEDS_2812_M1   COLPIX_2812_M1 * ROWPIX_2812_M1

            #define ROW1_2812_T1   NEO_TILE_TOP
            #define COL1_2812_T1   NEO_TILE_LEFT
            #define DIR_2812_T1    NEO_TILE_COLUMNS
            #define ORI_2812_T1    NEO_TILE_ZIGZAG

            #define COLPIX_2812_T1 32
            #define ROWPIX_2812_T1 8
            #define COLTIL_2812_M1 4  // 0 = OFF
            #define ROWTIL_2812_M1 1
            #define ANZ_TILES_M1   COLTIL_2812_M1 * ROWPIX_2812_T1
              #if (USE_WS2812_MATRIX_OUT > 1)
                  #ifndef USE_OUTPUT_CYCLE
                      #define USE_OUTPUT_CYCLE
                    #endif
                  #define UPD_2812_M2_MS 8
                  #define LEDS_2812_M2   512
                  #define BRIGHT_2812_M2 5
                  #define TYPE_2812_M2   WS2812B
                  #define COLORD_2812_M2 NEO_GRB
                  #define COLPIX_2812_M2 128
                  #define ROWPIX_2812_M2 8
                  #define COLPIX_2812_T2 8
                  #define ROWPIX_2812_T2 8
                  #define COLTIL_2812_M2 4
                  #define ROWTIL_2812_M2 1
                #endif
          #endif

        #if (USE_WS2812_LINE_OUT > OFF)
            #ifndef USE_OUTPUT_CYCLE
                #define USE_OUTPUT_CYCLE
              #endif

            #define USE_FAST_LED
            #ifdef USE_FAST_LED
                #define UPD_2812_L1_MS 10
                #define LEDS_2812_L1   599
                #define BRIGHT_2812_L1 5
                #define TYPE_2812_L1   WS2812B
                #define COLORD_2812_L1 GRB
            #else
                #define UPD_2812_L1_MS 1
                #define LEDS_2812_L1   300
                #define BRIGHT_2812_L1 5
                #define TYPE_2812_L1   WS2812B
                #define COLORD_2812_L1 NEO_GRB
                #define COLPIX_2812_L1 300
                #define ROWPIX_2812_L1 1
                //#define COLPIX_2812_T1 8
                //#define ROWPIX_2812_T1 8
                #define COLTIL_2812_L1 4
                #define ROWTIL_2812_L1 1
              #endif
            #if (USE_WS2812_LINE_OUT > 1)
                #define UPD_2812_L2_MS 10
                #define LEDS_2812_L2   30
                #define BRIGHT_2812_L2 12
                #define TYPE_2812_L2   WS2812B
                #define COLORD_2812_L2 GRB
                #if (USE_WS2812_LINE_OUT > 2)
                    #define LEDS_2812_L3   30
                    #define BRIGHT_2812_L3 12
                    #define TYPE_2812_L3   WS2812
                    #define COLORD_2812_L3 GRB
                    #if (USE_WS2812_LINE_OUT > 3)
                        #define LEDS_2812_L4   30
                        #define BRIGHT_2812_L4 12
                        #define TYPE_2812_L4   WS2812
                        #define COLORD_2812_L4 GRB
                      #endif
                  #endif
              #endif
          #endif

    // --- network
      // --- WIFI
        #if (USE_WIFI > OFF)
            #define WIFI_ANZ_LOGIN  3
            #define WIFI_SSID0      "MAMD-HomeG" // WLAN Moosgrabenstrasse 26
            #define WIFI_SSID0_PW   "ElaNanniRalf3"
            #define WIFI_SSID1      "HS-HomeG" // WLAN Am Jungberg 9
            #define WIFI_SSID1_PW   "ElaNanniRalf3"
            #define WIFI_SSID2      "FairHandeln" //Weltladen
            #define WIFI_SSID2_PW   "WL&Fair2Live#"
            #define WIFI_CONN_DELAY 500000ul // Scan-Abstand [us]
            #define WIFI_CONN_REP   5        // Anzahle der Connect-Schleifen
            #define WIFI_CONN_CYCLE 4000ul   // Intervallzeit fuer Recoonect [us]
            #define NTPSERVER_CYCLE 1000ul   // Intervallzeit [us]

            #define WIFI_ANZ_LOCIP  WIFI_ANZ_LOGIN
            #if !(BOARD ^ MC_ESP32_Node)
                #define WIFI_FIXIP0     0x1400000Aul // 10.0.0.20   lowest first
            #elif !(BOARD ^ MC_ESP32_D1_MINI)
                #define WIFI_FIXIP0     0x1600000Aul // 10.0.0.22   lowest first
            #elif !(BOARD ^ MC_ESP32_D1_R32)
                #define WIFI_FIXIP0     0x1A00000Aul // 10.0.0.26   lowest first
              #endif
            #define WIFI_GATEWAY0   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP1     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY1   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP2     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY2   0x8a00000Aul // 10.0.0.138
            #define WIFI_SUBNET     0x00FFFFFFul // 255.255.255.0
          #endif

      // --- webserver
        #if (USE_WEBSERVER > OFF)
            #define WEBSERVER_CYCLE 1000ul  // Intervallzeit [us]
          #endif

    // --- user output
      // --- PWM
        #if (USE_RGBLED_PWM > OFF)
            #ifndef USE_OUTPUT_CYCLE
                #define USE_OUTPUT_CYCLE
              #endif
            #define PWM_LEDS_FREQ      4000u
            #define PWM_LEDS_RES       8
          #endif

        #if (USE_FAN_PWM > OFF)
            #ifndef USE_OUTPUT_CYCLE
                #define USE_OUTPUT_CYCLE
              #endif
            #define PWM_FAN_FREQ      4500u
            #define PWM_FAN_RES       8
          #endif

      // --- display
        #if (USE_DISP > 0)
            #define USE_STATUS
            #if (USE_OLED_I2C > OFF)
                #if ( OLED1  > OFF )
                    #define USE_STATUS1
                          // select OLED - device & GEO   GEO_RAWMODE?
                          // MC_UO_OLED_066_AZ   GEO_64_48    OLED_DRV_1306
                          // MC_UO_OLED_091_AZ   GEO_128_32   OLED_DRV_1306
                          // MC_UO_OLED_096_AZ   GEO_128_64   OLED_DRV_1306
                          // MC_UO_OLED_130_AZ   GEO_128_64   OLED_DRV_1106
                    #if !(OLED1 ^ MC_UO_OLED_066_AZ)
                        #define DISP1_MAXCOLS  OLED_066_MAXCOLS
                        #define DISP1_MAXROWS  OLED_066_MAXROWS
                        #define OLED1_GEO      GEO_64_48
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_091_AZ)
                        #define DISP1_MAXCOLS  OLED_091_MAXCOLS
                        #define DISP1_MAXROWS  OLED_091_MAXROWS
                        #define OLED1_GEO      GEO_128_32
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_096_AZ)
                        #define DISP1_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP1_MAXROWS  OLED_096_MAXROWS
                        #define OLED1_GEO      GEO_128_64
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_130_AZ)
                        #define DISP1_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP1_MAXROWS  OLED_130_MAXROWS
                        #define OLED1_GEO      GEO_128_64
                        #define OLED1_DRV      OLED_DRV_1106
                      #endif
                  #endif
                #if defined( OLED2 )
                    #define USE_STATUS2
                    #if !(OLED2 ^ MC_UO_OLED_066_AZ)
                        #define DISP2_MAXCOLS  OLED_066_MAXCOLS
                        #define DISP2_MAXROWS  OLED_066_MAXROWS
                        #define OLED2_GEO      GEO_64_48
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_091_AZ)
                        #define DISP2_MAXCOLS  OLED_091_MAXCOLS
                        #define DISP2_MAXROWS  OLED_091_MAXROWS
                        #define OLED2_GEO      GEO_128_32
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_096_AZ)
                        #define DISP2_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP2_MAXROWS  OLED_096_MAXROWS
                        #define OLED2_GEO      GEO_128_64
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_130_AZ)
                        #define DISP2_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP2_MAXROWS  OLED_130_MAXROWS
                        #define OLED2_GEO      GEO_128_64
                        #define OLED2_DRV      OLED_DRV_1106
                      #endif
                  #endif

              #endif
            #if ( USE_TOUCHSCREEN > 0 )
              #endif // USE_TOUCHSCREEN

            #if ( USE_TFT > 0 )
                #if !(DISP_TFT ^ MC_UO_TOUCHXPT2046_AZ)
                    #define DISP_ORIENT    0      // 0:USB oben, 1:USB links, 2:USB unten, 3:USB rechts
                    #define DATE_DISP_COL  0
                    #define DATE_DISP_LINE 9      // line on display for date / time
                    // text display area
                    #define DISP_X         0
                    #define DISP_Y         0
                    #define DISP_W         240
                    #define DISP_H         300
                    #define DISP_BCOL      0x000F // TFT_NAVY
                    #define DISP_ANZ_SP    20
                    #define DISP_ANZ_ZE    12
                    #define DISP_Hoe_ZE    25
                    #define DISP_TX_FCOL   0xB7E0 // TFT_GREENYELLOW
                    #define DISP_TX_BCOL   DISP_BCOL
                    // status line for messages
                    #define STATUS_XCENT   120 // Centred on this
                    #define STATUS_YCENT   315
                    #define STATUS_XLI     0
                    #define STATUS_XRE     239
                    #define STATUS_YOB     DISP_H // 300
                    #define STATUS_YUN     319
                    #define STATUS_FCOL    0xF81F // TFT_MAGENTA
                    #define STATUS_BCOL    0x0000 // TFT_BLACK
                  #endif
                //
                #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                    #define DATE_DISP_COL   0
                    #define DATE_DISP_LINE  0    // line on display for date / time
                    #define LCD_ROWS        2
                    #define LCD_CULS        2
                  #endif
              #endif
          #endif // DISP

      // --- acoustic output
        #if (USE_BUZZER_PWM > OFF)
            #if !(BUZZER1 ^ AOUT_PAS_BUZZ_3V5V)
                #define PLAY_MUSIC
                #define MUSIC_BASE_OCTA 5        // base oktave for musik
                //#define PLAY_START_MUSIC
                #define PLAY_START_DINGDONG
              #endif
          #endif // USE_BUZZER_PWM

    // --- user input
      // --- keypads
        #if defined(KEYS)
            #ifndef USE_MEASURE_CYCLE
                #define USE_MEASURE_CYCLE
              #endif
            #if !(KEYS ^ MC_UI_Keypad_ANA0_RO)
                #define USE_KEYPADSHIELD
                #define KEYS_ADC        34   // ADC Pin GPIO02
                #define ADC_STD_RES     12   // default resolution 12 Bit
                #define ADC_RES         12   // active resolution
                #define KP_NUM_KEYS     5
                #define KP_KEY_VAL_1    200  // max ADC value of button 0
                #define KP_KEY_VAL_2    750
                #define KP_KEY_VAL_3    1470
                #define KP_KEY_VAL_4    2330
                #define KP_KEY_VAL_5    3200
              #endif // keypad

            #if !(KEYS ^ MC_UI_TOUCHXPT2046_AZ)
                // Keypad start position, key sizes and spacing
                #define KEY_X          40 // Centre of key
                #define KEY_Y          287
                #define KEY_W          62 // Width and height
                #define KEY_H          26
                #define KEY_SPACING_X  18 // X and Y gap
                #define KEY_SPACING_Y  20
                #define KEY_TEXTSIZE   1   // Font size multiplier
                #define KEY_NUM_LEN    3 // Anzahl der Tasten
              #endif // touchpad

          #endif

      // --- counter
        #if (USE_CNT_INP > OFF)
            #define PCNT_H_LIM_VAL      0   // not used
            #define PCNT_L_LIM_VAL      0   // not used
            #define PNCT_AUTO_SWDN      5000000ul  // > 5 sec period
            #define PNCT_AUTO_SWUP      50000ul    // < 50 msec period
            // counter 1
            #define PCNT1_INP_FILT      10  // glitch filter (clock 80 MHz)
            #define PCNT1_UFLOW         3000000ul  // timedelay due to 0 Hz [us]
            #define PCNT1_INP_SIG_IO    PIN_CNT_FAN_1       // Pulse Input GPIO
            #define PCNT1_INP_CTRL_IO   PIN_CNT_FAN_1       // Control GPIO HIGH=count up, LOW=count down
            //#define PCNT1_THRESH1_VAL   5
            #define PCNT1_THRESH0_VAL   3
            #define PCNT1_EVT_0         PCNT_EVT_THRES_0
            //#define PCNT1_EVT_1         PCNT_EVT_THRES_1
            #define PCNT1_UNIT          0
            #define PCNT1_CHAN          0
            #define PCNT1_ID            0
            #if (USE_CNT_INP > 1)
                // counter 2
                #define PCNT2_INP_FILT      10  // glitch filter (clock 80 MHz)
                #define PCNT2_UFLOW         3000000ul  // timedelay due to 0 Hz [us]
                #define PCNT2_INP_SIG_IO    PIN_CNT_FAN_2   // Pulse Input GPIO
                #define PCNT2_INP_CTRL_IO   PIN_CNT_FAN_2       // Control GPIO HIGH=count up, LOW=count down
                //#define PCNT2_THRESH1_VAL   2
                #define PCNT2_THRESH0_VAL   400
                #define PCNT2_EVT_0         PCNT_EVT_THRES_0
                #define PCNT2_UNIT          1
                #define PCNT2_CHAN          0
                #define PCNT2_ID            1
              #endif
            #ifndef USE_MEASURE_CYCLE
                #define USE_MEASURE_CYCLE
              #endif
          #endif
      // --- dutycycle (pwm) input
        #if (USE_PWM_INP > OFF)

          #endif
        #if (USE_ADC1 > OFF)
            typedef struct
              {
                uint8_t  pin;
                uint8_t  channel;
                uint8_t  atten;
              } md_adc_conf_t;

            const md_adc_conf_t PIN_ADC_CONF[4] =
              {
                { PIN_INP_POTI_1, 7, 1 },
                { NO_PIN,         0, 0 },
                { NO_PIN,         0, 0 },
                { NO_PIN,         0, 0 }
              };
            #define INP_POTI_CTRL  0
            #ifndef USE_MEASURE_CYCLE
                #define USE_MEASURE_CYCLE
              #endif
          #endif

        #if (USE_DIG_INP > OFF)
            #ifndef USE_MEASURE_CYCLE
                #define USE_MEASURE_CYCLE
              #endif
            const uint8_t PIN_DIG_INP[8] =
              {
                PIN_INP_SW_1,   NO_PIN,
                NO_PIN,         NO_PIN,
                NO_PIN,         NO_PIN,
                NO_PIN,         NO_PIN
              };
            #define INP_SW_CTRL  0
          #endif

    // --- memories
      // --- FRAM
        #if (USE_FRAM_I2C > OFF)
            #define SIZE_FRAM     0x8000
          #endif
    // --- sensors
      #if (USE_DS18B20_1W_IO > OFF)
          #define DS_T_PRECISION   9
          #define DS18B20_ANZ      1
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
        #endif

      #if (USE_BME280_I2C > OFF)
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
        #endif

      #if (USE_MQ135_GAS_ADC > OFF)
          #define MQ135_FILT      15       // floating  measure filtering
          //#define MQ135_ThresFilt 25       // threshold measure filtering
          #define MQ135_EM_WIN    100      // window for traffic light
          //#define MQ135_EM_MID    2350    // green < (MID-(WIN/2) < yellow < (MID+(WIN/2) < red
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
        #endif

      #if (USE_TYPE_K_SPI > OFF)
          #define TYPEK_FILT      11       // floating  measure filtering
          #define TYPEK_DROP_PEEK 2        // drop biggest / lowest
          #define TYPEK1_OFFSET   0.       // offset unit °C
          #define TYPEK1_GAIN     1.       // result = (measure * gain) + offset
          #define TYPEK2_OFFSET   0.       // offset unit °C
          #define TYPEK2_GAIN     1.       // result = (measure * gain) + offset
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
        #endif

      #ifdef USE_MEASURE_CYCLE
          #define MEASURE_CYCLE_MS  500u
        #endif

      #ifdef USE_OUTPUT_CYCLE
          #define OUTPUT_CYCLE_MS  500u
        #endif

    // --- test features --------------------------------
      #define TEST_RGBLED_PWM       ON
      #define USE_WEBCTRL_RGB       OFF
      #define USE_WEBCTRL_FAN       OFF
      #define USE_POTICTRL_RGB      OFF
      #define USE_POTICTRL_FAN      OFF
      #define USE_SWCTRL_RGB        ON
      #define USE_SWCTRL_FAN        ON
      #define USE_SWCTRL_1812       ON
    // ******************************************
#endif // _PRJ_CONFIG_H_