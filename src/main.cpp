#include <main.h>
#include <prj_config.h>

// ---------------------------------------
// --- declarations
  // ------ system -----------------------
    //static uint64_t _tmp = 0ul;
    static bool firstrun = true;
    static uint16_t     md_error  = 0    // Error-Status bitkodiert -> 0: alles ok
                             #if (USE_WIFI > OFF)
                               + ERRBIT_WIFI
                               #if (USE_NTP_SERVER > OFF)
                                 + ERRBIT_NTPTIME
                               #endif
                             #endif
                             #if (USE_WEBSERVER > OFF)
                               + ERRBIT_SERVER
                             #endif
                             #if (USE_TOUCHSCREEN > OFF)
                               + ERRBIT_TOUCH
                             #endif
                             ;
    TwoWire i2c1 = TwoWire(0);
      // cycletime measurement
    static uint64_t anzUsCycles = 0ul;
    static uint64_t usLast      = 0ul;
    static uint64_t usPerCycle  = 0ul;
      //static uint64_t anzMsCycles = 0;
      //static uint64_t msLast = 0;
      //static uint64_t msPerCycle = 0;

    #if ( USE_I2C > 1 )
        TwoWire i2c2 = TwoWire(1);
      #endif

    #if ( USE_LED_BLINK_OUT > 0 )
        msTimer ledT = msTimer(BLINKTIME_MS);
        bool SYS_LED_ON = FALSE;
      #endif

    #if ( USE_DISP > 0 )
        msTimer       dispT  = msTimer(DISP_CYCLE);
        uint32_t      ze     = 1;      // aktuelle Schreibzeile
        char          outBuf[DISP1_MAXCOLS + 1] = "";
        String        outStr;
      #endif

        //
    #ifdef USE_STATUS
        msTimer     statT  = msTimer(STAT_TIMEDEF);
        char        statOut[DISP1_MAXCOLS + 1] = "";
        bool        statOn = false;
        bool        statDate = false;
        //char        timeOut[STAT_LINELEN + 1] = "";
      #endif

  // ------ user input ---------------
    #if (USE_CNT_INP > OFF)
      #endif

    #if (USE_TOUCHSCREEN > OFF)
        md_touch touch = md_touch();
      #endif

    #if (USE_KEYPADSHIELD > OFF)
        md_kpad kpad(KEYS_ADC);
        uint8_t key;
      #endif // USE_KEYPADSHIELD


    #if (USE_WS2812_MATRIX_OUT > OFF)
        md_ws2812_matrix matrix_1 = md_ws2812_matrix
          ( COLPIX_2812_M1, ROWPIX_2812_M1,
            COLTIL_2812_M1, ROWTIL_2812_M1, PIN_WS2812_M1,
            #if (ANZ_TILES_M1 > OFF)
                NEO_TILE_TOP       + NEO_TILE_LEFT +
                NEO_TILE_ROWS      + NEO_TILE_PROGRESSIVE +
              #endif
            NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
            NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
            (neoPixelType) COLORD_2812_M1 + NEO_KHZ800 );
        msTimer ws2812MT   = msTimer(UPD_2812_M1_MS);
        #if (USE_WS2812_MATRIX_OUT > 1)
            md_ws2812_matrix matrix_2 = md_ws2812_matrix
              ( COLPIX_2812_M2, ROWPIX_2812_M2,
                COLTIL_2812_M2, ROWTIL_2812_M2, PIN_WS2812_M2,
                #if (ANZ_TILES_M2 > OFF)
                    NEO_TILE_TOP       + NEO_TILE_LEFT +
                    NEO_TILE_ROWS      + NEO_TILE_PROGRESSIVE +
                  #endif
                NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                (neoPixelType) COLORD_2812_M1 + NEO_KHZ800 );
            msTimer ws2812MT   = msTimer(UPD_2812_M1_MS);
          #endif
        const char text2812[] = "Willkommen im Weltladen";
        const scroll2812_t outM2812 =
          {
            { MD_BITMAP_SMILY,  COL16_YELLOW_HIGH, (uint8_t) BRIGHT_2812_M1 },
            { (char*) text2812, COL16_RED_HIGH,    (uint8_t) BRIGHT_2812_M1 },
            { MD_BITMAP_SMILY,  COL16_YELLOW_HIGH, (uint8_t) BRIGHT_2812_M1 }
          };
        static int16_t posM2812 = sizeof(text2812) * COLCHAR_2812 + 2 * COLBMP_2812;
      #endif

    #if (USE_WS2812_LINE_OUT > OFF)
        msTimer         ws2812LT    = msTimer(UPD_2812_L1_MS);
        #ifdef USE_FAST_LED
            //extern CRGBPalette16 myRedWhiteBluePalette;
            //extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
            unsigned long ws2812L1_alt = 0;
            uint32_t      ws2812L1_cnt = 0;
            uint32_t      ws2812L1_v   = 0;
            uint16_t      idx2812L1    = 0;
            CRGBPalette16 curPalette1;
            TBlendType    curBlending1;
            CRGB leds1[LEDS_2812_L1];
            #if (USE_WS2812_LINE_OUT > 1)
                unsigned long ws2812L2_alt = 0;
                uint32_t      ws2812L2_cnt = 0;
                uint32_t      ws2812L2_v   = 0;
                uint16_t      idx2812L2    = 0;
                CRGBPalette16 curPalette2;
                TBlendType    curBlending2;
                CRGB leds2[LEDS_2812_L2];
                #if (USE_WS2812_LINE_OUT > 2)
                    uint16_t idx2812L3 = 0;
                    CRGB leds3[LEDS_2812_L3];
                    #if (USE_WS2812_LINE_OUT > 3)
                        uint16_t idx2812L4 = 0;
                        CRGB leds4[LEDS_2812_L4];
                      #endif
                  #endif
              #endif
        #else
            md_ws2812_matrix line_1 = md_ws2812_matrix
              ( COLPIX_2812_L1, ROWPIX_2812_L1,
                COLTIL_2812_L1, ROWTIL_2812_L1, PIN_WS2812_LD1,
                NEO_TILE_TOP       + NEO_TILE_LEFT +
                NEO_TILE_ROWS      + NEO_TILE_PROGRESSIVE +
                NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                (neoPixelType) COLORD_2812_L1 + NEO_KHZ800 );
            /*
              const char text2812[] = "Willkommen im Weltladen";
              const scroll2812_t outM2812 =
                {
                  { MD_BITMAP_SMILY,  COL16_YELLOW_HIGH, (uint8_t) BRIGHT_2812_M1 },
                  { (char*) text2812, COL16_RED_HIGH,    (uint8_t) BRIGHT_2812_M1 },
                  { MD_BITMAP_SMILY,  COL16_YELLOW_HIGH, (uint8_t) BRIGHT_2812_M1 }
                };
            */
            static int16_t posL2812 = -10000;
          #endif
      #endif

    #if (USE_CTRL_POTI_ADC > OFF)
        uint16_t inpValADC[USE_CTRL_POTI_ADC];
      #endif

    #if (USE_DIG_INP > OFF)
        uint8_t  inpValDig[USE_DIG_INP];
      #endif

    #if (USE_CNT_INP > OFF)
        const pcnt_config_t config_cnt[USE_CNT_INP] =
          {
            {
              PCNT1_INP_SIG_IO,      // Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
              PCNT_PIN_NOT_USED,     // Control signal input GPIO number, a negative value will be ignored
              PCNT_MODE_KEEP,        // PCNT low control mode
              PCNT_MODE_KEEP,        // PCNT high control mode
              PCNT_COUNT_DIS,        // PCNT positive edge count mode
              PCNT_COUNT_INC,        // PCNT negative edge count mode
              PCNT_H_LIM_VAL,        // Maximum counter value
              PCNT_L_LIM_VAL,        // Minimum counter value
              (pcnt_unit_t)    PCNT1_UNIT,  // PCNT unit number
              (pcnt_channel_t) PCNT1_CHAN  // the PCNT channel
            },
            #if (USE_CNT_INP > 1)
                {
                  PCNT2_INP_SIG_IO,   //< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
                  PCNT_PIN_NOT_USED,  //< Control signal input GPIO number, a negative value will be ignored
                  PCNT_MODE_KEEP,     //< PCNT low control mode
                  PCNT_MODE_KEEP,     // PCNT high control mode
                  PCNT_COUNT_DIS,     // PCNT positive edge count mode
                  PCNT_COUNT_INC,     // PCNT negative edge count mode
                  PCNT_H_LIM_VAL,     // Maximum counter value
                  PCNT_L_LIM_VAL,     // Minimum counter value
                  (pcnt_unit_t)    PCNT2_UNIT, // PCNT unit number
                  (pcnt_channel_t) PCNT2_CHAN  // the PCNT channel
                },
              #endif
          };
        //typedef intr_handle_t pcnt_isr_handle_t;


        /* A sample structure to pass events from the PCNT
         * interrupt handler to the main program.
         */
        typedef struct
          {
            int16_t  pulsCnt;   // the PCNT unit that originated an interrupt
            uint16_t tres;
            uint32_t freq;   // information on the event type that caused the interrupt
            uint64_t usCnt;
          } pcnt_evt_t;

        static xQueueHandle pcnt_evt_queue[USE_CNT_INP];   // A queue to handle pulse counter events
        static pcnt_evt_t cntErg[USE_CNT_INP];
        static pcnt_evt_t tmpErg;
          //static uint64_t oldClk[USE_CNT_INP] = {NULL};
        static uint64_t oldUs[USE_CNT_INP];
          //static uint16_t intCnt[USE_CNT_INP];
          //static uint16_t anzNoCnt[2] = {0, 0};
          //static uint64_t utmp64 = 0;
          //uint32_t cntLowPulse[8];
          //uint32_t cntHighPulse[8];

        /* Decode what PCNT's unit originated an interrupt
         * and pass this information together with the event type
         * the main program using a queue.
         */

        #ifndef USE_INT_EVTHDL
            #define USE_INT_EVTHDL
          #endif
        //pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
          //int16_t count = 0;
          //uint32_t pcnt_cnt[2] = {0, 0};
          //pcnt_evt_t evt;
          //pcnt_evt_t pcnt_evt;
          //portBASE_TYPE res;
        portBASE_TYPE pcnt_res;
        //static const char *PCNTTAG = "pcnt_int ";

        static void IRAM_ATTR pcnt1_intr_hdl(void *arg)
          {
            //SOUTLN(" _int_1_ ");
            BaseType_t port_status = pdFALSE;
            pcnt_evt_t event;
            port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT1_UNIT, &(event.pulsCnt));
            event.usCnt  = micros();
            //event.intCnt = intCnt[PCNT1_UNIT];
            xQueueSendFromISR(pcnt_evt_queue[PCNT1_UNIT], &event, &port_status);
            //xQueueOverwriteFromISR(pcnt_evt_queue[PCNT1_UNIT], &event, &port_status);
            port_status = pcnt_counter_clear((pcnt_unit_t) PCNT1_UNIT);
            //intCnt[0] = 0;
          }

        #if (USE_CNT_INP > 1)
            static void IRAM_ATTR pcnt2_intr_hdl(void *arg)
              {
                //SOUTLN(" _int_2_ ");
                BaseType_t port_status = pdFALSE;
                pcnt_evt_t event;
                port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT2_UNIT, &(event.pulsCnt));
                event.usCnt  = micros();
                //event.intCnt = intCnt[PCNT2_UNIT];
                xQueueSendFromISR(pcnt_evt_queue[PCNT2_UNIT], &event, &port_status);
                //xQueueOverwriteFromISR(pcnt_evt_queue[PCNT2_UNIT], &event, &port_status);
                port_status = pcnt_counter_clear((pcnt_unit_t) PCNT2_UNIT);
                //intCnt[0] = 0;
              }
          #endif
      #endif
    #if (USE_PWM_INP > OFF)
        typedef struct
          {
            uint32_t lowVal;
            uint32_t highVal;
          } pwm_val_t;
        pwm_val_t pwmInVal[USE_PWM_INP];
      #endif
  // ------ user output ---------------
    #if (USE_RGBCOL16_PWM > OFF)
        outRGBVal_t outValRGB[USE_RGBCOL16_PWM];
        #if (TEST_RGBCOL16_PWM > OFF)
            uint8_t  colRGBLED = 0;
            uint16_t incRGBLED = 10;
            uint32_t RGBCOL16_gr = 0;
            uint32_t RGBCOL16_bl = 0;
            uint32_t RGBCOL16_rt = 0;
          #endif
      #endif

    #if (USE_BUZZER_PWM > OFF)
        md_buzzer     buzz       = md_buzzer();
        #ifdef PLAY_MUSIC
            tone_t test = {0,0,0};
          #endif
      #endif // USE_BUZZER_PWM

    #if (USE_FAN_PWM > OFF)
        #if (USE_POTICTRL_FAN > OFF)
          #endif
        uint32_t valFanPWM[USE_FAN_PWM];
      #endif

    #if (USE_OLED_I2C > OFF)
        #ifdef OLED1
            #if !(OLED1_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled1 = md_oled_1106((uint8_t) I2C_ADDR_OLED1, (uint8_t) I2C_SDA_OLED1,
                                        (uint8_t) I2C_SCL_OLED1, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
            #else
                md_oled_1306 oled1 = md_oled_1306((uint8_t) I2C_ADDR_OLED1, (uint8_t) I2C_SDA_OLED1,
                                        (uint8_t) I2C_SCL_OLED1, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
              #endif
          #endif
        #ifdef OLED2
            #if !(OLED2_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled2 = md_oled_1106((uint8_t) I2C_ADDR_OLED2, (uint8_t) I2C_SDA_OLED2,
                                        (uint8_t) I2C_SCL_OLED2, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #else
                md_oled_1306 oled2 = md_oled_1306((uint8_t) I2C_ADDR_OLED2, (uint8_t) I2C_SDA_OLED2,
                                        (uint8_t) I2C_SCL_OLED2, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #endif
          #endif
        msTimer oledT   = msTimer(DISP_CYCLE);
        uint8_t oledIdx = 0;
      #endif //USE_OLED_I2C

    #if (defined(USE_TFT1602_GPIO_RO_3V3) || defined(USE_TFT1602_GPIO_RO_3V3))
        LiquidCrystal  lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
        void*          plcd = (void*) &lcd;
        md_lcd         mlcd(plcd);
      #endif
  // ------ network ----------------------
    #if (USE_WIFI > OFF)
        md_wifi wifi  = md_wifi();
        msTimer wifiT = msTimer(WIFI_CONN_CYCLE);
        #if (USE_LOCAL_IP > OFF)
          #endif // USE_LOCAL_IP
        #if (USE_NTP_SERVER > OFF)
            msTimer ntpT    = msTimer(NTPSERVER_CYCLE);
            time_t  ntpTime = 0;
            bool    ntpGet  = true;
          #endif // USE_WEBSERVER
      #endif

    #if (USE_WEBSERVER > OFF)
        md_server webMD = md_server();
        msTimer   servT = msTimer(WEBSERVER_CYCLE);
      #endif // USE_WEBSERVER

  // ------ sensors ----------------------
    #ifdef USE_MEASURE_CYCLE
        msTimer measT   = msTimer(MEASURE_CYCLE_MS);
      #endif
    #ifdef USE_OUTPUT_CYCLE
        msTimer outpT   = msTimer(OUTPUT_CYCLE_MS);
      #endif
    #if (USE_DS18B20_1W_IO > OFF)
        OneWire dsOneWire(DS_ONEWIRE_PIN);
        DallasTemperature dsSensors(&dsOneWire);
        DeviceAddress     dsAddr[DS18B20_ANZ];
        float dsTemp[DS18B20_ANZ];
      #endif

    #if (USE_BME280_I2C > OFF)
        Adafruit_BME280 bme;
        Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
        Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
        Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
      #endif

    #if (USE_MQ135_GAS_ADC > OFF)
        filterValue valGas(MQ135_FILT, 1);
        //filterValue tholdGas(MQ135_ThresFilt,1);
        int16_t gasValue;
        int16_t gasThres;
      #endif

    #if (USE_TYPE_K_SPI > 0)
        //SPIClass* tkSPI = new SPIClass();
        //md_31855_ktype TypeK1(TYPEK1_CS_PIN, tkSPI);
        md_31855_ktype TypeK1(TYPEK1_CS_PIN, TYPEK_CLK_PIN, TYPEK_DATA_PIN);
        filterValue valTK1(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        filterValue valTK1ref(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        int16_t     tk1Val;
        int16_t     tk1ValRef;
        #if ( USE_TYPE_K_SPI > 1)
            //md_31855_ktype TypeK2(TYPEK2_CS_PIN, tkSPI);
            md_31855_ktype TypeK2(TYPEK2_CS_PIN, TYPEK_CLK_PIN, TYPEK_DATA_PIN);
            filterValue valTK2(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK2_OFFSET, TYPEK2_GAIN);
            filterValue valTK2ref(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK2_OFFSET, TYPEK2_GAIN);
            int16_t     tk2Val;
            int16_t     tk2ValRef;
          #endif
      #endif

  // ------ memories
    #if (USE_FRAM_I2C > OFF)
        md_FRAM fram = md_FRAM();
      #endif
// --- system setup -----------------------------------
  void setup()
    {
      // --- system
        // start system
          Serial.begin(SER_BAUDRATE);
          usleep(30000); // power-up safety delay
          SOUTLN(); SOUTLN("setup start ...");
          #ifdef SCAN_I2C
              scanI2C(I2C1, 0, SCAN_I2C, PIN_I2C1_SDA, PIN_I2C1_SCL);
              #if (USE_I2C > 1)
                  scanI2C(I2C2, 0, SCAN_I2C, PIN_I2C2_SDA, PIN_I2C2_SCL);
                #endif
            #endif

          #if (USE_LED_BLINK_OUT > 0)
              pinMode(PIN_BOARD_LED, OUTPUT);
              digitalWrite(PIN_BOARD_LED, SYS_LED_ON);
            #endif

      // --- user output
        // start display - output to user
          #if (USE_TRAFFIC_COL16_OUT > 0)
              pinMode(PIN_TL_GREEN, OUTPUT);
              pinMode(PIN_TL_YELLOW, OUTPUT);
              pinMode(PIN_TL_RED, OUTPUT);
              digitalWrite(PIN_TL_GREEN, ON);
              digitalWrite(PIN_TL_RED, ON);
              digitalWrite(PIN_TL_YELLOW, ON);
              usleep(500000);
              digitalWrite(PIN_TL_GREEN, OFF);
              digitalWrite(PIN_TL_RED, OFF);
              digitalWrite(PIN_TL_YELLOW, OFF);
            #endif
          #if (USE_RGBCOL16_PWM > 0)
              // RGB red
                pinMode(PIN_RGB_RED, OUTPUT);
                ledcSetup(PWM_RGB_RED,    PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_RED,   PWM_RGB_RED);
                ledcWrite(PWM_RGB_RED, 255);
                SOUTLN("LED rot");
                usleep(300000);
                ledcWrite(PWM_RGB_RED, 0);

              // RGB green
                pinMode(PIN_RGB_GREEN, OUTPUT);
                ledcSetup(PWM_RGB_GREEN,  PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_GREEN, PWM_RGB_GREEN);
                ledcWrite(PWM_RGB_GREEN, 255);
                SOUTLN("LED gruen");
                usleep(500000);
                ledcWrite(PWM_RGB_GREEN, 0);

              // RGB blue
                pinMode(PIN_RGB_BLUE, OUTPUT);
                ledcSetup(PWM_RGB_BLUE,   PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_BLUE,  PWM_RGB_BLUE);
                ledcWrite(PWM_RGB_BLUE, 255);
                SOUTLN("LED blau");
                usleep(500000);
                ledcWrite(PWM_RGB_BLUE, 0);
            #endif
          startDisp();
          dispStatus("setup start ...");

        // WS2812 LEDs
          #if (USE_WS2812_MATRIX_OUT > OFF)
              SOUT("start WS2812 matrix ...");
              matrix_1.begin();
              //usleep(5000);
              //matrix_1.start_scroll_task((scroll2812_t*) &outM2812, &posM2812);
              matrix_1.start_scroll_matrix((scroll2812_t*) &outM2812, &posM2812);
              SOUTLN(" ok");
            #endif

          #if (USE_WS2812_LINE_OUT > OFF)
              #ifdef USE_FAST_LED
                  FastLED.addLeds<TYPE_2812_L1, PIN_WS2812_LD1, COLORD_2812_L1>(leds1, LEDS_2812_L1).setCorrection(TypicalLEDStrip);
                  FastLED.setBrightness(BRIGHT_2812_L1);
                  curPalette1 = RainbowStripeColors_p;
                  curBlending1 = NOBLEND;
                  #if (USE_WS2812_LINE_OUT > 1)
                        FastLED.addLeds<TYPE_2812_L2, PIN_WS2812_LD2, COLORD_2812_L2>(leds2, LEDS_2812_L2).setCorrection(TypicalLEDStrip);
                        FastLED.setBrightness(BRIGHT_2812_L2);
                        curPalette2  = RainbowStripeColors_p;
                        curBlending2 = NOBLEND;
                      #if (USE_WS2812_LINE_OUT > 2)
                            FastLED.addLeds<TYPE_2812_L3, PIN_WS2812_LD3, COLORD_2812_L3>(leds3, LEDS_2812_L3).setCorrection(TypicalLEDStrip);
                            FastLED.setBrightness(BRIGHT_2812_L3);
                            curPalette1 = RainbowColors_p;
                            curBlending = LINEARBLEND;
                          #if (USE_WS2812_LINE_OUT > 3)
                              FastLED.addLeds<TYPE_2812_L4, PIN_WS2812_LD4, COLORD_2812_L4>(leds3, LEDS_2812_L4).setCorrection(TypicalLEDStrip);
                              FastLED.setBrightness(BRIGHT_2812_L4);
                              curPalette1 = RainbowColors_p;
                              curBlending = LINEARBLEND;
                            #endif
                        #endif
                    #endif
              #else
                  SOUT("start WS2812 line ...");
                  line_1.begin();
                  //usleep(5000);
                  line_1.scroll_colorLine(true);
                  SOUTLN(" ok");
                #endif
            #endif

        // start buzzer (task)
          #if (USE_BUZZER_PWM > OFF)
              pinMode(PIN_BUZZ, OUTPUT);                                                                               // Setting pin 11 as output
              #ifdef PLAY_MUSIC
                buzz.initMusic(PIN_BUZZ, PWM_BUZZ);
                #if defined(PLAY_START_MUSIC)
                    playSong();
                  #endif
                #if defined(PLAY_START_DINGDONG)
                    buzz.playDingDong();
                  #endif
              #endif
            #endif
        // start key device
          #if defined(KEYS)
              startKeys();
            #endif
        // start fans
          #if (USE_FAN_PWM > OFF)
              // Fan 1
                pinMode(PIN_PWM_FAN_1, OUTPUT);
                ledcSetup(PWM_FAN_1, PWM_FAN_FREQ, PWM_FAN_RES);
                ledcAttachPin(PIN_PWM_FAN_1, PWM_FAN_1);
                ledcWrite(PWM_FAN_1, 255);
                SOUTLN("Test Fan 1");
                sleep(1);
                ledcWrite(PWM_FAN_1, 0);

              // Fan 2
                pinMode(PIN_PWM_FAN_2, OUTPUT);
                ledcSetup(PWM_FAN_2, PWM_FAN_FREQ, PWM_FAN_RES);
                ledcAttachPin(PIN_PWM_FAN_2, PWM_FAN_2);
                ledcWrite(PWM_FAN_2, 255);
                SOUTLN("Test Fan 2");
                sleep(1);
                ledcWrite(PWM_FAN_2, 0);

            #endif

        // start freq generator
          #if (USE_BUZZER_PWM > OFF)

            #endif
      // --- user input
        // start digital inputs
          #if (USE_DIG_INP > OFF)
              SOUT("config digSW Pins " );
              for (uint8_t i = 0 ; i < USE_DIG_INP ; i++ )
                {
                  pinMode(PIN_DIG_INP[i], INPUT_PULLUP);
                  SOUT(PIN_DIG_INP[i]); SOUT(" ");
                }
              SOUTLN();
            #endif
        // start poti inputs
          #if (USE_ADC1 > OFF)
              SOUT("config ADCs Pins " );
              for (uint8_t i = 0 ; i < USE_CTRL_POTI_ADC ; i++ )
                {
                  pinMode(PIN_ADC_CONF[i].pin, INPUT);
                  adc1_config_channel_atten((adc1_channel_t) PIN_ADC_CONF[i].channel,
                                            (adc_atten_t)   PIN_ADC_CONF[i].atten);
                  SOUT(PIN_ADC_CONF[i].pin); SOUT(" (");
                  SOUT(PIN_ADC_CONF[i].channel); SOUT(", ");
                  SOUT(PIN_ADC_CONF[i].atten) ; SOUT(") ");
                }
              SOUTLN();
            #endif
        // start dutycycle (pwm) inputs
          #if (USE_PWM_INP > OFF)
              //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_OUT);
              mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, PIN_PWM_INP_1); // MAGIC LINE to define WHICH GPIO
              // gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal
              mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 1);
            #endif
      // --- network
        // start WIFI
          #if (USE_WIFI > OFF)
            startWIFI(true);
            if ((md_error & ERRBIT_WIFI) == 0)
                dispStatus("WIFI connected");
              else
                dispStatus("WIFI error");
              if ((md_error & ERRBIT_WIFI) == 0)
                {
                  dispStatus("WIFI connected");
                }
                else
                {
                  dispStatus("WIFI error");
                }
              #endif // USE_WIFI
        // start Webserer
          #if (USE_WEBSERVER > OFF)
              {
                servT.startT();
                startWebServer();
                //md_error = setBit(md_error, ERRBIT_SERVER, webMD.md_handleClient());

              }
            #endif

      // --- sensors
        // temp. sensor DS18D20
          #if (USE_DS18B20_1W_IO > OFF)
                  SOUT(millis()); SOUT(" DS18D20 ... " );
              dsSensors.begin();
              String DS18Str = getDS18D20Str();
              dispStatus(DS18Str);
                  SOUTLN(DS18Str);
            #endif
        // BME280 temperature, pessure, humidity
          #if ( USE_BME280_I2C > OFF )
                    SOUT(millis()); SOUT(" BME280 ... " );
                bool bmeda = false;
                #if defined( I2C_BME2801_USE_I2C1 )
                    bmeda = bme.begin(I2C_ADDR_BME2801, &i2c1);
                  #endif
                #if defined( I2C_BME2801_USE_I2C2 )
                    bmeda = bme280.begin(I2C_ADDR_BME2801, &i2c2);
                  #endif
                if (bmeda)
                    {
                            SOUTLN(" gefunden");
                      bme.setSampling(bme.MODE_SLEEP);
                      String stmp = getBME280Str();
                            SOUTLN(stmp);
                      //bme_temp->printSensorDetails();
                      //bme_pressure->printSensorDetails();
                      //bme_humidity->printSensorDetails();

                    }
                  else
                    {
                      SOUT(" nicht gefunden");
                    }
            #endif

        // K-type thermoelementation
          #if ( USE_TYPE_K_SPI > 0)
                    SOUT(millis()); SOUT(" Tcouple1 ... " );
                uint8_t tkerr = TypeK1.begin();
                if (!tkerr)
                    {
                            SOUT(" gefunden TK1 ");
                      int16_t itmp = TypeK1.actT();
                            SOUT(itmp); SOUT(" TK1cold ");
                      itmp = TypeK1.refT();
                            SOUTLN(itmp);
                    }
                  else
                    {
                      SOUTLN(" nicht gefunden");
                    }
                #if ( USE_TYPE_K_SPI > 1)
                          SOUT(millis()); SOUT(" Tcouple2 ... " );
                      int16_t itmp = 0;
                      tkerr = TypeK2.begin();
                      if (!tkerr)
                          {
                                  SOUT(" gefunden TK2 ");
                            itmp = TypeK2.actT();
                                  SOUT(itmp); SOUT(" TK2cold ");
                            itmp = TypeK2.refT();
                                  SOUTLN(itmp);
                          }
                        else
                          {
                            SOUTLN(" nicht gefunden");
                          }
                  #endif
            #endif

      // --- memories
        // FRAM
          #if (USE_FRAM_I2C > OFF) // NIO funktioniert nicht
            // Read the first byte
            SOUT("FRAM addr "); SOUTHEX(I2C_ADDR_FRAM1);
            bool ret = !fram.begin(I2C_SDA_FRAM1, I2C_SCL_FRAM1, I2C_ADDR_FRAM1);
            if (ret == ISOK)
              {
                SOUT(" ok ProdID= ");
                uint16_t prodID, manuID;
                fram.getDeviceID(&manuID, &prodID);
                SOUT(" product "); SOUT(prodID); SOUT(" producer "); SOUTLN(manuID);

                SOUTLN(" FRAM selftest "); SOUTLN(fram.selftest());
              }
            #endif

      // --- services using interrupt
        // start counter
          #if (USE_CNT_INP > OFF)
              SOUT("config counter Pins " );
              for (uint8_t i = 0 ; i < USE_CNT_INP ; i++ )
                {
                  switch (i)
                    {
                      case 0: pinMode(PIN_CNT_FAN_1, INPUT_PULLUP); SOUT(PIN_CNT_FAN_1); SOUT(" "); break;
                      #if (USE_CNT_INP > 1)
                          case 1: pinMode(PIN_CNT_FAN_2, INPUT_PULLUP); SOUT(PIN_CNT_FAN_2); SOUT(" "); break;
                        #endif
                      default: break;
                    }
                }
              SOUTLN();
              initFanPCNT();
                      //attachInterrupt(digitalPinToInterrupt(PIN_PWM_FAN_1), &pcnt_intr_handler, FALLING);

              #ifdef USE_MCPWM
                  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, PIN_CNT_FAN_1); // MAGIC LINE to define WHICH GPIO
                  // gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal
                  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
                #endif
            #endif
      // --- finish setup
          #if (DEBUG_MODE >= CFG_DEBUG_STARTUP)
              #if (USE_COL16_BLINK_OUT > 0)
                  digitalWrite(PIN_BOARD_LED, OFF);
                  SYS_LED_ON = OFF;
                #endif
              SOUTLN();
              SOUT("... end setup -- error="); SOUTLN(md_error);
              SOUTLN();
            #endif
    }

// ---------------------------------------
  #if (USE_WS2812_MATRIX_OUT > OFF)

    #endif
// ---------------------------------------
// --- system run = endless loop
  void loop()
    {
      if (firstrun == true)
        {
          String taskMessage = "loop task running on core ";
          taskMessage = taskMessage + xPortGetCoreID();
          SOUTLN(taskMessage);
          usLast = micros();
          firstrun = false;
        }
      anzUsCycles++;

      //uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
      #if (USE_WIFI > OFF)  // restart WIFI if offline
          if(wifiT.TOut())
            {
              //Serial.print("WiFi md_error = "); Serial.println(md_error);
              wifiT.startT();
              if((md_error & ERRBIT_WIFI) > 0)
                {
                  SOUTLN("WiFi startWIFI");
                  dispStatus("WiFi startWIFI");
                  startWIFI(false);
                }
            }
        #endif // USE_WIFI

      // ----------------------
      #if (USE_NTP_SERVER > OFF)   // get time from NTP server
        if (ntpT.TOut() == true)
          {
            setTime(++ntpTime);
            if ((md_error & ERRBIT_WIFI) == 0)
              { // WiFi online
                if (((md_error & ERRBIT_NTPTIME) > 0) || (year() < 2000))   // time not initialized
                  {
                    initNTPTime();
                    ntpGet = true;
                  }
                if (ntpGet == true)
                  {
                    ntpGet = wifi.getNTPTime(&ntpTime);
                    setTime(ntpTime);
                  }
              }
            ntpT.startT();
                  #if (DEBUG_MODE == CFG_DEBUG_DETAILS)
                      //SOUT("Datum "); SOUT(day()); SOUT("."); SOUT(month()); SOUT("."); SOUT(year()); SOUT(" ");
                      //SOUT("Zeit "); SOUT(hour()); SOUT("."); SOUT(minute()); SOUT(":"); SOUTLN(second());
                    #endif
          }
        #endif // USE_NTP_SERVER
      // ----------------------
      #if (USE_WEBSERVER > OFF)    // run webserver -> restart/run not allowed in loop task
          if (servT.TOut())
            {
              servT.startT();
              /*
              if ((md_error & ERRBIT_SERVER) != 0)
                {;}//startWebServer();
              else
                //bool ret = webMD.md_handleClient();
                md_error = setBit(md_error, ERRBIT_SERVER, webMD.md_handleClient());
              */
            }

        #endif
      // ----------------------
      #if (USE_TOUCHSCREEN > OFF)
        touch.runTouch(outBuf);
        #endif // USE_TOUCHSCREEN

      // ----------------------
      #if (USE_KEYPADSHIELD > OFF)
        key = getKey();
        if (key)
          {
            sprintf(outBuf,"key %d", key);
            dispStatus(outBuf);
          }
        #endif
      // ----------------------
      #if (USE_CNT_INP > OFF)
          for ( uint8_t i = 0; i < USE_CNT_INP ; i++ )
            {
              uint64_t    tmp = micros();
              uint64_t    lim = 0ul;
              pcnt_unit_t unit;
                          //SOUT(tmp);
              switch (i)
                {
                  #if (USE_CNT_INP > 1)
                      case 1:
                        lim = PCNT2_UFLOW;
                        unit = (pcnt_unit_t) PCNT2_UNIT;
                        pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT2_UNIT], &tmpErg, 0);
                          //SOUT(" res1 "); SOUT(pcnt_res);
                        break;
                    #endif
                  #if (USE_CNT_INP > 2)
                      case 2:
                        lim = PCNT3_UFLOW;
                        pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT3_UNIT], &tmpErg, 0);
                        break;
                    #endif
                  #if (USE_CNT_INP > 3)
                      case 3:
                        lim = PCNT4_UFLOW;
                        pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT4_UNIT], &tmpErg, 0);
                        break;
                    #endif
                  default:   // case 0
                    lim = PCNT1_UFLOW;
                    pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT1_UNIT], &tmpErg, 0);
                          //SOUT(" res0 "); SOUT(pcnt_res);
                    break;
                }
              if (pcnt_res == pdTRUE)
                {
                  cntErg[i].usCnt   = (tmpErg.usCnt - oldUs[i]);
                  oldUs[i]          = tmpErg.usCnt;
                  cntErg[i].pulsCnt = tmpErg.pulsCnt;
                  //uint16_t pulsCnt  = tmpErg.pulsCnt;
                  // check for auto range switching
                    /*
                    if (cntErg[i].usCnt > PNCT_AUTO_SWDN)
                      {
                        pulsCnt = (tmpErg.pulsCnt * 2) / 3;
                        if (pulsCnt <= 0) pulsCnt = 1;
                        SOUTLN(); SOUT(i); SOUT(" dn "); SOUT(cntErg[i].usCnt); SOUT(" "); SOUTLN(pulsCnt);
                      }
                    if ( (cntErg[i].usCnt < PNCT_AUTO_SWUP) && (pulsCnt > 0) )
                      {
                        pulsCnt = (tmpErg.pulsCnt * 3) / 2;
                        SOUTLN(); SOUT(i); SOUT(" up "); SOUT(cntErg[i].usCnt); SOUT(" "); SOUTLN(pulsCnt);
                      }

                    pcnt_event_disable(unit, PCNT_EVT_THRES_0);
                    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, pulsCnt);
                    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
                    */

                  if ( cntErg[i].usCnt > 55 )
                    { // low freq
                          //cntErg[i].usCnt /= tmpErg.pulsCnt;
                      if ( (cntErg[i].usCnt > 0) && (cntErg[i].usCnt > tmpErg.pulsCnt) )
                        {
                          cntErg[i].freq = (uint32_t) (1000000ul / (cntErg[i].usCnt / tmpErg.pulsCnt) );
                        }
                          //SOUT(" l "); SOUT(cntErg[i].freq);
                    }
                    else
                    { // high freq
                      cntErg[i].freq = (uint32_t) (tmpErg.pulsCnt * 1000000ul / cntErg[i].usCnt);
                          //SOUT(" h "); SOUT(cntErg[i].freq);
                    }
                }
                else
                {
                  if ( tmp - oldUs[i] > lim )
                    {
                      oldUs[i] = tmp;
                      cntErg[i].freq = 0;
                    }
                }
                          //SOUT(" erg "); SOUT(i); SOUT(" "); SOUTLN(cntErg[i].freq);
            }
        #endif
      // ----------------------
      #if (USE_PWM_INP > OFF)
          mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 1);
          pwmInVal->lowVal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);

          mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 1);
          pwmInVal->highVal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
        #endif
      // ----------------------
      #ifdef USE_MEASURE_CYCLE
          if (measT.TOut())
            {
              measT.startT();
              #if (USE_MQ135_GAS_ADC > OFF)
                  gasValue = analogRead(PIN_MQ135);
                        //SOUT(millis()); SOUT(" gas measurment val = "); SOUTLN(gasValue);
                  gasValue = (int16_t) valGas.value((double) gasValue);
                        //SOUT(millis()); SOUT("    gasValue = "); SOUTLN(gasValue);
                  //gasThres = analogRead(PIN_CO2_THOLD);
                        //SOUT(millis()); SOUT(" gas threshold val = "); SOUTLN(gasThres);
                  //gasThres = (int16_t) tholdGas.value((double) gasThres);
                        //SOUT(millis()); SOUT("    gasThres = "); SOUTLN(gasThres);
                #endif
              #if (USE_CNT_INP > OFF)
                  #ifdef USE_PW
                      getCNTIn();
                    #endif
                #endif
              #if (USE_TYPE_K_SPI > OFF)
                  int8_t  tkerr = (int8_t) ISOK;
                  int16_t ival = TypeK1.actT();
                  tkerr = TypeK1.readErr();
                        //SOUT(" typeK1 err "); SOUT(tkerr);
                        //SOUT(" val "); SOUT( ival);
                  if (!tkerr)
                    {
                      tk1Val    = valTK1.value((double) ival);
                        //SOUT(" / "); SOUT(tk1Val);
                      ival      = TypeK1.refT();
                            //SOUT(millis()); SOUT(" ref raw = "); SOUT((int) ival);
                      tk1ValRef = valTK1ref.value((double) ival);
                        //SOUT(millis()); SOUT(" ival = "); SOUT((int) tk1ValRef);
                    }
                        //SOUTLN();
                  #if (USE_TYPE_K_SPI > 1)
                      ival    = TypeK2.actT();
                      tkerr     = TypeK2.readErr() % 8;
                            //SOUT(" typeK2 err "); SOUT(tkerr);
                            //SOUT(" val "); SOUT(ival);
                      if (!tkerr)
                        {
                          tk2Val    = valTK2.value((double) ival);
                            //SOUT(" / "); SOUT((int) tk2Val);
                          ival      = TypeK2.refT();
                            //SOUT(millis()); SOUT(" ref raw = "); SOUT(ival);
                          tk2ValRef = valTK2ref.value((double) ival);
                            //SOUT(millis()); SOUT(" ival = "); SOUT(tk2ValRef);
                        }
                            //SOUTLN();
                  #else
                            SOUTLN();
                    #endif
                #endif
              #if (USE_ADC1 > OFF)
                  getADCIn();
                #endif

              #if (USE_DIG_INP > OFF)
                  getDIGIn();
                #endif

              #if (USE_MCPWM > OFF)
                  getCNTIn();
                #endif
            }
        #endif
      // ----------------------
      #ifdef USE_OUTPUT_CYCLE
          #if (USE_WS2812_MATRIX_OUT > OFF)
              //if (ws2812T.TOut())
              //  {
              //    ws2812T.startT();
                  //SOUTLN(); SOUT(micros());
                  matrix_1.scroll_matrix();
                  //SOUT(" "); SOUTLN(micros());
              //  }
            #endif

          #if (USE_WS2812_LINE_OUT > OFF)
              #ifdef USE_FAST_LED
                //SOUT(" FASTLED ... ");
                //ChangePalettePeriodically(0);
                idx2812L1 = idx2812L1 + 1; /* motion speed */
                ws2812L1_cnt++;
                FillLEDsFromPaletteColors(0, idx2812L1);
                      //SOUT(micros()); SOUT(" "); SOUT(ws2812L1_cnt);
                      //SOUT(" show ... ");
                #if (USE_WS2812_LINE_OUT > 1)
                      //idx2812L2 = idx2812L2 + 1; /* motion speed */
                      //ws2812L2_cnt++;
                      //FillLEDsFromPaletteColors(1, idx2812L2);
                    #if (USE_WS2812_LINE_OUT > 2)
                        #if (USE_WS2812_LINE_OUT > 3)
                          #endif
                      #endif
                  #endif
                FastLED.show();
                      //SOUTLN(" ready ");
              #else
                  if (ws2812LT.TOut())
                    {
                      ws2812LT.startT();
                      line_1.scroll_colorLine();
                    }
                #endif
            #endif

          if (outpT.TOut())
            {
              outpT.startT();
              #if (USE_RGBCOL16_PWM > OFF)
                  #if (TEST_RGBCOL16_PWM > OFF)
                      switch (colRGBLED)
                        {
                          case 0:
                            if (RGBCOL16_rt >= 254)
                              {
                                RGBCOL16_rt = 0;
                                RGBCOL16_gr += incRGBLED;
                                colRGBLED++;
                              }
                              else
                              { RGBCOL16_rt += incRGBLED; }
                            break;
                          case 1:
                            if (RGBCOL16_gr >= 254)
                              {
                                RGBCOL16_gr = 0;
                                RGBCOL16_bl += incRGBLED;
                                colRGBLED++;
                              }
                              else
                              { RGBCOL16_gr += incRGBLED; }
                            break;
                          case 2:
                            if (RGBCOL16_bl >= 254)
                              {
                                RGBCOL16_bl = 0;
                                RGBCOL16_rt += incRGBLED;
                                colRGBLED = 0;
                              }
                              else
                              { RGBCOL16_bl += incRGBLED; }
                            break;
                          default:
                            break;
                        }

                      #if (USE_WEBCTRL_RGB > OFF)
                          _tmp += 4;
                          if (_tmp > 50)
                            { _tmp = 0; }
                          //SOUT(millis()); SOUT(" _tmp = "); SOUTLN(_tmp);
                          ledcWrite(PWM_RGB_RED,   webMD.getDutyCycle(0));
                          ledcWrite(PWM_RGB_GREEN, webMD.getDutyCycle(1));
                          ledcWrite(PWM_RGB_BLUE,  webMD.getDutyCycle(2));
                        #endif

                      ledcWrite(PWM_RGB_RED,   RGBCOL16_rt);
                      ledcWrite(PWM_RGB_GREEN, RGBCOL16_gr);
                      ledcWrite(PWM_RGB_BLUE,  RGBCOL16_bl);
                    #endif

                #endif

              #if (USE_FAN_PWM > OFF)
                  valFanPWM[0] += 1;
                  valFanPWM[1] += 1;
                  if (valFanPWM[1] >= 127) // -50%
                    {
                      valFanPWM[0] = 0;
                      valFanPWM[1] = 0;
                    }

                  #if (USE_POTICTRL_FAN > 0)
                      valFan[INP_CNT_FAN_1] = map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 255);
                      valFan[INP_CNT_FAN_2] = map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 255);
                      //SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" "); SOUTLN(valFan[INP_CNT_FAN_1]);
                      valFanPWM[0] = valFan[INP_CNT_FAN_1];
                      valFanPWM[1] = valFan[INP_CNT_FAN_2];
                    #endif

                  ledcWrite(PWM_FAN_1, valFanPWM[0]);
                  ledcWrite(PWM_FAN_2, valFanPWM[1]);
                #endif

            }
        #endif
      // --- Display -------------------
      #if (USE_DISP > 0)
        if (dispT.TOut())    // handle touch output
          {
            #ifdef RUN_OLED_TEST
                oled.clearBuffer();
                switch (oledIdx)
                  {
                    case 0:
                      oled.prepare();
                      oled.box_frame();
                      break;
                    case 1:
                      oled.disc_circle();
                      oled.sendBuffer();
                      break;
                    case 2:
                      oled.r_frame_box();
                      break;
                    case 3:
                      oled.prepare();
                      oled.string_orientation();
                      oledIdx--;
                      break;
                    case 4:
                      oled.line();
                      break;
                    case 5:
                      oled.triangle();
                      break;
                    case 6:
                      oled.bitmap();
                      break;
                    default:
                      break;
                  }
                if (++oledIdx > 6) { oledIdx = 0; }
                oled.sendBuffer();
              #endif // RUN_OLED_TEST
            oledIdx++;
            switch (oledIdx)
              {
              case 1: // system output
                  usPerCycle = (millis() - usLast) / anzUsCycles;
                    //SOUT(usLast); SOUT(" "); SOUT(micros()); SOUT(" "); SOUTLN(usPerCycle);
                  usLast      = millis();
                    //SOUTLN(); SOUT(usLast); SOUT(" ms/cyc "); SOUT((uint32_t) usPerCycle); SOUT(" ");
                  anzUsCycles = 0ul;
                break;

              case 2: // webserver nu
                #if (USE_WEBSERVER > OFF)
                  #endif
                break;

              case 3: // k-type sensor
                #if (USE_TYPE_K_SPI > OFF)
                  outStr = "";
                  outStr = "TK1 ";
                  outStr.concat(tk1Val);
                  outStr.concat("°");
                  //dispText(outStr ,  0, 1, outStr.length());
                  #if (USE_TYPE_K_SPI > 1)
                      //outStr = "";
                      outStr.concat(" TK2 ");
                      outStr.concat(tk2Val);
                      outStr.concat("° ");
                    #endif
                  dispText(outStr ,  0, 2, outStr.length());
                  outStr.concat(" (");
                  outStr.concat(tk1ValRef);
                  #if (USE_TYPE_K > 1)
                      outStr.concat("° / ");
                      outStr.concat(tk2ValRef);
                    #endif
                  outStr.concat("°)");
                          SOUTLN(outStr); SOUT(" ");
                  #endif
                break;

              case 4: // gas sensor
                #if (USE_MQ135_GAS_ADC > OFF)
                  outStr = "";
                  //_tmp = showTrafficLight(gasValue, gasThres); // -> rel to defined break point
                  outStr = "CO2 ";
                  outStr.concat(gasValue);
                  //outStr.concat(" (");
                  //outStr.concat(_tmp);
                  //outStr.concat(")");
                  dispText(outStr ,  0, 1, outStr.length());
                          //SOUT(outStr); SOUT("  ");
                  #endif
                break;

              case 5: // temp sesor
                #if (USE_DS18B20_1W_IO > OFF)
                  outStr = "";
                  outStr = getDS18D20Str();
                  dispText(outStr ,  0, 4, outStr.length());
                  #endif
                break;

              case 6: // BME 280 temp, humidity, pressure
                #if ( USE_BME280_I2C > OFF )
                  outStr = getBME280Str();
                          //SOUT(outStr); SOUT(" ");
                  dispText(outStr , 0, 3, outStr.length());
                  #endif
                break;

              case 7: // test values
                #if (USE_DIG_INP > OFF)
                    //SOUT("SW1 "); SOUT(inpValDig[INP_SW_CTRL]); SOUT(" ");
                  #endif
                #if (USE_CTRL_POTI_ADC > OFF)
                    //SOUT("POT "); SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" ");
                  #endif
                break;

              case 8: // WS2812 lines
                #if (USE_WS2812_LINE_OUT > OFF)
                    outStr = "              ";
                    dispText(outStr ,  0, 0, outStr.length());
                    //outStr = "LED ";
                    outStr = "";
                        //outStr += (String) ws2812_cnt; outStr += " ";
                    ws2812L1_v = millis() - ws2812L1_alt; // dispT.getTout();
                    ws2812L1_alt = millis();
                    if (ws2812L1_cnt > 0)
                      {
                        ws2812L1_v = ws2812L1_v / ws2812L1_cnt;
                        ws2812L1_cnt = 0;
                      }
                    outStr += (String) ws2812L1_v;
                    outStr += ("ms");
                          //SOUT((uint32_t) millis()); SOUT(" ");
                              //SOUT(outStr); SOUT(" ");
                    dispText(outStr ,  0, 0, outStr.length());
                  #endif
                break;
              case 9: // counter values
                #if (USE_CNT_INP > OFF)
                      outStr = "              ";
                      dispText(outStr ,  0, 2, outStr.length());
                      outStr = "f";
                      for (uint8_t i = 0; i < USE_CNT_INP ; i++ )
                        {
                          outStr += " ";
                          outStr += (String) cntErg[i].freq;
                                //SOUT("/"); SOUT(cntErg[i].pulsCnt); SOUT(" "); SOUT("/"); SOUT(cntErg[i].usCnt); SOUT(" ");
                        }
                      //SOUT(outStr); SOUT(" "); //SOUT(valFanPWM[0]); SOUT(" ");
                      dispText(outStr ,  0, 2, outStr.length());
                    #ifdef USE_MCPWM
                        outStr = "              ";
                        dispText(outStr ,  0, 0, outStr.length());
                        outStr = "LH ";
                        outStr += (String) cntLowPulse[0]; outStr += (" ");
                        outStr += (String) cntHighPulse[0];
                                  SOUT(outStr); SOUT(" ");
                        dispText(outStr ,  0, 2, outStr.length());
                      #endif
                  #endif
                break;
              case 10: // pwm counter values
                #if (USE_PWM_INP > OFF)
                      outStr = "              ";
                      dispText(outStr ,  0, 1, outStr.length());
                      outStr = "pwm ";
                      for (uint8_t i = 0; i < USE_PWM_INP ; i++ )
                        {
                          outStr += " ";
                          outStr += (String) pwmInVal[i].lowVal;
                          outStr += " ";
                          outStr += (String) pwmInVal[i].highVal;
                                //SOUT("/"); SOUT(cntErg[i].pulsCnt); SOUT(" "); SOUT("/"); SOUT(cntErg[i].usCnt); SOUT(" ");
                        }
                      //SOUT(outStr); SOUT(" ");
                      dispText(outStr ,  0, 1, outStr.length());
                  #endif
                break;

              default:
                //SOUT("disp end "); SOUT(" "); SOUTLN(millis());
                if (SYS_LED_ON == ON) { SYS_LED_ON = OFF; }
                else                  { SYS_LED_ON = ON ; }
                digitalWrite(PIN_BOARD_LED, SYS_LED_ON);

                oledIdx = 0;
                dispT.startT();
                break;
              }
            #ifdef USE_STATUS
                dispStatus("");
              #endif
          }
        #endif // defined(DISP)

    // --- system control --------------------------------
      #if (USE_COL16_BLINK_OUT > 0)
        if (ledT.TOut())    // handle touch output
          {
            ledT.startT();
            if (SYS_LED_ON == TRUE)
                {
                  digitalWrite(PIN_BOARD_LED, OFF);
                  SYS_LED_ON = OFF;
                }
              else
                {
                  digitalWrite(PIN_BOARD_LED, ON);
                  SYS_LED_ON = ON;
                }
          }
        #endif

      if (firstrun == true)
        {
          String taskMessage = "loop task running on core ";
          taskMessage = taskMessage + xPortGetCoreID();
          SOUTLN(taskMessage);
          usLast = micros();
          firstrun = false;
        }
      anzUsCycles++;
      //usleep(250);
    }
//
// --- subroutine and drivers ----------------
  // --- user output-.
    // --- display
      void clearDisp()
        {
          #if (USE_DISP > 0)
              #if defined(OLED1)
                oled1.clear();
                #endif
              #if defined(OLED2)
                oled2.clear();
                #endif
            #endif
        }

      void dispStatus(String msg)
        {
          #ifdef USE_STATUS
            size_t statLen = msg.length();
            bool   doIt    = false;

            if (statLen)
              {
                if ( statLen > DISP1_MAXCOLS)
                  {
                    msg.remove(DISP1_MAXCOLS);
                  }
                statOn = true;
                statT.startT();
                doIt = true;    // output statOut
                statT.startT();
              }
            else // empty input
              if (statOn && statT.TOut())
                statOn = false;

            if (!statOn) // disp actual time
              {
                sprintf(statOut,"%02d.%02d. %02d:%02d:%02d ", day(), month(), hour(), minute(), second());
                msg = statOut;
                doIt = true;
              }
            if (doIt)
              {
                #if (USE_TOUCHSCREEN > OFF)
                  touch.wrStatus(msg);
                  #endif
                #if (USE_OLED_I2C > OFF)
                    #if defined( USE_STATUS1 )
                        oled1.wrStatus(msg);
                      #endif
                    #if defined( USE_STATUS2 )
                        oled2.wrStatus(msg);
                      #endif
                         //SOUT("  md_error="); SOUTLN(md_error);
                  #endif
                #if (USE_TFT > 0)
                    mlcd.wrStatus((char*) statOut);
                        #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                            SOUT("  md_error="); SOUTLN(md_error);
                          #endif
                  #endif // USE_DISP
              }
            #endif // USE_STATUS
        }
      void dispStatus(const char* msg)
        {
          dispStatus((String) msg);
        }

      void dispText(char* msg, uint8_t col, uint8_t row, uint8_t len)
        {
          #if (USE_DISP > 0)
              #if (USE_TOUCHSCREEN > OFF)
                touch.wrTouch(msg, col, row);
                #endif
              #if defined(OLED1)
                oled1.wrText(msg, col, row, len);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                        SOUT("  md_error="); SOUTLN(md_error);
                      #endif
                #endif
              #if defined(OLED2)
                oled2.wrText(msg, col, row, len);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                        SOUT("  md_error="); SOUTLN(md_error);
                      #endif
                #endif
              #if (USE_TFT > 0)
                mlcd.wrText(msg, row, col);
                #endif
            #endif
        }
      void dispText(String msg, uint8_t col, uint8_t row, uint8_t len)
        {
          #if (USE_DISP > 0)
              #if (USE_TOUCHSCREEN > OFF)
                touch.wrTouch(msg, col, row);
                #endif
              #if defined(OLED1)
                oled1.wrText(msg, col, row, len);
                          //SOUT((uint32_t) millis); SOUT(" dispText oled1 '"); SOUT(msg); SOUTLN("'");
                #endif
              #if defined(OLED2)
                oled2.wrText(msg, col, row, len);
                          //SOUT((uint32_t) millis); SOUT(" dispText oled2 '"); SOUT(msg); SOUTLN("'");
                #endif
              #if (USE_TFT > 0)
                mlcd.wrText(msg, row, col);
                #endif
            #endif
        }

      void startDisp()
        {
          #if (USE_DISP > 0)
              #ifdef USE_STATUS
                statOut[DISP1_MAXCOLS] = 0;  // limit strlen
                #endif

              #if (USE_TFT > 0)
                mlcd.start(plcd);
                #endif

              #if (USE_TOUCHSCREEN > OFF)
                bool ret = touch.startTouch();
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        SOUT("startTouch ret="); SOUT(ret);
                      #endif
                md_error = setBit(md_error, ERRBIT_TOUCH, ret);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        SOUT("  md_error="); SOUTLN(md_error);
                      #endif
                #endif

              #if defined (OLED1)
                  oled1.begin((uint8_t) DISP1_MAXCOLS, (uint8_t) DISP1_MAXROWS);
                #endif

              #if defined (OLED2)
                  oled2.begin((uint8_t) DISP2_MAXCOLS, (uint8_t) DISP2_MAXROWS);
                #endif
            #endif
        }
    // --- WS2812 lines
      #if (USE_WS2812_LINE_OUT > OFF)
          #ifdef USE_FAST_LED
              void FillLEDsFromPaletteColors(uint8_t lineNo, uint8_t colorIndex)
                {
                  uint8_t brightness = 255;
                  for( int i = 0; i < LEDS_2812_L1; i++)
                    {
                      leds1[i] = ColorFromPalette( curPalette1, colorIndex, brightness, curBlending1);
                      #if (USE_WS2812_LINE_OUT > 1)
                          leds2[i] = ColorFromPalette( curPalette2, colorIndex, brightness, curBlending2);
                          #if (USE_WS2812_LINE_OUT > 2)
                              leds3[i] = ColorFromPalette( curPalette3, colorIndex, brightness, curBlending3);
                              #if (USE_WS2812_LINE_OUT > 3)
                                  leds4[i] = ColorFromPalette( curPalette4, colorIndex, brightness, curBlending4);
                                #endif
                            #endif
                        #endif
                      colorIndex += 3;
                    }
              }
              /* There are several different palettes of colors demonstrated here.
                //
                // FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
                // OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
                //
                // Additionally, you can manually define your own color palettes, or you can write
                // code that creates color palettes on the fly.  All are shown here.
                */

              void ChangePalettePeriodically(uint8_t lineNo)
                {
                  uint8_t secondHand = (millis() / 1000) % 60;
                  static uint8_t lastSecond = 99;

                  if( lastSecond != secondHand)
                    {
                      lastSecond = secondHand;
                      //if( secondHand ==  0)  { curPalette1 = RainbowColors_p;         curBlending = LINEARBLEND; SOUTLN("RainbowColors_p"); }
                      curPalette1 = RainbowStripeColors_p;
                      curBlending1 = NOBLEND;
                      #if (USE_WS2812_LINE_OUT > 1)
                          curPalette2 = RainbowStripeColors_p;
                          curBlending2 = NOBLEND;
                          #if (USE_WS2812_LINE_OUT > 2)
                              #if (USE_WS2812_LINE_OUT > 3)
                                #endif
                            #endif
                        #endif
                          //if( secondHand == 15)  { curPalette1 = RainbowStripeColors_p;   curBlending = LINEARBLEND; SOUTLN("RainbowStripeColors_p"); }
                          //if( secondHand == 20)  { SetupPurpleAndGreenPalette();             curBlending = LINEARBLEND; SOUTLN("PurpleAndGreenPalette"); }
                          //if( secondHand == 25)  { SetupTotallyRandomPalette();              curBlending = LINEARBLEND; SOUTLN("TotallyRandomPalette"); }
                          //if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       curBlending = NOBLEND;     SOUTLN("BlackAndWhiteStripedPalette"); }
                          //if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       curBlending = LINEARBLEND; SOUTLN("BlackAndWhiteStripedPalette"); }
                          //if( secondHand == 40)  { curPalette1 = CloudColors_p;           curBlending = LINEARBLEND; SOUTLN("CloudColors_p"); }
                          //if( secondHand == 45)  { curPalette1 = PartyColors_p;           curBlending = LINEARBLEND; SOUTLN("PartyColors_p"); }
                          //if( secondHand == 50)  { curPalette1 = myRedWhiteBluePalette_p; curBlending = NOBLEND;     SOUTLN("myRedWhiteBluePalette_p"); }
                          //if( secondHand == 55)  { curPalette1 = myRedWhiteBluePalette_p; curBlending = LINEARBLEND; SOUTLN("myRedWhiteBluePalette_p"); }
                    }
                }

              #ifdef XXXX
                  // This function fills the palette with totally random colors.
                  void SetupTotallyRandomPalette()
                    {
                      for( int i = 0; i < 16; i++)
                        {
                          curPalette1[i] = CHSV( random8(), 255, random8());
                        }
                    }

                  /* This function sets up a palette of black and white stripes,
                    // using code.  Since the palette is effectively an array of
                    // sixteen CRGB colors, the various fill_* functions can be used
                    // to set them up.
                    */
                  void SetupBlackAndWhiteStripedPalette()
                    {
                      // 'black out' all 16 palette entries...
                      fill_solid( curPalette1, 16, CRGB::Black);
                      // and set every fourth one to white.
                      curPalette1[0] = CRGB::White;
                      curPalette1[4] = CRGB::White;
                      curPalette1[8] = CRGB::White;
                      curPalette1[12] = CRGB::White;

                    }

                  // This function sets up a palette of purple and green stripes.
                  void SetupPurpleAndGreenPalette()
                    {
                      CRGB purple = CHSV( HUE_PURPLE, 255, 255);
                      CRGB green  = CHSV( HUE_GREEN, 255, 255);
                      CRGB black  = CRGB::Black;

                      curPalette1 = CRGBPalette16(
                                                     green,  green,  black,  black,
                                                     purple, purple, black,  black,
                                                     green,  green,  black,  black,
                                                     purple, purple, black,  black );
                    }

                  /* This example shows how to set up a static color palette
                    // which is stored in PROGMEM (flash), which is almost always more
                    // plentiful than RAM.  A static PROGMEM palette like this
                    // takes up 64 bytes of flash.
                    */
                  const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
                    {
                      CRGB::Red,
                      CRGB::Gray, // 'white' is too bright compared to red and blue
                      CRGB::Blue,
                      CRGB::Black,

                      CRGB::Red,
                      CRGB::Gray,
                      CRGB::Blue,
                      CRGB::Black,

                      CRGB::Red,
                      CRGB::Red,
                      CRGB::Gray,
                      CRGB::Gray,
                      CRGB::Blue,
                      CRGB::Blue,
                      CRGB::Black,
                      CRGB::Black
                    };

                  /* Additionl notes on FastLED compact palettes:
                    //
                    // Normally, in computer graphics, the palette (or "color lookup table")
                    // has 256 entries, each containing a specific 24-bit RGB color.  You can then
                    // index into the color palette using a simple 8-bit (one byte) value.
                    // A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
                    // is quite possibly "too many" bytes.
                    //
                    // FastLED does offer traditional 256-element palettes, for setups that
                    // can afford the 768-byte cost in RAM.
                    //
                    // However, FastLED also offers a compact alternative.  FastLED offers
                    // palettes that store 16 distinct entries, but can be accessed AS IF
                    // they actually have 256 entries; this is accomplished by interpolating
                    // between the 16 explicit entries to create fifteen intermediate palette
                    // entries between each pair.
                    //
                    // So for example, if you set the first two explicit entries of a compact
                    // palette to Green (0,255,0) and Blue (0,0,255), and then retrieved
                    // the first sixteen entries from the virtual palette (of 256), you'd get
                    // Green, followed by a smooth gradient from green-to-blue, and then Blue.
                    */
                #endif
          #else
            #endif
        #endif

    // --- passive buzzer
      #ifdef PLAY_MUSIC
          void playSong(int8_t songIdx)
            {
              if (buzz.setSong(SONG0_LEN,(void*) SONG0_NOTES) == ISOK)
                {
                  #ifndef USE_SONGTASK
                    buzz.playSong();
                  #endif
                }
            }

          void playSong()
            { playSong(0); }

        #endif

    // --- frequency generator
      #if (_USE_OUT_FREQ_PWM > OFF)
          void init_oscillator ()                                              // Inicializa gerador de pulsos
            {
              resolucao = (log (80000000 / oscilador)  / log(2)) / 2 ;                // Calculo da resolucao para o oscilador
              if (resolucao < 1) resolucao = 1;                                       // Resolu�ao m�nima
              // Serial.println(resolucao);                                           // Print
              mDuty = (pow(2, resolucao)) / 2;                                        // Calculo do ciclo de carga 50% do pulso
              // Serial.println(mDuty);                                               // Print

              ledc_timer_config_t ledc_timer = {};                                    // Instancia a configuracao do timer do LEDC

              ledc_timer.duty_resolution =  ledc_timer_bit_t(resolucao);              // Configura resolucao
              ledc_timer.freq_hz    = oscilador;                                      // Configura a frequencia do oscilador
              ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Modo de operacao em alta velocidade
              ledc_timer.timer_num = LEDC_TIMER_0;                                    // Usar timer0 do LEDC
              ledc_timer_config(&ledc_timer);                                         // Configura o timer do LEDC

              ledc_channel_config_t ledc_channel = {};                                // Instancia a configuracao canal do LEDC

              ledc_channel.channel    = LEDC_CHANNEL_0;                               // Configura canal 0
              ledc_channel.duty       = mDuty;                                        // Configura o ciclo de carga
              ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // Configura GPIO da saida do LEDC - oscilador
              ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // Desabilita interrup��o do LEDC
              ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Modo de operacao do canal em alta velocidade
              ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Seleciona timer 0 do LEDC
              ledc_channel_config(&ledc_channel);                                     // Configura o canal do LEDC
            }
        #endif
  // --- user input
    // --- keypad
      #if defined(KEYS)
          void startKeys()
            {
              #if (USE_KEYPADSHIELD > OFF)
                  kpad.init(KEYS_ADC);
                #endif // USE_KEYPADSHIELD
            }

          uint8_t getKey()
            {
              #if (USE_KEYPADSHIELD > OFF)
                  return kpad.getKey();
                #else
                  return NOKEY;
                #endif // USE_KEYPADSHIELD
            }
        #endif

    // --- counter
      #if (USE_CNT_INP > OFF)
          static void initFanPCNT()
            {
              /* Initialize PCNT event queue and PCNT functions */
              pcnt_unit_t unit;
              SOUT("init pcnt ");
              for (uint8_t i = 0 ; i < USE_CNT_INP ; i++)
                {
                  // Initialize PCNT unit and  queue
                    pcnt_unit_config(&config_cnt[i]);
                    pcnt_evt_queue[i] = xQueueCreate(1, sizeof(pcnt_evt_t));
                  switch (i)
                    {
                      #if (USE_CNT_INP > 1)
                          case 1:
                            unit = (pcnt_unit_t) PCNT2_UNIT;
                            // Configure and enable the input filter
                              pcnt_set_filter_value(unit, PCNT2_INP_FILT);
                              pcnt_filter_enable(unit);
                            // Set threshold 0 and 1 values and enable events to watch //
                              pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT2_THRESH0_VAL);
                              pcnt_event_enable(unit, PCNT_EVT_THRES_0);
                              //pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT2_THRESH1_VAL);
                              //pcnt_event_enable(unit, PCNT_EVT_THRES_1);
                            // Enable events on zero, maximum and minimum limit values //
                              //pcnt_event_enable(unit, PCNT_EVT_ZERO);
                              //pcnt_event_enable(unit, PCNT_EVT_H_LIM);
                              //pcnt_event_enable(unit, PCNT_EVT_L_LIM);
                            break;
                        #endif
                      default:  // = case 0
                        unit = (pcnt_unit_t) PCNT1_UNIT;
                        // Configure and enable the input filter
                          pcnt_set_filter_value(unit, PCNT1_INP_FILT);
                          pcnt_filter_enable(unit);
                        // Set threshold 0 and 1 values and enable events to watch //
                          pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT1_THRESH0_VAL);
                          pcnt_event_enable(unit, PCNT_EVT_THRES_0);
                          //pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT1_THRESH0_VAL);
                          //pcnt_event_enable(unit, PCNT_EVT_THRES_1);
                        // Enable events on zero, maximum and minimum limit values //
                          //pcnt_event_enable(unit, PCNT1_EVT_ZERO);
                          //pcnt_event_enable(unit, PCNT1_EVT_H_LIM);
                          //pcnt_event_enable(unit, PCNT1_EVT_L_LIM);
                        break;
                    }
                  SOUT(i); SOUT(" ");
                  // Initialize PCNT's counter //
                    pcnt_counter_pause(unit);
                    pcnt_counter_clear(unit);

                  // Install interrupt service and add isr callback handler //
                  pcnt_isr_service_install(unit);
                  switch (i)
                    {
                      #if (USE_CNT_INP > 1)
                          case 1: pcnt_isr_handler_add(unit, pcnt2_intr_hdl, &unit); break;
                        #endif
                      default: pcnt_isr_handler_add(unit, pcnt1_intr_hdl, &unit); break;
                    }
                  // Everything is set up, now go to counting //
                  pcnt_counter_resume(unit);
                }
            }
          #ifdef USE_MCPWM
              static void getCNTIn()
                {
                      // count LOW width
                      mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
                      cntLowPulse[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0) / 1000;
                      // count HIGH width
                      mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
                      cntHighPulse[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0) /1000 ;
                }
            #endif
        #endif

      #if (USE_DIG_INP > OFF)
          void getDIGIn()
            {
              for ( uint8_t i=0 ; i < USE_DIG_INP ; i++ )
                {
                  inpValDig[i] = digitalRead(PIN_DIG_INP[i]);
                }
            }
        #endif

      #if (USE_CTRL_POTI_ADC > OFF)
          void getADCIn()
            {
              for (uint8_t i = 0 ; i < USE_CTRL_SW_INP ; i++ )
                {
                  inpValADC[i] = analogRead(PIN_ADC_CONF[i].pin);
                }
            }
        #endif

  // --- sensors
    // --- DS18B20
      String getDS18D20Str()
        {
          String outS = "";
          #if (USE_DS18B20_1W_IO > OFF)
              dsSensors.requestTemperatures(); // Send the command to get temperatures
              for (uint8_t i = 0 ; i < DS18B20_ANZ ; i++ )
                {
                  dsTemp[i] = dsSensors.getTempCByIndex(i);
                        //SOUTLN(dsTemp[i]);
                  if (i < 1) { outS  = "T1 "; }
                  else       { outS += "    T2  ";}
                  outS += (String) ((int) (dsTemp[i] + 0.5)) + "°";
                }
            #endif
          return outS;
        }
    // --- BME280
      String getBME280Str()
        {
          String _outS = "";
          #if ( USE_BME280_I2C > OFF )
            bme.init();
            usleep(1000);
            _outS  = ((String) ((int) (bme.readTemperature()         + 0.5))) + "° ";
            usleep(1000);
            _outS += ((String) ((int) (bme.readHumidity()            + 0.5))) + "% ";
            usleep(1000);
            _outS += ((String) ((int) ((bme.readPressure() / 100.0F) + 0.5))) + "mb";
            //outS += "  Alt~ " + (String) bme280.readAltitude(1013.25);

                    //SOUT(" BME280 "); SOUTLN(_outS);
            #endif
          return _outS;
        }
    // --- T-element type K
  // ------ network -------------------------
    // --- WIFI
      void startWIFI(bool startup)
        {
          #if (USE_WIFI > OFF)
            bool ret = ISERR;
            dispStatus("  start WIFI");
            if (startup)
              {
                ip_list ipList = ip_list(); // temporary object
                          #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                              SOUT(millis()); SOUT(" setup startWIFI created ipList "); SOUTHEXLN((int) &ipList);
                              SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 0");
                            #endif
                ipList.append(WIFI_FIXIP0, WIFI_GATEWAY0, WIFI_SUBNET, WIFI_SSID0, WIFI_SSID0_PW);
                #if (WIFI_ANZ_LOGIN > 1)
                          #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                              SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 1");
                            #endif
                    ipList.append(WIFI_FIXIP1, WIFI_GATEWAY1, WIFI_SUBNET, WIFI_SSID1, WIFI_SSID1_PW);
                  #endif
                #if (WIFI_ANZ_LOGIN > 2)
                          #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                              SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 2");
                            #endif
                    ipList.append(WIFI_FIXIP2, WIFI_GATEWAY2, WIFI_SUBNET, WIFI_SSID2, WIFI_SSID2_PW);
                  #endif
                #if (WIFI_ANZ_LOGIN > 3)
                          #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                              SOUT(millis()); SOUTLN(" setup add WIFI 3");
                            #endif
                    ipList.append(WIFI_FIXIP3, WIFI_GATEWAY3, WIFI_SUBNET, WIFI_SSID3, WIFI_SSID3_PW);
                  #endif
                #if (WIFI_ANZ_LOGIN > 4)
                          #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                              SOUT(millis()); SOUTLN(" setup add WIFI 4");
                            #endif
                    ipList.append(WIFI_FIXIP3, WIFI_GATEWAY4, WIFI_SUBNET, WIFI_SSID4, WIFI_SSID4_PW);
                  #endif
                          SOUT(millis()); SOUTLN(" setup startWIFI locWIFI fertig");

                          //ip_cell* pip = (ip_cell*) ipList.pFirst();
                          //char stmp[NET_MAX_SSID_LEN] = "";
                                  /*
                                    SOUT(" setup ip_list addr "); SOUT((u_long) &ipList);
                                    SOUT(" count "); SOUTLN(ipList.count());
                                    SOUT(" ip1: addr "); SOUTHEX((u_long) pip);
                                    SOUT(" locIP "); SOUTHEX(pip->locIP());
                                    SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                    SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                    pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                    pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                    pip = (ip_cell*) pip->pNext();
                                    SOUT(" ip2: addr "); SOUTHEX((u_long) pip);
                                    SOUT(" locIP "); SOUTHEX(pip->locIP());
                                    SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                    SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                    pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                    pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                    pip = (ip_cell*) pip->pNext();
                                    SOUT(" ip3: addr "); SOUTHEX((u_long) pip);
                                    SOUT(" locIP "); SOUTHEX(pip->locIP());
                                    SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                    SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                    pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                    pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                  */

                ret = wifi.scanWIFI(&ipList);
                          SOUT(millis()); SOUT(" scanWIFI ret="); SOUTLN(ret);
              }
            ret = wifi.startWIFI();
                        SOUT("startWIFI ret="); SOUT(ret);
            md_error = setBit(md_error, ERRBIT_WIFI, ret);
                  #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                    SOUT("  md_error="); SOUTLN(md_error);
                    #endif

            if ((md_error & ERRBIT_WIFI) == 0)
                dispStatus("WIFI connected");
              else
                dispStatus("WIFI error");

            #if (USE_NTP_SERVER > OFF)
              if((md_error & ERRBIT_WIFI) == 0) // WiFi ok
                  if((md_error & ERRBIT_NTPTIME) != 0) // WiFi ok
                    wifi.initNTP();
              #endif
            #endif // USE_WIFI
        }
    // --- NTP server
      void initNTPTime()
        {
          #if (USE_NTP_SERVER > OFF)
            bool ret = wifi.initNTP();
                  #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                    Serial.print("initNTPTime ret="); Serial.print(ret);
                  #endif
            md_error = setBit(md_error, ERRBIT_NTPTIME, ret);
                  #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                    Serial.print("  md_error="); Serial.println(md_error);
                  #endif
            if ((md_error & ERRBIT_WIFI) == 0)
              {
                dispStatus("NTPTime ok");
              }
              else
              {
                dispStatus("NTPTime error");
              }
            #endif // USE_NTP_SERVER
        }

    // --- webserver
      void configWebsite()
        {
          #if (USE_WEBSERVER > OFF)
              webMD.createElement(EL_TYPE_SLIDER, "LED red", "%");
              webMD.createElement(EL_TYPE_SLIDER, "LED green", "%");
              webMD.createElement(EL_TYPE_SLIDER, "LED blue", "%");

              webMD.createElement(EL_TYPE_ANALOG, "DS18B20 Temp", "°C");
              webMD.createElement(EL_TYPE_ANALOG, "Type-K Temp", "°C");
              webMD.createElement(EL_TYPE_ANALOG, "BME_Temp", "°C");
              webMD.createElement(EL_TYPE_ANALOG, "BME_Humidity", "%");
              webMD.createElement(EL_TYPE_ANALOG, "BME_Pressure", "mb");
              webMD.createElement(EL_TYPE_ANALOG, "Gaswert", "");
            #endif // USE_WEBSERVER
        }

      void startWebServer()
        {
          #if (USE_WEBSERVER > OFF)
            bool ret = ISERR;
            if ((md_error & ERRBIT_SERVER) != 0)
              {
                dispStatus("start webserver");
                if ((md_error & ERRBIT_WIFI) == 0)
                  {
                    ret = webMD.md_startServer(0, 3, 0);
                        #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                          // SOUT("startServer ret="); SOUT(ret);
                        #endif
                  }
                md_error = setBit(md_error, ERRBIT_SERVER, ret);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        // SOUT("  md_error="); SOUTLN(md_error);
                      #endif

                if ((md_error & ERRBIT_SERVER) == 0)
                  {
                    dispStatus("Webserver online");
                  }
                  else
                  {
                    dispStatus("Webserver error");
                  }
              }
            #endif // USE_WEBSERVER
        }

// --- end of code --------------------------