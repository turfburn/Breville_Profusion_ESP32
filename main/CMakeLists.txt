idf_component_register(
    SRCS 
        "./main.c" 
        "./time_sync.c"
        "./TCA9554PWR/TCA9554PWR.c"
        "./LCD_Driver/esp_lcd_st77916/esp_lcd_st77916.c" 
        "./LCD_Driver/ST77916.c"       
        "./LVGL_Driver/LVGL_Driver.c"
        "./LVGL_UI/LVGL_Coffee_UI.c"
        "./SD_Card/SD_MMC.c" 
        "./I2C_Driver/I2C_Driver.c"
        "./PCF85063/PCF85063.c"
        "./Wireless/Wireless.c"
        "./profile_manager.c"
        "./background.c"
        

    INCLUDE_DIRS 
        "./LCD_Driver/esp_lcd_st77916" 
        "./LCD_Driver"   
        "./LVGL_Driver" 
        "./LVGL_UI" 
        "./TCA9554PWR"
        "./SD_Card" 
        "./I2C_Driver"
        "./PCF85063"
        "./Wireless"
        "."
        
    REQUIRES
        esp_lcd
        esp_timer 
        driver
        lvgl
        fatfs
        # esp_wifi          # COMMENTED OUT
        # esp_netif         # COMMENTED OUT  
        # esp_event         # COMMENTED OUT (if only used for WiFi)
        bt                  # Bluetooth controller
        nvs_flash
        esp_http_client
        esp_wifi
        json
)