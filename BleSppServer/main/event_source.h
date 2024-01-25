#include "esp_event.h"

// Declare event bases
//SPP服务
ESP_EVENT_DECLARE_BASE(LED_EVENTS);             // declaration of the event family
enum {                                          // declaration of the specific events under the event family
    LED_EVENT_READ,            
    LED_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(URL_EVENTS);
enum {
    URL_EVENT_READ,            
    URL_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(MAC_EVENTS);
enum {
    MAC_EVENT_READ,            
    MAC_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(SSID_EVENTS);
enum {
    SSID_EVENT_READ,            
    SSID_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(Wifi_EVENTS);
enum {
    Wifi_EVENT_READ,            
    Wifi_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(AOTA_EVENTS);
enum {
    AOTA_EVENT_READ,            
    AOTA_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(LIT_EVENTS);
enum {
    LIT_EVENT_READ,            
    LIT_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(STRIP_LIT_EVENTS);
enum {
    STRIP_LIT_EVENT_READ,            
    STRIP_LIT_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(R_LIT_EVENTS);
enum {
    R_LIT_EVENT_READ,            
    R_LIT_EVENT_WRITE,                              
};

ESP_EVENT_DECLARE_BASE(MATRIX_LIT_EVENTS);
enum {
    MATRIX_LIT_EVENT_READ,            
    MATRIX_LIT_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(PID_EVENTS);
enum {
    PID_EVENT_READ,            
    PID_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(ARMOR_ID_EVENTS);
enum {
    ARMOR_ID_EVENT_READ,            
    ARMOR_ID_EVENT_WRITE,                             
};
//OPS服务
ESP_EVENT_DECLARE_BASE(RUN_EVENTS);
enum {        
    RUN_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(GPA_EVENTS);
enum {        
    GPA_EVENT_READ,                             
};

ESP_EVENT_DECLARE_BASE(UNLK_EVENTS);
enum {        
    UNLK_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(STOP_EVENTS);
enum {        
    STOP_EVENT_WRITE,                             
};

ESP_EVENT_DECLARE_BASE(OTA_EVENTS);
enum {        
    OTA_EVENT_WRITE,                             
};