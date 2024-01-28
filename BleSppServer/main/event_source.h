#include "esp_event.h"
//SPP服务
enum {                                          // declaration of the specific events under the event family
    LED_EVENT_READ,            
    LED_EVENT_WRITE,                             
};

enum {
    URL_EVENT_READ,            
    URL_EVENT_WRITE,                             
};

enum {
    MAC_EVENT_READ,            
    MAC_EVENT_WRITE,                             
};

enum {
    SSID_EVENT_READ,            
    SSID_EVENT_WRITE,                             
};

enum {
    Wifi_EVENT_READ,            
    Wifi_EVENT_WRITE,                             
};
enum {
    AOTA_EVENT_READ,            
    AOTA_EVENT_WRITE,                             
};

enum {
    LIT_EVENT_READ,            
    LIT_EVENT_WRITE,                             
};

enum {
    STRIP_LIT_EVENT_READ,            
    STRIP_LIT_EVENT_WRITE,                             
};

enum {
    R_LIT_EVENT_READ,            
    R_LIT_EVENT_WRITE,                              
};

enum {
    MATRIX_LIT_EVENT_READ,            
    MATRIX_LIT_EVENT_WRITE,                             
};

enum {
    PID_EVENT_READ,            
    PID_EVENT_WRITE,                             
};

enum {
    ARMOR_ID_EVENT_READ,            
    ARMOR_ID_EVENT_WRITE,                             
};

//OPS服务
enum {        
    RUN_EVENT_WRITE,                             
};

enum {        
    GPA_EVENT_READ,                             
};

enum {        
    UNLK_EVENT_WRITE,                             
};

enum {        
    STOP_EVENT_WRITE,                             
};

enum {        
    OTA_EVENT_WRITE,                             
};