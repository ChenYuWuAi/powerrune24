// led
#include <stdio.h>
#include "driver/gpio.h"
#include "LED_Strip.h"
#include "sdkconfig.h"

// event loop
#include "event_source.h"
#include "PowerRune_Events.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_spp_server_demo.h"

#define GATTS_TABLE_TAG "GATTS_SPP"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "PowerRune24" // The Device Name Characteristics in GAP

// 服务注册ID
#define SPP_SVC_INST_ID 0
#define OPS_SVC_INST_ID 1

// SPP Service(系统参数设置服务)的UUID
static const uint16_t spp_service_uuid = 0x1827; // Mesh Proxy Service
// 特征值的UUID
#define UUID_URL 0x2AA6        // Central Address Resoluton
#define UUID_MAC 0x2AC9        // Resolvable Private Address Only
#define UUID_SSID 0x2AC3       // Object ID
#define UUID_Wifi 0x2A3E       // Network Availability
#define UUID_AOTA 0x2AC5       // Object Action Control Point
#define UUID_LIT 0x2A0D        // DST Offset
#define UUID_STRIP_LIT 0x2A01  // Appearance
#define UUID_R_LIT 0x2A9B      // Body Composition Feature
#define UUID_MATRIX_LIT 0x2A9C // Body Composition Measurement
#define UUID_PID 0x2A66        // Cycling Power Control Point
#define UUID_ARMOR_ID 0x2B1F   // Reconnection Configuration Control Point

// 大符操作服务的UUID
static const uint16_t ops_service_uuid = 0x1828;
// 特征值的UUID
#define UUID_RUN 0x2A65  // Cycling Power Control Feature
#define UUID_GPA 0x2A69  // Position Quality
#define UUID_UNLK 0x2A3B // Service Required
#define UUID_STOP 0x2AC8 // Object Changed
#define UUID_OTA 0x2A9F  // User Control Point

// LED_Strip
LED_Strip LED_Strip_0(GPIO_NUM_10, 49);

/**
 * @brief Static array containing the advertising data for the BLE SPP server.
 *
 * The advertising data includes the following:
 * - Flags: 0x02, 0x01, 0x06
 * - Complete List of 16-bit Service Class UUIDs: 0x03, 0x03, 0xF0, 0xAB
 * - Complete Local Name in advertising: "PowerRune24"
 */
static const uint8_t spp_adv_data[] = {
    /* Flags */
    0x02,
    0x01,
    0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,
    0x03,
    0xF0,
    0xAB,
    /* Complete Local Name in advertising */
    0x0C,
    0x09,
    'P',
    'o',
    'w',
    'e',
    'r',
    'R',
    'u',
    'n',
    'e',
    '2',
    '4',
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};

// spp服务的句柄存储(系统参数设置服务)
static uint16_t spp_handle_table[SPP_IDX_NB];

// 大符操作服务的句柄存储
static uint16_t ops_handle_table[OPS_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node
{
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff
{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num = 0,
    .buff_size = 0,
    .first_node = NULL};

extern "C" void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// 服务端APP注册结构表
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler, // 回调函数
        .gatts_if = ESP_GATT_IF_NONE,            /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};
/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// 系统参数设置服务的属性表的相关全局变量
// url
static const uint16_t url_uuid = UUID_URL;
static const u_int8_t url_val[1] = {0};
static const uint8_t url_ccc[1] = {0};
// mac
static const uint16_t mac_uuid = UUID_MAC;
static const u_int8_t mac_val[1] = {0};
static const uint8_t mac_ccc[1] = {0};
// ssid
static const uint16_t ssid_uuid = UUID_SSID;
static const u_int8_t ssid_val[1] = {0};
static const uint8_t ssid_ccc[1] = {0};
// wifi
static const uint16_t wifi_uuid = UUID_Wifi;
static const u_int8_t wifi_val[1] = {0};
static const uint8_t wifi_ccc[1] = {0};
// aota
static const uint16_t aota_uuid = UUID_AOTA;
static const u_int8_t aota_val[1] = {0};
static const uint8_t aota_ccc[1] = {0};
// lit
static const uint16_t lit_uuid = UUID_LIT;
static const u_int8_t lit_val[1] = {0};
static const uint8_t lit_ccc[1] = {0};
// strip_lit
static const uint16_t strip_lit_uuid = UUID_STRIP_LIT;
static const u_int8_t strip_lit_val[1] = {0};
static const uint8_t strip_lit_ccc[1] = {0};
// r_lit
static const uint16_t r_lit_uuid = UUID_R_LIT;
static const u_int8_t r_lit_val[1] = {0};
static const uint8_t r_lit_ccc[1] = {0};
// matrix_lit
static const uint16_t matrix_lit_uuid = UUID_MATRIX_LIT;
static const u_int8_t matrix_lit_val[1] = {0};
static const uint8_t matrix_lit_ccc[1] = {0};
// pid
static const uint16_t pid_uuid = UUID_PID;
static const u_int8_t pid_val[1] = {0};
static const uint8_t pid_ccc[1] = {0};
// armor_id
static const uint16_t armor_id_uuid = UUID_ARMOR_ID;
static const u_int8_t armor_id_val[1] = {0};
static const uint8_t armor_id_ccc[1] = {0};

// 系统参数设置服务的属性表
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = {
    // SPP -  Service Declaration
    [SPP_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    // url
    [URL_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [URL_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&url_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(url_val), sizeof(url_val), (uint8_t *)url_val}},

    [URL_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(url_ccc), (uint8_t *)url_ccc}},

    // mac
    [MAC_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [MAC_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&mac_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(mac_val), sizeof(mac_val), (uint8_t *)mac_val}},

    [MAC_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(mac_ccc), (uint8_t *)mac_ccc}},

    // ssid
    [SSID_CHAR] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [SSID_VAL] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&ssid_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ssid_val), sizeof(ssid_val), (uint8_t *)ssid_val}},

    [SSID_CFG] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ssid_ccc), (uint8_t *)ssid_ccc}},

    // wifi
    [Wifi_CHAR] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [Wifi_VAL] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&wifi_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(wifi_val), sizeof(wifi_val), (uint8_t *)wifi_val}},

    [Wifi_CFG] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(wifi_ccc), (uint8_t *)wifi_ccc}},

    // aota
    [AOTA_CHAR] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [AOTA_VAL] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&aota_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(aota_val), sizeof(aota_val), (uint8_t *)aota_val}},

    [AOTA_CFG] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(aota_ccc), (uint8_t *)aota_ccc}},

    // lit
    [LIT_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [LIT_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&lit_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(lit_val), sizeof(lit_val), (uint8_t *)lit_val}},

    [LIT_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(lit_ccc), (uint8_t *)lit_ccc}},

    // strip_lit
    [STRIP_LIT_CHAR] = {{ESP_GATT_AUTO_RSP},
                        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [STRIP_LIT_VAL] = {{ESP_GATT_AUTO_RSP},
                       {ESP_UUID_LEN_16, (uint8_t *)&strip_lit_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(strip_lit_val), sizeof(strip_lit_val), (uint8_t *)strip_lit_val}},

    [STRIP_LIT_CFG] = {{ESP_GATT_AUTO_RSP},
                       {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(strip_lit_ccc), (uint8_t *)strip_lit_ccc}},

    // r_lit
    [R_LIT_CHAR] = {{ESP_GATT_AUTO_RSP},
                    {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [R_LIT_VAL] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&r_lit_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(r_lit_val), sizeof(r_lit_val), (uint8_t *)r_lit_val}},

    [R_LIT_CFG] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(r_lit_ccc), (uint8_t *)r_lit_ccc}},

    // matrix_lit
    [MATRIX_LIT_CHAR] = {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [MATRIX_LIT_VAL] = {{ESP_GATT_AUTO_RSP},
                        {ESP_UUID_LEN_16, (uint8_t *)&matrix_lit_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(matrix_lit_val), sizeof(matrix_lit_val), (uint8_t *)matrix_lit_val}},

    [MATRIX_LIT_CFG] = {{ESP_GATT_AUTO_RSP},
                        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(matrix_lit_ccc), (uint8_t *)matrix_lit_ccc}},

    // pid
    [PID_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [PID_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&pid_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(pid_val), sizeof(pid_val), (uint8_t *)pid_val}},

    [PID_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(pid_ccc), (uint8_t *)pid_ccc}},

    // armor_id
    [ARMOR_ID_CHAR] = {{ESP_GATT_AUTO_RSP},
                       {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [ARMOR_ID_VAL] = {{ESP_GATT_AUTO_RSP},
                      {ESP_UUID_LEN_16, (uint8_t *)&armor_id_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(armor_id_val), sizeof(armor_id_val), (uint8_t *)armor_id_val}},

    [ARMOR_ID_CFG] = {{ESP_GATT_AUTO_RSP},
                      {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(armor_id_ccc), (uint8_t *)armor_id_ccc}},
};

// 大符操作服务的属性表的相关全局变量
// run
static const uint16_t ops_run_uuid = UUID_RUN;
static const u_int8_t ops_run_val[3] = {0};
static const uint8_t ops_run_ccc[1] = {0};
// gpa
static const uint16_t ops_gpa_uuid = UUID_GPA;
static const u_int8_t ops_gpa_val[1] = {0};

// unlk
static const uint16_t ops_unlk_uuid = UUID_UNLK;
static const u_int8_t ops_unlk_val[1] = {0};
static const uint8_t ops_unlk_ccc[1] = {0};

// stop
static const uint16_t ops_stop_uuid = UUID_STOP;
static const u_int8_t ops_stop_val[1] = {0};
static const uint8_t ops_stop_ccc[1] = {0};

// ota
static const uint16_t ops_ota_uuid = UUID_OTA;
static const u_int8_t ops_ota_val[1] = {0};
static const uint8_t ops_ota_ccc[1] = {0};

// 大符操作服务的属性表                       添加属性表下标
static const esp_gatts_attr_db_t ops_gatt_db[OPS_IDX_NB] = {
    // OPS -  Service Declaration
    [OPS_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(ops_service_uuid), sizeof(ops_service_uuid), (uint8_t *)&ops_service_uuid}},

    // run
    [RUN_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

    [RUN_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&ops_run_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ops_run_val), sizeof(ops_run_val), (uint8_t *)ops_run_val}},

    [RUN_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ops_run_ccc), (uint8_t *)ops_run_ccc}},

    // gpa
    [GPA_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    [GPA_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&ops_gpa_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ops_gpa_val), sizeof(ops_gpa_val), (uint8_t *)ops_gpa_val}},

    // unlk
    [UNLK_CHAR] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

    [UNLK_VAL] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&ops_unlk_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ops_unlk_val), sizeof(ops_unlk_val), (uint8_t *)ops_unlk_val}},

    [UNLK_CFG] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ops_unlk_ccc), (uint8_t *)ops_unlk_ccc}},

    // stop
    [STOP_CHAR] = {{ESP_GATT_AUTO_RSP},
                   {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

    [STOP_VAL] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&ops_stop_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ops_stop_val), sizeof(ops_stop_val), (uint8_t *)ops_stop_val}},

    [STOP_CFG] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ops_stop_ccc), (uint8_t *)ops_stop_ccc}},

    // ota
    [OTA_CHAR] = {{ESP_GATT_AUTO_RSP},
                  {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

    [OTA_VAL] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&ops_ota_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ops_ota_val), sizeof(ops_ota_val), (uint8_t *)ops_ota_val}},

    [OTA_CFG] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ops_ota_ccc), (uint8_t *)ops_ota_ccc}},
};

// event loop中的handle
// Handler which executes when the ble started event gets executed by the loop.
// spp服务
static void url_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void url_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void mac_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void mac_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void ssid_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void ssid_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void wifi_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void wifi_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void aota_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void aota_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void lit_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void lit_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void strip_lit_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void strip_lit_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void r_lit_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void r_lit_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void matrix_lit_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void matrix_lit_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void pid_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void pid_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void armor_id_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void armor_id_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}
// ops服务

class RUNE
{
private:
public:
    int ID;
    int rand_num;
    RUNE()
    {
    }
    RUNE(int id)
        : ID(id)
    {
        rand_num = rand() % 5 + 1;
    }
    void randagain()
    {
        rand_num = rand() % 5 + 1;
    }
};
RUNE rune1(1);
RUNE rune2(2);
RUNE rune3(3);
RUNE rune4(4);
RUNE rune5(5);
void rune_rand()
{
    srand(xTaskGetTickCount());
    rune1.randagain();
    while (rune1.rand_num == rune2.rand_num)
    {
        rune2.randagain();
    }
    while (rune1.rand_num == rune3.rand_num || rune2.rand_num == rune3.rand_num)
    {
        rune3.randagain();
    }
    while (rune1.rand_num == rune4.rand_num || rune2.rand_num == rune4.rand_num || rune3.rand_num == rune4.rand_num)
    {
        rune4.randagain();
    }
    while (rune1.rand_num == rune5.rand_num || rune2.rand_num == rune5.rand_num || rune3.rand_num == rune5.rand_num || rune4.rand_num == rune5.rand_num)
    {
        rune5.randagain();
    }
}

int IS_PRM_SPEED_STABLE = 0;
int IS_HIT = 0;

int hitted_ID = 10;
int start_pra_and_wait_hit(int expected_ID)
{
    esp_event_post(PRA, PRA_START_EVENT, NULL, 0, portMAX_DELAY);

    int old_time = xTaskGetTickCount();
    while (IS_HIT == 0)
    {
        if (xTaskGetTickCount() - old_time >= 2500)
        {
            esp_event_post(PRA, PRA_STOP_EVENT, NULL, 0, portMAX_DELAY);
            return -1;
        }
    }
    IS_HIT = 0;
    if (expected_ID == hitted_ID)
    {
        esp_event_post(PRA, PRA_HIT_EVENT, NULL, 0, portMAX_DELAY);
        return 1;
    }
    else
    {
        return 0;
    }
}
void run_task()
{
    const u_int8_t *value;
    u_int16_t len;

    esp_ble_gatts_get_attr_value(ops_handle_table[RUN_VAL], &len, &value);

    printf("run特征值:\n");
    printf("value0(颜色) = %d\r\n", value[0]);
    printf("value1(大小) = %d\r\n", value[1]);
    printf("value2(自动)0/1 = %d\r\n\n", value[2]);

    rune_rand();

    printf("rune1 = %d\r\n", rune1.rand_num);
    printf("rune2 = %d\r\n", rune2.rand_num);
    printf("rune3 = %d\r\n", rune3.rand_num);
    printf("rune4 = %d\r\n", rune4.rand_num);
    printf("rune5 = %d\r\n", rune5.rand_num);

    int a[5];
    a[0] = rune1.rand_num;
    a[1] = rune2.rand_num;
    a[2] = rune3.rand_num;
    a[3] = rune4.rand_num;
    a[4] = rune5.rand_num;
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);

    static int only_once = 0;
    if(only_once == 0)
    {
        esp_event_post(PRM, PRM_START_EVENT, NULL, 0, portMAX_DELAY);
        while (IS_PRM_SPEED_STABLE == 0){}
    }

    RUNE rune_start_sequence[5];
    if (rune1.rand_num == 1)
    {
        rune_start_sequence[0] = rune1;
    }
    else if (rune2.rand_num == 1)
    {
        rune_start_sequence[0] = rune2;
    }
    else if (rune3.rand_num == 1)
    {
        rune_start_sequence[0] = rune3;
    }
    else if (rune4.rand_num == 1)
    {
        rune_start_sequence[0] = rune4;
    }
    else if (rune5.rand_num == 1)
    {
        rune_start_sequence[0] = rune5;
    }
    if (rune1.rand_num == 2)
    {
        rune_start_sequence[1] = rune1;
    }
    else if (rune2.rand_num == 2)
    {
        rune_start_sequence[1] = rune2;
    }
    else if (rune3.rand_num == 2)
    {
        rune_start_sequence[1] = rune3;
    }
    else if (rune4.rand_num == 2)
    {
        rune_start_sequence[1] = rune4;
    }
    else if (rune5.rand_num == 2)
    {
        rune_start_sequence[1] = rune5;
    }
    if (rune1.rand_num == 3)
    {
        rune_start_sequence[2] = rune1;
    }
    else if (rune2.rand_num == 3)
    {
        rune_start_sequence[2] = rune2;
    }
    else if (rune3.rand_num == 3)
    {
        rune_start_sequence[2] = rune3;
    }
    else if (rune4.rand_num == 3)
    {
        rune_start_sequence[2] = rune4;
    }
    else if (rune5.rand_num == 3)
    {
        rune_start_sequence[2] = rune5;
    }
    if (rune1.rand_num == 4)
    {
        rune_start_sequence[3] = rune1;
    }
    else if (rune2.rand_num == 4)
    {
        rune_start_sequence[3] = rune2;
    }
    else if (rune3.rand_num == 4)
    {
        rune_start_sequence[3] = rune3;
    }
    else if (rune4.rand_num == 4)
    {
        rune_start_sequence[3] = rune4;
    }
    else if (rune5.rand_num == 4)
    {
        rune_start_sequence[3] = rune5;
    }
    if (rune1.rand_num == 5)
    {
        rune_start_sequence[4] = rune1;
    }
    else if (rune2.rand_num == 5)
    {
        rune_start_sequence[4] = rune2;
    }
    else if (rune3.rand_num == 5)
    {
        rune_start_sequence[4] = rune3;
    }
    else if (rune4.rand_num == 5)
    {
        rune_start_sequence[4] = rune4;
    }
    else if (rune5.rand_num == 5)
    {
        rune_start_sequence[4] = rune5;
    }
    int hit_count = 0;
    for (; hit_count < 5; hit_count++)
    {
        int res;
        int all_stop_arg = 0;
        res = start_pra_and_wait_hit(rune_start_sequence[hit_count].ID);
        if (res == 0)
        {
            printf("击错\n");
            esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
            char a[6] = "wrong";
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
            hit_count = 0;
            break;
        }
        else if (res == -1)
        {
            printf("超时\n");
            esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
            char a[5] = "miss";
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
            hit_count = 0;
            break;
        }
    }
    if(hit_count == 4)
    {
        printf("完成\n");
        esp_event_post(PRM, PRA_COMPLETE_EVENT, NULL, 0, portMAX_DELAY);
    }
    if (value[2] == 1)
    {
        printf("自动循环\n");
        esp_event_post(RUN_EVENTS, RUN_EVENT_WRITE, NULL, 0, portMAX_DELAY);
    }
    only_once = 1;
    vTaskDelete(NULL);
}
static void run_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    xTaskCreate((TaskFunction_t)run_task, "run_task", 4096, NULL, 10, NULL);
}

static void gpa_read_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void unlk_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}

static void stop_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
}
void ota_task()
{
    esp_event_post(PRC, OTA_BEGIN_EVENT, NULL, 0, portMAX_DELAY);

    int old_time = xTaskGetTickCount();
    // while (/*条件*/)
    // {
    //     if (xTaskGetTickCount() - old_time >= 2500)
    //     {//超时
    //         esp_event_post(PRA, PRA_STOP_EVENT, NULL, 0, portMAX_DELAY);
    //     }
    // }
    vTaskDelete(NULL);
    //esprestart();
}
static void ota_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    xTaskCreate((TaskFunction_t)ota_task, "ota_task", 4096, NULL, 10, NULL);
}

// PowerRune_Events handles
static void pra_stop(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    switch (*((int *)event_data))
    {
    case 0:

        break;
    default:
        break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;
    // 系统参数设置服务
    if (handle < spp_handle_table[0] + SPP_IDX_NB)
    {
        for (int i = 0; i < SPP_IDX_NB; i++)
        {
            if (handle == spp_handle_table[i])
            {
                return i;
            }
        }
    }
    // 大符操作服务
    else
    {
        for (int i = 0; i < OPS_IDX_NB; i++)
        {
            if (handle == ops_handle_table[i])
            {
                return i + SPP_IDX_NB;
            }
        }
    }
    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL)
    {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0)
    {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }
    else
    {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

// GAP的回调函数
extern "C" void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    printf("GAP的回调函数\n");
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        // 设置广播原始数据完成
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // 开始广播完成
        //  advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

// GATTS最终的回调函数
extern "C" void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        // 注册事件
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        // 创建属性表
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        printf("注册spp属性表结束\n");
        esp_ble_gatts_create_attr_tab(ops_gatt_db, gatts_if, OPS_IDX_NB, OPS_SVC_INST_ID);
        printf("注册ops属性表结束\n");
        break;
    case ESP_GATTS_READ_EVT:
    {
        // read事件
        res = find_char_and_desr_index(p_data->read.handle);
        switch (res)
        {
        case URL_VAL:
            // URL_read事件
            printf("URL_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(URL_EVENTS, URL_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("URL_read事件结束\n");
            break;
        case URL_CFG:
            // URL_cfg
            printf("URL_cfg(read)\n");
            break;
        case MAC_VAL:
            // MAC_read事件
            printf("MAC_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(MAC_EVENTS, MAC_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("MAC_read事件结束\n");
            break;
        case MAC_CFG:
            // MAC_cfg
            printf("MAC_cfg(read)\n");
            break;
        case SSID_VAL:
            // SSID_read事件
            printf("SSID_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(SSID_EVENTS, SSID_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("SSID_read事件结束\n");
            break;
        case SSID_CFG:
            // SSID_cfg
            printf("SSID_cfg(read)\n");
            break;
        case Wifi_VAL:
            // Wifi_read事件
            printf("Wifi_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(Wifi_EVENTS, Wifi_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("Wifi_read事件结束\n");
            break;
        case Wifi_CFG:
            // Wifi_cfg
            printf("Wifi_cfg(read)\n");
            break;
        case AOTA_VAL:
            // AOTA_read事件
            printf("AOTA_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(AOTA_EVENTS, AOTA_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("AOTA_read事件结束\n");
            break;
        case AOTA_CFG:
            // AOTA_cfg
            printf("AOTA_cfg(read)\n");
            break;
        case LIT_VAL:
            // LIT_read事件
            printf("LIT_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(LIT_EVENTS, LIT_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("LIT_read事件结束\n");
            break;
        case LIT_CFG:
            // LIT_cfg
            printf("LIT_cfg(read)\n");
            break;
        case STRIP_LIT_VAL:
            // STRIP_LIT_read事件
            printf("STRIP_LIT_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("STRIP_LIT_read事件结束\n");
            break;
        case STRIP_LIT_CFG:
            // STRIP_LIT_cfg
            printf("STRIP_LIT_cfg(read)\n");
            break;
        case R_LIT_VAL:
            // R_LIT_read事件
            printf("R_LIT_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(R_LIT_EVENTS, R_LIT_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("R_LIT_read事件结束\n");
            break;
        case R_LIT_CFG:
            // R_LIT_cfg
            printf("R_LIT_cfg(read)\n");
            break;
        case MATRIX_LIT_VAL:
            // MATRIX_LIT_read事件
            printf("MATRIX_LIT_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("MATRIX_LIT_read事件结束\n");
            break;
        case MATRIX_LIT_CFG:
            // MATRIX_LIT_cfg
            printf("MATRIX_LIT_cfg(read)\n");
            break;
        case PID_VAL:
            // PID_read事件
            printf("PID_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(PID_EVENTS, PID_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("PID_read事件结束\n");
            break;
        case PID_CFG:
            // PID_cfg
            printf("PID_cfg(read)\n");
            break;
        case ARMOR_ID_VAL:
            // ARMOR_ID_read事件
            printf("ARMOR_ID_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("ARMOR_ID_read事件结束\n");
            break;
        case ARMOR_ID_CFG:
            // ARMOR_ID_cfg
            printf("ARMOR_ID_cfg(read)\n");
            break;
        case GPA_VAL + SPP_IDX_NB:
            // GPA_read事件
            printf("GPA_read事件\n");
            ESP_ERROR_CHECK(esp_event_post(GPA_EVENTS, GPA_EVENT_READ, NULL, 0, portMAX_DELAY));
            printf("GPA_read事件结束\n");
            break;
        default:
            printf("未知read事件\n");
        }
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        // write事件
        res = find_char_and_desr_index(p_data->write.handle);
        printf("write事件  pdata handle: %d\n", res);
        if (p_data->write.is_prep == false)
        {
            switch (res)
            {
            case URL_VAL:
                // URL_write事件
                printf("URL_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case URL_CFG:
                // URL_cfg
                printf("URL_cfg(write)\n");
                break;
            case MAC_VAL:
                // MAC_write事件
                printf("MAC_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case MAC_CFG:
                // MAC_cfg
                printf("MAC_cfg(write)\n");
                break;
            case SSID_VAL:
                // SSID_write事件
                printf("SSID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case SSID_CFG:
                // SSID_cfg
                printf("SSID_cfg(write)\n");
                break;
            case Wifi_VAL:
                // Wifi_write事件
                printf("Wifi_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case Wifi_CFG:
                // Wifi_cfg
                printf("Wifi_cfg(write)\n");
                break;
            case AOTA_VAL:
                // AOTA_write事件
                printf("AOTA_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case AOTA_CFG:
                // AOTA_cfg
                printf("AOTA_cfg(write)\n");
                break;
            case LIT_VAL:
                // LIT_write事件
                printf("LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case LIT_CFG:
                // LIT_cfg
                printf("LIT_cfg(write)\n");
                break;
            case STRIP_LIT_VAL:
                // STRIP_LIT_write事件
                printf("STRIP_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case STRIP_LIT_CFG:
                // STRIP_LIT_cfg
                printf("STRIP_LIT_cfg(write)\n");
                break;
            case R_LIT_VAL:
                // R_LIT_write事件
                printf("R_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case R_LIT_CFG:
                // R_LIT_cfg
                printf("R_LIT_cfg(write)\n");
                break;
            case MATRIX_LIT_VAL:
                // MATRIX_LIT_write事件
                printf("MATRIX_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case MATRIX_LIT_CFG:
                // MATRIX_LIT_cfg
                printf("MATRIX_LIT_cfg(write)\n");
                break;
            case PID_VAL:
                // PID_write事件
                printf("PID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case PID_CFG:
                // PID_cfg
                printf("PID_cfg(write)\n");
                break;
            case ARMOR_ID_VAL:
                // ARMOR_ID_write事件
                printf("ARMOR_ID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case ARMOR_ID_CFG:
                // ARMOR_ID_cfg
                printf("ARMOR_ID_cfg(write)\n");
                break;
            case RUN_VAL + SPP_IDX_NB:
                // RUN_write事件
                printf("RUN_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case RUN_CFG + SPP_IDX_NB:
                // RUN_cfg
                printf("RUN_cfg(write)\n");
                break;
            case UNLK_VAL + SPP_IDX_NB:
                // UNLK_write事件
                printf("UNLK_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case UNLK_CFG + SPP_IDX_NB:
                // UNLK_cfg
                printf("UNLK_cfg(write)\n");
                break;
            case STOP_VAL + SPP_IDX_NB:
                // STOP_write事件
                printf("STOP_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case STOP_CFG + SPP_IDX_NB:
                // STOP_cfg
                printf("STOP_cfg(write)\n");
                break;
            case OTA_VAL + SPP_IDX_NB:
                // OTA_write事件
                printf("OTA_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case OTA_CFG + SPP_IDX_NB:
                // OTA_cfg
                printf("OTA_cfg(write)\n");
                break;
            default:
                printf("未知write事件\n");
            }
        }
        else if ((p_data->write.is_prep == true))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        if (p_data->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        // 经过esp_ble_gatts_send_indicate会到此处
        printf("经过send_indicate后\n");
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        // 创建属性表后启动服务
        else if (param->add_attr_tab.svc_inst_id == 0)
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);

            printf("spp_handle_table[0] = %d\n", spp_handle_table[0]);
        }
        else if (param->add_attr_tab.svc_inst_id == 1)
        {
            memcpy(ops_handle_table, param->add_attr_tab.handles, sizeof(ops_handle_table));
            esp_ble_gatts_start_service(ops_handle_table[OPS_IDX_SVC]);

            printf("ops_handle_table[0] = %d\n", ops_handle_table[0]);
        }
        break;
    }
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    {
        // 当设置属性表完成时，到这里
        res = find_char_and_desr_index(param->set_attr_val.attr_handle);
        switch (res)
        {
        case URL_VAL:
            // URL
            esp_event_post(URL_EVENTS, URL_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case MAC_VAL:
            // MAC
            esp_event_post(MAC_EVENTS, MAC_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case SSID_VAL:
            // SSID
            esp_event_post(SSID_EVENTS, SSID_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case Wifi_VAL:
            // Wifi
            esp_event_post(Wifi_EVENTS, Wifi_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case AOTA_VAL:
            // AOTA
            esp_event_post(AOTA_EVENTS, AOTA_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case LIT_VAL:
            // LIT
            esp_event_post(LIT_EVENTS, LIT_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case STRIP_LIT_VAL:
            // STRIP_LIT
            esp_event_post(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case R_LIT_VAL:
            // R_LIT
            esp_event_post(R_LIT_EVENTS, R_LIT_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case MATRIX_LIT_VAL:
            // MATRIX_LIT
            esp_event_post(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case PID_VAL:
            // PID
            esp_event_post(PID_EVENTS, PID_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case ARMOR_ID_VAL:
            // ARMOR_ID
            esp_event_post(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case RUN_VAL + SPP_IDX_NB:
            // RUN
            esp_event_post(RUN_EVENTS, RUN_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case UNLK_VAL + SPP_IDX_NB:
            // UNLK
            esp_event_post(UNLK_EVENTS, UNLK_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case STOP_VAL + SPP_IDX_NB:
            // STOP
            esp_event_post(STOP_EVENTS, STOP_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        case OTA_VAL + SPP_IDX_NB:
            // OTA
            esp_event_post(OTA_EVENTS, OTA_EVENT_WRITE, NULL, 0, portMAX_DELAY);
            break;
        }
        break;
    }
    default:
        break;
    }
}

// GATTS的回调函数
extern "C" void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    printf("\nGATTS事件回调\n\n");
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    // 如果是注册APP事件
    if (event == ESP_GATTS_REG_EVT)
    {
        // 如果注册成功
        if (param->reg.status == ESP_GATT_OK)
        {
            // APP记录描述符
            spp_profile_tab[SPP_PROFILE_APP_IDX /*只用一个APP所以下标为0*/].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        // 遍历注册APP的结构表
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                // 执行注册的APP的回调函数
                if (spp_profile_tab[idx].gatts_cb)
                {
                    printf("去往profile\n");
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
    printf("从profile回来\n");
}

extern "C" void app_main(void)
{
    printf("开始\n");
    // Initialize NVS
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    // Initialize GPIO
    gpio_config_t GPIO_config = {};
    GPIO_config.mode = GPIO_MODE_OUTPUT;
    GPIO_config.intr_type = GPIO_INTR_DISABLE;
    GPIO_config.pin_bit_mask = (1U << 2);
    GPIO_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&GPIO_config);

    // 获取ble默认配置
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // GATT的回调注册
    esp_ble_gatts_register_callback(gatts_event_handler);
    // GAP事件的函数
    esp_ble_gap_register_callback(gap_event_handler);
    // 注册APP
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    // event loop
    // 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 创建大符事件循环

    // 注册事件handle
    // spp服务
    // Register url event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(URL_EVENTS, URL_EVENT_READ, url_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(URL_EVENTS, URL_EVENT_WRITE, url_write_handler, NULL, NULL));
    // Register mac event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MAC_EVENTS, MAC_EVENT_READ, mac_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MAC_EVENTS, MAC_EVENT_WRITE, mac_write_handler, NULL, NULL));
    // Register ssid event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(SSID_EVENTS, SSID_EVENT_READ, ssid_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(SSID_EVENTS, SSID_EVENT_WRITE, ssid_write_handler, NULL, NULL));
    // Register wifi event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(Wifi_EVENTS, Wifi_EVENT_READ, wifi_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(Wifi_EVENTS, Wifi_EVENT_WRITE, wifi_write_handler, NULL, NULL));
    // Register aota event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(AOTA_EVENTS, AOTA_EVENT_READ, aota_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(AOTA_EVENTS, AOTA_EVENT_WRITE, aota_write_handler, NULL, NULL));
    // Register lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(LIT_EVENTS, LIT_EVENT_READ, lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(LIT_EVENTS, LIT_EVENT_WRITE, lit_write_handler, NULL, NULL));
    // Register strip_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_READ, strip_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_WRITE, strip_lit_write_handler, NULL, NULL));
    // Register r_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(R_LIT_EVENTS, R_LIT_EVENT_READ, r_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(R_LIT_EVENTS, R_LIT_EVENT_WRITE, r_lit_write_handler, NULL, NULL));
    // Register matrix_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_READ, matrix_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_WRITE, matrix_lit_write_handler, NULL, NULL));
    // Register pid event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PID_EVENTS, PID_EVENT_READ, pid_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PID_EVENTS, PID_EVENT_WRITE, pid_write_handler, NULL, NULL));
    // Register armor_id event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_READ, armor_id_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_WRITE, armor_id_write_handler, NULL, NULL));
    // ops服务
    // Register run event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(RUN_EVENTS, RUN_EVENT_WRITE, run_write_handler, NULL, NULL));
    // Register gpa event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(GPA_EVENTS, GPA_EVENT_READ, gpa_read_handler, NULL, NULL));
    // Register unlk event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(UNLK_EVENTS, UNLK_EVENT_WRITE, unlk_write_handler, NULL, NULL));
    // Register stop event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STOP_EVENTS, STOP_EVENT_WRITE, stop_write_handler, NULL, NULL));
    // Register ota event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(OTA_EVENTS, OTA_EVENT_WRITE, ota_write_handler, NULL, NULL));
    // PowerRune_Events
    // Register pra_stop event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PRA, PRA_STOP_EVENT, pra_stop, NULL, NULL));
}