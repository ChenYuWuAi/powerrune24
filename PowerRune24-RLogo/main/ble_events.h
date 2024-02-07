#pragma once
#ifndef _BLE_EVENTS_H_
#define _BLE_EVENTS_H_
#include "esp_event.h"

// 系统参数设置服务
enum
{
    SPP_IDX_SVC,

    URL_CHAR,
    URL_VAL,
    URL_CFG,

    MAC_CHAR,
    MAC_VAL,
    MAC_CFG,

    SSID_CHAR,
    SSID_VAL,
    SSID_CFG,

    Wifi_CHAR,
    Wifi_VAL,
    Wifi_CFG,

    AOTA_CHAR,
    AOTA_VAL,
    AOTA_CFG,

    LIT_CHAR,
    LIT_VAL,
    LIT_CFG,

    STRIP_LIT_CHAR,
    STRIP_LIT_VAL,
    STRIP_LIT_CFG,

    R_LIT_CHAR,
    R_LIT_VAL,
    R_LIT_CFG,

    MATRIX_LIT_CHAR,
    MATRIX_LIT_VAL,
    MATRIX_LIT_CFG,

    PID_CHAR,
    PID_VAL,
    PID_CFG,

    ARMOR_ID_CHAR,
    ARMOR_ID_VAL,
    ARMOR_ID_CFG,

    SPP_IDX_NB,
};

// 大符操作服务
enum
{
    OPS_IDX_SVC,

    RUN_CHAR,
    RUN_VAL,
    RUN_CFG,

    GPA_CHAR,
    GPA_VAL,

    UNLK_CHAR,
    UNLK_VAL,
    UNLK_CFG,

    STOP_CHAR,
    STOP_VAL,
    STOP_CFG,

    OTA_CHAR,
    OTA_VAL,
    OTA_CFG,

    OPS_IDX_NB,
};

#define SPP_DATA_MAX_LEN (512)
#define SPP_CMD_MAX_LEN (20)
#define SPP_STATUS_MAX_LEN (20)

#define GATTS_TABLE_TAG "GATTS_SPP"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define DEVICE_NAME "PowerRune24" // The Device Name Characteristics in GAP

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
// 服务端APP注册结构表static
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

// Declare event bases
// SPP服务
ESP_EVENT_DECLARE_BASE(LED_EVENTS); // declaration of the event family
enum
{ // declaration of the specific events under the event family
    LED_EVENT_READ,
    LED_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(URL_EVENTS);
enum
{
    URL_EVENT_READ,
    URL_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(MAC_EVENTS);
enum
{
    MAC_EVENT_READ,
    MAC_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(SSID_EVENTS);
enum
{
    SSID_EVENT_READ,
    SSID_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(Wifi_EVENTS);
enum
{
    Wifi_EVENT_READ,
    Wifi_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(AOTA_EVENTS);
enum
{
    AOTA_EVENT_READ,
    AOTA_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(LIT_EVENTS);
enum
{
    LIT_EVENT_READ,
    LIT_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(STRIP_LIT_EVENTS);
enum
{
    STRIP_LIT_EVENT_READ,
    STRIP_LIT_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(R_LIT_EVENTS);
enum
{
    R_LIT_EVENT_READ,
    R_LIT_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(MATRIX_LIT_EVENTS);
enum
{
    MATRIX_LIT_EVENT_READ,
    MATRIX_LIT_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(PID_EVENTS);
enum
{
    PID_EVENT_READ,
    PID_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(ARMOR_ID_EVENTS);
enum
{
    ARMOR_ID_EVENT_READ,
    ARMOR_ID_EVENT_WRITE,
};
// OPS服务
ESP_EVENT_DECLARE_BASE(RUN_EVENTS);
enum
{
    RUN_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(GPA_EVENTS);
enum
{
    GPA_EVENT_READ,
};

ESP_EVENT_DECLARE_BASE(UNLK_EVENTS);
enum
{
    UNLK_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(STOP_EVENTS);
enum
{
    STOP_EVENT_WRITE,
};

ESP_EVENT_DECLARE_BASE(OTA_EVENTS);
enum
{
    OTA_EVENT_WRITE,
};

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
#endif