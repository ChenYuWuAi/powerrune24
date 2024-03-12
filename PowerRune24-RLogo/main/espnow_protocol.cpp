/**
 * @file espnow_protocol.cpp
 * @brief ESP-NOW协议类定义，用于ESP-NOW通信
 * @version 1.7
 * @date 2024-0
 * @note 本文件存放ESP-NOW协议类的定义，Wifi硬件初始化，esp-now的发送和接收回调函数，事件处理
 */

#include "espnow_protocol.h"

// #define HEAPDEBUG

static const char *TAG_MESSAGER = "Messager";
static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
// 初始化静态成员
// espnow MAC地址
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
uint8_t ESPNowProtocol::mac_addr[6][ESP_NOW_ETH_ALEN] = {0};
// 优化为哈希查找
// std::unordered_map<mac_address_t, uint8_t, hash> ESPNowProtocol::mac_to_address_map;
uint16_t ESPNowProtocol::packet_tx_id[6] = {0};
uint16_t ESPNowProtocol::packet_rx_id[6] = {0};
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
uint8_t ESPNowProtocol::mac_addr[ESP_NOW_ETH_ALEN] = {0};
uint16_t ESPNowProtocol::packet_tx_id = 0;
uint16_t ESPNowProtocol::packet_rx_id = 0;
#endif
// espnow 数据包队列
QueueHandle_t ESPNowProtocol::espnow_rx_queue = NULL;
// tx mutex
SemaphoreHandle_t ESPNowProtocol::tx_semaphore = xSemaphoreCreateMutex();
// peer list是否成功建立
bool ESPNowProtocol::peer_list_established = false;
// 自身MAC
uint8_t ESPNowProtocol::self_mac[ESP_NOW_ETH_ALEN] = {0};
// 广播地址
uint8_t ESPNowProtocol::NULL_mac[ESP_NOW_ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 广播地址
uint8_t ESPNowProtocol::broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// 发送数据EventBits
EventGroupHandle_t ESPNowProtocol::send_state = NULL;
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
// Beacon
TickType_t ESPNowProtocol::beacon_send_time = 0;
TickType_t ESPNowProtocol::beacon_time = 0;
uint8_t ESPNowProtocol::beacon_mode = 1;
TaskHandle_t ESPNowProtocol::beacon_task_handle = NULL;
esp_event_handler_t ESPNowProtocol::beacon_timeout_handler = NULL;
#endif
// Function
void ESPNowProtocol::log_packet(const espnow_DATA_pack_t *data, uint16_t crc16)
{
    ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
    ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
    ESP_LOGD(TAG_MESSAGER, "event_base: %s", data->event_base);
    ESP_LOGD(TAG_MESSAGER, "event_id: 0x%02X", data->event_id);
    ESP_LOGD(TAG_MESSAGER, "event_data_len: %d", data->event_data_len);
    ESP_LOGD(TAG_MESSAGER, "CRC16: 0x%04X", data->crc16);
}

void ESPNowProtocol::log_packet(const ACK_OK_pack_t *data)
{
    ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
    ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
}

void ESPNowProtocol::log_packet(const ACK_FAIL_pack_t *data)
{
    ESP_LOGD(TAG_MESSAGER, "header: 0x%04X", data->header);
    ESP_LOGD(TAG_MESSAGER, "pack_id: 0x%08X", data->pack_id);
}

void ESPNowProtocol::log_raw_packet(const uint8_t *data, uint8_t len)
{
    printf("DATA: ");
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
uint8_t ESPNowProtocol::mac_to_address(uint8_t *mac)
{
    // 顺序查找
    for (uint8_t i = 0; i < 6; i++)
        if (memcmp(mac, mac_addr + i, ESP_NOW_ETH_ALEN) == 0)
            return i;

    return 0xFF;
}

void ESPNowProtocol::update_mac_to_address_map()
{
    return;
}

#endif
void ESPNowProtocol::rx_callback(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    // 中断回调函数参数参考: const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len
    ESP_LOGD(TAG_MESSAGER, "[rx_callback]Received %i bytes from " MACSTR, len, MAC2STR(esp_now_info->src_addr));

    if (esp_now_info->src_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG_MESSAGER, "Receive cb arg error");
        return;
    }

// // 比较MAC地址，如果不是自己的MAC地址或者广播地址，直接丢弃
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
    if (memcmp(esp_now_info->des_addr, self_mac, ESP_NOW_ETH_ALEN) != 0 && memcmp(esp_now_info->des_addr, broadcast_mac, ESP_NOW_ETH_ALEN) != 0)
#else
    if (memcmp(esp_now_info->des_addr, self_mac, ESP_NOW_ETH_ALEN) != 0)
#endif
    {
        ESP_LOGE(TAG_MESSAGER, "Receive cb MAC " MACSTR " error", MAC2STR(esp_now_info->des_addr));
        return;
    }

    // 存入数据
    received_struct received_data;

    memcpy(received_data.src_MAC, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
    received_data.data = (uint8_t *)malloc(len);
    assert(received_data.data != NULL);
    memcpy(received_data.data, data, len);
    received_data.len = len;
    // 发送到espnow_rx_queue
    xQueueSendFromISR(espnow_rx_queue, &received_data, &xHigherPriorityTaskWoken);
}

#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
void ESPNowProtocol::beacon_task(void *pvParameter)
{
    beacon_send_time = xTaskGetTickCount();
    while (1)
    {
        if ((xTaskGetTickCount() - beacon_time) > CONFIG_BEACON_TIMEOUT / portTICK_PERIOD_MS)
        {
            ESP_LOGE(TAG_MESSAGER, "beacon timeout");
            esp_event_post_to(pr_events_loop_handle, PRC, BEACON_TIMEOUT_EVENT, NULL, 0, 0);
        }
        // 十秒发送一次beacon PING
        if (xEventGroupGetBits(send_state) & SEND_BUSY)
        {
            ESP_LOGW(TAG_MESSAGER, "beacon busy, skip beacon send");
        }
        else
        {
// PING EVENT 数据
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
            ESP_LOGD(TAG_MESSAGER, "[beacon] Sending PING_EVENT to " MACSTR, MAC2STR(mac_addr));
            int32_t event_id = PRA_PING_EVENT;
            PRA_PING_EVENT_DATA ping_data;
            // 如果已经设置ID，则广播ID，否则维持0xFF
            const PowerRune_Armour_config_info_t *config_info = config->get_config_info_pt();
            if (config_info->armour_id != 0xFF)
                ping_data.address = config_info->armour_id - 1;
            else
                ping_data.address = 0xFF;
            memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Armour_config_info_t));
            send_data(broadcast_mac, PRA, event_id, &ping_data, sizeof(PRA_PING_EVENT_DATA), 0);
            vTaskDelayUntil(&beacon_send_time, 1000 / portTICK_PERIOD_MS);
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
            ESP_LOGD(TAG_MESSAGER, "Sending PING_EVENT to " MACSTR, MAC2STR(broadcast_mac));
            int32_t event_id = PRM_PING_EVENT;
            PRM_PING_EVENT_DATA ping_data;
            ping_data.address = MOTOR;
            memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Motor_config_info_t));
            send_data(broadcast_mac, PRM, event_id, &ping_data, sizeof(PRM_PING_EVENT_DATA), 0);
            vTaskDelayUntil(&beacon_send_time, 1000 / portTICK_PERIOD_MS);

#endif
        }
        // 如果有接收正常包视作beacon正常，继续等待
        while (xTaskGetTickCount() - beacon_time < 5000 / portTICK_PERIOD_MS)
        {
            // 重新等待
            vTaskDelayUntil(&beacon_send_time, 9000 / portTICK_PERIOD_MS);
        }
    }
}
#endif

void ESPNowProtocol::parse_data_task(void *pvParameter)
{

    received_struct *received_data = (received_struct *)malloc(sizeof(received_struct));

    while (1)
    {
        if (xQueueReceive(espnow_rx_queue, received_data, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG_MESSAGER, "Failed to receive data from espnow_rx_queue");
            continue;
        }
        uint8_t *data = received_data->data;
        ESP_LOGD(TAG_MESSAGER, "[parse_task] Received %i bytes from " MACSTR, received_data->len, MAC2STR(received_data->src_MAC));
        log_raw_packet(data, received_data->len);

        // 小端发送
        uint16_t header = (data[1] << 8) | data[0];
        // 解析大符事件数据包
        if (header == POWERRUNE_DATA_HEADER)
        {
            espnow_DATA_pack_t *espnow_data_pack = (espnow_DATA_pack_t *)malloc(received_data->len);
            memset((void *)espnow_data_pack, 0, received_data->len);
            // 按字节解析
            espnow_data_pack->header = header;
            espnow_data_pack->pack_id = (data[3] << 8) | data[2];
            memcpy(espnow_data_pack->event_base, data + 6, 4);
            espnow_data_pack->event_id = data[12];

// ID校验
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
            // PING包交给establish_peer_list处理
            if ((espnow_data_pack->event_base[2] == 'A' && espnow_data_pack->event_id == PRA_PING_EVENT) || (espnow_data_pack->event_base[2] == 'M' && espnow_data_pack->event_id == PRM_PING_EVENT))
            {
                ESP_LOGD(TAG_MESSAGER, "Received PING packet from " MACSTR ", sending RESPONSE packet", MAC2STR(received_data->src_MAC));
                // data的第三个变量是地址位
                send_data(received_data->src_MAC, PRC, RESPONSE_EVENT, NULL, 0, 0, 0); // 注意包的TX ID是大一的

                // 处理ID，将rx_id更新为pack_id - 1
                packet_rx_id[mac_to_address(received_data->src_MAC)] = espnow_data_pack->pack_id - 1;

                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
            // 接到的包ID小于等于系统记录已经接收的包ID，说明这个包已经无效
            if (espnow_data_pack->pack_id <= packet_rx_id[mac_to_address(received_data->src_MAC)])
            {
                ESP_LOGE(TAG_MESSAGER, "packet %i ID check failed, current rx_id %i", espnow_data_pack->pack_id, packet_rx_id[mac_to_address(received_data->src_MAC)]);
                // 发送ACK_FAIL，提示ID已更新，但不处理
                send_ACK(packet_rx_id[mac_to_address(received_data->src_MAC)] + 1, received_data->src_MAC, ACK_FAIL);
                // 直接丢包，释放内存
                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
            // RESPONSE包用于维护beacon time
            if (espnow_data_pack->event_id == RESPONSE_EVENT && espnow_data_pack->event_base[2] == 'C') // PRC:RESPONSE_EVENT
            {
                beacon_time = xTaskGetTickCount();
                ESP_LOGI(TAG_MESSAGER, "update beacon: %i", (int)beacon_time);
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
                // 开启led blink
                if (config->get_config_info_pt()->armour_id != 0xFF)
                    led->set_mode(LED_MODE_BLINK, config->get_config_info_pt()->armour_id);
                else
                    led->set_mode(LED_MODE_BLINK, 0);
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
                // 开启led fade
                led->set_mode(LED_MODE_FADE, 0);
#endif
                // 更新ID
                packet_rx_id = espnow_data_pack->pack_id - 1;
                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
            // 已发送的包ID小于等于接收的包ID，说明这个包已经无效
            if (espnow_data_pack->pack_id <= packet_rx_id)
            {
                ESP_LOGE(TAG_MESSAGER, "DATA packet %i ID check failed, current rx_id %i", espnow_data_pack->pack_id, packet_rx_id);
                // 发送ACK_FAIL，更新ID，但不处理
                send_ACK(packet_rx_id + 1, received_data->src_MAC, ACK_FAIL);
                // 直接丢包，释放内存
                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
            // 接包，说明beacon正常
            beacon_time = xTaskGetTickCount();
            ESP_LOGI(TAG_MESSAGER, "update beacon: %i", (int)beacon_time);
#endif

            uint16_t crc16_pack = (data[5] << 8) | data[4];
            espnow_data_pack->event_data_len = (data[11] << 8) | data[10];
            // 拷贝事件数据
            memcpy(espnow_data_pack->event_data, data + 13, espnow_data_pack->event_data_len);
            // CRC校验
            log_raw_packet((uint8_t *)espnow_data_pack, 13 + espnow_data_pack->event_data_len);
            uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)espnow_data_pack, 13 + espnow_data_pack->event_data_len);
            log_packet(espnow_data_pack, crc16_pack);

            if (crc16 != crc16_pack)
            {
                ESP_LOGE(TAG_MESSAGER, "DATA packet %i CRC16 check (result: 0x%04X) failed", espnow_data_pack->pack_id, crc16);
                // 发送ACK_FAIL，然后丢包
                send_ACK(espnow_data_pack->pack_id, received_data->src_MAC, ACK_FAIL);
                // 释放内存
                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
            else
            {
// 包有效，更新ID
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
                uint8_t address = mac_to_address(received_data->src_MAC);
                packet_rx_id[address] = espnow_data_pack->pack_id;
                // 将Address存入事件数据
                espnow_data_pack->event_data[0] = address;
#else
                packet_rx_id = espnow_data_pack->pack_id;
                espnow_data_pack->event_data[0] = 0x06; // 表示从主控发出
#endif
                ESP_LOGD(TAG_MESSAGER, "DATA packet %i CRC16 check success, address %i", espnow_data_pack->pack_id, espnow_data_pack->event_data[0]);
                // 发送ACK_OK
                send_ACK(espnow_data_pack->pack_id, received_data->src_MAC, ACK_OK);
                esp_event_base_t base = NULL;
                switch (espnow_data_pack->event_base[2])
                {
                case 'A':
                    base = PRA;
                    break;
                case 'M':
                    base = PRM;
                    break;
                case 'C':
                    base = PRC;
                    break;
                default:
                    break;
                }
                // 发送到事件循环
                esp_event_post_to(pr_events_loop_handle, base,
                                  espnow_data_pack->event_id, espnow_data_pack->event_data,
                                  espnow_data_pack->event_data_len,
                                  CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
                // 释放内存
                free(received_data->data);
                free(espnow_data_pack);
                continue;
            }
        }
        // 正在等待ACK的包
        else if (header == POWERRUNE_ACK_OK_HEADER && (xEventGroupGetBits(send_state) & SEND_ACK_PENDING_BIT))
        {
            ESP_LOGD(TAG_MESSAGER, "Received ACK_OK packet");
            // 这是一个ACK_OK包,
            ACK_OK_pack_t ack_ok_pack = {0};
            ack_ok_pack.header = header;
            ack_ok_pack.pack_id = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
            log_packet(&ack_ok_pack);
            // ID校验，丢弃过期ACK包
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
            if (ack_ok_pack.pack_id < packet_tx_id[mac_to_address(received_data->src_MAC)] + 1)
            {
                ESP_LOGE(TAG_MESSAGER, "ACK_OK packet %i ID check failed", ack_ok_pack.pack_id);
                // 释放内存
                free(received_data->data);
                continue;
            }
#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
            if (ack_ok_pack.pack_id < packet_tx_id + 1)
            {
                ESP_LOGE(TAG_MESSAGER, "ACK_OK packet %i ID check failed", ack_ok_pack.pack_id);
                // 释放内存
                free(received_data->data);
                continue;
            }
#endif
            // 发送ACK_OK
            xEventGroupSetBits(send_state, SEND_ACK_OK_BIT_INTERNAL);
            xEventGroupSetBits(send_state, SEND_ACK_OK_BIT);
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
            // 视作beacon更新依据
            beacon_time = xTaskGetTickCount();
            ESP_LOGI(TAG_MESSAGER, "update beacon: %i", (int)beacon_time);
#endif
            // 释放内存
            free(received_data->data);
        }
        else if (header == POWERRUNE_ACK_FAIL_HEADER && (xEventGroupGetBits(send_state) & SEND_ACK_PENDING_BIT))
        {
            // 这是一个ACK_FAIL包
            ACK_FAIL_pack_t ack_fail_pack = {0};
            ack_fail_pack.header = header;
            ack_fail_pack.pack_id = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
            log_packet(&ack_fail_pack);
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
            // 视作beacon更新依据
            beacon_time = xTaskGetTickCount();
            ESP_LOGI(TAG_MESSAGER, "update beacon: %i", (int)beacon_time);
#endif
            // ID校验，丢弃过期ACK包
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
            if (ack_fail_pack.pack_id < packet_tx_id[mac_to_address(received_data->src_MAC)])
            {

#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
            if (ack_fail_pack.pack_id < packet_tx_id)
            {
#endif
                ESP_LOGE(TAG_MESSAGER, "ACK_FAIL packet %i ID check failed", ack_fail_pack.pack_id);
                // 释放内存
                free(received_data->data);
                continue;
            }

#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
            else if (ack_fail_pack.pack_id > packet_tx_id[mac_to_address(received_data->src_MAC)] + 1)
            {
                // 更新ID
                packet_tx_id[mac_to_address(received_data->src_MAC)] = ack_fail_pack.pack_id - 1;
                ESP_LOGW(TAG_MESSAGER, "ACK_FAIL packet %i ID check failed, TX id updated to %i", ack_fail_pack.pack_id, packet_tx_id[mac_to_address(received_data->src_MAC)]);
#elif ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
            else if (ack_fail_pack.pack_id > packet_tx_id + 1)
            {
                // 更新ID
                packet_tx_id = ack_fail_pack.pack_id - 1;
                ESP_LOGW(TAG_MESSAGER, "ACK_FAIL packet %i ID check failed, TX id updated to %i", ack_fail_pack.pack_id, packet_tx_id);
#endif
                xEventGroupSetBits(send_state, SEND_ACK_FAIL_ID_BIT);
                free(received_data->data);
            }
            else
            {
                // 发送ACK_FAIL
                xEventGroupSetBits(send_state, SEND_ACK_FAIL_BIT);
                // 释放内存
                free(received_data->data);
            }
        }
        else
        {
            // 收到未知包，丢弃
            ESP_LOGE(TAG_MESSAGER, "Unknown packet header: 0x%04X", header);
            // 释放内存
            free(received_data->data);
            continue;
        }
    }
}

esp_err_t ESPNowProtocol::send_data(uint8_t *dest_mac, esp_event_base_t event_base, int32_t event_id, void *data, uint16_t data_len, uint8_t wait_ack, uint8_t mutex, uint8_t id_plus)
{
    /*
    发送DATA包时自动申请tx id
    关于ID的说明
    1. 调用函数时传参使用的ID会在未收到ACK_FAIL的情况下始终被发送循环使用
    2. 收到ACK_FAIL后，如果ACK_FAIL的ID大于当前ID，说明对方重启，在parse_data_task中会更新ID，本函数中则直接选取更新后的ID进行重传
    关于data_len的说明
    data_len应该与事件数据结构体内的data_len一致，即事件结构体的大小
    */

    EventBits_t bits;
    // 编码包数据
    espnow_DATA_pack_t *packet = (espnow_DATA_pack_t *)malloc(sizeof(espnow_DATA_pack_t));
    // 谨慎操作，因为malloc没有完全包含整个espnow_DATA_pack_t结构体
    // assert if malloc failed
    assert(packet != NULL);

    packet->header = POWERRUNE_DATA_HEADER;
    packet->event_data_len = data_len;
    memcpy(packet->event_data, data, data_len);
    memcpy(packet->event_base, event_base, 4);
    packet->event_id = event_id;
    // CRC置零后进行运算
    packet->crc16 = 0;
    log_raw_packet((uint8_t *)packet, 13 + data_len);
    // 申请ID
#if CONFIG_POWERRUNE_TYPE == 1
    uint8_t address = mac_to_address(dest_mac);
    if (address == 0xFF)
    {
        ESP_LOGE(TAG_MESSAGER, "Failed to find address of " MACSTR, MAC2STR(dest_mac));
        free(packet);
        return ESP_FAIL;
    }
    packet->pack_id = id_plus ? ESPNowProtocol::packet_tx_id[address] + 1 : ESPNowProtocol::packet_tx_id[mac_to_address(dest_mac)];
#else
    packet->pack_id = id_plus ? ESPNowProtocol::packet_tx_id + 1 : ESPNowProtocol::packet_tx_id;
#endif
    // 将CRC16校验和添加到数据包
    packet->crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)packet, 13 + data_len);
    log_raw_packet((uint8_t *)packet, 13 + data_len);
    memcpy(packet->dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
    log_packet(packet, packet->crc16);
    // 调用发送函数，含超时重传
    for (uint8_t trial = 0; trial < RETRY_COUNT; trial++)
    {
#ifdef HEAPDEBUG
        heap_caps_check_integrity_all(1);
#endif
        if (wait_ack)
            // 置位使能ACK等待
            xEventGroupSetBits(send_state, SEND_ACK_PENDING_BIT);

        // 发送锁
        // 获取互斥锁
        if (xSemaphoreTake(tx_semaphore, mutex ? portMAX_DELAY : CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS) != pdTRUE)
        {
            ESP_LOGE(TAG_MESSAGER, "Failed to take tx_semaphore");
            return ESP_FAIL;
        }

        // 清除ACK_OK和ACK_FAIL
        xEventGroupClearBits(send_state, SEND_ACK_OK_BIT_INTERNAL | SEND_ACK_FAIL_BIT);
        esp_err_t err = esp_now_send(dest_mac, (uint8_t *)packet, 13 + data_len);
        if (err != ESP_OK)
        {
            xEventGroupClearBits(send_state, SEND_ACK_PENDING_BIT);
            ESP_LOGE(TAG_MESSAGER, "packet %i send error %s, trial %i", packet->pack_id, esp_err_to_name(err), trial);
            // 释放互斥锁
            xSemaphoreGive(tx_semaphore);
            continue;
        }
        // 等待发送成功或失败
        bits = xEventGroupWaitBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);

        if (bits & SEND_COMPLETE_BIT)
        {
            ESP_LOGD(TAG_MESSAGER, "packet %i send successfully to " MACSTR, packet->pack_id, MAC2STR(dest_mac));
            trial = 0; // 重置重传次数

            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
            // 释放互斥锁
            xSemaphoreGive(tx_semaphore);
            if (wait_ack == 0)
            {
                free(packet);
                return ESP_OK;
            }
        }
        else if (bits & SEND_FAIL_BIT)
        {
            ESP_LOGE(TAG_MESSAGER, "packet %i send error in tx callback, trial %i", packet->pack_id, trial);
            xEventGroupClearBits(send_state, SEND_FAIL_BIT);
            // 释放互斥锁
            xSemaphoreGive(tx_semaphore);
            // 等待100ms
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        else
        {
            ESP_LOGE(TAG_MESSAGER, "packet %i send timeout, trial %i", packet->pack_id, trial);
            // 释放互斥锁
            xSemaphoreGive(tx_semaphore);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT);
            continue;
        }

        // 等待ACK_OK或ACK_FAIL，原则上只有一个线程在等待，所以不需考虑线程安全
        bits = xEventGroupWaitBits(send_state, SEND_ACK_OK_BIT_INTERNAL | SEND_ACK_FAIL_BIT | SEND_ACK_FAIL_ID_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);

        if (bits & SEND_ACK_OK_BIT_INTERNAL)
        {
            ESP_LOGD(TAG_MESSAGER, "packet %i ACK_OK received", packet->pack_id);
#if CONFIG_POWERRUNE_TYPE == 1
            ESPNowProtocol::packet_tx_id[mac_to_address(dest_mac)]++;
#else
            ESPNowProtocol::packet_tx_id++;
#endif
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_ACK_PENDING_BIT);
            free(packet);
            return ESP_OK;
        }
        else if (bits & SEND_ACK_FAIL_BIT)
        {
            ESP_LOGE(TAG_MESSAGER, "packet %i ACK_FAIL received", packet->pack_id);
            xEventGroupClearBits(send_state, SEND_ACK_FAIL_BIT | SEND_FAIL_BIT | SEND_COMPLETE_BIT | SEND_ACK_PENDING_BIT);
            // 重传
            continue;
        }
        else if (bits & SEND_ACK_FAIL_ID_BIT)
        {
            ESP_LOGE(TAG_MESSAGER, "packet %i ACK_FAIL received, ID updated", packet->pack_id);
            xEventGroupClearBits(send_state, SEND_ACK_FAIL_ID_BIT | SEND_FAIL_BIT | SEND_COMPLETE_BIT | SEND_ACK_PENDING_BIT);
            // 判断ID是否需要更改，如果对方的ID更大，说明ACK丢包，则丢包后更新ID
            free(packet);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG_MESSAGER, "packet %i ACK timeout", packet->pack_id);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_ACK_PENDING_BIT | SEND_ACK_OK_BIT_INTERNAL | SEND_ACK_FAIL_BIT);
            // 重传
            continue;
        }
    }
    // 超时重传次数用完
    xEventGroupSetBits(send_state, SEND_ACK_TIMEOUT_BIT);
    xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_ACK_OK_BIT_INTERNAL | SEND_ACK_FAIL_BIT);
    ESP_LOGE(TAG_MESSAGER, "packet %i send failed", packet->pack_id);
    free(packet);
    return ESP_FAIL;
}

esp_err_t ESPNowProtocol::send_ACK(uint16_t packet_tx_id, uint8_t *dest_mac, PacketType type)
{
    bool mutex_flag = true;
    // 获取互斥锁
    if (xSemaphoreTake(tx_semaphore, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS) != pdTRUE)
    {
        ESP_LOGE(TAG_MESSAGER, "Failed to take tx_semaphore, forbid operation to EventGroup");
        mutex_flag = false; // 互斥锁获取失败，不进行互斥操作，进行风险发送操作
    }
    ACK_OK_pack_t *packet = (ACK_OK_pack_t *)malloc(sizeof(ACK_OK_pack_t));
    packet->header = (type == ACK_OK) ? POWERRUNE_ACK_OK_HEADER : POWERRUNE_ACK_FAIL_HEADER;
    packet->pack_id = packet_tx_id;
    memcpy(packet->dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
    // send
    esp_err_t err = esp_now_send(packet->dest_mac, (uint8_t *)packet, sizeof(ACK_OK_pack_t) - 6);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send error %s", type, packet_tx_id, esp_err_to_name(err));
        free(packet);
        return err;
    }
    if (mutex_flag)
    {
        // 等待发送成功或失败
        xEventGroupWaitBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT, pdFALSE, pdFALSE, CONFIG_ESPNOW_TIMEOUT / portTICK_PERIOD_MS);
        if (xEventGroupGetBits(send_state) & SEND_COMPLETE_BIT)
        {
            ESP_LOGD(TAG_MESSAGER, "ACK %i packet %i send success", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT);
        }
        else if (xEventGroupGetBits(send_state) & SEND_FAIL_BIT)
        {
            ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send error in tx callback", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_FAIL_BIT);
            free(packet);
            return ESP_FAIL;
        }
        else
        {
            ESP_LOGE(TAG_MESSAGER, "ACK %i packet %i send timeout", type, packet_tx_id);
            xEventGroupClearBits(send_state, SEND_COMPLETE_BIT | SEND_FAIL_BIT);
            free(packet);
            return ESP_FAIL;
        }
    }
    free(packet);
    // 释放互斥锁
    if (mutex_flag)
        xSemaphoreGive(tx_semaphore);
    return ESP_OK;
}

void ESPNowProtocol::tx_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        xEventGroupSetBitsFromISR(send_state, SEND_COMPLETE_BIT, &xHigherPriorityTaskWoken);
        ESP_LOGD(TAG_MESSAGER, "Send packet successfully in tx callback");
    }
    else
    {
        xEventGroupSetBitsFromISR(send_state, SEND_FAIL_BIT, &xHigherPriorityTaskWoken);
        ESP_LOGE(TAG_MESSAGER, "Sending data to " MACSTR " failed", MAC2STR(mac_addr));
    }
}

void ESPNowProtocol::tx_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // 允许阻塞整个事件循环，事件循环本身就是消息队列

    // 地址判断
    // data的第一个变量是地址位
    if (peer_list_established)
    {
        // 忙锁，对外可见
        xEventGroupSetBits(send_state, SEND_BUSY);
#if CONFIG_POWERRUNE_TYPE == 1 // 主设备
        uint8_t address = ((uint8_t *)event_data)[0];
        if (address > 5)
        {
            if (address != 0xFF && address != 0x06)
                ESP_LOGE(TAG_MESSAGER, "Address %i out of range", address);
            else
                ESP_LOGW(TAG_MESSAGER, "Send skipped");
            return;
        }
        uint8_t *dest_mac = mac_addr[address];
        send_data(dest_mac, event_base, event_id, event_data, ((uint8_t *)event_data)[1]);

#endif
#if ((CONFIG_POWERRUNE_TYPE == 2) || (CONFIG_POWERRUNE_TYPE == 0)) // 从设备
        uint8_t *dest_mac = mac_addr;
        send_data(dest_mac, event_base, event_id, event_data, ((uint8_t *)event_data)[1]);

#endif
        // 解除忙锁
        xEventGroupClearBits(send_state, SEND_BUSY);
    }
    else
    {
        ESP_LOGE(TAG_MESSAGER, "Peer list not established, cannot send data");
        return;
    }
}
#if CONFIG_POWERRUNE_TYPE == 1
void ESPNowProtocol::reset_id_PRA_HIT_handler(void *event_handler_arg,
                                              esp_event_base_t event_base,
                                              int32_t event_id,
                                              void *event_data)
{
    reset_armour_id_t *reset_armour_id_data = (reset_armour_id_t *)event_handler_arg;
    PRA_HIT_EVENT_DATA *pra_hit_event_data = (PRA_HIT_EVENT_DATA *)event_data;
    if (pra_hit_event_data->address > 4)
    {
        ESP_LOGE(TAG_MESSAGER, "[Reset ID] Address %i out of range", pra_hit_event_data->address);
        return;
    }

    ESP_LOGI(TAG_MESSAGER, "[Reset ID] Hit event received from address %i", pra_hit_event_data->address);

    EventBits_t n = xEventGroupGetBits(reset_armour_id_data->event_group);
    // 数已激活设备数
    uint8_t count = 0;
    while (n)
    {
        count++;
        n &= (n - 1);
    }
    // 判断是否重复
    if (xEventGroupGetBits(reset_armour_id_data->event_group) & (1 << pra_hit_event_data->address))
    {
        ESP_LOGW(TAG_MESSAGER, "[Reset ID] Hit event from address %i repeated", pra_hit_event_data->address);
        // 更新ID
        reset_armour_id_data->rx_id_new[count] = packet_rx_id[pra_hit_event_data->address];
        reset_armour_id_data->tx_id_new[count] = packet_tx_id[pra_hit_event_data->address];
        return;
    }
    // 写入新MAC
    memcpy(reset_armour_id_data->mac_addr_new + count, mac_addr[pra_hit_event_data->address], ESP_NOW_ETH_ALEN);
    // 写入新Config
    memcpy(reset_armour_id_data->armour_config + count, config->get_config_armour_info_pt(pra_hit_event_data->address), sizeof(PowerRune_Armour_config_info_t));
    reset_armour_id_data->armour_config[count].armour_id = count + 1; // 1 2 3 4 5
    // 写入新ID
    reset_armour_id_data->rx_id_new[count] = packet_rx_id[pra_hit_event_data->address];
    reset_armour_id_data->tx_id_new[count] = packet_tx_id[pra_hit_event_data->address];
    xEventGroupSetBits(reset_armour_id_data->event_group, 1 << pra_hit_event_data->address); // 0 1 2 3 4
    ESP_LOGD(TAG_MESSAGER, "[Reset ID] Hit bits: %i, counts %i", (int)xEventGroupGetBits(reset_armour_id_data->event_group), count);
    if (count == 4)
        esp_event_handler_unregister_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, reset_id_PRA_HIT_handler);
}

esp_err_t ESPNowProtocol::reset_armour_id()
{
    // DEBUG TODO
    // return ESP_OK;
    ESP_LOGI(TAG_MESSAGER, "[Reset ID] Resetting PowerRune Armour ID...");

    // 事件位和任务通知序列
    EventGroupHandle_t reset_armour_id_event_group = xEventGroupCreate();
    reset_armour_id_t reset_armour_id_data;
    reset_armour_id_data.event_group = reset_armour_id_event_group;
    // 交换配置信息
    PowerRune_Armour_config_info_t *config_info = new PowerRune_Armour_config_info_t[5];
    reset_armour_id_data.armour_config = config_info;

    // 注册所需事件
    esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, tx_event_handler, NULL);
    esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, tx_event_handler, NULL);
    esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, tx_event_handler, NULL);
    esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, reset_id_PRA_HIT_handler, &reset_armour_id_data);

    // 遍历address_old中的五个armour_id
    PRA_START_EVENT_DATA pra_start_event_data;
    for (size_t i = 0; i < 5; i++)
    {
        pra_start_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_START_EVENT, &pra_start_event_data, sizeof(PRA_START_EVENT_DATA), portMAX_DELAY);
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    // 等待全部击中，0b11111
    xEventGroupWaitBits(reset_armour_id_event_group, 0b11111, pdTRUE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_MESSAGER, "[Reset ID] All PowerRune Armour ID hit");
    // 写入新MAC、新ID和新配置
    memcpy(mac_addr, reset_armour_id_data.mac_addr_new, 5 * ESP_NOW_ETH_ALEN);
    update_mac_to_address_map();
    memcpy(packet_rx_id, reset_armour_id_data.rx_id_new, 5 * sizeof(uint16_t));
    memcpy(packet_tx_id, reset_armour_id_data.tx_id_new, 5 * sizeof(uint16_t));

    CONFIG_EVENT_DATA pra_config_event_data;
    memcpy(&pra_config_event_data.config_common_info, config->get_config_common_info_pt(), sizeof(PowerRune_Common_config_info_t));

    for (size_t i = 0; i < 5; i++)
    {
        memcpy(config->get_config_armour_info_pt(i), reset_armour_id_data.armour_config + i, sizeof(PowerRune_Armour_config_info_t));
        pra_config_event_data.address = i;
        memcpy(&pra_config_event_data.config_armour_info, reset_armour_id_data.armour_config + i, sizeof(PowerRune_Armour_config_info_t));
        // 发送CONFIG_EVENT
        esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_EVENT, &pra_config_event_data, sizeof(CONFIG_EVENT_DATA), portMAX_DELAY);
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }

    ESP_LOGI(TAG_MESSAGER, "[Reset ID] PowerRune Armour ID reset done");
    // 给所有设备发送PRA_STOP_EVENT
    PRA_STOP_EVENT_DATA pra_stop_event_data;
    for (size_t i = 0; i < 5; i++)
    {
        pra_stop_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    // 释放内存
    delete[] config_info;
    vEventGroupDelete(reset_armour_id_event_group);

    return ESP_OK;
}
#endif
esp_err_t ESPNowProtocol::establish_peer_list(uint8_t *response_mac)
{
    ESP_LOGI(TAG_MESSAGER, "Establishing peer list");
    esp_now_peer_info_t *peer = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG_MESSAGER, "Malloc broadcast peer information fail");
        return ESP_FAIL;
    }
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
    /* Add broadcast peer information to peer list. */
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
#else
    bool reset_id = false;
#endif
    // 等待RESPONSE_EVENT或者PING_EVENT
    received_struct *received_data = (received_struct *)malloc(sizeof(received_struct));
    while (1)
    {
#ifdef HEAPDEBUG
        heap_caps_check_integrity_all(1);
#endif
        // 从设备只需要向0xFFFFFFFFFFFF发送PING_EVENT广播包即可，直到收到RESPONSE_EVENT包，然后建立peer list
// PING EVENT 数据
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
        ESP_LOGD(TAG_MESSAGER, "Sending PING_EVENT to " MACSTR, MAC2STR(broadcast_mac));
        int32_t event_id = PRA_PING_EVENT;
        PRA_PING_EVENT_DATA ping_data;
        // 如果已经设置ID，则广播ID，否则维持0xFF
        const PowerRune_Armour_config_info_t *config_info = config->get_config_info_pt();
        if (config_info->armour_id != 0xFF)
            ping_data.address = config_info->armour_id - 1;
        else
            ping_data.address = 0xFF;
        memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Armour_config_info_t));
        send_data(broadcast_mac, PRA, event_id, &ping_data, sizeof(PRA_PING_EVENT_DATA), 0);

#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
        ESP_LOGD(TAG_MESSAGER, "Sending PING_EVENT to " MACSTR, MAC2STR(broadcast_mac));
        int32_t event_id = PRM_PING_EVENT;
        PRM_PING_EVENT_DATA ping_data;
        memcpy(&ping_data.config_info, config->get_config_info_pt(), sizeof(PowerRune_Motor_config_info_t));
        send_data(broadcast_mac, PRM, event_id, &ping_data, sizeof(PRM_PING_EVENT_DATA), 0);
#endif
        // 接收RESPONSE_EVENT或者PING_EVENT
        // 随机数乱序等待
        if (xQueueReceive(espnow_rx_queue, received_data, esp_random() % 3000 + 2000 / portTICK_PERIOD_MS) != pdTRUE)
        {
            continue;
        }
        // 小端数据
        uint16_t header = (received_data->data[1] << 8) | received_data->data[0];
        ESP_LOGD(TAG_MESSAGER, "Received %i bytes from " MACSTR, received_data->len, MAC2STR(received_data->src_MAC));
        if (header == POWERRUNE_DATA_HEADER)
        {
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
            // 解包
            espnow_DATA_pack_t packet = {0};
            memcpy(packet.event_base, received_data->data + 6, 4);
            packet.event_id = received_data->data[12];
            packet.pack_id = (received_data->data[3] << 8) | received_data->data[2];
            log_packet(&packet);
            if (memcmp(packet.event_base, PRC, 4) != 0 || packet.event_id != RESPONSE_EVENT)
            {
                ESP_LOGE(TAG_MESSAGER, "RESPONSE packet %i event check failed", packet.pack_id);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                // 释放内存
                free(received_data->data);
                continue;
            }
            ESP_LOGI(TAG_MESSAGER, "Received RESPONSE_EVENT from " MACSTR, MAC2STR(received_data->src_MAC));
            // 建立peer list，此处peer已经在前述代码中分配内存
            memset(peer, 0, sizeof(esp_now_peer_info_t));
            peer->channel = CONFIG_ESPNOW_CHANNEL;
            peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
            peer->encrypt = false;
            memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
            memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
            ESP_ERROR_CHECK(esp_now_add_peer(peer));
            // 写入mac_addr
            memcpy(mac_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
            // 更新ID
            packet_rx_id = packet.pack_id - 1;
#elif CONFIG_POWERRUNE_TYPE == 1 // RLOGO
            // 解包
            espnow_DATA_pack_t packet = {0};
            packet.header = header;
            packet.pack_id = (received_data->data[3] << 8) | received_data->data[2];
            memcpy(packet.event_base, received_data->data + 6, 4);
            if (memcmp(packet.event_base, PRA, 4) != 0 && memcmp(packet.event_base, PRM, 4) != 0)
            {
                ESP_LOGE(TAG_MESSAGER, "PING packet %i event_base check failed", packet.pack_id);
                // 释放内存
                free(received_data->data);
                continue;
            }
            packet.event_data_len = (received_data->data[11] << 8) | received_data->data[10];
            packet.event_id = received_data->data[12];
            if (packet.event_id != PRA_PING_EVENT && packet.event_id != PRM_PING_EVENT)
            {
                ESP_LOGE(TAG_MESSAGER, "PING packet %i event_base check failed", packet.pack_id);
                // 释放内存
                free(received_data->data);
                continue;
            }
            memcpy(packet.event_data, received_data->data + 13, packet.event_data_len);
            // CRC校验
            uint16_t crc16 = esp_crc16_le(UINT16_MAX, (uint8_t *)&packet, 13 + packet.event_data_len);
            uint16_t crc16_pack = (received_data->data[5] << 8) | received_data->data[4];
            log_packet(&packet, crc16_pack);
            log_raw_packet((uint8_t *)&packet, 13 + packet.event_data_len);
            if (crc16 != crc16_pack)
            {
                ESP_LOGE(TAG_MESSAGER, "PING packet %i CRC16 0x%04X check failed", packet.pack_id, crc16);
                // 释放内存
                free(received_data->data);
                continue;
            }

            // 识别地址
            if ((packet.event_data[0] <= ARMOUR5 || packet.event_data[0] == 0xFF))
            {
                // 地址已知，而且有空位存放
                if (packet.event_data[0] != 0xFF && memcmp(mac_addr[packet.event_data[0]], NULL_mac, ESP_NOW_ETH_ALEN) == 0)
                {
                    // 建立peer list，此处peer已经在前述代码中分配内存
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
                    peer->encrypt = false;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
                    esp_err_t err = esp_now_add_peer(peer);
                    if (err == ESP_OK)
                    {
                        ESP_LOGI(TAG_MESSAGER, "Peer added");
                        memcpy(mac_addr[packet.event_data[0]], received_data->src_MAC, ESP_NOW_ETH_ALEN);
                        update_mac_to_address_map();
                    }
                    else
                    {
                        ESP_LOGE(TAG_MESSAGER, "Peer add failed: %s", esp_err_to_name(err));
                        free(received_data->data);
                        continue;
                    }
                    // 更新ID
                    packet_rx_id[packet.event_data[0]] = packet.pack_id - 1;
                    // 保存设备配置
                    memcpy(config->get_config_armour_info_pt(packet.event_data[0]), packet.event_data + 2, sizeof(PowerRune_Armour_config_info_t));
                    ESP_LOGD(TAG_MESSAGER, "Config info of armour %i:", packet.event_data[0]);
                    esp_log_buffer_hex(TAG_MESSAGER, config->get_config_armour_info_pt(packet.event_data[0]), sizeof(PowerRune_Armour_config_info_t));
                    // 发送RESPONSE_EVENT
                    int32_t event_id = RESPONSE_EVENT;
                    send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                }
                // 地址已知，但没有空位，说明ID设置有问题，在mac_addr中找到空位，重新建立。注意防止连续收到两个地址一样的。
                else if (packet.event_data[0] == 0xFF || memcmp(mac_addr[packet.event_data[0]], received_data->src_MAC, ESP_NOW_ETH_ALEN) != 0)
                {
                    // 寻找空位
                    int8_t first_empty = -1;
                    for (int8_t i = 0; i < 5; i++) // TODO，改成设备数
                    {                              // 寻找空位的同时确认这个MAC没有被添加过。
                        if (memcmp(mac_addr[i], received_data->src_MAC, ESP_NOW_ETH_ALEN) == 0)
                        {
                            // 添加过该设备
                            first_empty = -1;
                            break;
                        }
                        if (first_empty == -1 && memcmp(mac_addr[i], NULL_mac, ESP_NOW_ETH_ALEN) == 0)
                        {
                            // 未添加过，而且这个时候还没有找到空位
                            first_empty = i;
                            break;
                        }
                    }
                    // 如果没有空位或者重复，丢弃该包
                    if (first_empty == -1)
                    {
                        ESP_LOGE(TAG_MESSAGER, "PING packet %i address check failed", packet.pack_id);
                        // 发送RESPONSE_EVENT
                        int32_t event_id = RESPONSE_EVENT;
                        send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                        free(received_data->data); // 释放内存
                        continue;
                    }
                    // 置位重新设置ID
                    reset_id = true;
                    if (reset_id == false)
                        ESP_LOGI(TAG_MESSAGER, "Reset ID flag set");

                    // 建立peer list，此处peer已经在前述代码中分配内存
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
                    peer->encrypt = false;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);

                    esp_err_t err = esp_now_add_peer(peer);
                    if (err == ESP_OK)
                    {
                        ESP_LOGI(TAG_MESSAGER, "Peer added");
                        memcpy(mac_addr[first_empty], received_data->src_MAC, ESP_NOW_ETH_ALEN);
                        update_mac_to_address_map();
                    }
                    else
                    {
                        ESP_LOGE(TAG_MESSAGER, "Peer add failed: %s", esp_err_to_name(err));
                        // 发送RESPONSE_EVENT
                        int32_t event_id = RESPONSE_EVENT;
                        send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                        free(received_data->data);
                        continue;
                    }
                    // 更新ID
                    packet_rx_id[first_empty] = packet.pack_id - 1;
                    // 保存设备配置
                    memcpy(config->get_config_armour_info_pt(first_empty), packet.event_data + 2, sizeof(PowerRune_Armour_config_info_t));
                    ESP_LOGD(TAG_MESSAGER, "[unreset] Config info of armour %i:", first_empty);
                    esp_log_buffer_hex(TAG_MESSAGER, config->get_config_armour_info_pt(first_empty), sizeof(PowerRune_Armour_config_info_t));

                    // 发送RESPONSE_EVENT
                    int32_t event_id = RESPONSE_EVENT;
                    send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                }
            }
            else if (packet.event_data[0] == MOTOR && memcmp(mac_addr[MOTOR], NULL_mac, ESP_NOW_ETH_ALEN) == 0) // Address
            {
                // 建立peer list，此处peer已经在前述代码中分配内存
                memset(peer, 0, sizeof(esp_now_peer_info_t));
                peer->channel = CONFIG_ESPNOW_CHANNEL;
                peer->ifidx = (wifi_interface_t)ESP_IF_WIFI_STA;
                peer->encrypt = false;
                memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                memcpy(peer->peer_addr, received_data->src_MAC, ESP_NOW_ETH_ALEN);
                esp_err_t err = esp_now_add_peer(peer);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG_MESSAGER, "Peer added");
                    memcpy(mac_addr[MOTOR], received_data->src_MAC, ESP_NOW_ETH_ALEN);
                    update_mac_to_address_map();
                }
                else
                {
                    ESP_LOGE(TAG_MESSAGER, "Peer add failed: %s", esp_err_to_name(err));
                    // 发送RESPONSE_EVENT
                    int32_t event_id = RESPONSE_EVENT;
                    send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                    free(received_data->data);
                    continue;
                }
                // 更新ID
                packet_rx_id[MOTOR] = packet.pack_id - 1;
                // 保存设备配置
                memcpy(config->get_config_motor_info_pt(), packet.event_data + 2, sizeof(PowerRune_Motor_config_info_t));
                ESP_LOGD(TAG_MESSAGER, "Config info of motor:");
                esp_log_buffer_hex(TAG_MESSAGER, config->get_config_motor_info_pt(), sizeof(PowerRune_Motor_config_info_t));
                // 发送RESPONSE_EVENT
                int32_t event_id = RESPONSE_EVENT;
                send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
            }
            else
            {
                ESP_LOGE(TAG_MESSAGER, "PING packet %i address check failed", packet.pack_id);
                free(received_data->data);
                // 发送RESPONSE_EVENT
                int32_t event_id = RESPONSE_EVENT;
                send_data(received_data->src_MAC, PRC, event_id, NULL, 0, 0);
                continue;
            }
            // 检查是否完成设置
            // 正式上线版本
            if (memcmp(mac_addr[0], NULL_mac, ESP_NOW_ETH_ALEN) != 0 &&
                memcmp(mac_addr[1], NULL_mac, ESP_NOW_ETH_ALEN) != 0 &&
                memcmp(mac_addr[2], NULL_mac, ESP_NOW_ETH_ALEN) != 0 &&
                memcmp(mac_addr[3], NULL_mac, ESP_NOW_ETH_ALEN) != 0 &&
                memcmp(mac_addr[4], NULL_mac, ESP_NOW_ETH_ALEN) != 0 &&
                memcmp(mac_addr[5], NULL_mac, ESP_NOW_ETH_ALEN) != 0)
            // DEBUG版本
            // if (memcmp(mac_addr[5], NULL_mac, ESP_NOW_ETH_ALEN) != 0)
            {
                break;
            }

#endif

            free(received_data->data);
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
            break;
#endif
        }
        else
        {
            // 收到未知包，丢弃
            ESP_LOGE(TAG_MESSAGER, "Unknown packet header: 0x%04X", header);
            // 释放内存
            free(received_data->data);
            continue;
        }
    }
    // 释放内存
    free(peer);

#if CONFIG_POWERRUNE_TYPE == 1
    // 等待5s并清空队列，丢弃多余的PING包
    while (xQueueReceive(espnow_rx_queue, received_data, 5000 / portTICK_PERIOD_MS) == pdTRUE)
    {
        free(received_data->data);
        send_data(received_data->src_MAC, PRC, RESPONSE_EVENT, NULL, 0, 0);
    }
#endif
    free(received_data);
    ESP_LOGD(TAG_MESSAGER, "Peer list established");
    peer_list_established = true;
#if CONFIG_POWERRUNE_TYPE == 1
    if (reset_id)
    {
        return ESP_ERR_INVALID_STATE;
    }
#endif
    return ESP_OK;
}

#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
/**
 * @brief beacon控制器
 * @param mode 0: 关闭beacon 1: 开启beacon
 */
esp_err_t ESPNowProtocol::beacon_control(uint8_t mode)
{
    if (mode == 0)
    {
        esp_event_handler_unregister_with(pr_events_loop_handle, PRC, BEACON_TIMEOUT_EVENT, beacon_timeout_handler);
        vTaskDelete(beacon_task_handle);
        ESP_LOGI(TAG_MESSAGER, "Beacon disabled");
    }
    else if (mode == 1)
    {
        beacon_time = xTaskGetTickCount();
        beacon_send_time = xTaskGetTickCount();
        xTaskCreate(beacon_task, "beacon_task", 4096, NULL, 1, &beacon_task_handle);
        esp_event_handler_register_with(pr_events_loop_handle, PRC, BEACON_TIMEOUT_EVENT, beacon_timeout_handler, NULL);
        ESP_LOGI(TAG_MESSAGER, "Beacon enabled");
    }
    else
    {
        ESP_LOGE(TAG_MESSAGER, "Unknown mode");
        return ESP_FAIL;
    }
    return ESP_OK;
}
#endif

ESPNowProtocol::ESPNowProtocol(esp_event_handler_t beacon_timeout)
{
    // 初始化espnow，PMK和LMK
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    // set pmk
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));
    // 获取自身MAC
    esp_wifi_get_mac((wifi_interface_t)ESP_IF_WIFI_STA, self_mac);
    espnow_rx_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(received_struct));
    send_state = xEventGroupCreate();
    assert(espnow_rx_queue != NULL);
    esp_now_register_recv_cb(rx_callback);
    esp_now_register_send_cb(tx_callback);
// establish peer list: 发送/接收PING_EVENT广播包，然后接收/回复RESPONSE_EVENT包
#if CONFIG_POWERRUNE_TYPE == 1
    esp_err_t err = establish_peer_list();
#else
    establish_peer_list();
#endif
// led闪烁
#if CONFIG_POWERRUNE_TYPE == 1
    led->set_mode(LED_MODE_FADE, 0);
#endif
    // 创建解析数据任务
    xTaskCreate(parse_data_task, "parse_data_task", 4096, NULL, 6, NULL);
#if CONFIG_POWERRUNE_TYPE == 1 // 主控
    // Reset ID
    if (err == ESP_ERR_INVALID_STATE)
        reset_armour_id();
#endif
        // 创建Beacon任务
#if CONFIG_POWERRUNE_TYPE == 0 || CONFIG_POWERRUNE_TYPE == 2 // Armour || Motor
    assert(beacon_timeout != NULL);
    beacon_timeout_handler = beacon_timeout;
    // beacon_control(1);
#endif
}
