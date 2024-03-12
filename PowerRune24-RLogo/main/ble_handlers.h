/**
 * @file "ble_handlers.h"
 * @note 本文件存放蓝牙事件回调函数
 * @version 1.0
 * @date 2024-02-19
 */
#pragma once
#ifndef _BLE_HANDLERS_H_
#define _BLE_HANDLERS_H_
#include "main.h"

const char *TAG_BLE = "BLE";

// 函数作用；找到handle对应的index
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

// GATTS最终的回调函数
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// GATTS的回调函数
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(TAG_BLE, "\nGATTS事件回调\n\n");
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
                    ESP_LOGD(TAG_BLE, "去往profile\n");
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
    ESP_LOGD(TAG_BLE, "从profile回来\n");
}

// GAP的回调函数
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(TAG_BLE, "GAP的回调函数\n");
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
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));

        break;
    default:
        break;
    }
}
#endif