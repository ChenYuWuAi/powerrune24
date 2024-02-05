/**
 * @file "ble_handlers.h"
 * @note 本文件存放蓝牙事件回调函数
 */
#pragma once
#include "main.h"

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

static void ota_write_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    xTaskCreate((TaskFunction_t)ota_task, "ota_task", 4096, NULL, 10, NULL);
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

// GATTS最终的回调函数
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        // 注册事件
        esp_ble_gap_set_device_name(DEVICE_NAME);
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
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
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

// GAP的回调函数
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
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
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));

        break;
    default:
        break;
    }
}