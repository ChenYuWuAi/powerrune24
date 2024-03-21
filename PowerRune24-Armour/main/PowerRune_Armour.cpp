/**
 * @file PowerRune_Armour.cpp
 * @brief 装甲板类
 * @version 0.2
 * @date 2024-02-19
 * @note 装甲板类，用于装甲板控制
 */
#include "PowerRune_Armour.h"
static const char *TAG_ARMOUR = "Armour";
// 变量初始化
LED_Strip *PowerRune_Armour::led_strip[5];
SemaphoreHandle_t PowerRune_Armour::ISR_mutex = xSemaphoreCreateBinary();
DEMUX PowerRune_Armour::demux_led = DEMUX(DEMUX_IO, DEMUX_IO_enable);
TaskHandle_t PowerRune_Armour::LED_update_task_handle;
SemaphoreHandle_t PowerRune_Armour::LED_Strip_FSM_Semaphore;
LED_Strip_FSM_t PowerRune_Armour::state;

void PowerRune_Armour::clear_armour(bool refresh)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        demux_led = i;
        led_strip[i]->clear_pixels();
        if (refresh)
            led_strip[i]->refresh();
    }
}

// LED更新任务
void PowerRune_Armour::LED_update_task(void *pvParameter)
{
    // 状态
    LED_Strip_FSM_t state_task;
    const PowerRune_Armour_config_info_t *config_info;
    while (1)
    {
        // 状态机，状态转移：IDLE->TARGET->HIT->BLINK->IDLE，TARGET->HIT和BLINK->IDLE过程之后task阻塞，等待信号量
        switch (state.LED_Strip_State)
        {
        case LED_STRIP_IDLE:
            clear_armour();

            // 等待信号量
            xSemaphoreTake(LED_Strip_FSM_Semaphore, portMAX_DELAY);
            // 转移状态
            state_task = state;
            break;
        case LED_STRIP_TARGET:
        {
            clear_armour(false);
            config_info = config->get_config_info_pt();

            // 点亮靶状图案、上下装甲板，灯臂刷新一次
            demux_led = LED_STRIP_MAIN_ARMOUR;
            if (state_task.color == PR_RED)
            {
                for (uint16_t i = 0; i < sizeof(target_pic) / sizeof(uint16_t); i++)
                {
                    led_strip[LED_STRIP_MAIN_ARMOUR]->set_color_index(target_pic[i], config_info->brightness, 0, 0);
                }
            }
            else
            {
                for (uint16_t i = 0; i < sizeof(target_pic) / sizeof(uint16_t); i++)
                {
                    led_strip[LED_STRIP_MAIN_ARMOUR]->set_color_index(target_pic[i], 0, 0, config_info->brightness);
                }
            }
            led_strip[LED_STRIP_MAIN_ARMOUR]->refresh();
            demux_led = LED_STRIP_UPPER;
            led_strip[LED_STRIP_UPPER]->set_color(state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
            led_strip[LED_STRIP_UPPER]->refresh();
            demux_led = LED_STRIP_LOWER;
            led_strip[LED_STRIP_LOWER]->set_color(state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
            led_strip[LED_STRIP_LOWER]->refresh();
            demux_led = LED_STRIP_ARM;
            led_strip[LED_STRIP_ARM]->refresh();
            // 开启矩阵流水灯
            uint8_t i = 0;
            do
            {
                TickType_t xLastWakeTime = xTaskGetTickCount();

                demux_led = LED_STRIP_MATRIX;

                if (state_task.color == PR_RED)
                    for (uint16_t j = 0; j < 165; j++)
                    {
                        led_strip[LED_STRIP_MATRIX]->set_color_index(j, single_arrow[(j + i * 5) % 25] * config_info->brightness, 0, 0);
                    }
                else
                    for (uint16_t j = 0; j < 165; j++)
                    {
                        led_strip[LED_STRIP_MATRIX]->set_color_index(j, 0, 0, single_arrow[(j + i * 5) % 25] * config_info->brightness);
                    }
                led_strip[LED_STRIP_MATRIX]->refresh();
                i = (i + 1) % 5;
                vTaskDelayUntil(&xLastWakeTime, MATRIX_REFRESH_PERIOD / portTICK_PERIOD_MS);

            } while (xSemaphoreTake(LED_Strip_FSM_Semaphore, 0) == pdFALSE);
            // 信号量释放后，重新加载state_task
            state_task = state;
            break;
        }
        case LED_STRIP_HIT:
        {
            config_info = config->get_config_info_pt();
            // 命中图案，大符为对应环数（10环特殊，点亮），小符为1环
            switch (state_task.mode)
            {
            case PRA_RUNE_BIG_MODE:
                demux_led = LED_STRIP_MAIN_ARMOUR;
                led_strip[LED_STRIP_MAIN_ARMOUR]->clear_pixels();
                for (uint16_t j = hit_ring_cutoff[state_task.score - 1]; j < hit_ring_cutoff[state_task.score]; j++)
                    led_strip[LED_STRIP_MAIN_ARMOUR]->set_color_index(j, state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);

                if (state_task.score == 10)
                {
                    // 点亮1，3，5，7，9环
                    for (uint16_t i = 0; i < 5; i++)
                        for (uint16_t j = hit_ring_cutoff[i * 2]; j < hit_ring_cutoff[i * 2 + 1]; j++)
                            led_strip[LED_STRIP_MAIN_ARMOUR]->set_color_index(j, state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
                }
                led_strip[LED_STRIP_MAIN_ARMOUR]->refresh();
                demux_led = LED_STRIP_ARM;
                led_strip[LED_STRIP_ARM]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_edge : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_edge);
                led_strip[LED_STRIP_ARM]->refresh();
                demux_led = LED_STRIP_MATRIX;
                led_strip[LED_STRIP_MATRIX]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_matrix : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_matrix);
                led_strip[LED_STRIP_MATRIX]->refresh();
                // 等待信号量
                xSemaphoreTake(LED_Strip_FSM_Semaphore, portMAX_DELAY);
                // 转移状态
                state_task = state;
                break;
            case PRA_RUNE_SMALL_MODE:
                demux_led = LED_STRIP_MAIN_ARMOUR;
                led_strip[LED_STRIP_MAIN_ARMOUR]->clear_pixels();
                for (uint16_t j = hit_ring_cutoff[0]; j < hit_ring_cutoff[1]; j++)
                {
                    led_strip[LED_STRIP_MAIN_ARMOUR]->set_color_index(j, state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
                }
                led_strip[LED_STRIP_MAIN_ARMOUR]->refresh();
                demux_led = LED_STRIP_ARM;
                led_strip[LED_STRIP_ARM]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_edge : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_edge);
                led_strip[LED_STRIP_ARM]->refresh();
                demux_led = LED_STRIP_MATRIX;
                led_strip[LED_STRIP_MATRIX]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_matrix : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_matrix);
                led_strip[LED_STRIP_MATRIX]->refresh();
                // 等待信号量
                xSemaphoreTake(LED_Strip_FSM_Semaphore, portMAX_DELAY);
                // 转移状态
                state_task = state;
                break;
            }
            break;
        }
        case LED_STRIP_BLINK:
        {

            config_info = config->get_config_info_pt();
            // 按ID进行同步化延迟
            vTaskDelay((BLINK_DELAY * (6 - config_info->armour_id)) / portTICK_PERIOD_MS);
            // UPPER，LOWER，MATRIX，ARM闪烁十次，MAIN_ARMOUR不闪烁
            for (uint8_t i = 0; i < 10; i++)
            {
                demux_led = LED_STRIP_UPPER;
                led_strip[LED_STRIP_UPPER]->clear_pixels();
                led_strip[LED_STRIP_UPPER]->refresh();
                demux_led = LED_STRIP_LOWER;
                led_strip[LED_STRIP_LOWER]->clear_pixels();
                led_strip[LED_STRIP_LOWER]->refresh();
                demux_led = LED_STRIP_MATRIX;
                led_strip[LED_STRIP_MATRIX]->clear_pixels();
                led_strip[LED_STRIP_MATRIX]->refresh();
                demux_led = LED_STRIP_ARM;
                led_strip[LED_STRIP_ARM]->clear_pixels();
                led_strip[LED_STRIP_ARM]->refresh();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                demux_led = LED_STRIP_UPPER;
                led_strip[LED_STRIP_UPPER]->set_color(state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
                led_strip[LED_STRIP_UPPER]->refresh();
                demux_led = LED_STRIP_LOWER;
                led_strip[LED_STRIP_LOWER]->set_color(state_task.color == PR_RED ? config_info->brightness : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness);
                led_strip[LED_STRIP_LOWER]->refresh();
                demux_led = LED_STRIP_MATRIX;
                led_strip[LED_STRIP_MATRIX]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_matrix : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_matrix);
                led_strip[LED_STRIP_MATRIX]->refresh();
                demux_led = LED_STRIP_ARM;
                led_strip[LED_STRIP_ARM]->set_color(state_task.color == PR_RED ? config_info->brightness_proportion_edge : 0, 0, state_task.color == PR_RED ? 0 : config_info->brightness_proportion_edge);
                led_strip[LED_STRIP_ARM]->refresh();
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            // 等待信号量
            xSemaphoreTake(LED_Strip_FSM_Semaphore, portMAX_DELAY);
            // 转移状态
            state_task = state;
            break;
        }
        }
    }
    vTaskDelete(NULL);
}

void IRAM_ATTR PowerRune_Armour::GPIO_ISR_handler(void *arg)
{
    // 操作过程中激活互斥锁，屏蔽其他中断
    if (xSemaphoreTake(ISR_mutex, 0) == pdFALSE)
        return;
    uint8_t io = (*(uint8_t *)arg);
    // 发送事件
    PRA_HIT_EVENT_DATA hit_event_data;
    hit_event_data.address = config->get_config_info_pt()->armour_id - 1;
    hit_event_data.score = io;
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_HIT_EVENT, &hit_event_data, sizeof(PRA_HIT_EVENT_DATA), portMAX_DELAY);
    xTaskCreate(restart_ISR_task, "restart_ISR_task", 4096, NULL, 5, NULL);
}

void PowerRune_Armour::GPIO_init()
{
    // 初始化GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    for (uint8_t i = 0; i < 10; i++)
    {
        io_conf.pin_bit_mask = (1ULL << TRIGGER_IO[i]);
        gpio_config(&io_conf);
    }
}

void PowerRune_Armour::GPIO_ISR_enable()
{
    // 初始化GPIO ISR
    for (uint8_t i = 0; i < 10; i++)
    {
        gpio_set_intr_type(TRIGGER_IO[i], GPIO_INTR_NEGEDGE);
    }
}

void PowerRune_Armour::restart_ISR_task(void *pvParameter)
{
    // 屏蔽1s
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // 释放信号量
    xSemaphoreGive(ISR_mutex);
    vTaskDelete(NULL);
}

// Class PowerRune_Armour 定义
PowerRune_Armour::PowerRune_Armour()
{
    // 初始化GPIO
    GPIO_init();
    // 开启ISR服务
    gpio_install_isr_service(0);
    for (uint8_t i = 0; i < 10; i++)
    {
        gpio_isr_handler_add(TRIGGER_IO[i], GPIO_ISR_handler, (void *)&TRIGGER_IO_TO_SCORE[i]);
    }

    GPIO_ISR_enable();
    // 初始化LED_Strip
    led_strip[LED_STRIP_MAIN_ARMOUR] = new LED_Strip(STRIP_IO, 301);
    led_strip[LED_STRIP_UPPER] = new LED_Strip(STRIP_IO, 86);
    led_strip[LED_STRIP_LOWER] = new LED_Strip(STRIP_IO, 92);
    led_strip[LED_STRIP_ARM] = new LED_Strip(STRIP_IO, 60);
    led_strip[LED_STRIP_MATRIX] = new LED_Strip(STRIP_IO, 165);

    // 状态机更新信号量
    LED_Strip_FSM_Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(ISR_mutex);
    // 创建LED更新任务
    xTaskCreate(LED_update_task, "LED_update_task", 8192, NULL, 5, &LED_update_task_handle);

    // 注册装甲板事件处理
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, global_pr_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, global_pr_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, global_pr_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, global_pr_event_handler, NULL));
}

void PowerRune_Armour::trigger(RUNE_MODE mode, RUNE_COLOR color)
{
    ESP_LOGI(TAG_ARMOUR, "Trigger Armour with mode: %s, color: %s", mode == PRA_RUNE_BIG_MODE ? "Big" : "Small", color == PR_RED ? "Red" : "Blue");
    // 状态机更新
    state.LED_Strip_State = LED_STRIP_TARGET;
    state.mode = mode;
    state.color = color;
    state.score = 0;
    // 释放信号量
    xSemaphoreGive(LED_Strip_FSM_Semaphore);
}

void PowerRune_Armour::stop()
{
    ESP_LOGI(TAG_ARMOUR, "Stop Armour");
    // 状态机更新
    state.LED_Strip_State = LED_STRIP_IDLE;
    // 释放信号量
    xSemaphoreGive(LED_Strip_FSM_Semaphore);
}

void PowerRune_Armour::hit(uint8_t score)
{
    ESP_LOGI(TAG_ARMOUR, "Hit Armour with score: %d", score);
    // 状态机更新
    state.LED_Strip_State = LED_STRIP_HIT;
    state.score = score;
    // 释放信号量
    xSemaphoreGive(LED_Strip_FSM_Semaphore);
}

void PowerRune_Armour::blink()
{
    ESP_LOGI(TAG_ARMOUR, "Activation Complete, Blink Armour");
    // 状态机更新
    state.LED_Strip_State = LED_STRIP_BLINK;
    // 释放信号量
    xSemaphoreGive(LED_Strip_FSM_Semaphore);
}

void PowerRune_Armour::global_pr_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    switch (id)
    {
    case PRA_START_EVENT:
    {
        PRA_START_EVENT_DATA *start_event_data = (PRA_START_EVENT_DATA *)event_data;
        trigger((RUNE_MODE)start_event_data->mode, (RUNE_COLOR)start_event_data->color);
        break;
    }
    case PRA_STOP_EVENT:
        stop();
        break;
    case PRA_HIT_EVENT:
    {
        // 检查状态机状态
        if (state.LED_Strip_State == LED_STRIP_TARGET)
        {
            PRA_HIT_EVENT_DATA *hit_event_data = (PRA_HIT_EVENT_DATA *)event_data;
            hit(hit_event_data->score);
        }
        break;
    }
    case PRA_COMPLETE_EVENT:
    {
        blink();
        break;
    }
    }
}
