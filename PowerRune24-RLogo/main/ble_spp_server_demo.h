/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)

//系统参数设置服务
enum{
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

//大符操作服务
enum{
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