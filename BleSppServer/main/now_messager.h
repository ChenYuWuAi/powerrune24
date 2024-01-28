/**
 * @file now_messager.h
 * @brief ESP NOW驱动程序，用于发送和接收数据，带有CRC检测和一次ACK握手
 * @version 0.1
 * @date 2024-01-26
 */

class Now_Messager
{
private:
public:
    Now_Messager();
    esp_err_t tx_handler();
    esp_err_t rx_handler();
    ~Now_Messager();
};