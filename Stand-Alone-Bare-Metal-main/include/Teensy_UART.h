#ifndef TEENSY_UART_H
#define TEENSY_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MIMXRT1062.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART_TX_BUFFER_SIZE     128
#define UART_RX_BUFFER_SIZE     128
#define UART_CLK_FREQ           CCM_UART_CLK_HZ

// #define UART_INIT(x, baud)    LPUART##x##_INIT(baud)

extern QueueHandle_t LPUART1_TX_QUEUE;
extern QueueHandle_t LPUART1_RX_QUEUE;

extern QueueHandle_t LPUART2_TX_QUEUE;
extern QueueHandle_t LPUART2_RX_QUEUE;

extern QueueHandle_t LPUART3_TX_QUEUE;
extern QueueHandle_t LPUART3_RX_QUEUE;

extern QueueHandle_t LPUART4_TX_QUEUE;
extern QueueHandle_t LPUART4_RX_QUEUE;

extern QueueHandle_t LPUART5_TX_QUEUE;
extern QueueHandle_t LPUART5_RX_QUEUE;

extern QueueHandle_t LPUART6_TX_QUEUE;
extern QueueHandle_t LPUART6_RX_QUEUE;

extern QueueHandle_t LPUART7_TX_QUEUE;
extern QueueHandle_t LPUART7_RX_QUEUE;

extern uint8_t LPUART_FIFO_SIZE;

#define LPUART_TXCOUNT(uart)    (((uart)->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT)

void LPUART1_INIT(uint32_t baudrate); // Pin 0/RX1, 1/TX1 - Pinler Yanlis olabilir, kontrol et.
void LPUART1_IRQHandler(void);
void LPUART1_Write_Char(char c);
void LPUART1_Write_String(const char *str);
char LPUART1_Read_Char(void);

void LPUART2_INIT(uint32_t baudrate); // Pin 7/RX2, 8/TX2 - Pinler Yanlis olabilir, kontrol et.
void LPUART2_IRQHandler(void);
void LPUART2_Write_Char(char c);
void LPUART2_Write_String(const char *str);
char LPUART2_Read_Char(void);

void LPUART3_INIT(uint32_t baudrate); // Pin 15/RX3, 14/TX3 - Pinler Yanlis olabilir, kontrol et.
void LPUART3_IRQHandler(void);
void LPUART3_Write_Char(char c);
void LPUART3_Write_String(const char *str);
char LPUART3_Read_Char(void);

void LPUART4_INIT(uint32_t baudrate); // Pin 16/RX4, 17/TX4 - Pinler Yanlis olabilir, kontrol et.
void LPUART4_IRQHandler(void);
void LPUART4_Write_Char(char c);
void LPUART4_Write_String(const char *str);
char LPUART4_Read_Char(void);

void LPUART5_INIT(uint32_t baudrate); // Pin 21/RX5, 20/TX5 - Pinler Yanlis olabilir, kontrol et.
void LPUART5_IRQHandler(void);
void LPUART5_Write_Char(char c);
void LPUART5_Write_String(const char *str);
char LPUART5_Read_Char(void);

void LPUART6_INIT(uint32_t baudrate); // Pin 25/RX6, 24/TX6 - Pinler Yanlis olabilir, kontrol et.
void LPUART6_IRQHandler(void);
void LPUART6_Write_Char(char c);
void LPUART6_Write_String(const char *str);
char LPUART6_Read_Char(void);

void LPUART7_INIT(uint32_t baudrate); // Pin 28/RX7, 29/TX7 - Pinler Yanlis olabilir, kontrol et.
void LPUART7_IRQHandler(void);
void LPUART7_Write_Char(char c);
void LPUART7_Write_String(const char *str);
char LPUART7_Read_Char(void);

//LUPART8 Teensy4.1 Tarafindan Ethernet icin kullaniliyor.

#ifdef __cplusplus
}
#endif

#endif