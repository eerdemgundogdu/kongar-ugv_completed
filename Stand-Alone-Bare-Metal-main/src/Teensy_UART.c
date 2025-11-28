#include "Teensy_UART.h"

QueueHandle_t LPUART1_TX_QUEUE;
QueueHandle_t LPUART1_RX_QUEUE;

QueueHandle_t LPUART2_TX_QUEUE;
QueueHandle_t LPUART2_RX_QUEUE;

QueueHandle_t LPUART3_TX_QUEUE;
QueueHandle_t LPUART3_RX_QUEUE;

QueueHandle_t LPUART4_TX_QUEUE;
QueueHandle_t LPUART4_RX_QUEUE;

QueueHandle_t LPUART5_TX_QUEUE;
QueueHandle_t LPUART5_RX_QUEUE;

QueueHandle_t LPUART6_TX_QUEUE;
QueueHandle_t LPUART6_RX_QUEUE;

QueueHandle_t LPUART7_TX_QUEUE;
QueueHandle_t LPUART7_RX_QUEUE;

uint8_t LPUART_FIFO_SIZE = 0;

/********************************************************************************************************************************/
/*                                                        LPUART1                                                               */
/********************************************************************************************************************************/

void LPUART1_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR5 |= CCM_CCGR5_CG12_MASK; // LPUART1 clock enable

    // Pin mux ayarları
    // LPUART1_TX = GPIO_AD_B0_12, LPUART1_RX = GPIO_AD_B0_13
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12] |= 0x2; // ALT2 = LPUART1_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13] |= 0x2; // ALT2 = LPUART1_RX

    // UART reset
    LPUART1->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART1->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART1->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART1->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART1->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART1->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART1->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART1->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART1->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART1->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART1->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART1->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART1->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART1_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART1_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART1_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART1_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART1)
    NVIC_EnableIRQ(LPUART1_IRQn);

    // Modulu ac
    LPUART1->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART1->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART1);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART1_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART1->DATA = c; // FIFO’ya yaz
            } else {
                LPUART1->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART1->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART1->DATA & 0xFF);
        xQueueSendFromISR(LPUART1_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART1_Write_Char(char c) {
    xQueueSend(LPUART1_TX_QUEUE, &c, portMAX_DELAY);
    LPUART1->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART1_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART1_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART1->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART1_Read_Char(void) {
    char c;
    xQueueReceive(LPUART1_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART2                                                               */
/********************************************************************************************************************************/

void LPUART2_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR0 |= CCM_CCGR0_CG14_MASK; // LPUART2 clock enable

    // Pin mux ayarları
    // LPUART2_TX = GPIO_AD_B1_02, LPUART2_RX = GPIO_AD_B1_03
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02] |= 0x2; // ALT2 = LPUART2_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03] |= 0x2; // ALT2 = LPUART2_RX

    // UART reset
    LPUART2->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART2->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART2->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART2->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART2->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART2->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART2->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART2->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART2->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART2->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART2->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART2->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART2->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART2_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART2_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART2_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART2_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART2)
    NVIC_EnableIRQ(LPUART2_IRQn);

    // Modulu ac
    LPUART2->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART2_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART2->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART2);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART2_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART2->DATA = c; // FIFO’ya yaz
            } else {
                LPUART2->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART2->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART2->DATA & 0xFF);
        xQueueSendFromISR(LPUART2_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART2_Write_Char(char c) {
    xQueueSend(LPUART2_TX_QUEUE, &c, portMAX_DELAY);
    LPUART2->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART2_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART2_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART2->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART2_Read_Char(void) {
    char c;
    xQueueReceive(LPUART2_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART3                                                               */
/********************************************************************************************************************************/

void LPUART3_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR0 |= CCM_CCGR0_CG6_MASK; // LPUART3 clock enable

    // Pin mux ayarları
    // LPUART3_TX = GPIO_B0_08, LPUART3_RX = GPIO_B0_09
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08] |= 0x3; // ALT3 = LPUART3_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09] |= 0x3; // ALT3 = LPUART3_RX

    // UART reset
    LPUART3->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART3->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART3->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART3->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART3->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART3->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART3->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART3->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART3->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART3->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART3->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART3->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART3->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART3_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART3_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART3_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART3_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART3)
    NVIC_EnableIRQ(LPUART3_IRQn);

    // Modulu ac
    LPUART3->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART3_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART3->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART3);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART3_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART3->DATA = c; // FIFO’ya yaz
            } else {
                LPUART3->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART3->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART3->DATA & 0xFF);
        xQueueSendFromISR(LPUART3_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART3_Write_Char(char c) {
    xQueueSend(LPUART3_TX_QUEUE, &c, portMAX_DELAY);
    LPUART3->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART3_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART3_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART3->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART3_Read_Char(void) {
    char c;
    xQueueReceive(LPUART3_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART4                                                               */
/********************************************************************************************************************************/

void LPUART4_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR1 |= CCM_CCGR1_CG12_MASK; // LPUART4 clock enable

    // Pin mux ayarları
    // LPUART4_TX = GPIO_B1_00, LPUART4_RX = GPIO_B1_01
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00] |= 0x2; // ALT2 = LPUART4_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01] |= 0x2; // ALT2 = LPUART4_RX

    // UART reset
    LPUART4->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART4->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART4->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART4->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART4->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART4->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART4->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART4->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART4->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART4->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART4->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART4->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART4->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART4_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART4_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART4_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART4_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART4)
    NVIC_EnableIRQ(LPUART4_IRQn);

    // Modulu ac
    LPUART4->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART4_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART4->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART4);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART4_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART4->DATA = c; // FIFO’ya yaz
            } else {
                LPUART4->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART4->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART4->DATA & 0xFF);
        xQueueSendFromISR(LPUART4_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART4_Write_Char(char c) {
    xQueueSend(LPUART4_TX_QUEUE, &c, portMAX_DELAY);
    LPUART4->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART4_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART4_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART4->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART4_Read_Char(void) {
    char c;
    xQueueReceive(LPUART4_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART5                                                               */
/********************************************************************************************************************************/

void LPUART5_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR3 |= CCM_CCGR3_CG1_MASK; // LPUART5 clock enable

    // Pin mux ayarları
    // LPUART5_TX = GPIO_B1_12, LPUART5_RX = GPIO_B1_13
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12] |= 0x1; // ALT1 = LPUART5_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13] &= ~0xF;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13] |= 0x1; // ALT1 = LPUART5_RX

    // UART reset
    LPUART5->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART5->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART5->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART5->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART5->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART5->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART5->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART5->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART5->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART5->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART5->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART5->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART5->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART5_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART5_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART5_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART5_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART5)
    NVIC_EnableIRQ(LPUART5_IRQn);

    // Modulu ac
    LPUART5->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART5_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART5->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART5);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART5_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART5->DATA = c; // FIFO’ya yaz
            } else {
                LPUART5->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART5->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART5->DATA & 0xFF);
        xQueueSendFromISR(LPUART5_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART5_Write_Char(char c) {
    xQueueSend(LPUART5_TX_QUEUE, &c, portMAX_DELAY);
    LPUART5->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART5_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART5_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART5->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART5_Read_Char(void) {
    char c;
    xQueueReceive(LPUART5_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART6                                                               */
/********************************************************************************************************************************/

void LPUART6_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR3 |= CCM_CCGR3_CG3_MASK; // LPUART6 clock enable

    // Pin mux ayarları
    // LPUART6_TX = GPIO_AD_B0_02, LPUART6_RX = GPIO_AD_B0_03
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02] |= 0x2; // ALT2 = LPUART6_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03] |= 0x2; // ALT2 = LPUART6_RX

    // UART reset
    LPUART6->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART6->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART6->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART6->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART6->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART6->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART6->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART6->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART6->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART6->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART6->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART6->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART6->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART6_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART6_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART6_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART6_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART6)
    NVIC_EnableIRQ(LPUART6_IRQn);

    // Modulu ac
    LPUART6->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART6_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART6->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART6);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART6_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART6->DATA = c; // FIFO’ya yaz
            } else {
                LPUART6->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART6->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART6->DATA & 0xFF);
        xQueueSendFromISR(LPUART6_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART6_Write_Char(char c) {
    xQueueSend(LPUART6_TX_QUEUE, &c, portMAX_DELAY);
    LPUART6->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART6_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART6_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART6->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART6_Read_Char(void) {
    char c;
    xQueueReceive(LPUART6_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}

/********************************************************************************************************************************/
/*                                                        LPUART7                                                               */
/********************************************************************************************************************************/

void LPUART7_INIT(uint32_t baudrate) {
    // LPUART saat ayari (CCM)
    CCM->CCGR5 |= CCM_CCGR5_CG13_MASK; // LPUART7 clock enable

    // Pin mux ayarları
    // LPUART7_TX = GPIO_SD_B1_08, LPUART7_RX = GPIO_SD_B1_09
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_08] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_08] |= 0x2; // ALT2 = LPUART7_TX

    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_09] &= ~0x7;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_09] |= 0x2; // ALT2 = LPUART7_RX

    // UART reset
    LPUART7->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    LPUART7->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    // Modulu kapat
    LPUART7->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART7->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);
    LPUART7->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

    LPUART7->WATER &= ~(LPUART_WATER_TXWATER_MASK | LPUART_WATER_RXWATER_MASK);
    LPUART7->WATER |= LPUART_WATER_TXWATER(2U) | LPUART_WATER_RXWATER(0U);

    if(!(LPUART_FIFO_SIZE))
        LPUART_FIFO_SIZE = 1U << (((LPUART7->FIFO & LPUART_FIFO_TXFIFOSIZE_MASK) >> LPUART_FIFO_TXFIFOSIZE_SHIFT) + 1U);

    // Baud rate ayari
    uint32_t sbr = (UART_CLK_FREQ / (16U * baudrate));
    LPUART7->BAUD &= ~LPUART_BAUD_SBR_MASK;
    LPUART7->BAUD |= LPUART_BAUD_SBR(sbr);

    // 8-bit, no parity
    LPUART7->CTRL = 0; // UART reset atinca sifirlandi zaten. Bu satir sadece okunabilirlik daha kolay olsun diye var.

    LPUART7->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK); // TX ve RX enable
    LPUART7->CTRL |= LPUART_CTRL_RIE_MASK; // RX interrupt enable - TX kesmesi LPUART7_Write_Char sonrasi acilacak

    // FreeRTOS queue oluştur
    LPUART7_TX_QUEUE = xQueueCreate(UART_TX_BUFFER_SIZE, sizeof(char));
    LPUART7_RX_QUEUE = xQueueCreate(UART_RX_BUFFER_SIZE, sizeof(char));

    NVIC_SetPriority(LPUART7_IRQn, 5);

    // NVIC interrupt enable (IRQn LPUART7)
    NVIC_EnableIRQ(LPUART7_IRQn);

    // Modulu ac
    LPUART7->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void LPUART7_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // TX interrupt
    if (LPUART7->STAT & LPUART_STAT_TDRE_MASK) {
        uint8_t space = LPUART_FIFO_SIZE - LPUART_TXCOUNT(LPUART7);
        while (space--) {
            char c;
            if (xQueueReceiveFromISR(LPUART7_TX_QUEUE, &c, &xHigherPriorityTaskWoken) == pdPASS) {
                LPUART7->DATA = c; // FIFO’ya yaz
            } else {
                LPUART7->CTRL &= ~LPUART_CTRL_TIE_MASK; // Queue bos → TIE disable
                break; // FIFO dolu olmayabilir.
            }
        }
    }

    // RX interrupt
    while (LPUART7->STAT & LPUART_STAT_RDRF_MASK) {
        char c = (char)(LPUART7->DATA & 0xFF);
        xQueueSendFromISR(LPUART7_RX_QUEUE, &c, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LPUART7_Write_Char(char c) {
    xQueueSend(LPUART7_TX_QUEUE, &c, portMAX_DELAY);
    LPUART7->CTRL |= LPUART_CTRL_TIE_MASK;
}

void LPUART7_Write_String(const char *str) {
    while (*str) {
        xQueueSend(LPUART7_TX_QUEUE, str++, portMAX_DELAY);
    }
    LPUART7->CTRL |= LPUART_CTRL_TIE_MASK;
}

char LPUART7_Read_Char(void) {
    char c;
    xQueueReceive(LPUART7_RX_QUEUE, &c, portMAX_DELAY);
    return c;
}