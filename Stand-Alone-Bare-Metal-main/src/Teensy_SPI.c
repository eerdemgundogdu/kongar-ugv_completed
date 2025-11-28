#include "Teensy_SPI.h"
#include "MIMXRT1062.h"

// SPI Configuration (LPSPI4)
// SCK:  Pin 13 (GPIO_B0_03)
// MOSI: Pin 11 (GPIO_B0_02)
// MISO: Pin 12 (GPIO_B0_01)
// CS:   Pin 10 (GPIO_B0_00)

void SPI_Init(void) {
    // 1. Enable Clocks
    CCM->CCGR1 |= CCM_CCGR1_CG0_MASK; // Enable LPSPI4
    CCM->CCGR0 |= CCM_CCGR0_CG6_MASK; // Enable GPIO2 (for CS)
    
    // 2. Configure Pins
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03] = 3; // SCK
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02] = 3; // MOSI
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01] = 3; // MISO
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00] = 5; // CS (GPIO)
    
    // 3. Configure CS Pin
    GPIO2->GDIR |= (1 << 0); // Output
    GPIO2->DR |= (1 << 0);   // High
    
    // 4. Configure LPSPI4
    LPSPI4->CR = 0; // Disable
    LPSPI4->IER = 0; // No interrupts
    LPSPI4->DER = 0; // No DMA
    LPSPI4->CFGR0 = 0;
    LPSPI4->CFGR1 = LPSPI_CFGR1_MASTER_MASK; // Master Mode
    
    // Clock Config (Assuming 60MHz source)
    // SCK = Source / (2^PRESCALE * (SCKDIV+2))
    LPSPI4->CCR = LPSPI_CCR_SCKDIV(4) | LPSPI_CCR_DBT(2) | LPSPI_CCR_PCSSCK(2) | LPSPI_CCR_SCKPCS(2);
    
    LPSPI4->TCR = LPSPI_TCR_CPOL(0) | LPSPI_TCR_CPHA(0) | LPSPI_TCR_PRESCALE(0) | LPSPI_TCR_FRAMESZ(7); // 8-bit
    
    LPSPI4->CR |= LPSPI_CR_MEN_MASK; // Enable
}

void SPI_CS_Low(void) {
    GPIO2->DR &= ~(1 << 0);
}

void SPI_CS_High(void) {
    GPIO2->DR |= (1 << 0);
}

uint8_t SPI_Transfer(uint8_t data) {
    while (!(LPSPI4->SR & LPSPI_SR_TDF_MASK)); // Wait for TX FIFO
    LPSPI4->TDR = data;
    
    while (!(LPSPI4->SR & LPSPI_SR_RDF_MASK)); // Wait for RX FIFO
    return (uint8_t)LPSPI4->RDR;
}
