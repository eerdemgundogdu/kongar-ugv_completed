#include "Teensy_Encoder.h"
#include "MIMXRT1062.h"

// Encoder Configuration
// Left Encoder:  Pins 0, 1 (QTIMER1)
// Right Encoder: Pins 6, 7 (QTIMER2)

void Encoder_Init(void) {
    // 1. Enable Clocks
    CCM->CCGR6 |= CCM_CCGR6_CG11_MASK; // QTIMER1
    CCM->CCGR6 |= CCM_CCGR6_CG12_MASK; // QTIMER2
    
    // 2. Configure Pin Muxing
    
    // Left Encoder Pins (0, 1)
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03] = 1; 
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02] = 1; 
    
    // Right Encoder Pins (6, 7)
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_10] = 1; 
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_11] = 1; 
    
    // 3. Configure QTIMER1 (Left) - Quadrature Mode
    // Use Timer 0 as the counter
    TMR1->CHANNEL[0].CTRL = 0;
    TMR1->CHANNEL[0].CNTR = 0;
    TMR1->CHANNEL[0].LOAD = 0;
    TMR1->CHANNEL[0].COMP1 = 0xFFFF;
    TMR1->CHANNEL[0].CMPLD1 = 0xFFFF;
    
    // Count on rising and falling edges of primary and secondary sources
    // Primary source = Counter 0 Input Pin
    // Secondary source = Counter 1 Input Pin
    TMR1->CHANNEL[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_SCS(1); // Quadrature count mode
    TMR1->CHANNEL[0].SCTRL = TMR_SCTRL_IPS(0); // External input
    
    // 4. Configure QTIMER2 (Right)
    TMR2->CHANNEL[0].CTRL = 0;
    TMR2->CHANNEL[0].CNTR = 0;
    TMR2->CHANNEL[0].LOAD = 0;
    TMR2->CHANNEL[0].COMP1 = 0xFFFF;
    TMR2->CHANNEL[0].CMPLD1 = 0xFFFF;
    
    TMR2->CHANNEL[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) | TMR_CTRL_SCS(1);
}

int32_t Encoder_ReadLeft(void) {
    return (int32_t)TMR1->CHANNEL[0].CNTR;
}

int32_t Encoder_ReadRight(void) {
    return (int32_t)TMR2->CHANNEL[0].CNTR;
}
