#include "Teensy_PWM.h"
#include "MIMXRT1062.h"

// Hardware Mapping:
// Pin 2 -> FlexPWM4_PWMA02
// Pin 3 -> FlexPWM4_PWMB02
// Pin 4 -> GPIO4_IO06
// Pin 5 -> GPIO4_IO08

#define PWM_MAX_COUNT 1000

void PWM_Init(void) {
    // 1. Enable Clocks
    CCM->CCGR2 |= CCM_CCGR2_CG6_MASK; // GPIO4
    CCM->CCGR4 |= CCM_CCGR4_CG11_MASK; // FlexPWM4
    
    // 2. Configure Pins (IOMUX)
    
    // PWM Pins (2, 3) -> ALT1
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04] = 1; 
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05] = 1; 
    
    // Direction Pins (4, 5) -> ALT5
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06] = 5; 
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08] = 5; 
    
    // 3. Configure GPIO for Direction
    GPIO4->GDIR |= (1 << 6); // Pin 4 Output
    GPIO4->GDIR |= (1 << 8); // Pin 5 Output
    
    // 4. Configure FlexPWM4 Module 2
    // Disable module first
    PWM4->SM[2].CTRL2 = 0;
    
    // Set Prescaler to 1 (IPBus Clock / 1)
    PWM4->SM[2].CTRL = PWM_CTRL_PRSC(0);
    
    // Set Period (Signed Center-Aligned)
    // INIT = -MOD, VAL0 = 0, VAL1 = MOD
    PWM4->SM[2].INIT = (uint16_t)(-(PWM_MAX_COUNT));
    PWM4->SM[2].VAL1 = (uint16_t)(PWM_MAX_COUNT);
    PWM4->SM[2].VAL0 = 0;
    
    // Set Initial Duty (0%)
    PWM4->SM[2].VAL2 = 0; // A On
    PWM4->SM[2].VAL3 = 0; // A Off
    PWM4->SM[2].VAL4 = 0; // B On
    PWM4->SM[2].VAL5 = 0; // B Off
    
    // Enable Output
    PWM4->OUTEN |= (1 << 8) | (1 << 9); // Enable PWM4_SM2_A and B
    
    // Load Registers
    PWM4->MCTRL |= PWM_MCTRL_LDOK(1 << 2);
    
    // Start PWM
    PWM4->MCTRL |= PWM_MCTRL_RUN(1 << 2);
}

void PWM_SetDutyLeft(float duty) {
    // Pin 4 is Dir, Pin 2 is PWM (Channel A)
    
    if (duty > 0) {
        GPIO4->DR |= (1 << 6); // Forward
    } else {
        GPIO4->DR &= ~(1 << 6); // Backward
        duty = -duty;
    }
    
    if (duty > 1.0f) duty = 1.0f;
    
    int16_t val = (int16_t)(duty * PWM_MAX_COUNT);
    
    // Center Aligned PWM
    PWM4->SM[2].VAL2 = -val;
    PWM4->SM[2].VAL3 = val;
    
    PWM4->MCTRL |= PWM_MCTRL_LDOK(1 << 2);
}

void PWM_SetDutyRight(float duty) {
    // Pin 5 is Dir, Pin 3 is PWM (Channel B)
    
    if (duty > 0) {
        GPIO4->DR |= (1 << 8); // Forward
    } else {
        GPIO4->DR &= ~(1 << 8); // Backward
        duty = -duty;
    }
    
    if (duty > 1.0f) duty = 1.0f;
    
    int16_t val = (int16_t)(duty * PWM_MAX_COUNT);
    
    PWM4->SM[2].VAL4 = -val;
    PWM4->SM[2].VAL5 = val;
    
    PWM4->MCTRL |= PWM_MCTRL_LDOK(1 << 2);
}
