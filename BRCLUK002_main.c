//********************************************************************
//*                    MEC4126F C template                           *
//*==================================================================*
//* WRITTEN BY: Jesse Arendse   	                 	             *
//* DATE CREATED: 07/04/2023                                         *
//* MODIFIED: Dylan Fanner                                           *
//* DATE MODIFIED: 23/01/2025                                        *
//*==================================================================*
//* PROGRAMMED IN: Visual Studio Code                                *
//* TARGET:        STM32F0                                           *
//*==================================================================*
//* DESCRIPTION:     Template for MEC4126F C Practicals              *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================

#define STM32F051

#include "stm32f0xx.h"											   
#include "stdio.h"
#include "stdint.h"

//====================================================================
// GLOBAL CONSTANTS
//====================================================================
const float P = 500; 
const float I = 1;
const float T_s = 0.001;

//====================================================================
// GLOBAL VARIABLES
//====================================================================
volatile float o_p = 0;
volatile float g_p = 0;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void set_to_48MHz(void);
void init_ADC(void);
void init_timer6(void); 
void init_timer3(void);
void TIM6_IRQHandler(void);
void main(void);
//====================================================================
// MAIN FUNCTION
//====================================================================

void main (void)
{
    set_to_48MHz();
    init_ADC();
    init_timer6();
    init_timer3();

    while (1)
    {

    }
}							
// End of main

//====================================================================
// ISR DEFINITIONS
//====================================================================
void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;            // Acknowledge interrupt

    ADC1->CR |= ADC_CR_ADSTART;                // Start conversion
    while ((ADC1->ISR & ADC_ISR_EOC)==0); 
    uint16_t dTheta_c = ADC1->DR;   
    while ((ADC1->ISR & ADC_ISR_EOC)==0); 
    uint16_t dTheta_f = ADC1->DR; 

    int32_t e = dTheta_c - dTheta_f; // calculate error
    float g = e * P;

    float o = (g * (2 + I*T_s) + g_p * (I*T_s - 2) + 2 * o_p)/2;

    if (o > 126)
    {
        o = 126;
    }
    else if (o < -127)
    {
        o = -127;
    }

    TIM3->CCR3 = (int8_t)o + 127 ;

    g_p = g;
    o_p = o;
}


//====================================================================
// FUNCTION DEFINITIONS
//====================================================================
/*
*   Function to configure system clock to 48 MHz
*/
void set_to_48MHz(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
    {
        RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    }
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);
    RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void init_ADC(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER5 | GPIO_MODER_MODER6;          // PA 5 and 6 to Analogue

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;          // Enable ADC clock  

    ADC1->CFGR1 |= ADC_CFGR1_RES_0;             // 10 bit
    ADC1->CFGR1 |= ADC_CFGR1_WAIT;
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6;          // Channel 5 & 6

    ADC1->IER |= ADC_IER_EOCIE;                 // Enable interrupts on EOC

    ADC1->CR |= ADC_CR_ADEN;                    // Turn on the ADC
    while ((ADC1->ISR & ADC_ISR_ADRDY)==0);     // Until the ADRDY flag is raised do nothing                                           

    NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

void init_timer6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         // Enable clock to TIM6
    TIM6->DIER |= TIM_DIER_UIE;                 // Enable update event interrupts
    
    TIM6->PSC = 119;                          // PSC and ARR for 0.001 s on UE
    TIM6->ARR = 400;
    
    TIM6->CR1 |= TIM_CR1_ARPE;                  // Use preload registers
    TIM6->CR1 |= TIM_CR1_CEN;                   // Enable TIM6
    
    NVIC_EnableIRQ(TIM6_IRQn);                  // Allow TIM6 interrupts on NVIC    
}

void init_timer3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;         // Enable clock to TIM3

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= 0b10;                       // COnfig MODER0 as AF
    GPIOB->AFR[0] |= 0b1;                       // AFR[0] == AFRL (AF1 = TIM3_CH3)
  
    TIM3->PSC = 5000;                            // PSC and ARR for High freq update
    TIM3->ARR = 255;

    // Setup PWM
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM3->CCER |= TIM_CCER_CC3E;

    TIM3->CCR3 |= 200;
    
    TIM3->CR1 |= TIM_CR1_CEN;                   // Enable TIM3
}


//********************************************************************
// END OF PROGRAM
//********************************************************************