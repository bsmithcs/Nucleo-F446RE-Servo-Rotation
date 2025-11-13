/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj4A, 11/14/25
 *    NucleoF466RE CMSIS STM32F4xx 
 *    Line-following Robot
 ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/************************* Definitions *************************/

#define FREQUENCY   16000000UL // 16 MHz  

#define BTN_PORT    GPIOC
#define BTN_PIN     13

// Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT     GPIOC
#define SERVO3_PIN      6 

// Assuming servo motor 1 control pin is connected to GPIOC pin 9
#define SERVO1_PORT     GPIOC
#define SERVO1_PIN      9 

#define IR_SENSOR_PORT GPIOC

/************************* Variables *************************/

volatile uint32_t IR_reading;
volatile int digit_select = 0;

/***** Movement State *****/
volatile bool left_stopped = false;
volatile bool right_stopped = false;
volatile bool hit_first = false; 

/************************* Function Declarations *************************/

void TIM2_Config(void); 
void TIM8_Config(void);
void PWM_Output_PC6_Init(void);
void PWM_Output_PC9_Init(void);
void servo_speed_update(void);
void BTN_Config(void);
void EXTI15_10_IRQHandler(void);
void TIM2_IRQHandler(void);
void IR_Sensor_Init(void);
int get_display_reading(void);

/************************* Main *************************/
int main(void) {

    /***** Enable necessary lines *****/
    // Enable AHB1 peripherals (GPIO ports)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN     // GPIOC for BTN, PWM
                 |  RCC_AHB1ENR_GPIOBEN;    // GPIOB for SSD segments

    // Enable APB1 peripherals (TIM2)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;      // SSD multiplexing timer

    // Enable APB2 peripherals (TIM8, SYSCFG)
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN      // Advanced timer for servo PWM
                 |  RCC_APB2ENR_SYSCFGEN;   // For EXTI button interrupt

    /***** Initializations *****/
    TIM2_Config();
    TIM8_Config();

    PWM_Output_PC6_Init();
    PWM_Output_PC9_Init();
    BTN_Config();
    SSD_init();
    IR_Sensor_Init();

    /***** Wait for Interrupt *****/
    //__WFI(); // Low power wait defined by CMSIS
    while (1);

    return 0;
}

/************************* Function Definitions *************************/

/***** Timer Configuration Functions *****/

/// @brief This function configures TIM2 with 500 us interrupts 
/// using 1MHz prescaler. Used to flicker SSD selection.
void TIM2_Config(void) {
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period)
    TIM2->ARR = 5000 - 1; // Auto-reload when CNT = XX: (period = XX usec)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

/// @brief This function configures TIM8 CH1 & CH4 to send PWM to Servo1 and Servo3
void TIM8_Config(void) {
	// Configure TIM8 for PWM output on CH1 & CH4 (PC6 & PC9)
	TIM8->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM8->ARR = 19999; // Period for 50 Hz

    // Channel 1 Setup (SERVO3 Line)
	TIM8->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M); // Clear
	TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM8->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
	TIM8->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)

    // Channel 4 Setup (SERVO1 Line)
	TIM8->CCR4 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM8->CCMR2 &= ~(TIM_CCMR2_OC4M); // Clear
	TIM8->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // PWM mode 1
	TIM8->CCMR2 |= TIM_CCMR2_OC4PE; // Preload enable
	TIM8->CCER |= TIM_CCER_CC4E; // Enable CH4 output (PC9)

	// For advanced-control timers (TIM8) the main output must be enabled
	// using the BDTR MOE bit or outputs will stay inactive even if CC1E is set.
	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM8->EGR = TIM_EGR_UG; // Generate update event
	TIM8->CR1 |= TIM_CR1_CEN; // Enable timer
}

/***** Servo Control Functions *****/

/// @brief Initializes PC6 as AF3 (TIM8_CH1)
void PWM_Output_PC6_Init(void) {
	// Set PC6 to alternate function (AF3 for TIM8_CH1)
	GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
	GPIOC->AFR[0] |=  (0x3 << (SERVO3_PIN * 4)); // AF3 = TIM8

    // Configure output type and speed for PC6: push-pull, high speed
    GPIOC->OTYPER &= ~(1 << SERVO3_PIN); // push-pull
    GPIOC->OSPEEDR &= ~(0x3 << (SERVO3_PIN * 2));
    GPIOC->OSPEEDR |=  (0x2 << (SERVO3_PIN * 2)); // high speed
}

/// @brief Initializes PC9 as AF3 (TIM8_CH4)
void PWM_Output_PC9_Init(void) {
	// Set PC9 to alternate function (AF3 for TIM8_CH4)
	GPIOC->MODER &= ~(0x3 << (SERVO1_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO1_PIN * 2)); // Alternate function
	GPIOC->AFR[1] &= ~(0xF << ((SERVO1_PIN - 8) * 4));
	GPIOC->AFR[1] |=  (0x3 << ((SERVO1_PIN - 8) * 4)); // AF3 = TIM8

    // Configure output type and speed for PC6: push-pull, high speed
    GPIOC->OTYPER &= ~(1 << SERVO1_PIN); // push-pull
    GPIOC->OSPEEDR &= ~(0x3 << (SERVO1_PIN * 2));
    GPIOC->OSPEEDR |=  (0x2 << (SERVO1_PIN * 2)); // high speed
}

/// @brief sends pulses to servos with pulsewidths corresponding to current reading
void servo_speed_update() {
    uint32_t left_pw; 
    uint32_t right_pw;

    if (left_stopped) left_pw = 1500;
    else left_pw = 1280;

    if (right_stopped) right_pw = 1500;
    else right_pw = 1720;

    TIM8->CCR1 = right_pw;
    TIM8->CCR4 = left_pw;
}

int get_display_reading(void) {
    int display_reading = 0;

    if (IR_reading & 0b1000) display_reading += 1000;
    if (IR_reading & 0b0100) display_reading += 100;
    if (IR_reading & 0b0010) display_reading += 10;
    if (IR_reading & 0b0001) display_reading += 1;

    return display_reading;
}

/***** Peripheral Configuration Functions *****/

/// @brief Configures USER button with pullup and associated interrupt
void BTN_Config(void) {
    // Set BTN Pin to be an input with a pullup
    BTN_PORT->MODER &= ~(0x3 << (BTN_PIN * 2));
    BTN_PORT->PUPDR &= ~(0x3 << (BTN_PIN * 2));
    BTN_PORT->PUPDR |= (0x1 << (BTN_PIN * 2));

    // Configure interrupts for BTN
    EXTI->IMR |= (1 << 13); // Unmask EXTI line 13
    EXTI->FTSR |= (1 << 13); // Trigger on falling edge
    SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // Clear EXTI13 bits
    SYSCFG->EXTICR[3] |= (2 << (1 * 4)); // Map EXTI13 to PC13

    // Configure NVIC
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
}

/// @brief Initializes PC0-PC3 as inputs for IR sensor
void IR_Sensor_Init(void) {
    GPIOC->MODER &= ~(0x3 << (0 * 2));  // PC0 input
    GPIOC->MODER &= ~(0x3 << (1 * 2));  // PC1 input
    GPIOC->MODER &= ~(0x3 << (2 * 2));  // PC2 input
    GPIOC->MODER &= ~(0x3 << (3 * 2));  // PC3 input
}

/***** Interrupts *****/

/// @brief Handles button presses to cycle through directions: CW, CCW 
/// stopping between transitions. Triggered by interrupt on [15:10] line
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << BTN_PIN)) { // Check if the interrupt is from BTN_PIN
        EXTI->PR |= (1 << BTN_PIN); // Clear the pending interrupt

        if (left_stopped && right_stopped) {
            left_stopped = false;
            right_stopped = false;
        }
        else {
            left_stopped = true;
            right_stopped = true;
        }
    }
}

/// @brief Handles getting IR reading and interpreting it; updates SSD with IR_reading
/// Triggered by TIM2 interrupt every 500 us (0.5 ms).
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF) {
        // Read IR and invert so a 1 represents line hit
        IR_reading = (~IR_SENSOR_PORT->IDR) & 0b1111; 

        // STOP Condition
        if (IR_reading == 0b1111) { 
            if (hit_first){
                left_stopped = true;
                right_stopped = true;
            }
            else hit_first = true;
        }
        // RIGHT Condition
        else if ( 
            (IR_reading == 0b0001)
            | (IR_reading == 0b0010)
            | (IR_reading == 0b0011)
            | (IR_reading == 0b0111)
        ) {
            left_stopped = false;
            right_stopped = true;
        }
        // LEFT Condition
        else if (
            (IR_reading == 0b0100)
            | (IR_reading == 0b1000)
            | (IR_reading == 0b1100)
            | (IR_reading == 0b1110)
        ) {
            left_stopped = true;
            right_stopped = false;
        }
        // ERROR Condition (Unlikely Reading)
        else { 
            left_stopped = true;
            right_stopped = true;
        }

        servo_speed_update();

        /***** Update SSD *****/
        digit_select = (digit_select + 1) % 4;
        SSD_update(digit_select, get_display_reading(), 0);

        TIM2->SR &= ~TIM_SR_UIF; // clear update interrupt flag
    }
}
 