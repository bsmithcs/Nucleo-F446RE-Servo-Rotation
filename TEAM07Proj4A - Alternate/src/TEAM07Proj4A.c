/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj4A, 10/31/25
 *    NucleoF466RE CMSIS STM32F4xx 
 *    Continuous Rotation Servo Control w/ Feedback
 ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/***** Definitions *****/

#define FREQUENCY   16000000UL // 16 MHz  

#define UART_PORT   GPIOA
#define UART_TX_PIN 2 
#define UART_RX_PIN 3 
#define BAUDRATE    115200

#define BTN_PORT    GPIOC
#define BTN_PIN     13

// Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT     GPIOC
#define SERVO3_PIN      6 
#define SERVO3_FEED_PIN 7

#define POT_PORT    GPIOA
#define POT_PIN     1

#define ADC_CHANNEL 1

/***** Variables *****/

volatile uint32_t pot_val;
volatile uint32_t signal_pulse;

volatile bool stopped = true;
volatile bool going_cw = true;

volatile uint32_t rise;
volatile uint32_t feedback_pulse;

volatile int angular_position = 0;
int min_pulse = 32;
int max_pulse = 1076;

volatile uint32_t last_measurement_time;
volatile uint32_t last_angular_position;

volatile float angular_speed = 0.0;

volatile uint32_t digit_select = 0;

/***** Helper Functions *****/

/***** Timer Configuration Functions *****/

/// @brief This function configures TIM2 with 500 us interrupts 
/// using 1MHz prescaler. Used to flicker SSD selection.
void TIM2_Config(void) {
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period)
    TIM2->ARR = 500 - 1; // Auto-reload when CNT = XX: (period = XX usec)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

/// @brief This function configures TIM5 as a free-running clock w/ 1us period
/// (1 MHz). Used to count pulse length. 
void TIM5_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
    TIM5->PSC = 15; // Prescaler: (16MHz/16 = 1MHz, 1usec period)
    TIM5->ARR = 0xFFFFFFFF; // Auto-reload: Max value for free running (32-bits)
    TIM5->EGR = TIM_EGR_UG; // Update event generation register
    TIM5->CR1 = TIM_CR1_CEN; // Enable TIM5
}

void TIM8_Config(void) {
	// Configure TIM8 for PWM output on CH1 (PC6)
	TIM8->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM8->ARR = 19999; // Period for 50 Hz
	TIM8->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM8->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
	TIM8->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)

	// For advanced-control timers (TIM8) the main output must be enabled
	// using the BDTR MOE bit or outputs will stay inactive even if CC1E is set.
	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM8->EGR = TIM_EGR_UG; // Generate update event
	TIM8->CR1 |= TIM_CR1_CEN; // Enable timer
}

/***** Servo Control Functions *****/

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

void PWM_Input_PC7_Init(void) {
    // Configure PC7 as input
    SERVO3_PORT->MODER &= ~(0x3 << (SERVO3_PIN * 2));

    // Configure EXTI
    SYSCFG->EXTICR[0] &= ~(0xF << (SERVO3_FEED_PIN * 4));
    SYSCFG->EXTICR[0] |= (0x2 << (SERVO3_FEED_PIN * 4));
    EXTI->IMR |= (1 << SERVO3_FEED_PIN);
    EXTI->RTSR |= (1 << SERVO3_FEED_PIN);
    EXTI->FTSR |= (1 << SERVO3_FEED_PIN);

    // Configure NVIC
    NVIC_SetPriority(EXTI9_5_IRQn, 1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/// @brief sends pulse to servo with pulsewidth corresponding to the desired speed
void servo_speed_set() {
    // Assuming speeds from 0 to 4095 (12-bit ADC range)
    uint32_t speed = ( pot_val * 200u ) / 4095u;
    if (stopped) signal_pulse = 1500u;
    else if (going_cw) signal_pulse = 1480u - speed;
    else signal_pulse = speed + 1520u;
    // Write to TIM8 CH1 CCR (PC6 PWM output)
    TIM8->CCR1 = signal_pulse;
}

/***** UART Functions *****/

/// @brief Sends character to USART to be displayed on serial monitor 
/// @param c Character to be sent 
void uart_sendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE)); // Wait until transmit data register is empty
    USART2->DR = c;
}

/// @brief Sends string to USART to be displayed on serial monitor
/// @param str String to be sent 
void uart_sendString(const char* str) {
    while (*str) {
        uart_sendChar(*str++);
    }
}

/// @brief Sends int to USART to be displayed on serial monitor
/// @param val int to be sent 
void uart_send_int32(int32_t val) {
	char buf[12];
	sprintf(buf, "%ld", (long)val);
	uart_sendString(buf);
}

/// @brief Sends int to USART to be displayed on serial monitor
/// @param val int to be sent 
void uart_send_float(float val) {
	char buf[12];
	sprintf(buf, "%.2f", (float)val);
	uart_sendString(buf);
}

/// @brief Configures UART to display to serial monitor and 
/// writes a startup message 
void uart_Config(void) {
    // Set PA2 (TX) and PA3 (RX) to alternate function
    GPIOA->MODER &= ~((3 << (UART_TX_PIN*2)) | (3 << (UART_RX_PIN*2))); // Clear mode bits
    GPIOA->MODER |= (2 << (UART_TX_PIN*2)) | (2 << (UART_RX_PIN*2)); // AF mode
    GPIOA->AFR[0] |= (7 << (UART_TX_PIN*4)) | (7 << (UART_RX_PIN*4)); // AF7 for USART2
    // Configure USART2: BAUDRATE baud, 8N1, enable TX and RX
    USART2->BRR = 16000000UL / BAUDRATE; // Assuming 16 MHz clock
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART

    uart_sendString("\r\nCPEG222: TEAM07Proj4A\r\nCont. Servo\r\n\r\n");
}

/// @brief Displays relevant info on the serial monitor given the current angle
void uart_display() {
    uart_sendString("ADC Value: ");
    uart_send_int32(pot_val);

    uart_sendString("\t\tDirection: ");
    if (stopped) uart_sendString("STOP");
    else if (going_cw) uart_sendString("CW");
    else uart_sendString("CCW");

    uart_sendString("\t\tservo (us): ");
    uart_send_int32(signal_pulse);

    uart_sendString("\t\trpm: ");
    uart_send_float(angular_speed);

    uart_sendString("\r\n");
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

void Pot_Config(void) {
    // Set PA1 (ADC) to analog mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    POT_PORT->MODER &= ~(0x3 << (POT_PIN * 2));
    POT_PORT->MODER |= (0x3 << (POT_PIN * 2));
    // Initialize ADC, Default resolution is 12 bits
    ADC1->SQR3 = ADC_CHANNEL; // Select channel
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1; // Sample time 56 cycles
    ADC1->CR1 |= ADC_CR1_EOCIE; // Enable End of Conversion interrupt
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT; // Enable ADC
    
    // Configure NVIC for ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1);

    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/***** Interrupts *****/

/// @brief Handles button presses to cycle through directions: CW, CCW 
/// stopping between transitions. Triggered by interrupt on [15:10] line
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << BTN_PIN)) { // Check if the interrupt is from BTN_PIN
        EXTI->PR |= (1 << BTN_PIN); // Clear the pending interrupt

        if (!stopped) {
            stopped = true;
        }
        else {
            stopped = false;
            going_cw = !going_cw;
        }
    }
}

/// @brief Handles updating SSD with most recent distance measurement.
/// Triggered by TIM2 interrupt every 500 us (0.5 ms).
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // clear update interrupt flag
        digit_select = (digit_select + 1) % 4;
        
        SSD_update(digit_select, (int)(angular_speed * 10), 3);

        // Set speed here so it isn't updated too frequently
        servo_speed_set();
    }
}

/// @brief If rising edge, capture current time
//         If falling edge, calculate PW and convert to angular position and store last_measurement_time
//              If a previous angular position exists, calculate angle_delta and time_delta
//              where rpm = angle_delta in rotations / time_delta in minutes
//              (time_delta = TIM5->CNT - last_measurement_time)
//              (angle_delta = angular_position - last_angular_position)
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1 << SERVO3_FEED_PIN)) {
        EXTI->PR |= (1 << SERVO3_FEED_PIN);

        if (SERVO3_PORT->IDR & (1 << SERVO3_FEED_PIN)) {
            // Rising Edge
            rise = TIM5->CNT;
        }
        else {
            // Falling Edge
            if (TIM5->CNT >= rise) feedback_pulse = TIM5->CNT - rise;
            else feedback_pulse = (0xFFFFFFFF - rise) + TIM5->CNT + 1;

            // Calculate angular position based on feedback_pulse
            angular_position = (feedback_pulse - min_pulse)*360/(max_pulse - min_pulse);

            // Attempt rotational_speed calculation
            if (last_angular_position && last_measurement_time) {
                // Account for rollover and direction
                int angle_delta = last_angular_position - angular_position;
                int time_delta = TIM5->CNT - last_measurement_time;

                angular_speed = ((float)angle_delta/360.0f) / ((float)time_delta/60e6f);
            }

            last_measurement_time = TIM5->CNT;
            last_angular_position = angular_position;
        }

    } 
}


/// @brief Display to uart every 1 second
void SysTick_Handler(void) {
    uart_display();
}

/// @brief Handles ADC conversion complete interrupt
void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {  // Check if End of Conversion
        pot_val = ADC1->DR;      // Read the converted value
    }
}
 
/***** Main *****/
int main(void) {

    /***** Enable necessary clocks *****/
    // Enable AHB1 peripherals (GPIO ports)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN    // GPIOA for UART, ADC
                 |  RCC_AHB1ENR_GPIOCEN     // GPIOC for BTN, PWM
                 |  RCC_AHB1ENR_GPIOBEN;    // GPIOB for SSD segments

    // Enable APB1 peripherals (USART2, TIM2, TIM3)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN   // UART2
                 |  RCC_APB1ENR_TIM2EN      // SSD multiplexing timer
                 |  RCC_APB1ENR_TIM3EN     // Input capture timer
                 |  RCC_APB1ENR_TIM5EN;    // Free running timer

    // Enable APB2 peripherals (ADC, TIM8, SYSCFG)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN     // ADC1 for potentiometer
                 |  RCC_APB2ENR_TIM8EN      // Advanced timer for servo PWM
                 |  RCC_APB2ENR_SYSCFGEN;   // For EXTI button interrupt

    /***** Initializations *****/
    SysTick_Config(FREQUENCY * 1 - 1);  
    NVIC_SetPriority(SysTick_IRQn, 3); 
    TIM2_Config();
    TIM5_Config();
    TIM8_Config();

    PWM_Output_PC6_Init();
    PWM_Input_PC7_Init();
    BTN_Config();
    uart_Config();
    SSD_init();
    Pot_Config();

    /***** Loop *****/
    while(1);

    return 0;
}