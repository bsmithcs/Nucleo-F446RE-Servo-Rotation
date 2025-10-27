/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj3B, 10/24/25
 *    NucleoF466RE CMSIS STM32F4xx 
 *    Ultrasound-based Radar Scanning
 ****************************************************************/

#include "stm32f4xx.h"
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
#define WIPER_PIN   1

#define ADC_CHANNEL 1

/***** Variables *****/

volatile bool going_cw;
volatile bool stopped;

volatile uint32_t signal_pulse_width;
volatile uint32_t wiper_val;
volatile float rpm;
volatile int digit_select = 0;

/***** Helper Functions *****/

/// @brief This function configures TIM2 with 500 us interrupts 
/// using 1MHz prescaler. Used to flicker SSD selection.
void TIM2_Config(void) {
    // Configure TIM2 for XX microseconds interrupt (assuming 16MHz HSI clock)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period)
    TIM2->ARR = 500 - 1; // Auto-reload when CNT = XX: (period = XX usec)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

/// @brief Configures pulse width modulation via TIM3 for the servo on PC6
void PWM_Output_PC6_Init(void) {
	// Set PC6 to alternate function (AF2 for TIM3_CH1)
	GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
	GPIOC->AFR[0] |=  (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3
	// Configure TIM3 for PWM output on CH1 (PC6)
	TIM3->PSC = (FREQUENCY/1000000) - 1; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 19999; // Period for 50 Hz
	TIM3->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
	TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)
	TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM3->EGR = TIM_EGR_UG; // Generate update event
	TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

/// @brief sends pulse to servo with pulsewidth corresponding to the desired angle
/// @param angle angle to send the standard servo to  
void servo_angle_set(int speed) {
    // Assuming speeds from 0 to 140 rpm
    if (going_cw) signal_pulse_width = speed * (200 / 140) + 1520;
    else signal_pulse_width = speed * (200 / 140) + 1280;
	TIM3->CCR1 = signal_pulse_width;
}

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

    uart_sendString("\r\nCPEG222: TEAM07Proj3A\r\nUltrasound-based Radar\r\n\r\n");
}

/// @brief Displays relevant info on the serial monitor given the current angle
void uart_display() {
    uart_sendString("ADC Value: ");
    uart_send_int32(wiper_val);

    uart_sendString("\t\tDirection: ");
    if (stopped) uart_send_str("STOP");
    else if (going_cw) uart_send_str("CW");
    else uart_send_str("CCW");

    uart_sendString("\t\tservo (us): ");
    uart_send_int32(signal_pulse_width);

    uart_sendString("\t\trpm: ");
    uart_send_float(rpm);

    uart_sendString("\r\n");
}

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
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
}

void Pot_Config(void) {
    // Set PA1 (ADC) to analog mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    POT_PORT->MODER &= ~(0x3 << (WIPER_PIN * 2));
    POT_PORT->MODER |= (0x3 << (WIPER_PIN * 2));
    // Initialize ADC, Default resolution is 12 bits
    ADC1->SQR3 = ADC_CHANNEL; // Select channel
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1; // Sample time 56 cycles
    ADC1->CR2 = ADC_CR2_ADON; // Enable ADC

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
        SSD_update(digit_select, (int)(rpm * 10), 3);
    }
}

/// @brief Handles sending trigger pulse every 0.5 seconds
void SysTick_Handler(void) {
    uart_display();
}
 
/***** Main *****/
void main(void) {

    /***** Enable necessary ports *****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                 |  RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock

    /***** Initializations *****/
    TIM2_Config();
    SysTick_Config(16000000 * 0.5 - 1);  
    NVIC_SetPriority(SysTick_IRQn, 3); 

    PWM_Output_PC6_Init();
    BTN_Config();
    uart_Config();

    /***** Loop *****/
    while(1);
}