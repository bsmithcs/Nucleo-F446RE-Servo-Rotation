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
#define WIPER_PIN   1

#define ADC_CHANNEL 1

/***** Variables *****/

volatile bool going_cw = true;
volatile bool stopped = false;

volatile uint32_t signal_pulse_width;
volatile uint32_t pot_val;
volatile float rpm = 0.0;
volatile int digit_select = 0;

volatile bool waiting_for_falling = false;

volatile uint32_t rise;
volatile uint32_t fall;
volatile uint32_t pulse_width;

volatile uint32_t current_angle;
volatile uint32_t last_angle;

volatile uint32_t current_time;
volatile uint32_t last_time;

volatile float time_diff;
volatile float angle_diff;
uint32_t min_pulse_width = 32;
uint32_t max_pulse_width = 1076;

volatile int systick_tick = 0;



void uart_sendString(const char* str);
void uart_send_int32(int32_t val);
void uart_send_float(float val);

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

/// @brief Used for PWM Input from signal wire (Capture/Compare)
void TIM3_Config(void) {
	// Configure TIM3 for simple input capture on CH2 (PC7)
	TIM3->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 0xFFFF;
	TIM3->CCMR1 &= ~(0x3 << 8); // CC2S bits for CH2
	TIM3->CCMR1 |= (0x01 << 8); // CC2: IC2 mapped to TI2
	TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge
	TIM3->CCER |= TIM_CCER_CC2E; // Enable capture on CH2
	TIM3->DIER |= TIM_DIER_CC2IE; // Enable capture/compare 2 interrupt
	TIM3->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC
}

/// @brief This function configures TIM5 as a free-running clock w/ 1us period
/// (1 MHz). Used to count . 
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
	// Set PC7 to alternate function (AF2 for TIM3_CH2)
	GPIOC->MODER &= ~(0x3 << (7 * 2));
	GPIOC->MODER |=  (0x2 << (7 * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
	GPIOC->AFR[0] |=  (0x2 << (7 * 4)); // AF2 = TIM3
}

/// @brief sends pulse to servo with pulsewidth corresponding to the desired angle
/// @param angle angle to send the standard servo to  
void servo_angle_set(int speed) {
    // Assuming speeds from 0 to 4095 (12-bit ADC range)
    uint32_t delta = ( (uint32_t)speed * 200u ) / 4095u;
    if (stopped) signal_pulse_width = 1500u;
    else if (going_cw) signal_pulse_width = 1480u - delta;
    else signal_pulse_width = delta + 1510u;
    // Write to TIM8 CH1 CCR (PC6 PWM output)
    TIM8->CCR1 = signal_pulse_width;
}

void calculate_rpm(void) {
    if (current_time >= last_time) time_diff = (float)(current_time - last_time);
    else time_diff = (float)((0xFFFFFFFF - last_time) + current_time + 1);

    if (going_cw) {
        if (current_angle >= last_angle) angle_diff = (float)(current_angle - last_angle);
        else angle_diff = (float)((360.0f - last_angle) + current_angle);
    }
    else {
        if (last_angle > current_angle) angle_diff = (float)(last_angle - current_angle);
        else angle_diff = (float)((360.0f - current_angle) + last_angle);
    }

    if (time_diff == 0.0f) { 
        rpm = 0.0f;
        return;
    }

    rpm = (angle_diff / 360.0f) / (time_diff / 60000000.0f);
    if (rpm > 180.0f) rpm = 0.0f;
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
    uart_send_int32(signal_pulse_width);

    uart_sendString("\t\trpm: ");
    uart_send_float(rpm);
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
    POT_PORT->MODER &= ~(0x3 << (WIPER_PIN * 2));
    POT_PORT->MODER |= (0x3 << (WIPER_PIN * 2));
    // Initialize ADC, Default resolution is 12 bits
    ADC1->SQR3 = ADC_CHANNEL; // Select channel
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1; // Sample time 56 cycles
    ADC1->CR1 |= ADC_CR1_EOCIE; // Enable End of Conversion interrupt
    ADC1->CR2 = ADC_CR2_ADON; // Enable ADC
    
    // Configure NVIC for ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1);
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

/// @brief Handles updating SSD with most recent rpm measurement.
/// Triggered by TIM2 interrupt every 500 us (0.5 ms).
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // clear update interrupt flag
        digit_select = (digit_select + 1) % 4;

        SSD_update(digit_select, (int)(rpm * 10), 3);
    }
}

/// @brief TIM3 input capture interrupt handler for PC7 (CH2)
void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_CC2IF) { // Check if CC2IF is set
		TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
        if (!waiting_for_falling) {
            rise = TIM3->CCR2;
            // Switch to capture falling edge
            TIM3->CCER |= TIM_CCER_CC2P; // Set to falling edge
            waiting_for_falling = 1;
        } 
        else {
            fall = TIM3->CCR2;
			if (fall >= rise) {
				if (fall - rise < 1100) pulse_width = fall - rise;
			} 
            else pulse_width = (0xFFFF - rise) + fall + 1;

            current_angle = ((pulse_width - min_pulse_width)*360)/(max_pulse_width - min_pulse_width);

            current_time = TIM5->CNT;

            TIM3->CCER &= ~TIM_CCER_CC2P; // Set to rising edge
            waiting_for_falling = 0;
        }
	}
}

/// @brief Display to uart every 1 second
void SysTick_Handler(void) {
    if (systick_tick == 9) {
        uart_display();
        systick_tick = (systick_tick + 1) % 10;
    }

    if (last_time && last_angle && !stopped) calculate_rpm();
    else rpm = 0.0f;
    last_time = current_time;
    last_angle = current_angle;

    systick_tick = (systick_tick + 1) % 10;
}

/// @brief Handles ADC conversion complete interrupt
void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {  // Check if End of Conversion
        pot_val = ADC1->DR;      // Read the converted value

        servo_angle_set(pot_val);
        // Start next conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;
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
    SysTick_Config(16000000 * 0.1 - 1);  
    NVIC_SetPriority(SysTick_IRQn, 3); 
    TIM2_Config();
    TIM3_Config();
    TIM5_Config();
    TIM8_Config();

    PWM_Output_PC6_Init();
    PWM_Input_PC7_Init();
    BTN_Config();
    uart_Config();
    SSD_init();
    Pot_Config();

    ADC1->CR2 |= ADC_CR2_SWSTART;

    /***** Loop *****/
    while(1);

    return 0;
}