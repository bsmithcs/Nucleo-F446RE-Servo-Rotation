/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj4B, 10/24/25
 *    NucleoF466RE CMSIS STM32F4xx 
 *    Ultrasound-based Radar Scanning
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
volatile uint32_t wiper_val;
volatile float rpm;
volatile int digit_select = 0;

volatile bool waiting_for_falling = false;
volatile uint32_t last_rising;
volatile uint32_t last_falling;
volatile uint32_t pulse_width;

volatile uint32_t current_angle;
volatile uint32_t previous_angle;
volatile uint32_t current_time;
volatile uint32_t previous_time;
volatile uint32_t time_diff;
volatile uint32_t angle_diff;
uint32_t min_pulse_width = 32;
uint32_t max_pulse_width = 1076;

volatile float dcycle;
float max_dcycle = 97;
float min_dcycle = 3;

int sensor_LUT[16] = {
    0b0000, // STOP
    0b0001, // HARD RIGHT
    0b0010, // ERROR
    0b0011, // RIGHT
    0b0100, // ERROR
    0b0101, // ERROR
    0b0110, // FORWARD
    0b0111, // HARD RIGHT
    0b1000, // HARD LEFT
    0b1001, // ERROR
    0b1010, // ERROR
    0b1011, // ERROR
    0b1100, // LEFT
    0b1101, // ERROR
    0b1110, // HARD LEFT
    0b1111, // STOP on second encounter
};

/***** Helper Functions *****/

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

void PWM_Input_PC7_Init(void) {
	// Set PC7 to alternate function (AF2 for TIM3_CH2)
	GPIOC->MODER &= ~(0x3 << (7 * 2));
	GPIOC->MODER |=  (0x2 << (7 * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
	GPIOC->AFR[0] |=  (0x2 << (7 * 4)); // AF2 = TIM3

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

/// @brief sends pulse to servo with pulsewidth corresponding to the desired angle
/// @param angle angle to send the standard servo to  
void servo_angle_set(int speed) {
    // Assuming speeds from 0 to 4095 (12-bit ADC range)
    // Compute delta in the range 0..200 using integer math without losing precision
    uint32_t delta = ( (uint32_t)speed * 200u ) / 4095u;
    if (stopped) signal_pulse_width = 1500u;
    else if (going_cw) signal_pulse_width = 1480u - delta;
    else signal_pulse_width = delta + 1520u;
    // Write to TIM8 CH1 CCR (PC6 PWM output)
    TIM8->CCR1 = signal_pulse_width;
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

    uart_sendString("\r\nCPEG222: TEAM07Proj4A\r\nCont. Servo\r\n\r\n");
}

/// @brief Displays relevant info on the serial monitor given the current angle
void uart_display() {
    uart_sendString("ADC Value: ");
    uart_send_int32(wiper_val);

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

void calculate_rpm(void) {
    if (current_time > previous_time) {
        time_diff = current_time - previous_time;
    }
    else {
        time_diff = (0xFFFFFFFF - previous_time) + current_time + 1;
    }

    if (current_angle > previous_angle) {
        angle_diff = current_angle - previous_angle;
    }
    else {
        angle_diff = (360 - previous_angle) + current_angle;
    }

    time_diff /= 60000000;
    angle_diff /= 360;

    rpm = (float)angle_diff / (float)time_diff;
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

// TIM3 input capture interrupt handler for PC7 (CH2)
void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_CC2IF) { // Check if CC2IF is set
		TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
        if (!waiting_for_falling) {
            last_rising = TIM3->CCR2;
            // Switch to capture falling edge
            TIM3->CCER |= TIM_CCER_CC2P; // Set to falling edge
            waiting_for_falling = 1;
        } 
        else {
            last_falling = TIM3->CCR2;
            // Compute pulse width taking timer wrap into account
            if (last_falling >= last_rising) {
                if (last_falling - last_rising < 1100) {
                    pulse_width = last_falling - last_rising;
                }
                pulse_width = last_falling - last_rising;
            } 
            else {
                pulse_width = (0xFFFF - last_rising) + last_falling + 1;
            }

            current_angle = (pulse_width - min_pulse_width) * 360 /
                    (max_pulse_width - min_pulse_width);
            current_time = TIM5->CNT;
            calculate_rpm();
            previous_time = current_time;
            previous_angle = current_angle;

            TIM3->CCER &= ~TIM_CCER_CC2P; // Set to rising edge
            waiting_for_falling = 0;
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
        wiper_val = ADC1->DR;      // Read the converted value

        servo_angle_set(wiper_val);
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
                 |  RCC_APB1ENR_TIM3EN;     // Input capture timer

    // Enable APB2 peripherals (ADC, TIM8, SYSCFG)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN     // ADC1 for potentiometer
                 |  RCC_APB2ENR_TIM8EN      // Advanced timer for servo PWM
                 |  RCC_APB2ENR_SYSCFGEN;   // For EXTI button interrupt

    /***** Initializations *****/
    SysTick_Config(16000000 * 1 - 1);  
    NVIC_SetPriority(SysTick_IRQn, 3); 
    TIM2_Config();
    TIM5_Config();

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