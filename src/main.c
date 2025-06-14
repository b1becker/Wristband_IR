#include <stdio.h>
#include "stm32l432xx.h"
#include "lab1_helpers.h"
#include <stdbool.h>

#define PWM_FREQ       38000U
#define TIMER_CLOCK    SystemCoreClock

// UART hook for printf()
int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

// DWT-based microsecond delay
static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (TIMER_CLOCK / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {}
}

// 38 kHz IR carrier via TIM1_CH1 on PA8
static void ir_timer_init(void) {
    // enable clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // PA8 = AF1 (TIM1_CH1), push-pull, very-high speed
    GPIOA->MODER   = (GPIOA->MODER & ~(3U<<(2*8))) | (2U<<(2*8));
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU<<0))   | (1U<<0);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U<<(2*8))) | (3U<<(2*8));
    GPIOA->OTYPER  &= ~(1U<<8);
    GPIOA->PUPDR   &= ~(3U<<(2*8));
    // PSC & ARR for ~38 kHz
    const uint32_t presc = 7;  // SYSCLK/8 → 10 MHz
    TIM1->PSC = presc;
    TIM1->ARR = (TIMER_CLOCK/(presc+1)/PWM_FREQ) - 1;  // ≈263
    // 50% duty
    TIM1->CCR1 = (TIM1->ARR + 1)/2;
    // PWM mode 1 + preload
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;
    TIM1->CCER  = TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    // start timer
    TIM1->CR1 = TIM_CR1_ARPE|TIM_CR1_CEN;
}
static inline void ir_carrier_on(void)  { TIM1->CCER |=  TIM_CCER_CC1E; }
static inline void ir_carrier_off(void) { TIM1->CCER &= ~TIM_CCER_CC1E; }

// Test 1: Continuous / blinking patterns
static void test_signal_types(void) {
    printf("\r\n=== Signal Type Tests ===\r\n");
    // Continuous 5 s on
    printf("1) Continuous ON for 5s...\r\n");
    ir_carrier_on();
    delay_us(5000000);
    ir_carrier_off();
    printf("   Done\r\n\n");
    // Slow blink 500 ms ×10
    printf("2) Slow blink 500ms ×10...\r\n");
    for (int i = 0; i < 10; i++) {
      ir_carrier_on();  delay_us(500000);
      ir_carrier_off(); delay_us(500000);
    }
    printf("   Done\r\n\n");
    // Fast blink 50 ms ×20
    printf("3) Fast blink 50ms ×20...\r\n");
    for (int i = 0; i < 20; i++) {
      ir_carrier_on();  delay_us(50000);
      ir_carrier_off(); delay_us(50000);
    }
    printf("   Done\r\n\n");
    // Very fast 1 ms ×100
    printf("4) Very fast blink 1ms ×100...\r\n");
    for (int i = 0; i < 100; i++) {
      ir_carrier_on();  delay_us(1000);
      ir_carrier_off(); delay_us(1000);
    }
    printf("   Done\r\n");
}

// Test 2: Voltage levels
static void test_voltage_levels(void) {
    printf("\r\n=== Voltage Level Tests ===\r\n");
    printf("Carrier OFF → measure ~0V for 10s...\r\n");
    ir_carrier_off();
    delay_us(10000000);
    printf("Carrier ON → measure PWM ~3.3V for 10s...\r\n");
    ir_carrier_on();
    delay_us(10000000);
    ir_carrier_off();
    printf("   Done\r\n");
}

// Test 3: Drive modes
static void test_drive_modes(void) {
    printf("\r\n=== Drive Mode Tests ===\r\n");
    // Push-pull pulses
    printf("Push-pull pulses (5 × 20ms)...\r\n");
    for (int i = 0; i < 5; i++) {
      ir_carrier_on();  delay_us(20000);
      ir_carrier_off(); delay_us(20000);
    }
    // Open-drain pulses
    printf("Switching PA8 to open-drain...\r\n");
    GPIOA->OTYPER |=  (1U<<8);
    for (int i = 0; i < 5; i++) {
      ir_carrier_on();  delay_us(20000);
      ir_carrier_off(); delay_us(20000);
    }
    // Restore push-pull
    GPIOA->OTYPER &= ~(1U<<8);
    printf("   Done\r\n");
}

// Toggle the 38 kHz carrier on/off per raw[i]:
void send_raw(const uint16_t *raw, int len) {
  for (int i = 0; i < len; i++) {
    if ((i & 1) == 0) ir_carrier_on();
    else            ir_carrier_off();
    delay_us(raw[i]);
  }
  ir_carrier_off();  // ensure we finish with carrier off
}



int main(void) {
    // UART2 + DWT
    host_serial_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    // IR PWM
    ir_timer_init();
    printf("\r\n>> PixMob IR HW Test Suite\r\n");

    while (1) {
        uint16_t wake_prefix[] = {
        700, 700,        // ‘1’:   carrier on 700 µs, off 700 µs
        700, 700,        // ‘0’:   carrier off 700 µs, on 700 µs
        700, 700,        // repeat for nine zeros…
        700, 700,
        700, 700,
        700, 700,
        700, 700,
        700, 700,
        700, 700,
        700, 700,
        700, 700
        };
        send_raw(wake_prefix, sizeof(wake_prefix)/sizeof(*wake_prefix));
        uint16_t red_packet[] = {38, 6, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2,
               6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6, 2, 6};


        send_raw(red_packet, sizeof(red_packet)/sizeof(*red_packet));
        // test_signal_types();
        delay_us(1000000);
        // test_voltage_levels();
        // delay_us(1000000);
        // test_drive_modes();
        // delay_us(5000000);
    }
    // never reached
    return 0;
}
