#include <stdint.h>
#include <math.h>

// System control registers
#define SYSCTL_RCGCGPIO_R (*((volatile uint32_t *)0x400FE608)) // GPIO clock
#define SYSCTL_RCGCPWM_R (*((volatile uint32_t *)0x400FE640)) // PWM clock
#define SYSCTL_RCC_R        (*((volatile uint32_t *)0x400FE060))

// GPIO Port B registers (base: 0x40005000)
#define GPIO_PORTB_AFSEL_R (*((volatile uint32_t *)0x40005420)) // Alt function
#define GPIO_PORTB_DEN_R (*((volatile uint32_t *)0x4000551C)) // Digital enable
#define GPIO_PORTB_AMSEL_R (*((volatile uint32_t *)0x40005528)) // Analog mode
#define GPIO_PORTB_PCTL_R (*((volatile uint32_t *)0x4000552C)) // Port control


// Port F GPIO registers (base: 0x40025000)
#define GPIO_PORTF_DATA_R (*((volatile uint32_t *)0x400253FC)) // Data
#define GPIO_PORTF_DIR_R (*((volatile uint32_t *)0x40025400)) // Direction
#define GPIO_PORTF_AFSEL_R (*((volatile uint32_t *)0x40025420)) // Alt function
#define GPIO_PORTF_PUR_R (*((volatile uint32_t *)0x40025510)) // Pull-up
#define GPIO_PORTF_DEN_R (*((volatile uint32_t *)0x4002551C)) // Digital enable
#define GPIO_PORTF_LOCK_R (*((volatile uint32_t *)0x40025520)) // Lock
#define GPIO_PORTF_CR_R (*((volatile uint32_t *)0x40025524)) // Commit


// SysTick registers (Cortex-M core)
#define NVIC_ST_CTRL_R (*((volatile uint32_t *)0xE000E010)) // Control/Status
#define NVIC_ST_RELOAD_R (*((volatile uint32_t *)0xE000E014)) // Reload value
#define NVIC_ST_CURRENT_R (*((volatile uint32_t *)0xE000E018)) // Current value


// PWM Module 0, Generator 0 registers (base: 0x40028000)
#define PWM0_ENABLE_R (*((volatile uint32_t *)0x40028008)) // PWM output enable
#define PWM0_0_CTL_R (*((volatile uint32_t *)0x40028040)) // Generator control
#define PWM0_0_LOAD_R (*((volatile uint32_t *)0x40028050)) // Load (period)
#define PWM0_0_CMPA_R (*((volatile uint32_t *)0x40028058)) // Compare A (duty)
#define PWM0_0_GENA_R (*((volatile uint32_t *)0x40028060)) // Generator A action

#define TONE_HZ 440 // Frequency in Hz

#define SYSCLK          16000000UL  
#define PWM_FREQ        20000UL     
#define FS              8000UL 

#define PWM_LOAD        ((SYSCLK / (2UL * PWM_FREQ)) - 1UL) // pwm clock = SYSCLK/2, LOAD = (SYSCLK/2)/PWM_FREQ - 1 = 399
#define SYSTICK_RELOAD  ((SYSCLK / FS) - 1UL) // SysTick fires at FS  ->  RELOAD = SYSCLK/FS - 1 = 1999
#define TICKS_PER_100MS (FS / 10)   // 800 ticks = 100 ms


#define TABLE_SIZE  32
#define N_BITS      4
#define MAX_VAL     15 

volatile uint32_t indexFP = 0;
volatile uint32_t stepFP  = 0;

volatile uint32_t sysTicks = 0;

volatile uint8_t muted = 0;

const uint8_t sineTable[TABLE_SIZE] = {
8,10,11,13,14,15,15,15,
15,15,14,13,11,10,8,6,
5,3,2,1,1,1,1,1,
1,1,2,3,5,6,8,8
};

volatile uint16_t progression = 1;
volatile int ending = 0;

// pwm initilisation
void PWM_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCPWM_R  |= 0x01;
    while ((SYSCTL_RCGCGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_RCGCPWM_R  & 0x01) == 0) {}

    // pwm clock = SYSCLK / 2
    SYSCTL_RCC_R |=  (1UL << 20);
    SYSCTL_RCC_R &= ~(0x7UL << 17);

    GPIO_PORTB_AFSEL_R |=  0x40;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xF0FFFFFFUL) | 0x04000000UL;
    GPIO_PORTB_DEN_R   |=  0x40;
    GPIO_PORTB_AMSEL_R &= ~0x40;

    PWM0_0_CTL_R  = 0;
    PWM0_0_GENA_R = 0x8C;
    PWM0_0_LOAD_R = PWM_LOAD;
    PWM0_0_CMPA_R = PWM_LOAD;   // 0% duty to start
    PWM0_0_CTL_R  = 1;
    PWM0_ENABLE_R |= 0x01;
}



void SysTick_Handler(void) {
    sysTicks++;   // always increment for timing

    if (!muted) {
        uint32_t idx = (indexFP >> 8) % TABLE_SIZE;
        PWM0_0_CMPA_R = PWM_LOAD - (PWM_LOAD * (uint32_t)sineTable[idx]) / MAX_VAL;

        indexFP += stepFP;
        if ((indexFP >> 8) >= TABLE_SIZE) {
            indexFP -= ((uint32_t)TABLE_SIZE << 8);
        }
    } else {
        PWM0_0_CMPA_R = PWM_LOAD / 2;   // hold at mid-rail during rests
    }
}



static void waitTicks(uint32_t ticks) {
    uint32_t start = sysTicks;
    while ((sysTicks - start) < ticks) {}
}


// systick initialisation
void SysTick_Init(void) {
    NVIC_ST_CTRL_R    = 0;
    NVIC_ST_RELOAD_R  = SYSTICK_RELOAD;  // 1999
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R    = 0x07;           
}





void rest(float dur) { // 10 for 1 sec
    muted   = 1;
    indexFP = 0;

    waitTicks((uint32_t)(dur * TICKS_PER_100MS));

    muted = 0;
}

void note (int keynum, float dur){
    float frequency = TONE_HZ*pow(2, (keynum - 49)/12.0);
    stepFP  = (uint32_t)(TABLE_SIZE * frequency * 256.0f / FS + 0.5f);

    indexFP = 0;
    muted   = 0;

    waitTicks((uint32_t)(dur * TICKS_PER_100MS));

}


void G();
void C();
void F();
void E();
void A();
void D();


void G(){


    if (progression == 7){
        note(47,10);
    } else {
         note(47, 5);
    }

    //GPIO_PORTB_DATA_R = RED;
    rest(1);
    if ((progression == 1) || (progression == 3)) {
        progression++;
        G();
    }
    if ((progression == 2) || (progression == 7)) {
        progression = 1;
        F();
    }
    if ((progression == 4)) {
        progression++;
        A();
    }
}

void C(){
    if (progression == 7){
        note(40,10);
    } else {
         note(40, 5);
    }

    rest(1);
    if ((progression == 1)) {
        progression++;
        C();
    }
    if ((progression == 2)) {
        progression++;
        G();
    }
    if ((progression == 7)) {
        progression = 1;
        G();
    }
}

void F(){
    note(45, 5);
    rest(1);
    if ((progression == 1) || (progression == 3)) {
        progression++;
        F();
    }
    if ((progression == 2) || (progression == 4)) {
        progression++;
        E();
    }
}

void E(){
    note(44, 5);
    rest(1);
    if ((progression == 3) || (progression == 5)) {
        progression++;
        E();
    }
    if ((progression == 4) || (progression == 6)) {
        progression++;
        D();
    }
}

void A(){
    note(49, 5);
    rest(1);
    if ((progression == 6)) {
        progression++;
        G();
    }
    if ((progression == 5)) {
        progression++;
        A();
    }
}

void D(){
    if (progression == 7){
        note(42,10);
    } else {
         note(42, 5);
    }

    rest(1);
    if ((progression == 6)) {
        progression++;
        C();
    }
    if ((progression == 7)) {
        if (ending == 0){
            progression = 1;
            ending = 1;
            G();
        }
        if (ending == 1){
            progression = 1;
            ending = 0;
            C();
        }
    }
    if ((progression == 5)) {
        progression++;
        D();
    }
}




int main(void) {
    PWM_Init();                    
    SysTick_Init();               

    while(1){
        C();       
    }
    
}