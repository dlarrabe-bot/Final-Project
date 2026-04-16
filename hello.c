#include <stdint.h>
#include <math.h>

// System control registers
#define SYSCTL_RCGCGPIO_R (*((volatile uint32_t *)0x400FE608)) // GPIO clock
#define SYSCTL_RCGCPWM_R (*((volatile uint32_t *)0x400FE640)) // PWM clock

// GPIO Port B registers (base: 0x40005000)
#define GPIO_PORTB_AFSEL_R (*((volatile uint32_t *)0x40005420)) // Alt function
#define GPIO_PORTB_DEN_R (*((volatile uint32_t *)0x4000551C)) // Digital enable
#define GPIO_PORTB_AMSEL_R (*((volatile uint32_t *)0x40005528)) // Analog mode
#define GPIO_PORTB_PCTL_R (*((volatile uint32_t *)0x4000552C)) // Port control
#define GPIO_PORTB_DATA_R (*((volatile uint32_t *)0x400053FC)) // I added, may need to remove
#define GPIO_PORTB_DIR_R (*((volatile uint32_t *)0x40005400)) // I added, may need to remove

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

#define SYSCLK 16000000 // 16 MHz default clock (no PLL)
#define TONE_HZ 440 // Frequency in Hz
#define COUNTFLAG (1U << 16) // Bit 16 of NVIC_ST_CTRL_R

#define TABLE_SIZE 32
const uint8_t sineTable[TABLE_SIZE] = {
8,10,11,13,14,15,15,15,
15,15,14,13,11,10,8,6,
5,3,2,1,1,1,1,1,
1,1,2,3,5,6,8,8
};

volatile uint16_t tableIndex = 0;
volatile uint16_t STEP = 1;
volatile uint32_t remaining_ticks;  
volatile int delaying = 0;



void SysTick_Handler(void) {
    if (delaying == 0){
        static uint8_t toggle = 0;
        if (toggle == 0) {
            GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~0x0F) | 0x0F; // high
        } else {
            GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~0x0F) | 0x00; // low
        }
        toggle ^= 1; //toggles the variable "toggle"
    }
    if (remaining_ticks > 0) remaining_ticks--;

}


/*
void SysTick_Handler(void) {
    // Write next sine sample directly no inversion needed
    GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R & ~0x0F)
    | (sineTable[tableIndex] & 0x0F);
    tableIndex += STEP;
    if (tableIndex >= TABLE_SIZE) {
        tableIndex -= TABLE_SIZE;
    }
    if (remaining_ticks > 0) remaining_ticks--;

}
*/



void R2R_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x02;      
    while ((SYSCTL_RCGCGPIO_R & 0x02) == 0) {}
    GPIO_PORTB_DIR_R |= 0x0F;       
    GPIO_PORTB_DEN_R |= 0x0F;       
    GPIO_PORTB_AMSEL_R &= ~0x0F;    
}

void SysTick_Init() {
    NVIC_ST_CTRL_R = 0; // Disable SysTick during setup
    NVIC_ST_RELOAD_R = (SYSCLK/8000)-1; // Set reload value, 8000 is sampling frequency
    NVIC_ST_CURRENT_R = 0; // Clear current value and COUNTFLAG
    NVIC_ST_CTRL_R = 0x07; // Enable + core clock
}


//void genSine(int nbits, int samples){
    
//    sineTable[nbits] = (int)7*sin( 2*3.1415*nbits*32 )+ 8;
//}

void delay(float dur){
    remaining_ticks = (uint32_t)(2.0f * 8000 * dur);
    delaying = 1;
    NVIC_ST_CTRL_R &= ~0x02;   
    NVIC_ST_RELOAD_R = (SYSCLK/8000)-1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |= 0x02;

    while (remaining_ticks > 0){

    }
    delaying = 0;

}

void note (int keynum, float dur){
    float frequency = TONE_HZ*pow(2, (keynum - 49)/12.0);
    remaining_ticks = (uint32_t)(2.0f * frequency * dur);
    STEP = (int)(32*(frequency)/(8000)+.5);
    uint32_t new_reload = (SYSCLK / (2 * frequency)) - 1;
    
    NVIC_ST_CTRL_R &= ~0x02;   
    NVIC_ST_RELOAD_R = new_reload;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |= 0x02;

    while (remaining_ticks > 0){

    }

}

//plays c major scale
void scale (void){
    note(40, 1);
    note(42, .5);
    note(44, .5);
    note(45, .5);
    note(47, .5);
    note(49, .5);
    note(51, .5);
    note(52, 1);
}

void twinkle(void){
    //C C G G A A F
    note (40, .25);
    delay(.05);
    note (40, .25);
    delay(.05);

    note (47, .25);
    delay(.05);
    note (47, .25);
    delay(.05);

    note (37, .25);
    delay(.05);
    note (37, .25);
    delay(.05);

    note (47, .5);
    delay(.05);


    //F F E E D D C
    note (45, .25);
    delay(.05);
    note (45, .25);
    delay(.05);

    note (44, .25);
    delay(.05);
    note (44, .25);
    delay(.05);

    note (42, .25);
    delay(.05);
    note (42, .25);
    delay(.05);

    note (40, .5);
    delay(.05);

    //G G F F E E D x2
    int i;
    for (i=0; i<2; i++){
        note (47, .25);
        delay(.05);
        note (47, .25);
        delay(.05);

        note (45, .25);
        delay(.05);
        note (45, .25);
        delay(.05);

        note (44, .25);
        delay(.05);
        note (44, .25);
        delay(.05);

        note (42, .5);
        delay(.05);
    }
}


int main(void) {
    R2R_Init();  
    SysTick_Init();
    scale();
    twinkle();
}
