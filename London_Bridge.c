#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_udma.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/udma.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// System control registers
#define SYSCTL_RCGCGPIO_R   (*((volatile uint32_t *)0x400FE608)) // GPIO clock
#define SYSCTL_RCGCPWM_R    (*((volatile uint32_t *)0x400FE640)) // PWM clock
#define SYSCTL_RCC_R        (*((volatile uint32_t *)0x400FE060))

// GPIO Port B registers (base: 0x40005000)
#define GPIO_PORTB_AFSEL_R  (*((volatile uint32_t *)0x40005420)) // Alt function
#define GPIO_PORTB_DEN_R    (*((volatile uint32_t *)0x4000551C)) // Digital enable
#define GPIO_PORTB_AMSEL_R  (*((volatile uint32_t *)0x40005528)) // Analog mode
#define GPIO_PORTB_PCTL_R   (*((volatile uint32_t *)0x4000552C)) // Port control

// Port F GPIO registers (base: 0x40025000)
#define GPIO_PORTF_DATA_R   (*((volatile uint32_t *)0x400253FC)) // Data
#define GPIO_PORTF_DIR_R    (*((volatile uint32_t *)0x40025400)) // Direction
#define GPIO_PORTF_AFSEL_R  (*((volatile uint32_t *)0x40025420)) // Alt function
#define GPIO_PORTF_PUR_R    (*((volatile uint32_t *)0x40025510)) // Pull-up
#define GPIO_PORTF_DEN_R    (*((volatile uint32_t *)0x4002551C)) // Digital enable
#define GPIO_PORTF_LOCK_R   (*((volatile uint32_t *)0x40025520)) // Lock
#define GPIO_PORTF_CR_R     (*((volatile uint32_t *)0x40025524)) // Commit

// SysTick registers (Cortex-M core)
#define NVIC_ST_CTRL_R      (*((volatile uint32_t *)0xE000E010)) // Control/Status
#define NVIC_ST_RELOAD_R    (*((volatile uint32_t *)0xE000E014)) // Reload value
#define NVIC_ST_CURRENT_R   (*((volatile uint32_t *)0xE000E018)) // Current value

// PWM Module 0, Generator 0 registers (base: 0x40028000)
#define PWM0_ENABLE_R       (*((volatile uint32_t *)0x40028008)) // PWM output enable
#define PWM0_0_CTL_R        (*((volatile uint32_t *)0x40028040)) // Generator control
#define PWM0_0_LOAD_R       (*((volatile uint32_t *)0x40028050)) // Load (period)
#define PWM0_0_CMPA_R       (*((volatile uint32_t *)0x40028058)) // Compare A (duty)
#define PWM0_0_GENA_R       (*((volatile uint32_t *)0x40028060)) // Generator A action

// UART0 data register offset
#define UART0_DR_OFFSET     0x000

// London Bridge (key of G):
// G A G F E F G | D E F | E F G
// G4=47, A4=49, F4=45, E4=44, D4=42
// LED colors per note
#define RED     0x02  // E = red
#define BLUE    0x04  // D = blue
#define MAGENTA 0x06  // A = magenta
#define GREEN   0x08  // G = green
#define CYAN    0x0C  // F = cyan

#define SYSCLK          16000000UL
#define PWM_FREQ        20000UL
#define FS              8000UL

#define PWM_LOAD        ((SYSCLK / (2UL * PWM_FREQ)) - 1UL) // pwm clock = SYSCLK/2, LOAD = (SYSCLK/2)/PWM_FREQ - 1 = 399
#define SYSTICK_RELOAD  ((SYSCLK / FS) - 1UL)               // SysTick fires at FS -> RELOAD = SYSCLK/FS - 1 = 1999
#define TICKS_PER_100MS (FS / 10)                            // 800 ticks = 100 ms

#define TABLE_SIZE  32
#define MAX_VAL     15

// DMA control table 
#pragma DATA_ALIGN(dmaCtrlTable, 1024)
static uint8_t dmaCtrlTable[1024];

// DMA receive buffer for UART0 RX
#define UART_RX_BUF_SIZE 4
static volatile uint8_t uartRxBuf[UART_RX_BUF_SIZE];

volatile uint32_t indexFP  = 0;
volatile uint32_t stepFP   = 0;
volatile uint32_t sysTicks = 0;
volatile uint8_t  muted    = 0;
volatile uint8_t  paused   = 0; // 1 = user paused via 'p' keypress, 0 = playing

const uint8_t sineTable[TABLE_SIZE] = {
8,10,11,13,14,15,15,15,
15,15,14,13,11,10,8,6,
5,3,2,1,1,1,1,1,
1,1,2,3,5,6,8,8
};

// London Bridge melody progression map:
// 1  = G  (Lon-)
// 2  = A  (-don)
// 3  = G  (Bridge)
// 4  = F  (is)
// 5  = E  (fall-)
// 6  = F  (-ing)
// 7  = G  (down,)
// 8  = D  (fall-)
// 9  = E  (-ing)
// 10 = F  (down,)
// 11 = E  (fall-)
// 12 = F  (-ing)
// 13 = G  (down,)
// 14 = G  (Lon-)
// 15 = A  (-don)
// 16 = G  (Bridge)
// 17 = F  (is)
// 18 = E  (fall-)
// 19 = F  (-ing)
// 20 = G  (down!) -- half note, then loop
//
// state: 1=G, 2=A, 3=F, 4=E, 5=D

volatile int progression = 1;
volatile int state       = 1; // start on G

// Send a string to the UART
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send
    while(ui32Count--)
    {
        // Write the next character to the UART
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

// Reload DMA channel for next block of UART RX bytes
static void
DMA_ReloadUARTRx(void)
{
    MAP_uDMAChannelTransferSet(
        UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        (void *)(UART0_BASE + UART0_DR_OFFSET), // source: UART0 data register (fixed)
        (void *)uartRxBuf,                      // destination: our rx buffer
        UART_RX_BUF_SIZE                        // number of bytes to transfer
    );
    MAP_uDMAChannelEnable(UDMA_CHANNEL_UART0RX);
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t i;
    char c;

    // Read and clear the asserted interrupts
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    // DMA finished transferring a full block of bytes
    if(ui32Status & UART_INT_DMARX)
    {
        // Loop through every byte the DMA deposited
        for(i = 0; i < UART_RX_BUF_SIZE; i++)
        {
            c = (char)uartRxBuf[i];
            MAP_UARTCharPutNonBlocking(UART0_BASE, c); // echo back

            // Toggle pause/play on 'p'
            if(c == 'p' || c == 'P')
            {
                paused = !paused;
                if(paused)
                    UARTSend((uint8_t *)"\r\n[PAUSED]  press p to resume\r\n", 31);
                else
                    UARTSend((uint8_t *)"\r\n[PLAYING] press p to pause\r\n", 30);
            }
        }
        // Reload DMA for the next block
        DMA_ReloadUARTRx();
    }

    if(ui32Status & UART_INT_RT)
    {
        // Loop while there are characters in the receive FIFO
        while(MAP_UARTCharsAvail(UART0_BASE))
        {
            // Read the next character from the UART and write it back
            c = (char)MAP_UARTCharGetNonBlocking(UART0_BASE);
            MAP_UARTCharPutNonBlocking(UART0_BASE, c);

            // Toggle pause/play on 'p'
            if(c == 'p' || c == 'P')
            {
                paused = !paused;
                if(paused)
                    UARTSend((uint8_t *)"\r\n[PAUSED]  press p to resume\r\n", 31);
                else
                    UARTSend((uint8_t *)"\r\n[PLAYING] press p to pause\r\n", 30);
            }
        }
        // Reload DMA
        if(!MAP_uDMAChannelIsEnabled(UDMA_CHANNEL_UART0RX))
            DMA_ReloadUARTRx();
    }
}

// PWM initialisation
void PWM_Init(void)
{
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
    PWM0_0_CMPA_R = PWM_LOAD; // 0% duty to start
    PWM0_0_CTL_R  = 1;
    PWM0_ENABLE_R |= 0x01;
}

// Port F initialisation
void PortFInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x20;                  // Enable clock for Port F
    while ((SYSCTL_RCGCGPIO_R & 0x20) == 0) {}  // Wait for clock
    GPIO_PORTF_LOCK_R  = 0x4C4F434B;            // Unlock Port F
    GPIO_PORTF_CR_R    = 0x1F;                  // Allow changes to PF4-PF0
    GPIO_PORTF_DIR_R   = 0x0E;                  // PF1-PF3 output, PF4,PF0 input
    GPIO_PORTF_AFSEL_R = 0x00;                  // Disable alternate functions
    GPIO_PORTF_PUR_R   = 0x11;                  // Enable pull-up on PF4 and PF0
    GPIO_PORTF_DEN_R   = 0x1F;                  // Enable digital I/O on PF4-PF0
}

void SysTick_Handler(void)
{
    sysTicks++;   // always increment for timing
    if (!muted && !paused) { // silence when muted (rests) OR when user has paused playback
        uint32_t idx = (indexFP >> 8) % TABLE_SIZE;
        PWM0_0_CMPA_R = PWM_LOAD - (PWM_LOAD * (uint32_t)sineTable[idx]) / MAX_VAL;

        indexFP += stepFP;
        if ((indexFP >> 8) >= TABLE_SIZE) {
            indexFP -= ((uint32_t)TABLE_SIZE << 8);
        }
    } else {
        PWM0_0_CMPA_R = PWM_LOAD / 2; // hold at mid-rail during rests
    }
}

static void waitTicks(uint32_t ticks)
{
    uint32_t counted = 0;
    uint32_t last    = sysTicks;

    while (counted < ticks) {
        if (!paused) {
            uint32_t now = sysTicks;
            counted += (now - last);
            last = now;
        } else {
            last = sysTicks; // reset reference so paused time is not counted
        }
    }
}

// SysTick initialisation
void SysTick_Init(void)
{
    NVIC_ST_CTRL_R    = 0;
    NVIC_ST_RELOAD_R  = SYSTICK_RELOAD; // 1999
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R    = 0x07;
}

void rest(float dur) // 10 for 1 sec
{
    muted   = 1;
    indexFP = 0;

    waitTicks((uint32_t)(dur * TICKS_PER_100MS));

    muted = 0;
}

void note(int keynum, float dur)
{
    float frequency = 440.0f * powf(2.0f, (keynum - 49) / 12.0f);
    stepFP  = (uint32_t)(TABLE_SIZE * frequency * 256.0f / FS + 0.5f);

    indexFP = 0;
    muted   = 0;

    waitTicks((uint32_t)(dur * TICKS_PER_100MS));
}

// DMA + UART initialisation
void
UART_DMA_Init(void)
{
    // Enable UART0, Port A, and uDMA peripherals
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Configure PA0/PA1 for UART function
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UART0: 115200 baud, 8N1
    MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable DMA on UART0 RX
    MAP_UARTDMAEnable(UART0_BASE, UART_DMA_RX);

    // Unmask DMA-done and RX-timeout interrupts
    MAP_UARTIntEnable(UART0_BASE, UART_INT_DMARX | UART_INT_RT);
    MAP_IntEnable(INT_UART0);

    // Enable uDMA controller and set control table base address
    MAP_uDMAEnable();
    MAP_uDMAControlBaseSet(dmaCtrlTable);

    // Assign channel 8 to UART0 RX (Table 9-1 in TM4C123 datasheet)
    MAP_uDMAChannelAssign(UDMA_CH8_UART0RX);

    // Clear all channel attributes before configuring
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX,
        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
        UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    // 8-bit transfers, source fixed (UART DR), destination increments by 1 byte
    // arbitrate every 4 bytes
    MAP_uDMAChannelControlSet(
        UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
        UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4
    );

    // Arm the first DMA transfer
    DMA_ReloadUARTRx();
}

// G note (keynum 47) - LED green
void G(void)
{
    GPIO_PORTF_DATA_R = GREEN;

    // "down!" final note is a half note
    if (progression == 20) {
        note(47, 10);
    } else {
        note(47, 5);
    }

    rest(1);

    if (progression == 1)  { progression++; state = 2; return; } // next A
    if (progression == 3)  { progression++; state = 3; return; } // next F
    if (progression == 7)  { progression++; state = 5; return; } // next D
    if (progression == 13) { progression++; state = 1; return; } // next G
    if (progression == 14) { progression++; state = 2; return; } // next A
    if (progression == 16) { progression++; state = 3; return; } // next F
    if (progression == 20) { progression = 1; state = 1; return; } // loop back
}

// A note (keynum 49) - LED magenta
void A(void)
{
    GPIO_PORTF_DATA_R = MAGENTA;
    note(49, 5);
    rest(1);

    if (progression == 2)  { progression++; state = 1; return; } // next G
    if (progression == 15) { progression++; state = 1; return; } // next G
}

// F note (keynum 45) - LED cyan
void F(void)
{
    GPIO_PORTF_DATA_R = CYAN;
    note(45, 5);
    rest(1);

    if (progression == 4)  { progression++; state = 4; return; } // next E
    if (progression == 6)  { progression++; state = 1; return; } // next G
    if (progression == 10) { progression++; state = 4; return; } // next E
    if (progression == 12) { progression++; state = 1; return; } // next G
    if (progression == 17) { progression++; state = 4; return; } // next E
    if (progression == 19) { progression++; state = 1; return; } // next G (final)
}

// E note (keynum 44) - LED red
void E(void)
{
    GPIO_PORTF_DATA_R = RED;
    note(44, 5);
    rest(1);

    if (progression == 5)  { progression++; state = 3; return; } // next F
    if (progression == 9)  { progression++; state = 3; return; } // next F
    if (progression == 11) { progression++; state = 3; return; } // next F
    if (progression == 18) { progression++; state = 3; return; } // next F
}

// D note (keynum 42) - LED blue
void D(void)
{
    GPIO_PORTF_DATA_R = BLUE;
    note(42, 5);
    rest(1);

    if (progression == 8)  { progression++; state = 4; return; } // next E
}

int main(void)
{
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();

    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    PWM_Init();
    SysTick_Init();
    PortFInit();

    MAP_IntMasterEnable();

    // Replaces the old manual UART + interrupt setup
    UART_DMA_Init();

    UARTSend((uint8_t *)"Ready. Press p to pause/play.\r\n", 31);

    while(1)
    {
        if (!paused) {
            if (state == 1) G();
            if (state == 2) A();
            if (state == 3) F();
            if (state == 4) E();
            if (state == 5) D();
        }
    }
}
