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

// system registers
#define SYSCTL_RCGCGPIO_R   (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCPWM_R    (*((volatile uint32_t *)0x400FE640))
#define SYSCTL_RCC_R        (*((volatile uint32_t *)0x400FE060))

// port B registers
#define GPIO_PORTB_AFSEL_R  (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DEN_R    (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_AMSEL_R  (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R   (*((volatile uint32_t *)0x4000552C))

// port F registers
#define GPIO_PORTF_DATA_R   (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R    (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_AFSEL_R  (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_PUR_R    (*((volatile uint32_t *)0x40025510))
#define GPIO_PORTF_DEN_R    (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_LOCK_R   (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_CR_R     (*((volatile uint32_t *)0x40025524))

// SysTick registers
#define NVIC_ST_CTRL_R      (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile uint32_t *)0xE000E018))

// PWM registers
#define PWM0_ENABLE_R       (*((volatile uint32_t *)0x40028008))
#define PWM0_0_CTL_R        (*((volatile uint32_t *)0x40028040))
#define PWM0_0_LOAD_R       (*((volatile uint32_t *)0x40028050))
#define PWM0_0_CMPA_R       (*((volatile uint32_t *)0x40028058))
#define PWM0_0_GENA_R       (*((volatile uint32_t *)0x40028060))

// UART0 data register offset
#define UART0_DR_OFFSET     0x000

// LED colors
#define RED     0x02
#define BLUE    0x04
#define GREEN   0x08
#define YELLOW  0x0A

#define SYSCLK          16000000UL
#define PWM_FREQ        20000UL
#define FS              8000UL

#define PWM_LOAD        ((SYSCLK / (2UL * PWM_FREQ)) - 1UL) // period register
#define SYSTICK_RELOAD  ((SYSCLK / FS) - 1UL)               // fires at 8kHz
#define TICKS_PER_100MS (FS / 10)                            // 800 ticks = 100ms

#define TABLE_SIZE  32
#define MAX_VAL     15

// DMA control table, must be 1024-byte aligned
#pragma DATA_ALIGN(dmaCtrlTable, 1024)
static uint8_t dmaCtrlTable[1024];

// UART RX buffer for DMA
#define UART_RX_BUF_SIZE 4
static volatile uint8_t uartRxBuf[UART_RX_BUF_SIZE];

volatile uint32_t indexFP  = 0;
volatile uint32_t stepFP   = 0;
volatile uint32_t sysTicks = 0;
volatile uint8_t  muted    = 0;
volatile uint8_t  paused   = 0; // 0 = playing, 1 = paused

const uint8_t sineTable[TABLE_SIZE] = {
8,10,11,13,14,15,15,15,
15,15,14,13,11,10,8,6,
5,3,2,1,1,1,1,1,
1,1,2,3,5,6,8,8
};

// Hot Cross Buns: E D C | E D C | C C C C D D D D | E D C
#define SONG_LEN 18
const int song[SONG_LEN][2] = {
    {44, 5},  // E  - Hot
    {42, 5},  // D  - Cross
    {40, 10}, // C  - Buns
    {44, 5},  // E  - Hot
    {42, 5},  // D  - Cross
    {40, 10}, // C  - Buns
    {40, 5},  // C  - One
    {40, 5},  // C  - a
    {40, 5},  // C  - pen
    {40, 5},  // C  - ny
    {42, 5},  // D  - two
    {42, 5},  // D  - a
    {42, 5},  // D  - pen
    {42, 5},  // D  - ny
    {44, 5},  // E  - Hot
    {42, 5},  // D  - Cross
    {40, 10}, // C  - Buns
    {-1, 0}   // end of song
};

// C=red, D=yellow, E=blue
volatile int songIdx = 0;

// send string over UART
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

// re-arm DMA for next block of RX bytes
static void
DMA_ReloadUARTRx(void)
{
    MAP_uDMAChannelTransferSet(
        UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        (void *)(UART0_BASE + UART0_DR_OFFSET), // source: UART DR
        (void *)uartRxBuf,                      // destination: rx buffer
        UART_RX_BUF_SIZE
    );
    MAP_uDMAChannelEnable(UDMA_CHANNEL_UART0RX);
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t i;
    char c;

    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    // DMA block complete
    if(ui32Status & UART_INT_DMARX)
    {
        for(i = 0; i < UART_RX_BUF_SIZE; i++)
        {
            c = (char)uartRxBuf[i];
            MAP_UARTCharPutNonBlocking(UART0_BASE, c); // echo

            // toggle pause on 'p'
            if(c == 'p' || c == 'P')
            {
                paused = !paused;
                if(paused)
                    UARTSend((uint8_t *)"\r\n[PAUSED]  press p to resume\r\n", 31);
                else
                    UARTSend((uint8_t *)"\r\n[PLAYING] press p to pause\r\n", 30);
            }
        }
        DMA_ReloadUARTRx();
    }

    // single keypress that didn't fill a full DMA block
    if(ui32Status & UART_INT_RT)
    {
        while(MAP_UARTCharsAvail(UART0_BASE))
        {
            c = (char)MAP_UARTCharGetNonBlocking(UART0_BASE);
            MAP_UARTCharPutNonBlocking(UART0_BASE, c);

            // toggle pause on 'p'
            if(c == 'p' || c == 'P')
            {
                paused = !paused;
                if(paused)
                    UARTSend((uint8_t *)"\r\n[PAUSED]  press p to resume\r\n", 31);
                else
                    UARTSend((uint8_t *)"\r\n[PLAYING] press p to pause\r\n", 30);
            }
        }
        if(!MAP_uDMAChannelIsEnabled(UDMA_CHANNEL_UART0RX))
            DMA_ReloadUARTRx();
    }
}

// PWM setup
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
    PWM0_0_CMPA_R = PWM_LOAD; // start at 0% duty
    PWM0_0_CTL_R  = 1;
    PWM0_ENABLE_R |= 0x01;
}

// port F setup
void PortFInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x20;
    while ((SYSCTL_RCGCGPIO_R & 0x20) == 0) {}
    GPIO_PORTF_LOCK_R  = 0x4C4F434B; // unlock
    GPIO_PORTF_CR_R    = 0x1F;
    GPIO_PORTF_DIR_R   = 0x0E;       // PF1-3 output, PF0/4 input
    GPIO_PORTF_AFSEL_R = 0x00;
    GPIO_PORTF_PUR_R   = 0x11;       // pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R   = 0x1F;
}

// updates PWM duty each sample to output sine wave
void SysTick_Handler(void)
{
    sysTicks++;
    if (!muted && !paused) {
        uint32_t idx = (indexFP >> 8) % TABLE_SIZE;
        PWM0_0_CMPA_R = PWM_LOAD - (PWM_LOAD * (uint32_t)sineTable[idx]) / MAX_VAL;

        indexFP += stepFP;
        if ((indexFP >> 8) >= TABLE_SIZE) {
            indexFP -= ((uint32_t)TABLE_SIZE << 8);
        }
    } else {
        PWM0_0_CMPA_R = PWM_LOAD / 2; // hold mid-rail during rests
    }
}

// waits given number of ticks, pausing if needed
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
            last = sysTicks; // don't count paused time
        }
    }
}

// SysTick setup
void SysTick_Init(void)
{
    NVIC_ST_CTRL_R    = 0;
    NVIC_ST_RELOAD_R  = SYSTICK_RELOAD;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R    = 0x07;
}

// silent gap between notes
void rest(float dur)
{
    muted   = 1;
    indexFP = 0;
    waitTicks((uint32_t)(dur * TICKS_PER_100MS));
    muted = 0;
}

// play a note by piano key number for a given duration
void note(int keynum, float dur)
{
    float frequency = 440.0f * powf(2.0f, (keynum - 49) / 12.0f);
    stepFP  = (uint32_t)(TABLE_SIZE * frequency * 256.0f / FS + 0.5f);
    indexFP = 0;
    muted   = 0;
    waitTicks((uint32_t)(dur * TICKS_PER_100MS));
}

// set LED color based on note
static void setLED(int keynum)
{
    if      (keynum == 40) GPIO_PORTF_DATA_R = RED;    // C
    else if (keynum == 42) GPIO_PORTF_DATA_R = YELLOW; // D
    else if (keynum == 44) GPIO_PORTF_DATA_R = BLUE;   // E
    else                   GPIO_PORTF_DATA_R = GREEN;  // rest
}

// play one note from the song array and advance index
static void playNext(void)
{
    int keynum = song[songIdx][0];
    int dur    = song[songIdx][1];

    // end of song, loop back
    if(keynum == -1)
    {
        songIdx = 0;
        return;
    }

    setLED(keynum);
    note(keynum, (float)dur);
    rest(1);

    songIdx++;

    if(songIdx >= SONG_LEN || song[songIdx][0] == -1)
        songIdx = 0;
}

// DMA + UART setup
void
UART_DMA_Init(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // 115200 baud, 8N1
    MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    MAP_UARTDMAEnable(UART0_BASE, UART_DMA_RX);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_DMARX | UART_INT_RT);
    MAP_IntEnable(INT_UART0);

    MAP_uDMAEnable();
    MAP_uDMAControlBaseSet(dmaCtrlTable);

    // channel 8 = UART0 RX on TM4C123
    MAP_uDMAChannelAssign(UDMA_CH8_UART0RX);
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX,
        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
        UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    // 8-bit, src fixed, dst increments, arbitrate every 4 bytes
    MAP_uDMAChannelControlSet(
        UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
        UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4
    );

    DMA_ReloadUARTRx();
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

    UART_DMA_Init();

    UARTSend((uint8_t *)"Ready. Press p to pause/play.\r\n", 31);

    while(1)
    {
        if (!paused)
            playNext();
    }
}