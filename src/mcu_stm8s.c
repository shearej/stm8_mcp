/**
  ******************************************************************************
  * @file mcu_stm8s.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
//#include <stm8s.h>

// app headers
#include "system.h" // platform specific delarations


/* Private defines -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


/*
 * some of this bulk could be reduced with macros
 * or otherwise would be suitable for STM8 Peripheral Library.
 * Noting the situation regarding unused IO pins ... I am trying to assert the 
 * configuration on the few pins needed as GPIO (general purpose IO and noting 
 * that the specific module initialization (A/D, TIMx - and these are done
 * mostly with STM8 PL) will handle setting up suitable IO pin behavior.

 * 11.4 Reset configuration
 *  All I/O pins are generally input floating under reset (i.e. during the reset phase) and at reset
 *  state (i.e. after reset release). However, a few pins may have a different behavior. Refer to
 *  the datasheet pinout description for all details.
 * 11.5 Unused I/O pins
 *  Unused I/O pins must not be left floating to avoid extra current consumption. They must be
 *  put into one of the following configurations:
 *  connected to VDD or VSS by external pull-up or pull-down resistor and kept as input
 *  floating (reset state), configured as input with internal pull-up/down resistor,
 *  configured as output push-pull low.
*/
void GPIO_Config(void)
{
// OUTPUTS
// built-in LED
    GPIOD->ODR |= (1 << LED); //LED initial state is OFF (cathode driven to Vcc)
    GPIOD->DDR |= (1 << LED); //PD.n as output
    GPIOD->CR1 |= (1 << LED); //push pull output

// Controls the /SD pins of IR2104s on PC5, PC7, PG1
///////////  // tried E2, E0, D1 but E2 not work as output ... ???
// C5, C7, and G1 are CN2 pin 6, 8, 12 so 3 leads can go in one connector shell ;)
#if 0
    GPIOC->ODR &=  ~(1<<5);
    GPIOC->DDR |=  (1<<5);
    GPIOC->CR1 |=  (1<<5);

    GPIOC->ODR &=  ~(1<<7);
    GPIOC->DDR |=  (1<<7);
    GPIOC->CR1 |=  (1<<7);

    GPIOG->ODR &=  ~(1<<1);
    GPIOG->DDR |=  (1<<1);
    GPIOG->CR1 |=  (1<<1);
#endif

// SD/A = PD2 SD/B=PD1  SD/C=PA5  .... however D1 not working for me as intended, E0 instead (GN: 10-2)
    GPIOD->ODR &=  ~(1<<2);
    GPIOD->DDR |=  (1<<2);
    GPIOD->CR1 |=  (1<<2);

    GPIOD->ODR &=  ~(1<<1); //?
    GPIOD->DDR |=  (1<<1); //?
    GPIOD->CR1 |=  (1<<1); //?

GPIOE->ODR &=  ~(1<<0); //E0 data 
    GPIOE->DDR |=  (1<<0); //E0 dir
    GPIOE->CR1 |=  (1<<0); //E0	cfg

    GPIOA->ODR &= ~(1 << 5); // set pin off 
    GPIOA->DDR |= (1 << 5);  // PA.5 as OUTPUT
    GPIOA->CR1 |= (1 << 5);  // push pull output


////////////
// use PG0 (CN2-11) as test pins
    GPIOG->ODR &= ~(1<<0);
    GPIOG->DDR |=  (1<<0);
    GPIOG->CR1 |=  (1<<0);


    GPIOC->ODR |=  (1<<4);  // PC4 .. tmp test
    GPIOC->DDR |=  (1<<4);
    GPIOC->CR1 |=  (1<<4);

// INPUTS
// PA4 as button input (Stop)
    GPIOA->DDR &= ~(1 << 4); // PA.4 as input
    GPIOA->CR1 |= (1 << 4);  // pull up w/o interrupts
// uses CN2.7 as GND

// PA6 as button input (B+)
    GPIOA->DDR &= ~(1 << 6); // PA.6 as input
    GPIOA->CR1 |= (1 << 6);  // pull up w/o interrupts

// PE5 as button input (B-)
    GPIOE->DDR &= ~(1 << 5); // PE.5 as input
    GPIOE->CR1 |= (1 << 5);  // pull up w/o interrupts

#if 0 // UartX_Init()
// UART2 D5: Rx, D6: Tx
    GPIOD->DDR &= ~(1 << 5); // PD.5 as input
    GPIOD->CR1 |= (1 << 5);  // pull up w/o interrupts
    GPIOD->DDR |= (1 << 6);  // PD.6 as output
    GPIOD->CR1 |= (1 << 6);  // push pull output
    GPIOD->ODR |= (1 << 6);  // use as hi-side of button
#endif

// PE.6 AIN9
    GPIOE->DDR &= ~(1 << 6);  // PE.6 as input
    GPIOE->CR1 &= ~(1 << 6);  // floating input
    GPIOE->CR2 &= ~(1 << 6);  // 0: External interrupt disabled   ???

// PE.7 AIN8
    GPIOE->DDR &= ~(1 << 7);  // PE.7 as input
    GPIOE->CR1 &= ~(1 << 7);  // floating input
    GPIOE->CR2 &= ~(1 << 7);  // 0: External interrupt disabled   ???

// AIN7
    GPIOB->DDR &= ~(1 << 7);  // PB.7 as input
    GPIOB->CR1 &= ~(1 << 7);  // floating input
    GPIOB->CR2 &= ~(1 << 7);  // 0: External interrupt disabled   ???

// AIN6
    GPIOB->DDR &= ~(1 << 6);  // PB.6 as input
    GPIOB->CR1 &= ~(1 << 6);  // floating input
    GPIOB->CR2 &= ~(1 << 6);  // 0: External interrupt disabled   ???

// AIN5
    GPIOB->DDR &= ~(1 << 5);  // PB.5 as input
    GPIOB->CR1 &= ~(1 << 5);  // floating input
    GPIOB->CR2 &= ~(1 << 5);  // 0: External interrupt disabled   ???

// AIN4
    GPIOB->DDR &= ~(1 << 4);  // PB.4 as input
    GPIOB->CR1 &= ~(1 << 4);  // floating input
    GPIOB->CR2 &= ~(1 << 4);  // 0: External interrupt disabled   ???

// AIN3
    GPIOB->DDR &= ~(1 << 3);  // PB.3 as input
    GPIOB->CR1 &= ~(1 << 3);  // floating input
    GPIOB->CR2 &= ~(1 << 3);  // 0: External interrupt disabled   ???

// AIN2
    GPIOB->DDR &= ~(1 << 2);  // PB.2 as input
    GPIOB->CR1 &= ~(1 << 2);  // floating input
    GPIOB->CR2 &= ~(1 << 2);  // 0: External interrupt disabled   ???

// AIN1
    GPIOB->DDR &= ~(1 << 1);  // PB.1 as input
    GPIOB->CR1 &= ~(1 << 1);  // floating input
    GPIOB->CR2 &= ~(1 << 1);  // 0: External interrupt disabled   ???

// AIN0
    GPIOB->DDR &= ~(1 << 0);  // PB.0 as input
    GPIOB->CR1 &= ~(1 << 0);  // floating input
    GPIOB->CR2 &= ~(1 << 0);  // 0: External interrupt disabled   ???
}

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/24/
 */
void UART_setup(void)
{
    UART2_DeInit();

    UART2_Init(115200,
               UART2_WORDLENGTH_8D,
               UART2_STOPBITS_1,
               UART2_PARITY_NO,
               UART2_SYNCMODE_CLOCK_DISABLE,
               UART2_MODE_TXRX_ENABLE);

    UART2_Cmd(ENABLE);
}

/*
*  Send a message to the debug port (UART1).
*    (https://blog.mark-stevens.co.uk/2012/08/using-the-uart-on-the-stm8s-2/)
*/
void UARTputs(char *message)
{
    char *ch = message;
    while (*ch)
    {
        UART2->DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
        while ( 0 == (UART2->SR & UART2_SR_TXE) ); //  Wait for transmission to complete.
        ch++;                               //  Grab the next character.
    }
}

/*
 * https://www.st.com/resource/en/application_note/cd00282842-rs232-communications-with-a-terminal-using-the-stm8sdiscovery-stmicroelectronics.pdf
 */
/*******************************************************************************
* Function Name  : SerialKeyPressed
* Description    : Test to see if a key has been pressed on the HyperTerminal
* Input          : - key: The key pressed
* Output         : None
* Return         : 1: Correct
*                  0: Error
*******************************************************************************/
uint8_t SerialKeyPressed(char *key)
{
    FlagStatus flag  ;
    /* First clear Rx buffer */
    flag = UART2_GetFlagStatus(UART2_FLAG_RXNE);
    if ( flag == SET)
    {
        *key = (char)UART2->DR;
        return 1;
    }
    else
    {
        return 0;
    }
}



/*
 * set ADC clock to 4Mhz  - sample time from the data sheet @ 4Mhz
 * min sample time .75 useconds @ fADC = 4Mhz
 * conversion time = 14 * 1/2000000 = 0.0000035 seconds (3.5 us)
 */
#ifdef CLOCK_16
 #define ADC_DIVIDER ADC1_PRESSEL_FCPU_D4  // 8 -> 16/4 = 4
#else
 #define ADC_DIVIDER ADC1_PRESSEL_FCPU_D2  // 4 ->  8/2 = 4 
#endif
/*
 * https://community.st.com/s/question/0D50X00009XkbA1SAJ/multichannel-adc
 */
void ADC1_setup(void)
{
// Port B[0..7]=floating input no interr
// STM8 Discovery, all PortB pins are on CN3
//    GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_IN_FL_NO_IT); // all AIN pins setup explicityly with the GPIO init

    ADC1_DeInit();

// configured range of A/D input pins: CH0, CH1, CH2 to use as b-EMF sensors
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, // don't care, see ConversionConfig below ..
              ADC1_CHANNEL_3,        // i.e. Ch 0, 1 and 2 are enabled
              ADC_DIVIDER,
              ADC1_EXTTRIG_TIM,      //  ADC1_EXTTRIG_GPIO ... not presently using any ex triggern
              DISABLE,               // ExtTriggerState
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL,
              DISABLE);              // SchmittTriggerState

    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE); // grab the sample in the ISR

    /*
    A single conversion is performed for each channel starting with AIN0 and the data is stored
    in the data buffer registers ADC_DBxR.
    */
    ADC1_ConversionConfig(ADC1_CONVERSIONMODE_CONTINUOUS,
                          ((ADC1_Channel_TypeDef)(ADC1_CHANNEL_0 | ADC1_CHANNEL_1 | ADC1_CHANNEL_2)),
                          ADC1_ALIGN_RIGHT);

//ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE); // Scan mode from channel 0 to n (as defined in ADC1_Init)

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
    ADC1_StartConversion(); // i.e. for scanning mode only has to start once ...
}

/*
 * Timer 1 Setup
 * from:
 *   http://www.emcu.it/STM8/STM8-Discovery/Tim1eTim4/TIM1eTIM4.html
 *
 * manual mode (ramp-up) commutation timing?
 * based on OTS ESC analysic, rampup start at
 *  pd = 0.008  sec
 *       0.003  sec  rampup to 3000 RPM
 *       0.004  sec  settle to 2500 RPM
 *       0.008  sec  1250 RPM slowest attainable after initial sync
 *
 *       0.0012 sec
*/
#ifdef CLOCK_16
#define TIM1_PRESCALER 8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
#define TIM1_PRESCALER 4  //    (1/8Mhz)  * 4 * 250 -> 0.000125 S
#endif

#define PWM_MODE  TIM1_OCMODE_PWM1

void TIM1_setup(void)
{
    const uint16_t T1_Period = TIM2_PWM_PD /* TIMx_PWM_PD */ ;  // 16-bit counter  ... 260 test

    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1, ENABLE);  // put this with clocks setup

    TIM1_DeInit();

    TIM1_TimeBaseInit(( TIM1_PRESCALER - 1 ), TIM1_COUNTERMODE_UP, T1_Period, 0); // ISR at and of "idle" time

    /* Channel 2 PWM configuration */
    TIM1_OC2Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_HIGH,
                  TIM1_OCNPOLARITY_HIGH,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);
    //   TIM2_OC2PreloadConfig(ENABLE); ??

    /* Channel 3 PWM configuration */
    TIM1_OC3Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_HIGH,
                  TIM1_OCNPOLARITY_HIGH,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);
// ?  TIM2_OC3PreloadConfig(ENABLE);

    /* Channel 4 PWM configuration */
    TIM1_OC4Init(PWM_MODE,
                 TIM1_OUTPUTSTATE_ENABLE,
                 0,
                 TIM1_OCPOLARITY_HIGH,
                 TIM1_OCIDLESTATE_RESET);
// TIM2_OC4PreloadConfig(ENABLE); // ???

    TIM1_CtrlPWMOutputs(ENABLE);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);
}

/*
 * Setup TIM2 PWM
 * Reference: AN3332
 */
#ifdef CLOCK_16
 #define TIM2_PRESCALER TIM2_PRESCALER_8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
 #define TIM2_PRESCALER TIM2_PRESCALER_4  //    (1/8Mhz)  * 4 * 250 -> 0.000125 S
#endif

void TIM2_setup(void)
{
    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz */
    TIM2_TimeBaseInit(TIM2_PRESCALER, TIM2_PWM_PD);
    /* Channel 1 PWM configuration */
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
    TIM2_OC1PreloadConfig(ENABLE);

    /* Channel 2 PWM configuration */
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
    TIM2_OC2PreloadConfig(ENABLE);

    /* Channel 3 PWM configuration */
    TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
    TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);


    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE); // GN: for eventual A/D triggering

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);
}


/*
 * Configure Timer 4 as general purpose fixed time-base reference
 * Timer 4 & 6 are 8-bit basic timers
 *
 *   https://lujji.github.io/blog/bare-metal-programming-stm8/
 *
 * Setting periodic task for fast-ish rate of A/D acquisition.
 * ISR must set 'TaskRdy' flag and not block on the task since A/D does a blocking wait.
 */
void TIM4_setup(void)
{
//    const uint8_t T4_Period = 32;     // Period =  256uS ... stable, any faster becomes jittery
    const uint8_t T4_Period = 64;    // Period =  0.000512 S  (512 uS) ... 
//    const uint8_t T4_Period = 32; // double the rate (for logging) ???

#ifdef CLOCK_16
    TIM4->PSCR = 0x07; // PS = 128  -> 0.0000000625 * 128 * p
#else
    TIM4->PSCR = 0x06; // PS =  64  -> 0.000000125  *  64 * p
#endif

    TIM4->ARR = T4_Period;

    TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
    TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

/*
 * Timers 2 3 & 5 are 16-bit general purpose timers
 *  Sets the commutation switching period.
 *
 *  @8Mhz, fMASTER period ==  0.000000125 S
 *   Timer Step:
 *     step = 1 / 8Mhz * prescaler = 0.000000125 * (2^1) = 0.000000250 S
 */
#ifdef CLOCK_16
#define TIM3_PSCR  0x02  // 2^2 == 4
#else
#define TIM3_PSCR  0x01  // 2^1 == 2
#endif

void TIM3_setup(uint16_t period)
{
    TIM3->PSCR = TIM3_PSCR;

    TIM3->ARRH = period >> 8;   // be sure to set byte ARRH first, see data sheet
    TIM3->ARRL = period & 0xff;

    TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
    TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
    TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM3
}

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/13/
 * GN:  by default  microcontroller uses   internal 16MHz RC oscillator
 * ("HSI", or high-speed internal) divided by eight  as a clock source. This results in a base timer frequency of 2MHz.
 * Using this function just to show the library way to explicit clock setup.
 */
void Clock_setup(void)
{
    CLK_DeInit();
#ifdef INTCLOCK
#else
    // Configure Quartz Clock
    CLK_DeInit();
    CLK_HSECmd(ENABLE);
#ifdef CLOCK_16
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // 16Mhz
#else
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2); // 8Mhz
#endif // CLK
#endif
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
}

/*
 * single point init for MCU module
 *
 */
void MCU_Init(void)
{
	    Clock_setup();
    GPIO_Config();
    UART_setup();
    ADC1_setup();
//    TIM1_setup();
    TIM2_setup();
    TIM4_setup();
}