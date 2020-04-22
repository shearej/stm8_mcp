/**
  ******************************************************************************
  * @file main.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version 
  * @date March-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "parameter.h" // app defines

/* Private defines -----------------------------------------------------------*/
#define PWM_DC_MIN 30
#define PWM_DC_MAX (TIM2_PWM_PD - 30)

// nbr of steps required to commutate 3 phase
#define N_CSTEPS   6


/* Public variables  ---------------------------------------------------------*/
uint16_t TIM2_pulse_0 ;
uint16_t TIM2_pulse_1 ;
uint16_t TIM2_pulse_2 ;


/* Private variables ---------------------------------------------------------*/
uint16_t global_uDC;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void _PWM_Config(void);

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
void PWM_Config(uint16_t pwm_dc)
{
    uint16_t uDC = pwm_dc;
#if 0
    if (uDC < PWM_DC_MIN)
    {
        uDC = PWM_DC_MIN;
    }
    if (uDC > PWM_DC_MAX)
    {
        uDC = PWM_DC_MAX;
    }
#endif
    global_uDC = uDC;
}

void PWM_set_outputs(u8 state0, u8 state1, u8 state2)
{

    TIM2_pulse_0 = state0 != 0 ? global_uDC : 0;
    TIM2_pulse_1 = state1 != 0 ? global_uDC : 0;
    TIM2_pulse_2 = state2 != 0 ? global_uDC : 0;

    _PWM_Config(); // seems to work here!
}

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
void _PWM_Config(void)
{
//    TIM2_pulse_0 =  TIM2_pulse_1 =  TIM2_pulse_2 = global_uDC;

    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz ... and period to ?    ( @2Mhz, fMASTER period == @ 0.5uS) */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, ( TIM2_PWM_PD - 1 ) ); // PS==1, 499   ->  8khz (period == .000125)

    /* Channel 1 PWM configuration */
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_0, TIM2_OCPOLARITY_LOW );
    TIM2_OC1PreloadConfig(ENABLE);


    /* Channel 2 PWM configuration */
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_1, TIM2_OCPOLARITY_LOW );
    TIM2_OC2PreloadConfig(ENABLE);


    /* Channel 3 PWM configuration */
    TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_2, TIM2_OCPOLARITY_LOW );
    TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);

#if 1
// GN: tmp test
    TIM2->IER |= TIM2_IER_UIE; // Enable Update Interrupt (sets manually-counted
    // pwm at 20mS with DC related to commutation/test-pot))
#endif
}

/*
 * drive /SD outputs and PWM channels
 */
void set_outputs(void)
{
    static u8 step = 0;

    step += 1;
    step %= N_CSTEPS;

    if ( PWM_Is_Active)
    {
// /SD outputs on C5, C7, and G1
// wait until switch time arrives (watching for voltage on the floating line to cross 0)
        switch(step)
        {
        default:

        case 0:
            GPIOC->ODR |=   (1<<5);
            GPIOC->ODR |=   (1<<7);
            GPIOG->ODR &=  ~(1<<1);
            PWM_set_outputs(1, 0, 0);
            break;
        case 1:
            GPIOC->ODR |=   (1<<5);
            GPIOC->ODR &=  ~(1<<7);
            GPIOG->ODR |=   (1<<1);
            PWM_set_outputs(1, 0, 0);
            break;
        case 2:
            GPIOC->ODR &=  ~(1<<5);
            GPIOC->ODR |=   (1<<7);
            GPIOG->ODR |=   (1<<1);
            PWM_set_outputs(0, 1, 0);
            break;
        case 3:
            GPIOC->ODR |=   (1<<5);
            GPIOC->ODR |=   (1<<7);
            GPIOG->ODR &=  ~(1<<1);
            PWM_set_outputs(0, 1, 0);
            break;
        case 4:
            GPIOC->ODR |=   (1<<5);
            GPIOC->ODR &=  ~(1<<7);
            GPIOG->ODR |=   (1<<1);
            PWM_set_outputs(0, 0, 1);
            break;
        case 5:
            GPIOC->ODR &=  ~(1<<5);
            GPIOC->ODR |=   (1<<7);
            GPIOG->ODR |=   (1<<1);
            PWM_set_outputs(0, 0, 1);
            break;
        }
    }
    else // motor drive output has been disabled
    {
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR &=   ~(1<<7);
        GPIOG->ODR &=   ~(1<<1);
        PWM_set_outputs(0, 0, 0);
    }
}
