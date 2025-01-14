/**
  ******************************************************************************
  * @file BLDC_sm.c
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
/**
 * \defgroup  BLDC_sm BLDC State
 * @brief  BLDC state management and timing control
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "bldc_sm.h" // external types used internally
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "faultm.h"
#include "sequence.h"

/* Private defines -----------------------------------------------------------*/

// if this is defined, stays in aligment state w/ 0 PWM
//#define TEST_ALIGNMENT


/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_ALIGN     25.0
#define PWM_DC_RAMPUP    15.0
#define PWM_DC_STARTUP   14.4
#define PWM_DC_SHUTOFF    7.2 // stalls if slower

// define pwm pulse times for operation states 
#define PWM_PD_ALIGN     PWM_GET_PULSE_COUNTS( PWM_DC_ALIGN )
#define PWM_PD_RAMPUP    PWM_GET_PULSE_COUNTS( PWM_DC_RAMPUP )
#define PWM_PD_STARTUP   PWM_GET_PULSE_COUNTS( PWM_DC_STARTUP )
#define PWM_PD_SHUTOFF   PWM_GET_PULSE_COUNTS( PWM_DC_SHUTOFF )

 
 // commutation period at start of ramp (est. @ 12v) - exp. det.
#define BL_CT_RAMP_START  (5632.0 * CTIME_SCALAR) // $1600

/**
 * @brief Control rate scalar
 * @details Scale factor relating the commutation-timing ramp data and variables
 *     with the control task rate
 */
#define CTRL_RATEM  4

// The control-frame rate becomes factored into the integer ramp-step
#define BL_ONE_RAMP_UNIT  (1.5 * CTRL_RATEM * CTIME_SCALAR)

// length of alignment step (experimentally determined w/ 1100kv @12.5v)
#define BL_TIME_ALIGN  (200 * 1) // N frames @ 1 ms / frame


/* Private types -----------------------------------------------------------*/

/**
 * @brief Type for BL operating state.
 */
typedef enum
{
  BL_MANUAL,
  BL_ALIGN,
  BL_RAMPUP,
  BL_OPN_LOOP,
  BL_CLS_LOOP,
  BL_STOPPED,
  BL_INVALID
}
BL_State_T;


/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint16_t BL_comm_period; // persistent value of ramp timing
static uint16_t BL_motor_speed; // persistent value of motor speed
static uint16_t BL_optimer; // allows for timed op state (e.g. alignment)
static BL_State_T BL_opstate; // BL operation state

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Commutation timing ramp control.
 *
 * At each iteration the commutation time period is ramped to the target value
 * stepped in increment of +/- step depending on the sign of the error.
 *
 * @param   tgt_commutation_per  Target value to track.
 */
static uint16_t timing_ramp_control(uint16_t setpoint, uint16_t target)
{
  uint16_t u16 = setpoint;

  // determine signage of error i.e. step increment
  if (u16 > target)
  {
    u16 -= (uint16_t)BL_ONE_RAMP_UNIT;
    if (u16 < target)
    {
      u16 = target;
    }
  }
  else if (u16 < target)
  {
    u16 += (uint16_t)BL_ONE_RAMP_UNIT;
    if (u16 > target)
    {
      u16 = target;
    }
  }
  return u16;
}

/**
 * @Brief common sub for stopping and fault states
 *
 * @Detail 
 * Allows motor to be stopped in a fault condition, while allowing the system to 
 * remain in whatever operating state - does not reset the control state, fault 
 * manageer etc. This is a developers "feature" allowing the fault state and 
 * other info to be examined. 
 */
static void BL_stop(void)
{
// have to clear the local UI_speed since that is the transition OFF->RAMP condition
  BL_motor_speed = 0;

  // kill the driver signals
  All_phase_stop();
}

/* Public functions ---------------------------------------------------------*/

/**
 * @brief Initialize/reset motor
 *
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 *
 * @details
 *    expect to be called from non-ISR/CS context (i.e. from  UI handler)
 */
void BL_reset(void)
{
  // assert PWM channels to disabled
  BL_stop();

  // Set initial commutation timing period upon state transition.
  // TIM3 is left enabled, so the commutation period (TIM3) is simply set to a 
  // arbitrarily large number. The TIM3 ISR will still fire but the commutation 
  // step logic has no effect as long as the PWM is disabled. 	
  BL_set_timing( U16_MAX ); // 0xFFFF;

  Faultm_init();

  BL_set_opstate( BL_STOPPED );  // set the initial control-state
}


/**
 * @brief Sets motor speed from commanded throttle/UI setting
 *
 * @details
 *  The motor is started once reaching the ramp speed threshold, and allowed to
 *  slow down to the low shutoff threshold.
 *  UI Speed is shared with background task so this function should
 *  be invoked only from within a CS.
 *
 * @param ui_mspeed_counts The desired motor output in tetms of timer counts
 */
void BL_set_speed(uint16_t ui_mspeed_counts)
{
  if( ui_mspeed_counts > PWM_PD_SHUTOFF )
  {
    // Update the dc if speed input greater than ramp start, OR if system already running
    if( ui_mspeed_counts > PWM_PD_STARTUP || 0 != BL_motor_speed  /* if Control_mode != STOPPED */ )
    {
      BL_motor_speed = ui_mspeed_counts;
    }
  }
  else
  {
    // commanded speed less than low limit so reset - has to ramp again to get started.
    BL_reset();
  }
}

/**
 * @brief Accessor for Commanded Duty Cycle
 *
 * @return PWM period
 */
uint16_t BL_get_speed(void)
{
  return BL_motor_speed;
}

/**
 * @brief adjust commutation timing by step amount
 */
void BL_timing_step_slower(void)
{
  BL_set_opstate( BL_MANUAL ); //tbd

  BL_comm_period += (uint16_t)BL_ONE_RAMP_UNIT; // slower
}

/**
 * @brief adjust commutation timing by step amount
 */
void BL_timing_step_faster(void)
{
  BL_set_opstate( BL_MANUAL ); // tbd

  BL_comm_period -= (uint16_t)BL_ONE_RAMP_UNIT; // faster
}

/**
  * @brief Accessor for commutation period.
  *
  * @return commutation period
  */
uint16_t BL_get_timing(void)
{
  return BL_comm_period;
}

/**
  * @brief Accessor for commutation period.
  */
void BL_set_timing(uint16_t u16)
{
  BL_comm_period = u16;
}

/**
 * @brief Accessor for state variable.
 *
 * @details
 *  External modules can query if the machine is running or not.
 *  There are only these two states bases on the set speed greater or less than
 *  the shutdown threshold.
 *
 * @return state value
 */
BL_RUNSTATE_t BL_get_state(void)
{
  if ( BL_motor_speed > PWM_PD_SHUTOFF )
  {
    return BL_IS_RUNNING;
  }
  // else
  return BL_NOT_RUNNING;
}

/**
 * @brief  Accessor for state variable
 */
void BL_set_opstate(uint8_t opstate)
{
  BL_opstate = opstate;
}

/**
 * @brief  Accessor for state variable
 *
 * @return  operation state
 */
uint8_t BL_get_opstate(void)
{
  return BL_opstate;
}

// TBD, test code
// 1 bit is /2 for sma, 1 bit is /2 for "gain"
//#define SMA_SH  1 // tmp
//#define GAIN_SH  2 // tmp
/**
 * @brief  Implement control task (fixed exec rate of ~1ms).
 */
void BL_State_Ctrl(void)
{
//  static const uint8_t CONTROL_GAIN_SH  = (GAIN_SH + SMA_SH);
//  static int16_t timing_error;

  uint16_t inp_dutycycle = 0; // in case of error, PWM output remains 0

  if ( 0 != Faultm_get_status() )
  {
    BL_stop(); // sets BL pwm period to 0 and disables timer PWM channels but
               // doesn't re-init the system state
  }
  else
  {
    inp_dutycycle = BL_motor_speed; // set pwm period from UI

    if( BL_STOPPED == BL_get_opstate() )
    {
      if (inp_dutycycle > 0)
      {
        BL_set_opstate( BL_ALIGN ); // state-transition
        BL_optimer = BL_TIME_ALIGN;

        // Set initial commutation timing period upon state transition.
        BL_set_timing( (uint16_t)BL_CT_RAMP_START );
      }
    }
    else if( BL_ALIGN == BL_get_opstate() )
    {
      if (BL_optimer > 0)
      {
        inp_dutycycle = PWM_PD_ALIGN;
        BL_optimer -=1;
      }
      else
      {
#ifndef TEST_ALIGNMENT
        BL_set_opstate(BL_RAMPUP);
#else
// force it to stay in alignment but kill the pwm
        inp_dutycycle = 0; //tmp
#endif
      }
    }
    else if( BL_RAMPUP == BL_get_opstate() )
    {
        // grab the current commutation period setpoint to handoff to ramp control
        uint16_t comm_perd_sp; // = BL_get_timing();

        // table-lookup for the target commutation timing period for the PWM duty-cycle (low speed-startup) 
        uint16_t olt = Get_OL_Timing( PWM_PD_STARTUP );

        // Set duty-cycle for rampup somewhere between 10-25% (tbd)
        inp_dutycycle = PWM_PD_RAMPUP;

        // PWM period ramped down by fixed rate of increment (decrement) ... linear ramp

        // this would only be ramping in one direction (commutation-period only decreasing in rampup!)
//        BL_set_timing( comm_perd_sp, (timing_ramp_control( olt ) );

        BL_timing_step_faster(); // decrement the commutation-period by 1 ramp-step

        // check state-transition .. has it reached the timing for the low speed setpoint?
        comm_perd_sp = BL_get_timing();

      if( comm_perd_sp > olt )
      {
        BL_set_opstate( BL_OPN_LOOP ); // state-transition
      }
    }
    else if( BL_OPN_LOOP == BL_get_opstate() )
    {
      uint16_t olt = Get_OL_Timing( inp_dutycycle );
      // grab the current commutation period setpoint to handoff to ramp control
      uint16_t comm_perd_sp = BL_get_timing();

      // update the commutation time period
      uint16_t temp16 = timing_ramp_control(comm_perd_sp, olt);
      BL_set_timing(temp16);

      // check plausibility condition for transition to closed-loop
      // if ( Seq_get_timing_error_p() )
      //           Control_mode = BL_CLS_LOOP; // state-transition
    }
    else if( BL_CLS_LOOP == BL_get_opstate() )
    {
#if 0 // test code
      // the control gain is macro'd together with the unscaling of the error term and also /2 of the sma
      uint16_t t16 = BLDC_OL_comm_tm ;
      timing_error = ( Seq_get_timing_error() + timing_error ) >> CONTROL_GAIN_SH;
      t16 += timing_error;
// if this overshoots, the control runs away until the TIM3 becomes so low the system locks up and no faultm can work .. not failsafe!
      if (t16 > LUDICROUS_SPEED)
      {
        BLDC_OL_comm_tm  = t16;
      }
#endif // test code
    }
    // else ...check for conditions if necessary to unlatch CL control mode? (too slow, lost sync)
  }

  // pwm duty-cycle will be upated to the timer peripheral at next commutation step.
  PWM_set_dutycycle( inp_dutycycle );
}

/**
 * @brief  commutation sequence step (timer ISR callback)
 *
 * @details 
 */ 
void BL_Commutation_Step(void)
{
//  static BL_RUNSTATE_t prev_state = BL_INVALID;

  switch( BL_get_opstate() )
  {
  case BL_ALIGN:
    //keep sector 0 on until timeout. Sequencer initializes to sector 0
    Sequence_Step_0();
    break;

  case BL_RAMPUP:
  case BL_OPN_LOOP:
  case BL_CLS_LOOP:

    Sequence_Step();
    break;

  case BL_STOPPED:
  case BL_MANUAL:
  default:
    break;
  }

//  prev_state = BL_get_state();
}

/**@}*/ // defgroup

