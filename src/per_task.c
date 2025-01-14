/**
  ******************************************************************************
  * @file per_task.c
  * @brief Background task / periodic task
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */
/**
 * \defgroup per_task Periodic Task
 * @brief Background task / periodic task
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stddef.h> // NULL

// app headers
#include "mcu_stm8s.h"
#include "sequence.h"
#include "bldc_sm.h"
#include "faultm.h"
#include "driver.h"
#include "spi_stm8s.h"
#include "pdu_manager.h"


/* Private defines -----------------------------------------------------------*/

/*
 * motor speed received from remote UI with integer scaling would ideally provide 1024 
 * steps of precision, however to fit uint16, scale factor limited to 65535/100 so 
 * slightly less precision than actual radio signal (timer capture) but acceptable  
 * i.e.
 *  pwm_motor_speed =   pwm_period_counts * (ui_speed_pcnt / 512) / 100 
 *
 * The UI motor speed is for now scaled to the range of the PWM period in 
 * clock counts i.e. (0:250) .. this is due to being used as the timing table index.
 */
#define UI_MSPEED_PCNT_SCALE  512.0  // 0.002% per bit ... note use power of 2 scale factor


// Threshold is set low enuogh that the machine doesn't stall
// thru the lower speed transition into closed-loop control.
// The fault can be tested by letting the spinning prop disc strike a flimsy
// obstacle like a 3x5 index card.
#if defined ( S105_DEV )
//  Vcc==3.3v  33k/10k @ Vbatt==12.4v
  #define V_SHUTDOWN_THR      0x02C0    // experimentally determined @ 12.4v
#else
  // applies presently only to the stm8s-Discovery, at 14.2v and ADCref == 5v
  #define V_SHUTDOWN_THR      0x02C0    // experimentally determined!
#endif


//#define ANLG_SLIDER


/* Private function prototypes -----------------------------------------------*/

// forward declarations for UI input handers
static void timing_plus(void);
static void timing_minus(void);
static void spd_plus(void);
static void spd_minus(void);
static void m_stop(void);
static void m_start(void);


/* Public variables  ---------------------------------------------------------*/

/* Private types     ---------------------------------------------------------*/

/**
 * @brief Data type for the key handler function.
 */
typedef void (*ui_handlrp_t)( void );

/**
 * @brief Data type for the key code lookup table.
 */
typedef enum
{
  COMM_PLUS   = ']',
  COMM_MINUS  = '[',
  M_STOP      = ' ', // space bar
  M_START     = '/', // /
  SPD_PLUS    = '.', // >
  SPD_MINUS   = ',', // <
  K_UNDEFINED = -1
} 
ui_keycode_t;

/**
 * @brief Data type for the key handler table.
 */
typedef struct
{
  ui_keycode_t   key_code;  /**< Key code. */
  ui_handlrp_t   phandler;  /**< Pointer to handler function. */
} 
ui_key_handler_t;


/* Private variables ---------------------------------------------------------*/

static uint8_t TaskRdy; // flag for timer interrupt for BG task timing
static uint8_t Log_Level;
static uint16_t Vsystem;
static uint16_t UI_Speed; // motor percent speed input from servo or remote UI 

/**
 * @brief Lookup table for UI input handlers
 */
static const ui_key_handler_t ui_keyhandlers_tb[] =
{
#ifdef ENABLE_MAN_TIMING
  {COMM_PLUS,  timing_plus},
  {COMM_MINUS, timing_minus},
#endif
  {SPD_PLUS,   spd_plus},
  {SPD_MINUS,  spd_minus},
  {M_STOP,     m_stop},
  {M_START,    m_start}	
};

// macros to help make the LUT slightly more encapsulated
#define _SIZE_K_LUT  ( sizeof( ui_keyhandlers_tb ) / sizeof( ui_key_handler_t ) )
#define _GET_KEY_CODE( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].key_code
#define _GET_UI_HDLRP( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].phandler


/* Private functions ---------------------------------------------------------*/

/**
 * @brief Print one line to the debug serial port.
 * @note: NOT appropriate in either an ISR or critical section because of printf
 *  to serial terminal is blocking.
 *
 * @param zeroflag set 1 to zero the line count
 */
static void Log_println(int zrof)
{
  static uint16_t Line_Count = 0;
  int faults = (int)Faultm_get_status();
  uint16_t ui_speed = UI_Speed;
  uint16_t bl_speed = BL_get_speed(); 
  uint16_t timing_error = Seq_get_timing_error();
  uint16_t comm_period = BL_get_timing();
//  uint16_t servo_pulse_period = Driver_get_pulse_perd();
  uint16_t servo_pulse_duration = Driver_get_pulse_dur();
  uint16_t display_speed_pcnt = (uint16_t)Driver_get_motor_spd_pcnt();
  uint16_t servo_posn_counts = Driver_get_servo_position_counts();

  // if flag is set then reset line counter
  if ( 0 != zrof)
  {
    Line_Count = 0;
  }

  // if logger is enabled (level>0) then invoke its output
  if ( Log_Level > 0)
  {
    printf(
      "{%04X) UIspd%=%X CtmCt=%04X BLdc=%04X Vs=%04X Sflt=%X RCsigCt=%04X MspdCt=%u Mspd%=%u ERR=%04X \r\n",
      Line_Count++,  // increment line countet
      ui_speed, comm_period, bl_speed, Vsystem, faults, 
      servo_pulse_duration, servo_posn_counts, display_speed_pcnt,
      timing_error
    );
     Log_Level -= 1;
  }
}

/*
 * Service the slider and trim inputs for speed setting.
 * The UI Speed value represents percent of motor speed (0% : 100%), which is
 * proportional to RC radio control servo signal. 
 *
 * Servo input will have range of (0:1600) which can directly be used as the 
 * pwm input if the timer prescaler is set such that 16Mhz works out to a 
 * PWM period of    0.0000000625 * 1600 = 100 uS so 10 kHz .
 * 
 * The UI motor speed is for now scaled to the range of the PWM period in 
 * clock counts i.e. (0:250) .. this is due to being used as the timing table index.
 *
 * The standard RC framerate is 50 Hz or 20 mS. With pertask is updating at 60Hz, 
 * then the timely response of the system should be assured. 
 */
static void ui_set_motor_spd(uint16_t ui_motor_speed)
{	
#ifdef ANLG_SLIDER
  uint16_t adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 ); // ISR safe ... hmmmm
  Analog_slider = adc_tmp16 / 4; // [ 0: 1023 ] -> [ 0: 255 ]
#endif

    BL_set_speed( ui_motor_speed );
}

/*
 * handlers for UI events must be short as they are invoked in ISR context
 */
#ifdef ENABLE_MAN_TIMING
// for development user only
static void timing_plus(void)
{
  BL_timing_step_slower();
}
// for development user only
static void timing_minus(void)
{
  BL_timing_step_faster();
}
#endif

/*
 * motor start
 */
static void m_start(void)
{
}

/*
 * motor stop
 */
static void m_stop(void)
{
  // reset the machine
  BL_reset();

  UI_Speed = 0;

  printf("###\r\n");

  Log_Level = 1; // allow one more status line printfd to terminal then stops log output
  Log_println(1 /* clear line count */ );
}

/*
 * motor speed up
 */
static void spd_plus(void)
{
// tbd: steps of 0.5% (scale factor of 512)
//  if (UI_Speed < S8_MAX)
  {
    UI_Speed += (uint16_t)MSPEED_PCNT_INCREM_STEP;
  }
}

/*
 * motor speed down
 */
static void spd_minus(void)
{
// tbd: steps of 0.5% (scale factor of 512)
  if (UI_Speed > 0)
  {
    UI_Speed -= (uint16_t)MSPEED_PCNT_INCREM_STEP;
  }
  Log_Level = 1;
}

/*
 * handle terminal input - these are simple 1-key inputs for now
 */
static ui_handlrp_t handle_term_inp(void)
{
  ui_handlrp_t fp = NULL;
  char key;

// Uses non-blocking/non-buffered scan for key input similar to getch()
  if (SerialKeyPressed(&key))
  {
    int n;
    for (n = 0; n < _SIZE_K_LUT ; n++)
    {
      if (key == (char)_GET_KEY_CODE( n ))
      {
// any terminal output specific to the key or handelr need to be done here and
// not in the handler itself because the handler is to be called from w/i the CS
        fp =_GET_UI_HDLRP( n );
        break;
      }
    }
// anykey ...
    Log_Level = 255;// default anykey enable continous/verbous log
  }
  return fp;
}

/**
 * @brief  The User Interface task
 *
 * @details   Service the UI and communication handlers. Invoked in the
 *   execution context of 'main()' (background task)
 */
static void Periodic_task(void)
{
  BL_RUNSTATE_t bl_state;

// invoke the terminal input and ui speed subs, 
// If there is a valid key input, a function pointer to the input handler is 
// returned. This is done prior to entering a Critical Section (DI/EI) in which
// it will then be safe to invoke the input handler function (e.g. can call 
// subfunctions that may be messing with global variables e.g. motor speed etc.
  ui_handlrp_t fp = handle_term_inp();

  disableInterrupts();  //////////////// DI

  if (NULL != fp)
  {
    fp();
  }

  // passes the UI percent motor speed to the BL controller
  ui_set_motor_spd( UI_Speed );

  bl_state = BL_get_state();

  Vsystem = Seq_Get_Vbatt();

  enableInterrupts();  ///////////////// EI EI O

#if defined( UNDERVOLTAGE_FAULT_ENABLED )
  // update system voltage diagnostic - check plausibilty of Vsys
  if( BL_IS_RUNNING == bl_state && Vsystem > 0 )
  {
    Faultm_upd(VOLTAGE_NG, (faultm_assert_t)( Vsystem < V_SHUTDOWN_THR) );
  }
#endif
}

/**
 * @brief  Run Periodic Task if ready
 *
 * @details
 * Called in non-ISR context - checks the background task ready flag which if !0
 * will invoke the Periodic Task function.
 * @note  Called at ~60 Hz (0.0167 ms) - see Driver_Update()
 * @return  True if task ran (allows caller to also sync w/ the time period)
 */
uint8_t Task_Ready(void)
{
  static uint8_t framecount = 0;
  
#ifdef UART_IT_RXNE_ENABLE
  Pdu_Manager_Handle_Rx();
#endif

  if (0 != TaskRdy)
  {
    TaskRdy = FALSE;
    Periodic_task();

// periodic task is enabled at ~60 Hz ... the modulus provides a time reference of
// approximately 2 Hz at which time the master attempts to read a few bytes from SPI

    if ( ! ((framecount++) % 0x20) )
    {
      Log_println(0); // note: no printf to serial terminal inside a CS

#if SPI_ENABLED == SPI_STM8_MASTER
      SPI_controld();
#endif
    }
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief  Trigger background task.
 *
 * @details
 * Called in ISR context - sets the background task ready flag which when seen
 * by polling Task_Ready in background task will invoke the Periodic Task function.
 */
void Periodic_Task_Wake(void)
{
  TaskRdy = TRUE; // notify background process
}

/**@}*/ // defgroup
