#include "canopen.h"
#include "timerscfg.h"

/* Define the timer registers */
uint32_t TimerAlarm_CAN;
uint32_t TimerCounter_CAN;
uint32_t last_time_set = 0;

/**
 * Initializes the timer, turn on the interrupt and put the interrupt time to zero
 * 
 */
void initTimer(void)
{
 	TimerCounter_CAN = 0;
	TimerAlarm_CAN   = 0;
	last_time_set = TIMEVAL_MAX;
}


/**
  * @brief  setTimer
  * @param  value:Set time value 0x0000-0xffff
  * @retval NONE
  */
void setTimer( TIMEVAL value )
{
    /* Add the desired time to timer interrupt time */
    TimerAlarm_CAN = ( TimerCounter_CAN + value ) % TIMEVAL_MAX;
}

/**
  * @brief  getElapsedTime
  * @param  NONE
	* @retval TIMEVAL:Return current timer value
  */
TIMEVAL getElapsedTime(void)
{
  uint32_t timer = TimerCounter_CAN;	// Copy the value of the running timer
  
  uint32_t res = ( timer >= last_time_set ) ?( timer - last_time_set ) :( TIMEVAL_MAX - last_time_set + timer );
  
  return res;
}
