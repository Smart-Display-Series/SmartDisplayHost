#include "canopentask.h"
#include "stm32f4xx_hal.h"
#include "SmartDisplayHost.h"
#include "canopen.h"

// == CAN Parameter Define ============================
#define NODE_ID  0x01
// ----------------------------------------------------

extern CAN_HandleTypeDef hcan2;

/*store date CAN receive*/
xQueueHandle xQ_CAN_MSG = NULL;

void vApplicationTickHook(void);


CAN_HandleTypeDef     CanHandle;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;


/**
  * @brief  canSend
	* @param  CANx:CAN1 or CAN2   m:can message
  * @retval 0ㄩSuccess
  */
unsigned char canSend(CAN_PORT Canport, Message *m)	                
{    
	TxHeader.StdId = (uint32_t)(m->cob_id);    
	TxHeader.ExtId = 0x00;
	//TxHeader.RTR = m->rtr;	
	if(m->rtr)
		TxHeader.RTR = CAN_RTR_REMOTE;
	else
		TxHeader.RTR = CAN_RTR_DATA;
    
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = m->len;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Set the data to be transmitted
	for( uint8_t i = 0; i < m->len; i++)    
		TxData[i] = m->data[i];

	if( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 ) {

		HAL_CAN_AbortTxRequest( &hcan2, TxMailbox );
	}

	// Start the Transmission process
	if( HAL_CAN_AddTxMessage( &hcan2, &TxHeader, TxData, &TxMailbox ) != HAL_OK ) {
		// Transmission request Error
		// Error_Handler();
		return 1;        
	}

	return 0;
}

void  vApplicationTickHook(void)
{
	TimerCounter_CAN++;

	if( TimerCounter_CAN >= TIMEVAL_MAX ) {

		TimerCounter_CAN = 0;        
	}

	/* ALARM */
	if( TimerCounter_CAN == TimerAlarm_CAN ){

		last_time_set = TimerCounter_CAN;
		/* Call the time handler of the stack to adapt the elapsed time  */
		TimeDispatch();
		/* This is important!!!!! */
	}
}

void initFunction(CO_Data* d)
{

}

void stoppedFunction( CO_Data* d )
{
  
}

void preOperationalFunction( CO_Data* d )
{
	
}

void dummyFunction(CO_Data* d)
{
	
}

/**
 * CANOpen Data Process Thread Program
 * @brief CANOpen Data Scan, if CANbus got a CAN message, then push the message into
 *        queue(xQ_CAN_MSG), this thread scan the queue at CANOpen_THREAD_SCAN_TIMER(
 *        default:20ms) rate.
 */
void canopen_dataprocess_thread(void const * argument)
{
	int i;
	Message RxMSG = Message_Initializer;    
	Message rxMessage;

	/* CANOpen Initialising */
	initTimer();

  SmartDisplayHost_Data.heartbeatError = (heartbeatError_t)dummyFunction;
	SmartDisplayHost_Data.initialisation = initFunction;
	SmartDisplayHost_Data.preOperational = (preOperational_t)preOperationalFunction;
	SmartDisplayHost_Data.operational = dummyFunction;
	SmartDisplayHost_Data.stopped = stoppedFunction;
	SmartDisplayHost_Data.post_sync = dummyFunction;
	SmartDisplayHost_Data.post_TPDO = dummyFunction;
	// SmartDisplay_Data.storeODSubIndex = storeODSubIndex;
	SmartDisplayHost_Data.post_emcy = (post_emcy_t)dummyFunction;

	setNodeId( &SmartDisplayHost_Data, NODE_ID );
  setState( &SmartDisplayHost_Data, Pre_operational );
  
	/*create a queue can store 20 data*/
	xQ_CAN_MSG = xQueueCreate( 20, sizeof( RxMSG ) );

	while(1) {
    
		/*Receive CANBUS data*/
		if( xQueueReceive( xQ_CAN_MSG, &rxMessage, ( portTickType )1 ) ) {

			RxMSG.cob_id = ( uint16_t )( rxMessage.cob_id );
			RxMSG.rtr = rxMessage.rtr;
			RxMSG.len = rxMessage.len;

			for(i=0;i<RxMSG.len;i++)
				RxMSG.data[i] = rxMessage.data[i];

			canDispatch( &SmartDisplayHost_Data, &RxMSG );
		}
	}

}
