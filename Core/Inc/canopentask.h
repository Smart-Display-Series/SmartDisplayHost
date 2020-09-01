#ifndef _CANOPENTASK_H_
#define _CANOPENTASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct
{
	unsigned short index;
	unsigned short subindex;
	unsigned int   val;
	
}CanOpenMessage_TypeDef;

extern xQueueHandle xQ_CAN_MSG;
extern xTaskHandle  xT_CANOpen;

void canopen_dataprocess_thread( void const * argument );

#endif /* _CANOPENTASK_H_ */
