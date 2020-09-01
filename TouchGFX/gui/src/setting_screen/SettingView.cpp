#include <gui/setting_screen/SettingView.hpp>
#include "cmsis_os.h"
#include "SmartDisplayHost.h"

#define GetTimeDiff( firstT, lastT ) ( ( firstT > lastT ) ?(0xFFFFFFFFL - firstT + 1 ) + lastT :lastT - firstT )

SettingView::SettingView():
    scrollListItemSelectedCallback(this, &SettingView::scrollListItemSelectedHandler)
{

}

void SettingView::scrollListItemSelectedHandler(int16_t itemSelected)
{
	const uint8_t convertArray[] = { 2, 4, 5, 6, 7, 8, 9, 10 };

	typeSelect = convertArray[ itemSelected ];
}

void SettingView::setupScreen()
{
	SettingViewBase::setupScreen();

	typeSelect = 2;
	style = 0;

	scrollWheel4.setItemSelectedCallback(scrollListItemSelectedCallback);

	// The item selected callbacks are registerd with scroll wheel and list
	// scrollList1.setItemSelectedCallback( scrollListItemSelectedCallback );
	tick = osKernelSysTick( );
}

void SettingView::tearDownScreen()
{
	SettingViewBase::tearDownScreen();
}

void SettingView::scrollWheel2UpdateItem(ScrollUnSelect& item, int16_t itemIndex)
{
	// Override and implement this function in Setting
	item.updateText(itemIndex);
}

void SettingView::scrollWheel2UpdateCenterItem(ScrollSelect& item, int16_t itemIndex)
{
	// Override and implement this function in Setting

	item.updateText(itemIndex);

	uint8_t array[] = { 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, };

	style = array[ itemIndex ];
}

void SettingView::handleTickEvent()
{ 
	tick = osKernelSysTick( );

	if( toggleButton1.getState() == 1 )
		application().gotoMonitorScreenNoTransition();
}

/*
 * Virtual Action Handlers
 */
void SettingView::ButtonIsClickedFun( )
{
    // Override and implement this function in Setting
    // Send SDO Msg
	UNS32 abortCode;
    
	uint8_t *pTypeArray[] = { 

		&Obj01_type, &Obj02_type, &Obj03_type, &Obj04_type, &Obj05_type, 
		&Obj06_type, &Obj07_type, &Obj08_type, &Obj09_type, &Obj10_type, 
	};
    
	uint16_t *pStyleArray[] = { 

		&Obj01_style, &Obj02_style, &Obj03_style, &Obj04_style, &Obj05_style, 
		&Obj06_style, &Obj07_style, &Obj08_style, &Obj09_style, &Obj10_style, 
	};
    
	int16_t *pXArray[] = { 

		&Obj01_posX, &Obj02_posX, &Obj03_posX, &Obj04_posX, &Obj05_posX, 
		&Obj06_posX, &Obj07_posX, &Obj08_posX, &Obj09_posX, &Obj10_posX, 
	};
	
	int16_t *pYArray[] = { 

		&Obj01_posY, &Obj02_posY, &Obj03_posY, &Obj04_posY, &Obj05_posY, 
		&Obj06_posY, &Obj07_posY, &Obj08_posY, &Obj09_posY, &Obj10_posY,
	};
    
	*pTypeArray[ presenter->getObjNum() ] = typeSelect;
	  
	*pStyleArray[ presenter->getObjNum() ] = style; 

	*pXArray[ presenter->getObjNum() ] = slider1.getValue();   

	*pYArray[ presenter->getObjNum() ] = slider1_1.getValue();

	masterSendNMTstateChange ( &SmartDisplayHost_Data, 0x7B, NMT_Enter_PreOperational );
 	// --------------------------------------------------------------------------
    
  tick = osKernelSysTick( );

  while( GetTimeDiff( tick, osKernelSysTick( ) ) < 10 );
    
	writeNetworkDict( &SmartDisplayHost_Data, 
										0x7B, 
										0x2000 + presenter->getObjNum(),
										1,
										1,
										uint8, 				                   
										( void * )pTypeArray[ presenter->getObjNum() ],
										0 );

	while( getWriteResultNetworkDict ( &SmartDisplayHost_Data, 0x7B, &abortCode ) == SDO_DOWNLOAD_IN_PROGRESS );

	writeNetworkDict( &SmartDisplayHost_Data, 
										0x7B, 			// node
										0x2000 + presenter->getObjNum(),	// index
										5,				  // subIndex
										2,				  // count
										int16, 			// dataType			                   
										( void * )pStyleArray[ presenter->getObjNum() ],
										0 );

	while( getWriteResultNetworkDict ( &SmartDisplayHost_Data, 0x7B, &abortCode ) == SDO_DOWNLOAD_IN_PROGRESS );

	writeNetworkDict( &SmartDisplayHost_Data, 
										0x7B, 		// node
										0x2000 + presenter->getObjNum(),	// index
										3,				// subIndex
										2,				// count
										int16, 		// dataType			                   
										( void* )pXArray[ presenter->getObjNum() ],
										0 );

	while( getWriteResultNetworkDict ( &SmartDisplayHost_Data, 0x7B, &abortCode ) == SDO_DOWNLOAD_IN_PROGRESS );

	writeNetworkDict( &SmartDisplayHost_Data, 
										0x7B, 		// node
										0x2000 + presenter->getObjNum(),	// index
										4,				// subIndex
										2,				// count
										int16, 		// dataType			                   
										( void* )pYArray[ presenter->getObjNum() ],
										0 );

	while( getWriteResultNetworkDict ( &SmartDisplayHost_Data, 0x7B, &abortCode ) == SDO_DOWNLOAD_IN_PROGRESS );
	// --------------------------------------------------------------------------

	UNS32 data;
	UNS32 size = 1;
	
 	readNetworkDict( &SmartDisplayHost_Data, 0x7B, 0x2102, 4, boolean, 0 ); // get the data index 0x2013 subindex 4 of node 0x7B
 	
	while( getReadResultNetworkDict( &SmartDisplayHost_Data, 0x7B, &data, &size, &abortCode ) == SDO_UPLOAD_IN_PROGRESS );

	Buzzer_Active = !( data & 0x01 );

	writeNetworkDict( &SmartDisplayHost_Data, 
										0x7B, 
										0x2102,
										4,
										1,
										boolean, 				                   
										( void * )&Buzzer_Active,
										0 );
	// wait
	while( getWriteResultNetworkDict ( &SmartDisplayHost_Data, 0x7B, &abortCode ) == SDO_DOWNLOAD_IN_PROGRESS );
	// --------------------------------------------------------------------------

	// change salve state to operation mode
	masterSendNMTstateChange ( &SmartDisplayHost_Data, 0x7B, NMT_Start_Node );
	
}


void SettingView::scrollWheel3UpdateItem(ScrollUnSelect& item, int16_t itemIndex)
{
	// Override and implement this function in Setting
	item.updateText(itemIndex);
}

void SettingView::scrollWheel3UpdateCenterItem(ScrollSelect& item, int16_t itemIndex)
{
	// Override and implement this function in Setting
	item.updateText(itemIndex);

	unsigned char array[] = { 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, };

	presenter->setObjNum( array[ itemIndex ] );
}

void SettingView::scrollWheel4UpdateItem(CustomContainer1& item, int16_t itemIndex)
{
	// Override and implement this function in Setting    
	item.updateText(itemIndex);
}

void SettingView::scrollWheel4UpdateCenterItem(ScrollListContainer& item, int16_t itemIndex)
{
	// Override and implement this function in Setting
	item.updateText( itemIndex );    
}
