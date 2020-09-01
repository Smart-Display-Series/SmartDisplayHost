#include <gui/monitor_screen/MonitorView.hpp>
#include "cmsis_os.h"
#include "SmartDisplayHost.h"
#include "BitmapDatabase.hpp"

#define NODEID 0x7B
#define SubIndexType 1
#define SubIndexSytel 5
#define GetTimeDiff( firstT, lastT ) ( ( firstT > lastT ) ?(0xFFFFFFFFL - firstT + 1 ) + lastT :lastT - firstT )

uint16_t SliderIndicatorBmp[ 16 ][ 11 ] = {
  
    { NULL },   // Empty
    { NULL },   // Image
    { 
        BITMAP_GAUGE_1_ID,  // gauge 
        BITMAP_GAUGE_1_ID, 
        BITMAP_GAUGE_2_ID, 
        BITMAP_GAUGE_1_ID, 
        BITMAP_GAUGE_1_ID, 
        BITMAP_GAUGE_5_ID,
        BITMAP_GAUGE_1_ID,
        BITMAP_GAUGE_1_ID,
        BITMAP_GAUGE_1_ID,
        BITMAP_GAUGE_1_ID,
        BITMAP_GAUGE_1_ID,
    },
    { NULL },               // bdi
    { 
        BITMAP_BUTTON_3_ID, // button
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_8_ID,
        BITMAP_BUTTON_9_ID,
        BITMAP_BUTTON_10_ID,
    },
    { 
        BITMAP_BUTTON_3_ID, // toogle button
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_3_ID,
        BITMAP_BUTTON_8_ID,
        BITMAP_BUTTON_9_ID,
        BITMAP_BUTTON_10_ID,
    },
    { 
        BITMAP_VERTICALSLIDER_0_ID, // vertical slider
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
        BITMAP_VERTICALSLIDER_0_ID,
    },
    {
        BITMAP_HORIZONTALSLIDER_0_ID,   
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
        BITMAP_HORIZONTALSLIDER_0_ID,
    },
    {
        BITMAP_BUTTON1_0_ID,    // check box
        BITMAP_BUTTON1_0_ID,  
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
        BITMAP_BUTTON1_0_ID, 
    },
    {
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
        BITMAP_TEMPERATURE_ID,
    },
    {
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_1_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
        BITMAP_BATTERY_0_ID,
    },
    {
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
        BITMAP_GRAPH_0_ID,
    },
    {
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
        BITMAP_INDICATOR_2_ID,
    },
    {
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_1_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
        BITMAP_CIRCLEPROGRESS_0_ID,
    },
    {
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_1_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
        BITMAP_IMAGEPROGRESS_0_ID,
    },
    {
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
        BITMAP_GROUPBUTTON_0_ID,
    },
        
};

static uint8_t ReadSDO( UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size )
{
	UNS32 abortCode = 0;
	UNS8 res = SDO_UPLOAD_IN_PROGRESS;

	// Read SDO
	UNS8 err = readNetworkDict ( &SmartDisplayHost_Data, nodeId, index, subIndex, dataType, 0 );

	if( err )
		return 0xFF;

	uint32_t tick = osKernelSysTick( );

	do
	{
		res = getReadResultNetworkDict (&SmartDisplayHost_Data, nodeId, data, size, &abortCode);

		if (res != SDO_UPLOAD_IN_PROGRESS)
			break;   

		// why????
		osDelay( 1 );

	}while( GetTimeDiff( tick, osKernelSysTick( ) ) < 10 );

	closeSDOtransfer( &SmartDisplayHost_Data, nodeId, SDO_CLIENT );

	if (res == SDO_FINISHED)
		return 0;

	return 0xFF;   
}


static uint16_t GetNetworkDictObj( uint16_t index )
{
	// UNS32 data;
	UNS32 size;
	// UNS32 abortCode;
	uint8_t type = 0;
	uint16_t style = 0;
	static uint8_t result;

	size = 1;
	result = ReadSDO( NODEID, index, SubIndexType, uint8, &type, &size );
    
	size = 2;
	result += ReadSDO( NODEID, index, SubIndexSytel, uint16, &style, &size );
    
	if( result != 0 )
		return BITMAP_RESERVED_ID;

	if( type == 0 || ( type >= 16 ) )
		return BITMAP_RESERVED_ID;

	if( ( style >= 11 )  )
		style = 10;

	return SliderIndicatorBmp[ type ][ style ]; 
}

MonitorView::MonitorView()
{

}

void MonitorView::setupScreen()
{
  MonitorViewBase::setupScreen();
  
  slider1.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2000 )) );
  slider2.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2001 )) );
  slider3.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2002 )) );
  slider4.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2003 )) );
  slider5.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2004 )) );
  slider6.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2005 )) );
  slider7.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2006 )) );
  slider8.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2007 )) );
  slider9.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2008 )) );
  slider10.setBitmaps(touchgfx::Bitmap(BITMAP_SILDE_2_ID), touchgfx::Bitmap(BITMAP_SILDE_1_ID), touchgfx::Bitmap( GetNetworkDictObj( 0x2009 )) );

	slider1.invalidate();
	slider2.invalidate();
	slider3.invalidate();
	slider4.invalidate();
	slider5.invalidate();
	slider6.invalidate();
	slider7.invalidate();
	slider8.invalidate();
	slider9.invalidate();
	slider10.invalidate();
  
  setState( &SmartDisplayHost_Data, Operational );

	indicate1 = Obj01_getValue;
	indicate2 = Obj02_getValue;
	indicate3 = Obj03_getValue;
	indicate4 = Obj04_getValue;
	indicate5 = Obj05_getValue;
	indicate6 = Obj06_getValue;
	indicate7 = Obj07_getValue;
	indicate8 = Obj08_getValue;
	indicate9 = Obj09_getValue;
	indicate10 = Obj10_getValue;

	uint16_t ArrayBmp[] = { BITMAP_INDICATOR_1_ID, BITMAP_INDICATOR_0_ID };

	image1.setBitmap( ( indicate1 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image2.setBitmap( ( indicate2 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image3.setBitmap( ( indicate3 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image4.setBitmap( ( indicate4 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image5.setBitmap( ( indicate5 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image6.setBitmap( ( indicate6 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image7.setBitmap( ( indicate7 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image8.setBitmap( ( indicate8 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image9.setBitmap( ( indicate9 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
	image10.setBitmap( ( indicate10 > 0 )?touchgfx::Bitmap(ArrayBmp[0]) :touchgfx::Bitmap(ArrayBmp[1]) );
}

void MonitorView::tearDownScreen()
{
	MonitorViewBase::tearDownScreen();
}

void MonitorView::handleTickEvent()
{
  static uint8_t tick = 0;
  
  if( GetTimeDiff( tick, osKernelSysTick( ) ) < 50 )
      return;

  tick = osKernelSysTick( );
    
  SmartDisplayHost_Data.PDO_status[ 0 ].last_message.data[ 0 ] = 0xFF;
  SmartDisplayHost_Data.PDO_status[ 1 ].last_message.data[ 0 ] = 0xFF;
  SmartDisplayHost_Data.PDO_status[ 2 ].last_message.data[ 0 ] = 0xFF;
  
	if( indicate1 != Obj01_setValue ) {
			
		indicate1 = Obj01_setValue;
		image1.setBitmap( ( indicate1 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image1.invalidate();

		slider1.setValue( indicate1 );
		slider1.invalidate( );
	}
	
	if( indicate2 != Obj02_setValue ) {
		
		indicate2 = Obj02_setValue;
		image2.setBitmap( ( indicate2 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image2.invalidate();

		slider2.setValue( indicate2 );
		slider2.invalidate( );
	}
	
	if( indicate3 != Obj03_setValue ) {
		
		indicate3 = Obj03_setValue;
		image3.setBitmap( ( indicate3 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image3.invalidate();

		slider3.setValue( indicate3 );
		slider3.invalidate( );
	}
	
	if( indicate4 != Obj04_setValue ) {
		
		indicate4 = Obj04_setValue;
		image4.setBitmap( ( indicate4 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image4.invalidate();

		slider4.setValue( indicate4 );
		slider4.invalidate( );
	}
	
	if( indicate5 != Obj05_setValue ) {
		
		indicate5 = Obj05_setValue;
		image5.setBitmap( ( indicate5 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image5.invalidate();

		slider5.setValue( indicate5 );
		slider5.invalidate( );
	}
	
	if( indicate6 != Obj06_setValue ) {
		
		indicate6 = Obj06_setValue;
		image6.setBitmap( ( indicate6 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image6.invalidate();

		slider6.setValue( indicate6 );
		slider6.invalidate( );
	}
	
	if( indicate7 != Obj07_setValue ) {
		
		indicate7 = Obj07_setValue;
		image7.setBitmap( ( indicate7 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image7.invalidate();

		slider7.setValue( indicate7 );
		slider7.invalidate( );
	}
	
	if( indicate8 != Obj08_setValue ) {
		
		indicate8 = Obj08_setValue;
		image8.setBitmap( ( indicate8 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image8.invalidate();

		slider8.setValue( indicate8 );
		slider8.invalidate( );
	}
	
	if( indicate9 != Obj09_setValue ) {
		
		indicate9 = Obj09_setValue;
		image9.setBitmap( ( indicate9 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image9.invalidate();

		slider9.setValue( indicate9 );
		slider9.invalidate( );
	}
	
	if( indicate10 != Obj10_setValue ) {
		
		indicate10 = Obj10_setValue;
		image10.setBitmap( ( indicate10 > 0 )?touchgfx::Bitmap(BITMAP_INDICATOR_1_ID) :touchgfx::Bitmap(BITMAP_INDICATOR_0_ID) );
		image10.invalidate();

		slider10.setValue( indicate10 );
		slider10.invalidate( );
	}
  
  sendOnePDOevent( &SmartDisplayHost_Data, 0 );
  sendOnePDOevent( &SmartDisplayHost_Data, 1 );
  sendOnePDOevent( &SmartDisplayHost_Data, 2 );
}


void MonitorView::Slider1ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj01_setValue = value;
}

void MonitorView::Slider2ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj02_setValue = value;
}

void MonitorView::Slider3ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj03_setValue = value;
}

void MonitorView::Slider4ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj04_setValue = value;
}

void MonitorView::Slider5ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj05_setValue = value;
}

void MonitorView::Slider6ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj06_setValue = value;
}

void MonitorView::Slider7ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj07_setValue = value;
}

void MonitorView::Slider8ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj08_setValue = value;
}

void MonitorView::Slider9ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj09_setValue = value;
}

void MonitorView::Slider10ChangeValue(int value)
{
	// Override and implement this function in Monitor
	Obj10_setValue = value;
}

void MonitorView::BuzzerOn()
{
  UNS32 data;
	UNS32 size = 1;
  UNS32 abortCode;
	
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
}



