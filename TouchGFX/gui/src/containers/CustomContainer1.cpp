#include <gui/containers/CustomContainer1.hpp>

CustomContainer1::CustomContainer1()
{

}

void CustomContainer1::initialize()
{
    CustomContainer1Base::initialize();
}

void CustomContainer1::updateText(int16_t value)
{
	static const char *array[ ] = { 

		"Gauge", 
		"Button", 
		"Toggle Button", 
		"Vertical Silder",
		"Horizontal Silder",
		"Check Box",
		"Temperatrue", 
		"Battery",
	};
    
	Unicode::strncpy( textArea1Buffer, array[ value ], TEXTAREA1_SIZE );

	textArea1.invalidate();
}
