#include <gui/containers/ScrollListContainer.hpp>

ScrollListContainer::ScrollListContainer()
{

}



void ScrollListContainer::initialize()
{
    ScrollListContainerBase::initialize();
}

void ScrollListContainer::updateText(int16_t value)
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


