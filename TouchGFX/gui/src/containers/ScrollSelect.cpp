#include <gui/containers/ScrollSelect.hpp>

ScrollSelect::ScrollSelect()
{

}

void ScrollSelect::initialize()
{
    ScrollSelectBase::initialize();
}

void ScrollSelect::updateText(int16_t value)
{
	const char *array[] = {

		"0", "1", "2", "3", "4", "5",
		"6", "7", "8", "9", "10"
	};

	Unicode::strncpy( textArea1Buffer, array[ value ], TEXTAREA1_SIZE );

    textArea1.invalidate();
}
