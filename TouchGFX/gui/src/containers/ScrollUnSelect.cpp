#include <gui/containers/ScrollUnSelect.hpp>

ScrollUnSelect::ScrollUnSelect()
{

}

void ScrollUnSelect::initialize()
{
    ScrollUnSelectBase::initialize();
}

void ScrollUnSelect::updateText(int16_t value)
{
	const char *array[] = {

		"0", "1", "2", "3", "4", "5",
		"6", "7", "8", "9", "10"
	};

	Unicode::strncpy( textArea1Buffer, array[ value ], TEXTAREA1_SIZE );

    textArea1.invalidate();
}

