#ifndef SCROLLUNSELECT_HPP
#define SCROLLUNSELECT_HPP

#include <gui_generated/containers/ScrollUnSelectBase.hpp>

class ScrollUnSelect : public ScrollUnSelectBase
{
public:
    ScrollUnSelect();
    virtual ~ScrollUnSelect() {}

    virtual void initialize();

	void updateText(int16_t value);

protected:
};

#endif // SCROLLUNSELECT_HPP
