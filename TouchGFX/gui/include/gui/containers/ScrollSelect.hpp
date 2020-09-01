#ifndef SCROLLSELECT_HPP
#define SCROLLSELECT_HPP

#include <gui_generated/containers/ScrollSelectBase.hpp>

class ScrollSelect : public ScrollSelectBase
{
public:
    ScrollSelect();
    virtual ~ScrollSelect() {}

    virtual void initialize();

	void updateText(int16_t value);
protected:
};

#endif // SCROLLSELECT_HPP
