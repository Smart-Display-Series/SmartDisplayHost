#ifndef SCROLLLISTCONTAINER_HPP
#define SCROLLLISTCONTAINER_HPP

#include <gui_generated/containers/ScrollListContainerBase.hpp>

class ScrollListContainer : public ScrollListContainerBase
{
public:
    ScrollListContainer();
    virtual ~ScrollListContainer() {}

    virtual void initialize();

    void updateText(int16_t value);
        
protected:
};

#endif // SCROLLLISTCONTAINER_HPP
