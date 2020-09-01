#ifndef MONITORPRESENTER_HPP
#define MONITORPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class MonitorView;

class MonitorPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    MonitorPresenter(MonitorView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~MonitorPresenter() {};

private:
    MonitorPresenter();

    MonitorView& view;
};


#endif // MONITORPRESENTER_HPP
