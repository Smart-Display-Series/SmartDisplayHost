#ifndef MONITORVIEW_HPP
#define MONITORVIEW_HPP

#include <gui_generated/monitor_screen/MonitorViewBase.hpp>
#include <gui/monitor_screen/MonitorPresenter.hpp>

class MonitorView : public MonitorViewBase
{
public:
    MonitorView();
    virtual ~MonitorView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

    /*
     * Virtual Action Handlers
     */
 
    virtual void BuzzerOn();
    virtual void Slider1ChangeValue(int value);
    virtual void Slider2ChangeValue(int value);
    virtual void Slider3ChangeValue(int value);
    virtual void Slider4ChangeValue(int value);
    virtual void Slider5ChangeValue(int value);
    virtual void Slider6ChangeValue(int value);
    virtual void Slider7ChangeValue(int value);
    virtual void Slider8ChangeValue(int value);
    virtual void Slider9ChangeValue(int value);
    virtual void Slider10ChangeValue(int value);

protected:

	uint8_t indicate1;
	uint8_t indicate2;
	uint8_t indicate3;
	uint8_t indicate4;
	uint8_t indicate5;
	uint8_t indicate6;
	uint8_t indicate7;
	uint8_t indicate8;
	uint8_t indicate9;
	uint8_t indicate10;
	
};

#endif // MONITORVIEW_HPP
