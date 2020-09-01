#ifndef SETTINGVIEW_HPP
#define SETTINGVIEW_HPP

#include <gui_generated/setting_screen/SettingViewBase.hpp>
#include <gui/setting_screen/SettingPresenter.hpp>
#include <touchgfx/mixins/MoveAnimator.hpp>

class SettingView : public SettingViewBase
{
public:
    SettingView();
    virtual ~SettingView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
        
	virtual void scrollWheel2UpdateItem(ScrollUnSelect& item, int16_t itemIndex);

    virtual void scrollWheel2UpdateCenterItem(ScrollSelect& item, int16_t itemIndex);
        
    virtual void scrollWheel3UpdateItem(ScrollUnSelect& item, int16_t itemIndex);

    virtual void scrollWheel3UpdateCenterItem(ScrollSelect& item, int16_t itemIndex);

    virtual void scrollWheel4UpdateItem(CustomContainer1& item, int16_t itemIndex);

    virtual void scrollWheel4UpdateCenterItem(ScrollListContainer& item, int16_t itemIndex);

	/*
     * Virtual Action Handlers
     */
    virtual void ButtonIsClickedFun();
        
    unsigned char getObjNum();
        
    void setObjNum( unsigned char num );
    
    uint8_t typeSelect;
    
    void scrollListItemSelectedHandler(int16_t itemSelected);
    
    Callback<SettingView, int16_t> scrollListItemSelectedCallback;
    
protected:
    
    uint32_t tick;
    uint8_t style;
    
	
};

#endif // SETTINGVIEW_HPP
