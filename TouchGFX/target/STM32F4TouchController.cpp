#include <STM32F4TouchController.hpp>

/* USER CODE BEGIN BSP user includes */
extern "C" I2C_HandleTypeDef hi2c2;
/* USER CODE END BSP user includes */

extern "C"
{

uint32_t LCD_GetXSize();
uint32_t LCD_GetYSize();
}

using namespace touchgfx;

void STM32F4TouchController::init()
{
   /* USER CODE BEGIN F4TouchController_init */

    /* Add code for touch controller Initialization*/
    //BSP_TS_Init(LCD_GetXSize(), LCD_GetYSize());

  /* USER CODE END  F4TouchController_init  */
}

bool STM32F4TouchController::sampleTouch(int32_t& x, int32_t& y)
{
  /* USER CODE BEGIN  F4TouchController_sampleTouch  */
    
    /*TS_StateTypeDef state;
    BSP_TS_GetState(&state);
    if (state.TouchDetected)
    {
        x = state.x;
        y = state.y;
        return true;
    }*/
    
    uint8_t Buf[5];
    uint8_t event;
    uint8_t ID;
    uint16_t sx,sy;
    
    /* USER CODE BEGIN  F4TouchController_sampleTouch  */
    if(HAL_I2C_Mem_Read( &hi2c2, 0x70, 0x02, I2C_MEMADD_SIZE_8BIT,&Buf[0], 5, 10 ) == HAL_OK)
    {
        event = (Buf[1] >> 6) & 0x03;
        ID = (Buf[3] >> 4) & 0x0F;
        if( ID == 0 )
        {
            if( ( event == 0x00 ) || ( event == 0x02 ) )
            {
                sx = ((Buf[1] & 0x0F) << 8) | Buf[2];
                sy = ((Buf[3] & 0x0F) << 8) | Buf[4];
                x = (sx * 100) / 224;
                y = (sy * 15) / 32;
                return true;
            }
        }
    }
    return false;
    
 /* USER CODE END F4TouchController_sampleTouch */    
}
