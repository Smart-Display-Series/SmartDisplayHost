#include "w25q128jv.h"

extern SPI_HandleTypeDef hspi1;

uint8_t Inital_SPI_Flash(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.Pin = FLASH_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
	HAL_GPIO_Init( FLASH_GPIO_CS, &GPIO_InitStruct );
    
    FLASH_CS_Select;
    
	Flash_SPI_Send_Byte( Enable_Reset );

	HAL_Delay( 5 );

	Flash_SPI_Send_Byte( Reset_Device );

	HAL_Delay( 5 );
	
	#if 1
	while( 1 ) {

		HAL_Delay( 10 );
		
		if( _Manufacturer_ID == Flash_Read_Status( JEDEC_ID ) ) {
			break;			
		}
	}
	#endif
	
	// W25Q SPI FLASH Manufacturer ID -> 0xEF
	if ( 0xEF != Flash_Read_Status( JEDEC_ID ) ) {

		return 1;
	}

	// Hardware Unprotected   
	// When /WP pin is high the Status register is unlocked and can 
	// be written to after a Write Enable instruction, WEL=1.  

	// SRL SRP /WP
	// 0   1   1 
	
    #if 1
	uint8_t status_1 = 0xFF;
	uint8_t status_2 = 0xFF;

	status_1 = Flash_Read_Status(Read_Status_Register_1);
	status_2 = Flash_Read_Status(Read_Status_Register_2);

	Flash_Write_Status( status_1 | 0x80, status_2 & 0xFE ); 	
	#endif	
    
    #if 0
    // Flash_Sector_Erase( 0 );
    // Flash_Byte_Program( 0, 0x77 );
    uint8_t array[] = { 0, 1, 2, 3, 4, 5 };
    
    Flash_Block_Erase( 0 );
    
    Flash_Program( 0, array, sizeof( array ) );
    
    uint8_t tmp[ 6 ] = { 0, 0, 0, 0, 0, 0 };
    
    Flash_Read( 0, tmp, sizeof( array ) );
    
    #endif
    
	return 0;
}

void SPI_DMA_Send_Flash(uint8_t *aTxBuffer,uint8_t *aRxBuffer,int BUFFERSIZE)
{
  unsigned int tmr_out = 0xFFFF;
	
  if(HAL_SPI_TransmitReceive_DMA( &hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
  {
  
  }

  while( ( HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY ) && ( tmr_out-- ) ) {
  
  }   
}

uint8_t Flash_Read_Status( uint8_t Register )
{
  unsigned char Flash_Status;
  uint8_t RX_Bf[ 2 ] = { 0, 0 };
  uint8_t TX_Bf[ 2 ] = { 0, 0 };
  
  TX_Bf[ 0 ] = Register;
  
  FLASH_CS_Select;

  SPI_DMA_Send_Flash( TX_Bf, RX_Bf, 2 );
  // HAL_SPI_TransmitReceive( &hspi1, TX_Bf, RX_Bf, 2, 10 );	
  
  FLASH_CS_DeSelect;
	
  Flash_Status = RX_Bf[1];
  return Flash_Status;
}

void Flash_Write_Status(uint8_t Flash_Status_1,uint8_t Flash_Status_2)
{
  uint8_t RX_Bf[3] = {0,0};
  uint8_t TX_Bf[3] = {0,0};
    
  TX_Bf[0] = Volatile_SR_Write_Enable;
  
  FLASH_CS_Select;
  SPI_DMA_Send_Flash(TX_Bf,RX_Bf,1);  
  FLASH_CS_DeSelect;
    
  TX_Bf[0] = Write_Status_Register_1;
  TX_Bf[1] = Flash_Status_1;
  TX_Bf[2] = Flash_Status_2;

  FLASH_CS_Select;
  SPI_DMA_Send_Flash(TX_Bf,RX_Bf,3);
  FLASH_CS_DeSelect;	
}

uint8_t Flash_SPI_Send_Byte(uint8_t Flash_Command)
{
  uint8_t RX_Bf[1] = {0};
  uint8_t TX_Bf[1] = {0};

  TX_Bf[0] = Flash_Command;  
  SPI_DMA_Send_Flash( TX_Bf, RX_Bf, 1 );
  
  return RX_Bf[0];  
}
 
void Flash_Chip_Erase( void )
{
	FLASH_CS_Select;
	Flash_SPI_Send_Byte( Chip_Erase );  
	FLASH_CS_DeSelect;

	// Check_Busy
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ); 
  
}

void Flash_Sector_Erase( unsigned long sector_addr )
{
	FLASH_CS_Select;
	
	Flash_SPI_Send_Byte( Sector_Erase );

	Flash_SPI_Send_Byte( sector_addr >> 16 );
	Flash_SPI_Send_Byte( sector_addr >> 8 );
	Flash_SPI_Send_Byte( sector_addr );

	FLASH_CS_DeSelect;

	// Check_Busy
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ); 
}


void Flash_Write_Enable(void)
{
  FLASH_CS_Select;  
  Flash_SPI_Send_Byte(Write_Enable);  
  FLASH_CS_DeSelect;
}

void Flash_Write_Disable(void)
{
  FLASH_CS_Select;  
  Flash_SPI_Send_Byte(Write_Disable);  
  FLASH_CS_DeSelect;
}

void Flash_Read( unsigned int ADDR, unsigned char *out, unsigned int size )
{
  uint8_t CMD_ADD[5];
  unsigned long start_addr;
 
  start_addr = ADDR;
      
	CMD_ADD[0] = Fast_Read_Data;
	CMD_ADD[1] = (start_addr >> 16);
	CMD_ADD[2] = (start_addr >> 8);
	CMD_ADD[3] = (start_addr & 0xFF);
	CMD_ADD[4] = 0x00;              // Dummy

	Flash_Write_Enable( ); 
	
	while( 0x01 & Flash_Read_Status(Read_Status_Register_1) );

	FLASH_CS_Select;  
   
  #if 1
	//Send Command and Address
	if( HAL_SPI_Transmit_DMA( &hspi1, CMD_ADD, 5 ) != HAL_OK) {

		// goto _Fail;
	}  
  #endif

  #if 0
  HAL_SPI_Transmit( &hspi1, CMD_ADD, 5, 10 );
	#endif
	
	while( ( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) );
        
	// Send Data

  #if 0
  HAL_SPI_Receive( &hspi1, out, size, 100 );
	#endif
    
  #if 1
	if( HAL_SPI_Receive_DMA( &hspi1, out, size ) != HAL_OK) {

		// goto _Fail;
	}    
  #endif

	while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY );

	FLASH_CS_DeSelect;
    
  while( ( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ) );
}

void Flash_Block_Erase(unsigned long Block_Address)
{
	// Check_Busy

	Flash_Write_Enable();
	
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) );

	FLASH_CS_Select;

	Flash_SPI_Send_Byte( _64K_Black_Erase);

	Flash_SPI_Send_Byte( Block_Address >> 16 );
	Flash_SPI_Send_Byte( Block_Address >> 8 );
	Flash_SPI_Send_Byte( Block_Address );

	FLASH_CS_DeSelect;

	// Check_Busy
	while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) ); 
}

unsigned char Flash_Byte_Program(unsigned long Flash_ADD,uint8_t Program_Data)
{
  
	if(Flash_ADD > 0x01000000) return 1;
	
	Flash_Write_Enable();

	while(0x01 & Flash_Read_Status(Read_Status_Register_1));

	FLASH_CS_Select;    
	Flash_SPI_Send_Byte(0x02);    
	Flash_SPI_Send_Byte(Flash_ADD >> 16);    
	Flash_SPI_Send_Byte(Flash_ADD >> 8);    
	Flash_SPI_Send_Byte(Flash_ADD);    
	Flash_SPI_Send_Byte(Program_Data);    
	FLASH_CS_DeSelect;    
    
    // 等待寫入結束
	while(0x01 & Flash_Read_Status(Read_Status_Register_1));    

	return 0;
}

unsigned char Flash_Program( unsigned long ADDR, uint8_t *Data, uint16_t size )
{
    uint8_t CMD_ADD[ 4 ];
	
    uint32_t current_addr, end_addr;
	
    uint16_t current_size;
    
    current_addr = 0;
	
    while( current_addr <= ADDR )
        current_addr += FLASH_PAGE_SIZE;
    
    current_size = current_addr - ADDR;
    
    // Check if the size of the data is less than the remaining place in the page
    if ( current_size > size )
        current_size = size;

    // Initialize the address variables
    current_addr = ADDR;
    end_addr = ADDR + size;
    
    // 16Mbyte 
    if( current_addr > 0x01000000 )
		return 1;
	
    // 16Mbyte
	if( end_addr >= 0x01000000 )
		return 1;     

	Flash_Write_Enable();

	do 
	{            
		CMD_ADD[0] = 0x02;
		CMD_ADD[1] = (current_addr >> 16);
		CMD_ADD[2] = (current_addr >> 8);
		CMD_ADD[3] = (current_addr & 0xFF);

		Flash_Write_Enable();    
		
		while( 0x01 & Flash_Read_Status( Read_Status_Register_1 ) );

		FLASH_CS_Select;  
		
		//Send Command and Address
		if(HAL_SPI_Transmit_DMA( &hspi1, CMD_ADD, 4 ) != HAL_OK)
		{
			// _Error_Handler(__FILE__, __LINE__);
            while( 1 );
        }
		
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY );   
			
		// Send Data
		if(HAL_SPI_Transmit_DMA(&hspi1, Data, current_size) != HAL_OK)
		{
			// _Error_Handler(__FILE__, __LINE__);
            while( 1 );
		}
		
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY );
		
		FLASH_CS_DeSelect;

		while( 0x01 & Flash_Read_Status(Read_Status_Register_1) );
			
		// Update the address and size variables for next page programming
		current_addr += current_size;
		Data += current_size;
		current_size = ( ( current_addr + FLASH_PAGE_SIZE ) > end_addr) ? (end_addr - current_addr) : FLASH_PAGE_SIZE;              
	
	}while( current_addr < end_addr );

    return 0;
}

