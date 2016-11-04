
#define Dummy_Byte 0xA5

/* Select SPI FLASH: ChipSelect pin low  */
#define Select_Flash()     GPIO_ResetBits(GPIOA, GPIO_Pin_4)
/* Deselect SPI FLASH: ChipSelect pin high */
#define NotSelect_Flash()    GPIO_SetBits(GPIOA, GPIO_Pin_4)


void SPI_Flash_Init(void);	        
u8 SPI_Flash_ReadByte(void);		
u8 SPI_Flash_SendByte(u8 byte);		


void FlashWaitBusy(void);			   
void FlashReadID(void);		
	

