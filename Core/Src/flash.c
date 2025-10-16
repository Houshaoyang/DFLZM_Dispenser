#include "stm32f0xx_hal.h"
#include "flash.h"


//typedef struct
//{
//uint16_t	last_state;
//uint16_t	last_fault;
//uint16_t	last_temper_setting;
//}storage_data;

uint16_t read_data[2];
storage_data Storage_Data={7,8,9};
static FLASH_EraseInitTypeDef EraseInitStruct;

uint8_t save_flash_data(void)
{
	uint8_t flash_err =0;
	uint16_t data=2;
	uint32_t Address = FLASH_STORAGE_START_ADDR, PageError = 0;
	uint16_t *data_ptr = (uint16_t *)&Storage_Data;
	uint16_t data_size = sizeof(storage_data) / sizeof(uint16_t);
	
	HAL_FLASH_Unlock();
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_STORAGE_START_ADDR;
	EraseInitStruct.NbPages = 1;
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
		flash_err = HAL_FLASH_GetError();
		HAL_FLASH_Unlock();
		return 1;//erro
	}
	for (uint16_t i = 0; i < data_size; i++)
  {
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address+(2*i), data_ptr[i]) != HAL_OK)
    {
      flash_err = HAL_FLASH_GetError();
			HAL_FLASH_Unlock();
			return 1;//erro
    }
	}	

	HAL_FLASH_Lock();
	return 0;
}

uint16_t read_flash_data(void)
{
	uint32_t Address = FLASH_STORAGE_START_ADDR;
	uint16_t *data_ptr = (uint16_t *)&Storage_Data;
	uint16_t data_size = sizeof(storage_data) / sizeof(uint16_t);
	for(uint16_t i = 0; i < data_size; i++)
  {
		data_ptr[i] = *(__IO uint16_t *)(Address + (i * 2));
  }
	
	return 0 ;
}