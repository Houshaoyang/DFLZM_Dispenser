//#include "stm32f0xx_hal.h"
//#include "flash.h"

//#define FLASH_STORAGE_PAGE       31                  // last page
//#define FLASH_TOTAL_SIZE        32768               // Total flash size (32KB)
//#define FLASH_STORAGE_START_ADDR       (0x08000000 + (FLASH_STORAGE_PAGE * FLASH_PAGE_SIZE))  // last page address

//uint16_t read_data;
//storage_data Storage_Data;
//static FLASH_EraseInitTypeDef EraseInitStruct;
//uint8_t save_storage_data_to_flash(void)
//{
//	uint8_t flash_err =0;
//	uint32_t Address = FLASH_STORAGE_START_ADDR;
//	uint32_t PageError = 0;
//	uint16_t *data_ptr = (uint16_t *)&Storage_Data;
//	uint16_t data_size = sizeof(storage_data) / sizeof(uint16_t);  // Calculate number of 16-bit units

//	HAL_FLASH_Unlock();
//	
//	    // 2. Disable interrupts to prevent operation interruption
//  __disable_irq();
//	
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//  EraseInitStruct.PageAddress = FLASH_STORAGE_START_ADDR;
//	EraseInitStruct.NbPages = 1;
//	
//	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
//		flash_err = HAL_FLASH_GetError();
//		__enable_irq();
//		HAL_FLASH_Lock();
//		return 1;//erro
//	}
//	
//	  // 4. Write structure data as consecutive 16-bit values
//    for (uint16_t i = 0; i < data_size; i++)
//    {
//        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + (i * 2), data_ptr[i]) != HAL_OK)
//        {
//            flash_err = HAL_FLASH_GetError();
//            __enable_irq();
//						HAL_FLASH_Lock();
//            return 1;
//        }
//    }

//		// 5. Restore interrupts
//    __enable_irq();

//    // 6. Lock flash memory
//    HAL_FLASH_Lock();

//	return 0;
//}

///**
// * @brief  Reads data from flash into the storage_data structure
// * @retval 0: Success 1: Failure
// */
//uint8_t read_storage_data_from_flash(void)
//{
//    uint32_t Address = FLASH_STORAGE_START_ADDR;
//    uint16_t *data_ptr = (uint16_t *)&Storage_Data;
//    uint16_t data_size = sizeof(storage_data) / sizeof(uint16_t);

//    // Check if address is within valid flash range (32KB total)
//    if (Address < 0x08000000 || Address >= (0x08000000 + FLASH_TOTAL_SIZE))
//    {
//        return 1;
//    }

//    // Read data as consecutive 16-bit values into structure
//    for (uint16_t i = 0; i < data_size; i++)
//    {
//        data_ptr[i] = *(__IO uint16_t *)(Address + (i * 2));
//    }

//    return 0;
//}

///**
// * @brief  Initializes storage data (reads from flash, sets defaults if invalid)
// */
//void init_storage_data(void)
//{
//    // Try to read data from flash
//    if (read_storage_data_from_flash() != 0)
//    {
//        // Read failed, set default values
//        Storage_Data.last_state = 1;               // Default state
//        Storage_Data.last_fault = 3;               // Default: no fault
//        Storage_Data.last_temper_setting = 45;     // Default temperature: 45
//        // Write default values to flash
//        save_storage_data_to_flash();
//    }
//}










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