#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>

#define FLASH_STORAGE_PAGE       31                  // last page
#define FLASH_STORAGE_START_ADDR       (0x08000000 + (FLASH_STORAGE_PAGE * FLASH_PAGE_SIZE))  // last page address

typedef struct
{
uint16_t	last_state;
uint16_t	last_fault;
uint16_t	last_temper_setting;
}storage_data;


uint8_t save_flash_data(void);
uint16_t read_flash_data(void);
#endif
