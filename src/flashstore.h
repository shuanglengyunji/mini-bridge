#ifndef __FLASHSTORE_H__
#define __FLASHSTORE_H__
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define STORE_DATA_ADDRESS		        ((uint32_t)0x08020000)  //sector5 start address 
#define STORE_DATA_SIZE			        ((uint32_t)0x0001F400)	//data size
#define SECTOR_SIZE				        ((uint32_t)0x0001F400)  

typedef struct {
    uint32_t begin_address;
    uint32_t area_size;
    uint32_t sector_num;
    uint32_t write_address;
}FLASH_HandleType;
extern FLASH_HandleType flash_store;
void flash_data_empty(FLASH_HandleType *handler);
void flash_write_data_begin(FLASH_HandleType *handler, uint32_t offset);
void flash_write_data_end(FLASH_HandleType *handler);
void flash_data_set_write_address(FLASH_HandleType *handler, uint32_t write_address);
void flash_data_write_data8(FLASH_HandleType *handler, uint8_t *data, uint32_t size);
void flash_data_write_data16(FLASH_HandleType *handler, uint16_t *data, uint32_t size);
void flash_data_write_data32(FLASH_HandleType *handler, uint32_t *data, uint32_t size);


#endif /* __FLASHSTORE_H__ */