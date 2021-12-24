#include "flashstore.h"
FLASH_HandleType flash_store = {
    STORE_DATA_ADDRESS,
    STORE_DATA_SIZE,
    1,
    0
};

void flash_data_empty(FLASH_HandleType *handle)
{
    uint32_t count;
    HAL_FLASH_Unlock();
    for(count = 0; count < handle->sector_num; count++)
    {
        FLASH_EraseInitTypeDef flash_erase_config;
        uint32_t page_error;
        flash_erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
        flash_erase_config.Sector = FLASH_SECTOR_5;
        flash_erase_config.NbSectors = 1;
        flash_erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        if(HAL_FLASHEx_Erase(&flash_erase_config, &page_error) != HAL_OK) {
			printf("erase flash failed!\r\n");
			break;
		}
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
    }
    HAL_FLASH_Lock();
}

void flash_write_data_begin(FLASH_HandleType *handler, uint32_t offset)
{
    if(offset >= SECTOR_SIZE) return;
    HAL_FLASH_Unlock();
    handler->write_address = handler->begin_address;
}
void flash_write_data_end(FLASH_HandleType *handler)
{
    HAL_FLASH_Lock();
}
void flash_data_set_write_address(FLASH_HandleType *handler, uint32_t write_address)
{
    if(write_address < handler->begin_address) return;
    if(write_address > handler->begin_address + handler->area_size) return;
    handler->write_address = write_address;
}
void flash_data_write_data8(FLASH_HandleType *handler, uint8_t *data, uint32_t size)
{
    if(handler->write_address < handler->begin_address) return;
    if((handler->write_address + size ) > handler->begin_address + handler->area_size) return;
    for(uint32_t i = 0; i < size; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, handler->write_address, (uint64_t)data[i]) == HAL_OK)
        {
            handler->write_address += 1;
        }
    }
}
void flash_data_write_data16(FLASH_HandleType *handler, uint16_t *data, uint32_t size)
{
    if(handler->write_address < handler->begin_address) return;
    if((handler->write_address + size * 2) > handler->begin_address + handler->area_size) return;
    for(uint32_t i = 0; i < size; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, handler->write_address, (uint64_t)data[i]) == HAL_OK)
        {
            handler->write_address += 2;
        }
    }
}
void flash_data_write_data32(FLASH_HandleType *handler, uint32_t *data, uint32_t size)
{
    if(handler->write_address < handler->begin_address) return;
    if((handler->write_address + size * 4) > handler->begin_address + handler->area_size) return;
    for(uint32_t i = 0; i < size; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, handler->write_address, (uint64_t)data[i]) == HAL_OK)
        {
            handler->write_address += 4;
        }
    }
}