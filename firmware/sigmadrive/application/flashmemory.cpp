/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */
#include <sstream>
#include <stdexcept>
#include "flashmemory.h"


/*
 * @param mem Pointer to the memory
 * @param nbytes The size of the memory region in bytes
 * @param sector The first sector of the flash the region occupies
 * @param nsectors The number of sectors the memory region occupies
 *
 * Note: All sectors will be erased together.
 *
 */
FlashMemory::FlashMemory(void *mem, size_t nbytes, size_t sector, size_t nsecors)
    : mem_((uint8_t*)mem)
    , nbytes_(nbytes)
    , sector_(sector)
    , nsectors_(nsecors)
{
}

FlashMemory::~FlashMemory()
{
}

void FlashMemory::erase()
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0xFFFFFFFF;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = sector_;
    EraseInitStruct.NbSectors = nsectors_;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_ERSERR);
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        HAL_FLASH_Lock();
        std::stringstream oss;
        oss << "Failed to erase sector: " << SectorError;
        throw std::runtime_error(oss.str());
    }
    HAL_FLASH_Lock();
}

void FlashMemory::program(const void *data, size_t datasize)
{
    uint8_t* memptr = (uint8_t*)mem_;
    uint8_t* dataptr = (uint8_t*)data;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0xFFFFFFFF;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = sector_;
    EraseInitStruct.NbSectors = nsectors_;

    if (datasize > nbytes_) {
        std::runtime_error("FlashMemory::program failed, the specified data size is too large.");
    }
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_ERSERR);
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        std::stringstream oss;
        oss << "Failed to erase sector: " << SectorError;
        throw std::runtime_error(oss.str());
    }

    while (datasize) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)memptr, *dataptr) != HAL_OK) {
            HAL_FLASH_Lock();
            std::runtime_error("FlashMemory::program failed");
        }
        datasize--;
        memptr++;
        dataptr++;
    }
    HAL_FLASH_Lock();
}

void* FlashMemory::data()
{
    return mem_;
}

size_t FlashMemory::size()
{
    return nbytes_;
}
