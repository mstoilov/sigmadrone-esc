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
#ifndef FLASHMEMORY_H_
#define FLASHMEMORY_H_

#include "stm32f7xx_hal.h"

class FlashMemory {
public:
    FlashMemory(void *mem, size_t nbytes, size_t sector, size_t nsecors);
    ~FlashMemory();

    void erase();
    void program(const void *data, size_t nbytes);
    void* data();
    size_t size();

protected:
public:
    uint8_t* mem_;
    size_t nbytes_;
    size_t sector_;
    size_t nsectors_;
};

#endif // FLASHMEMORY_H_
