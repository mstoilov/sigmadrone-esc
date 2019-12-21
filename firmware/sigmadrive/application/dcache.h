#ifndef _DCACHE_H_
#define _DCACHE_H_

#include "core_cm7.h"

//#define USE_DCACHE 1

#ifdef USE_DCACHE

__STATIC_INLINE uint32_t RoundDown(uint32_t value, uint32_t n)
{
	return value & ~(n - 1);
}

__STATIC_INLINE uint32_t RoundUp(uint32_t value, uint32_t n)
{
	return (value + n - 1) & ~(n - 1);
}


__STATIC_INLINE void EnableDCache()
{
	SCB_InvalidateDCache();
	SCB_EnableDCache();
}

__STATIC_INLINE void CleanInvalidateDCache_by_Addr(void *addr, uint32_t dsize)
{
	uint32_t address = RoundDown((uint32_t)addr, 32);
	uint32_t size = RoundUp((uint32_t)addr + dsize, 32) - address;
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)address, size);
}

__STATIC_INLINE void InvalidateDCache_by_Addr(void *addr, uint32_t dsize)
{
	uint32_t address = RoundDown((uint32_t)addr, 32);
	uint32_t size = RoundUp((uint32_t)addr + dsize, 32) - address;
	SCB_InvalidateDCache_by_Addr((uint32_t*)address, size);
}

__STATIC_INLINE void CleanDCache_by_Addr(void *addr, uint32_t dsize)
{
	uint32_t address = RoundDown((uint32_t)addr, 32);
	uint32_t size = RoundUp((uint32_t)addr + dsize, 32) - address;
	SCB_CleanDCache_by_Addr((uint32_t*)address, size);
}


#else

//#define CleanInvalidateDCache_by_Addr(addr, dsize) do {} while (0)
//#define InvalidateDCache_by_Addr(addr, dsize) do {} while (0)
//#define CleanDCache_by_Addr(addr, dsize) do {} while (0)
//#define EnableDCache() do {} while (0)

#endif


#endif
