/*
 * dma.h
 *
 *  Created on: May 11, 2017
 *      Author: mstoilov
 */

#ifndef DMA_H_
#define DMA_H_

#include "stm32f4xx_ll_dma.h"
#include "functionptr.h"

class Dma {
public:
	Dma(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config);
	virtual ~Dma();

	void ConfigAddresses(uint32_t SrcAddress, uint32_t DstAddress, uint32_t Direction);
	void SetDataLength(uint32_t nb);
	uint32_t GetDataLength()									{ return LL_DMA_GetDataLength(DMAx_, stream_); }
	uint32_t GetDataTransferDirection() 						{ return LL_DMA_GetDataTransferDirection(DMAx_, stream_); }
	void SetDataTransferDirection(uint32_t direction)			{ LL_DMA_SetDataTransferDirection(DMAx_, stream_, direction); }
	void Enable() 												{ LL_DMA_EnableStream(DMAx_, stream_); }
	void Disable()												{ LL_DMA_DisableStream(DMAx_, stream_); }
	void EnableIT_DME()											{ LL_DMA_EnableIT_DME(DMAx_, stream_); }
	void EnableIT_FE()											{ LL_DMA_EnableIT_FE(DMAx_, stream_); }
	void EnableIT_HT()											{ LL_DMA_EnableIT_HT(DMAx_, stream_); }
	void EnableIT_TC()											{ LL_DMA_EnableIT_TC(DMAx_, stream_); }
	void EnableIT_TE()											{ LL_DMA_EnableIT_TE(DMAx_, stream_); }
	void DisableIT_DME()										{ LL_DMA_DisableIT_DME(DMAx_, stream_); }
	void DisableIT_FE()											{ LL_DMA_DisableIT_FE(DMAx_, stream_); }
	void DisableIT_HT()											{ LL_DMA_DisableIT_HT(DMAx_, stream_); }
	void DisableIT_TC()											{ LL_DMA_DisableIT_TC(DMAx_, stream_); }
	void DisableIT_TE()											{ LL_DMA_DisableIT_TE(DMAx_, stream_); }
	virtual void IrqHandlerDME()								{ callback_DME_.call(); }
	virtual void IrqHandlerFE()									{ callback_FE_.call(); }
	virtual void IrqHandlerHT()									{ callback_HT_.call(); }
	virtual void IrqHandlerTC()									{ callback_TC_.call(); }
	virtual void IrqHandlerTE()									{ callback_TE_.call(); }
	DMA_TypeDef* GetDevice() 									{ return DMAx_; }
	uint32_t GetStream()										{ return stream_; }

	template<typename T>
	void callback_dme(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_DME_.attach(tptr, mptr);
	}
	void callback_dme(void (*fptr)(void))
	{
		callback_DME_.attach(fptr);
	}


	template<typename T>
	void callback_fe(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_FE_.attach(tptr, mptr);
	}
	void callback_fe(void (*fptr)(void))
	{
		callback_FE_.attach(fptr);
	}

	template<typename T>
	void callback_ht(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_HT_.attach(tptr, mptr);
	}
	void callback_ht(void (*fptr)(void))
	{
		callback_HT_.attach(fptr);
	}

	template<typename T>
	void callback_tc(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_TC_.attach(tptr, mptr);
	}
	void callback_tc(void (*fptr)(void))
	{
		callback_TC_.attach(fptr);
	}

	template<typename T>
	void callback_te(T* tptr, void (T::*mptr)(void))
	{
		if (tptr && mptr)
			callback_TE_.attach(tptr, mptr);
	}
	void callback_te(void (*fptr)(void))
	{
		callback_TE_.attach(fptr);
	}

protected:
	DMA_TypeDef* DMAx_;
	uint32_t stream_;
	uint32_t channel_;
	uint32_t id_;
	FunctionPointer callback_DME_;
	FunctionPointer callback_FE_;
	FunctionPointer callback_HT_;
	FunctionPointer callback_TC_;
	FunctionPointer callback_TE_;
};

#endif /* DMA_H_ */
