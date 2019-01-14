/*
 * dma.h
 *
 *  Created on: May 11, 2017
 *      Author: mstoilov
 */

#ifndef DMA_H_
#define DMA_H_

#include <functional>

#include "stm32f4xx_ll_dma.h"

class Dma {
public:
	Dma(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config, uint32_t irq_priority);
	virtual ~Dma();

	void ConfigAddresses(uint32_t SrcAddress, uint32_t DstAddress, uint32_t Direction);
	void SetDataLength(uint32_t nb)								{ LL_DMA_SetDataLength(DMAx_, stream_, nb); }
	uint32_t GetDataLength()									{ return LL_DMA_GetDataLength(DMAx_, stream_); }
	uint32_t GetDataTransferDirection() 						{ return LL_DMA_GetDataTransferDirection(DMAx_, stream_); }
	void SetDataTransferDirection(uint32_t direction)			{ LL_DMA_SetDataTransferDirection(DMAx_, stream_, direction); }
	void Enable() 												{ LL_DMA_EnableStream(DMAx_, stream_); }
	void Disable()												{ LL_DMA_DisableStream(DMAx_, stream_); }
	bool IsEnabled()											{ return LL_DMA_IsEnabledStream(DMAx_, stream_); }
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
	virtual void IrqHandlerDME()								{ callback_DME_(); }
	virtual void IrqHandlerFE()									{ callback_FE_(); }
	virtual void IrqHandlerHT()									{ callback_HT_(); }
	virtual void IrqHandlerTC()									{ callback_TC_(); }
	virtual void IrqHandlerTE()									{ callback_TE_(); }
	DMA_TypeDef* GetDevice() 									{ return DMAx_; }
	uint32_t GetStream()										{ return stream_; }
	uint32_t GetChannel()										{ return channel_; }
	void EnableIrq(uint32_t priority);
	void DisableIrq();

	template<typename T>
	void Callback_DME(T* tptr, void (T::*mptr)(void))
	{
		callback_DME_ = [=](void){(tptr->*mptr)();};
	}
	void Callback_DME(void (*fptr)(void))
	{
		callback_DME_ = [=](void){(*fptr)();};
	}

	template<typename T>
	void Callback_FE(T* tptr, void (T::*mptr)(void))
	{
		callback_FE_ = [=](void){(tptr->*mptr)();};
	}
	void Callback_FE(void (*fptr)(void))
	{
		callback_FE_ = [=](void){(*fptr)();};
	}

	template<typename T>
	void Callback_HT(T* tptr, void (T::*mptr)(void))
	{
		callback_HT_ = [=](void){(tptr->*mptr)();};
	}
	void Callback_HT(void (*fptr)(void))
	{
		callback_HT_ = [=](void){(*fptr)();};
	}

	template<typename T>
	void Callback_TC(T* tptr, void (T::*mptr)(void))
	{
		callback_TC_ = [=](void){(tptr->*mptr)();};
	}
	void Callback_TC(void (*fptr)(void))
	{
		callback_TC_ = [=](void){(*fptr)();};
	}

	template<typename T>
	void Callback_TE(T* tptr, void (T::*mptr)(void))
	{
		callback_TE_ = [=](void){(tptr->*mptr)();};
	}
	void Callback_TE(void (*fptr)(void))
	{
		callback_TE_ = [=](void){(*fptr)();};
	}

protected:
	void IrqHandlerStream0();
	void IrqHandlerStream1();
	void IrqHandlerStream2();
	void IrqHandlerStream3();
	void IrqHandlerStream4();
	void IrqHandlerStream5();
	void IrqHandlerStream6();
	void IrqHandlerStream7();

protected:
	DMA_TypeDef* DMAx_;
	uint32_t stream_;
	uint32_t channel_;
	uint32_t dma_num_;

	std::function<void(void)> callback_DME_ = [](void){};
	std::function<void(void)> callback_FE_ = [](void){};
	std::function<void(void)> callback_HT_ = [](void){};
	std::function<void(void)> callback_TC_ = [](void){};
	std::function<void(void)> callback_TE_ = [](void){};
};

#endif /* DMA_H_ */
