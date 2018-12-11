/*
 * dma.cpp
 *
 *  Created on: May 11, 2017
 *      Author: mstoilov
 */

#include "stm32f4xx_ll_bus.h"
#include "dma.h"

static Dma *g_dmas[] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
						nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

#define DMA_PTR_OFFSET(d, s) ((d - 1) * 8 + (s))

static void EnableDmaIrq(uint32_t dma, uint32_t stream);
static void DisableDmaIrq(uint32_t dma, uint32_t stream);

Dma::Dma(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config)
	: DMAx_(DMAx)
	, stream_(stream)
	, channel_(channel)
	, id_(-1)
{
	if (DMAx_ == DMA1) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
		id_ = 1;
	} else if (DMAx_ == DMA2) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
		id_ = 2;
	}
	g_dmas[DMA_PTR_OFFSET(id_, stream_)] = this;
	LL_DMA_ConfigTransfer(DMAx_, stream_, config);
	LL_DMA_SetChannelSelection(DMAx_, stream_, channel_);
	EnableDmaIrq(id_, stream_);
}

Dma::~Dma()
{
	DisableDmaIrq(id_, stream_);
	g_dmas[(id_ - 1) * 8 + stream_] = nullptr;
}

void Dma::ConfigAddresses(uint32_t srcAddress, uint32_t dstAddress, uint32_t direction)
{
	LL_DMA_ConfigAddresses(DMAx_, stream_, srcAddress, dstAddress, direction);
}

void Dma::SetDataLength(uint32_t nb)
{
	LL_DMA_SetDataLength(DMAx_, stream_, nb);
}


static void IrqHandlerStream0(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();

	if (LL_DMA_IsActiveFlag_DME0(DMAx)) {
		LL_DMA_ClearFlag_DME0(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE0(DMAx)) {
		LL_DMA_ClearFlag_FE0(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT0(DMAx)) {
		LL_DMA_ClearFlag_HT0(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC0(DMAx)) {
		LL_DMA_ClearFlag_TC0(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE0(DMAx)) {
		LL_DMA_ClearFlag_TE0(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream1(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME1(DMAx)) {
		LL_DMA_ClearFlag_DME1(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE1(DMAx)) {
		LL_DMA_ClearFlag_FE1(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT1(DMAx)) {
		LL_DMA_ClearFlag_HT1(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC1(DMAx)) {
		LL_DMA_ClearFlag_TC1(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE1(DMAx)) {
		LL_DMA_ClearFlag_TE1(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream2(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME2(DMAx)) {
		LL_DMA_ClearFlag_DME2(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE2(DMAx)) {
		LL_DMA_ClearFlag_FE2(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT2(DMAx)) {
		LL_DMA_ClearFlag_HT2(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC2(DMAx)) {
		LL_DMA_ClearFlag_TC2(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE2(DMAx)) {
		LL_DMA_ClearFlag_TE2(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream3(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME3(DMAx)) {
		LL_DMA_ClearFlag_DME3(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE3(DMAx)) {
		LL_DMA_ClearFlag_FE3(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT3(DMAx)) {
		LL_DMA_ClearFlag_HT3(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC3(DMAx)) {
		LL_DMA_ClearFlag_TC3(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE3(DMAx)) {
		LL_DMA_ClearFlag_TE3(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream4(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME4(DMAx)) {
		LL_DMA_ClearFlag_DME4(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE4(DMAx)) {
		LL_DMA_ClearFlag_FE4(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT4(DMAx)) {
		LL_DMA_ClearFlag_HT4(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC4(DMAx)) {
		LL_DMA_ClearFlag_TC4(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE4(DMAx)) {
		LL_DMA_ClearFlag_TE4(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream5(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME5(DMAx)) {
		LL_DMA_ClearFlag_DME5(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE5(DMAx)) {
		LL_DMA_ClearFlag_FE5(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT5(DMAx)) {
		LL_DMA_ClearFlag_HT5(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC5(DMAx)) {
		LL_DMA_ClearFlag_TC5(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE5(DMAx)) {
		LL_DMA_ClearFlag_TE5(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream6(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME6(DMAx)) {
		LL_DMA_ClearFlag_DME6(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE6(DMAx)) {
		LL_DMA_ClearFlag_FE6(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT6(DMAx)) {
		LL_DMA_ClearFlag_HT6(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC6(DMAx)) {
		LL_DMA_ClearFlag_TC6(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE6(DMAx)) {
		LL_DMA_ClearFlag_TE6(DMAx);
		dma->IrqHandlerTE();
	}
}

static void IrqHandlerStream7(Dma* dma)
{
	DMA_TypeDef* DMAx = dma->GetDevice();
	if (LL_DMA_IsActiveFlag_DME7(DMAx)) {
		LL_DMA_ClearFlag_DME7(DMAx);
		dma->IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE7(DMAx)) {
		LL_DMA_ClearFlag_FE7(DMAx);
		dma->IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT7(DMAx)) {
		LL_DMA_ClearFlag_HT7(DMAx);
		dma->IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TC7(DMAx)) {
		LL_DMA_ClearFlag_TC7(DMAx);
		dma->IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_TE7(DMAx)) {
		LL_DMA_ClearFlag_TE7(DMAx);
		dma->IrqHandlerTE();
	}
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 0)];
	if (dma)
		IrqHandlerStream0(dma);
}


extern "C" void DMA1_Stream1_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 1)];
	if (dma)
		IrqHandlerStream1(dma);
}


extern "C" void DMA1_Stream2_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 2)];
	if (dma)
		IrqHandlerStream2(dma);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 3)];
	if (dma)
		IrqHandlerStream3(dma);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 4)];
	if (dma)
		IrqHandlerStream4(dma);
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 5)];
	if (dma)
		IrqHandlerStream5(dma);
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 6)];
	if (dma)
		IrqHandlerStream6(dma);
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(1, 7)];
	if (dma)
		IrqHandlerStream7(dma);
}


extern "C" void DMA2_Stream0_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 0)];
	if (dma)
		IrqHandlerStream0(dma);
}


extern "C" void DMA2_Stream1_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 1)];
	if (dma)
		IrqHandlerStream1(dma);
}


extern "C" void DMA2_Stream2_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 2)];
	if (dma)
		IrqHandlerStream2(dma);
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 3)];
	if (dma)
		IrqHandlerStream3(dma);
}

extern "C" void DMA2_Stream4_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 4)];
	if (dma)
		IrqHandlerStream4(dma);
}

extern "C" void DMA2_Stream5_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 5)];
	if (dma)
		IrqHandlerStream5(dma);
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 6)];
	if (dma)
		IrqHandlerStream6(dma);
}

extern "C" void DMA2_Stream7_IRQHandler(void)
{
	Dma *dma = g_dmas[DMA_PTR_OFFSET(2, 7)];
	if (dma)
		IrqHandlerStream7(dma);
}

static void EnableDmaIrq(uint32_t dma, uint32_t stream)
{
	if (dma == 1 && stream == 0) {
		NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	} else if (dma == 1 && stream == 1) {
		NVIC_SetPriority(DMA1_Stream1_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	} else if (dma == 1 && stream == 2) {
		NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	} else if (dma == 1 && stream == 3) {
		NVIC_SetPriority(DMA1_Stream3_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	} else if (dma == 1 && stream == 4) {
		NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	} else if (dma == 1 && stream == 5) {
		NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	} else if (dma == 1 && stream == 6) {
		NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	} else if (dma == 1 && stream == 7) {
		NVIC_SetPriority(DMA1_Stream7_IRQn, 0);
		NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	} else if (dma == 2 && stream == 0) {
		NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	} else if (dma == 2 && stream == 1) {
		NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	} else if (dma == 2 && stream == 2) {
		NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	} else if (dma == 2 && stream == 3) {
		NVIC_SetPriority(DMA2_Stream3_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	} else if (dma == 2 && stream == 4) {
		NVIC_SetPriority(DMA2_Stream4_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	} else if (dma == 2 && stream == 5) {
		NVIC_SetPriority(DMA2_Stream5_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	} else if (dma == 2 && stream == 6) {
		NVIC_SetPriority(DMA2_Stream6_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	} else if (dma == 2 && stream == 7) {
		NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
		NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	}
}

static void DisableDmaIrq(uint32_t dma, uint32_t stream)
{
	if (dma == 1 && stream == 0) {
		NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream0_IRQn);
	} else if (dma == 1 && stream == 1) {
		NVIC_SetPriority(DMA1_Stream1_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	} else if (dma == 1 && stream == 2) {
		NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream2_IRQn);
	} else if (dma == 1 && stream == 3) {
		NVIC_SetPriority(DMA1_Stream3_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	} else if (dma == 1 && stream == 4) {
		NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream4_IRQn);
	} else if (dma == 1 && stream == 5) {
		NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	} else if (dma == 1 && stream == 6) {
		NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream6_IRQn);
	} else if (dma == 1 && stream == 7) {
		NVIC_SetPriority(DMA1_Stream7_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	} else if (dma == 2 && stream == 0) {
		NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	} else if (dma == 2 && stream == 1) {
		NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	} else if (dma == 2 && stream == 2) {
		NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	} else if (dma == 2 && stream == 3) {
		NVIC_SetPriority(DMA2_Stream3_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream3_IRQn);
	} else if (dma == 2 && stream == 4) {
		NVIC_SetPriority(DMA2_Stream4_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream4_IRQn);
	} else if (dma == 2 && stream == 5) {
		NVIC_SetPriority(DMA2_Stream5_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream5_IRQn);
	} else if (dma == 2 && stream == 6) {
		NVIC_SetPriority(DMA2_Stream6_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream6_IRQn);
	} else if (dma == 2 && stream == 7) {
		NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	}
}
