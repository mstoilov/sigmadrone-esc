/*
 * dma.cpp
 *
 *  Created on: May 11, 2017
 *      Author: mstoilov
 */

#include "sigmadrive.h"
#include "dma.h"
#include "interruptmanager.h"


Dma::Dma(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config, uint32_t irq_priority)
	: DMAx_(DMAx)
	, stream_(stream)
	, channel_(channel)
	, dma_num_(-1)
{
	if (DMAx_ == DMA1) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
		dma_num_ = 1;
	} else if (DMAx_ == DMA2) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
		dma_num_ = 2;
	}
	LL_DMA_ConfigTransfer(DMAx_, stream_, config);
	LL_DMA_SetChannelSelection(DMAx_, stream_, channel_);
	EnableIrq(irq_priority);
}

Dma::~Dma()
{
	DisableIrq();
}

void Dma::ConfigAddresses(uint32_t srcAddress, uint32_t dstAddress, uint32_t direction)
{
	LL_DMA_ConfigAddresses(DMAx_, stream_, srcAddress, dstAddress, direction);
}

void Dma::IrqHandlerStream0()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC0(DMAx)) {
		LL_DMA_ClearFlag_TC0(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME0(DMAx)) {
		LL_DMA_ClearFlag_DME0(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE0(DMAx)) {
		LL_DMA_ClearFlag_FE0(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT0(DMAx)) {
		LL_DMA_ClearFlag_HT0(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE0(DMAx)) {
		LL_DMA_ClearFlag_TE0(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream1()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC1(DMAx)) {
		LL_DMA_ClearFlag_TC1(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME1(DMAx)) {
		LL_DMA_ClearFlag_DME1(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE1(DMAx)) {
		LL_DMA_ClearFlag_FE1(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT1(DMAx)) {
		LL_DMA_ClearFlag_HT1(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE1(DMAx)) {
		LL_DMA_ClearFlag_TE1(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream2()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC2(DMAx)) {
		LL_DMA_ClearFlag_TC2(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME2(DMAx)) {
		LL_DMA_ClearFlag_DME2(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE2(DMAx)) {
		LL_DMA_ClearFlag_FE2(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT2(DMAx)) {
		LL_DMA_ClearFlag_HT2(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE2(DMAx)) {
		LL_DMA_ClearFlag_TE2(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream3()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC3(DMAx)) {
		LL_DMA_ClearFlag_TC3(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME3(DMAx)) {
		LL_DMA_ClearFlag_DME3(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE3(DMAx)) {
		LL_DMA_ClearFlag_FE3(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT3(DMAx)) {
		LL_DMA_ClearFlag_HT3(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE3(DMAx)) {
		LL_DMA_ClearFlag_TE3(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream4()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC4(DMAx)) {
		LL_DMA_ClearFlag_TC4(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME4(DMAx)) {
		LL_DMA_ClearFlag_DME4(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE4(DMAx)) {
		LL_DMA_ClearFlag_FE4(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT4(DMAx)) {
		LL_DMA_ClearFlag_HT4(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE4(DMAx)) {
		LL_DMA_ClearFlag_TE4(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream5()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC5(DMAx)) {
		LL_DMA_ClearFlag_TC5(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME5(DMAx)) {
		LL_DMA_ClearFlag_DME5(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE5(DMAx)) {
		LL_DMA_ClearFlag_FE5(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT5(DMAx)) {
		LL_DMA_ClearFlag_HT5(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE5(DMAx)) {
		LL_DMA_ClearFlag_TE5(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream6()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC6(DMAx)) {
		LL_DMA_ClearFlag_TC6(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME6(DMAx)) {
		LL_DMA_ClearFlag_DME6(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE6(DMAx)) {
		LL_DMA_ClearFlag_FE6(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT6(DMAx)) {
		LL_DMA_ClearFlag_HT6(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE6(DMAx)) {
		LL_DMA_ClearFlag_TE6(DMAx);
		IrqHandlerTE();
	}
}

void Dma::IrqHandlerStream7()
{
	DMA_TypeDef* DMAx = GetDevice();
	if (LL_DMA_IsActiveFlag_TC7(DMAx)) {
		LL_DMA_ClearFlag_TC7(DMAx);
		IrqHandlerTC();
	}
	if (LL_DMA_IsActiveFlag_DME7(DMAx)) {
		LL_DMA_ClearFlag_DME7(DMAx);
		IrqHandlerDME();
	}
	if (LL_DMA_IsActiveFlag_FE7(DMAx)) {
		LL_DMA_ClearFlag_FE7(DMAx);
		IrqHandlerFE();
	}
	if (LL_DMA_IsActiveFlag_HT7(DMAx)) {
		LL_DMA_ClearFlag_HT7(DMAx);
		IrqHandlerHT();
	}
	if (LL_DMA_IsActiveFlag_TE7(DMAx)) {
		LL_DMA_ClearFlag_TE7(DMAx);
		IrqHandlerTE();
	}
}


void Dma::EnableIrq(uint32_t priority)
{
	uint32_t stream = GetStream();
	if (dma_num_ == 1 && stream == 0) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream0_IRQn, priority, [=](void){IrqHandlerStream0();});
	} else if (dma_num_ == 1 && stream == 1) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream1_IRQn, priority, [=](void){IrqHandlerStream1();});
	} else if (dma_num_ == 1 && stream == 2) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream2_IRQn, priority, [=](void){IrqHandlerStream2();});
	} else if (dma_num_ == 1 && stream == 3) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream3_IRQn, priority, [=](void){IrqHandlerStream3();});
	} else if (dma_num_ == 1 && stream == 4) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream4_IRQn, priority, [=](void){IrqHandlerStream4();});
	} else if (dma_num_ == 1 && stream == 5) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream5_IRQn, priority, [=](void){IrqHandlerStream5();});
	} else if (dma_num_ == 1 && stream == 6) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream6_IRQn, priority, [=](void){IrqHandlerStream6();});
	} else if (dma_num_ == 1 && stream == 7) {
		InterruptManager::instance().Callback_EnableIRQ(DMA1_Stream7_IRQn, priority, [=](void){IrqHandlerStream7();});
	} else if (dma_num_ == 2 && stream == 0) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream0_IRQn, priority, [=](void){IrqHandlerStream0();});
	} else if (dma_num_ == 2 && stream == 1) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream1_IRQn, priority, [=](void){IrqHandlerStream1();});
	} else if (dma_num_ == 2 && stream == 2) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream2_IRQn, priority, [=](void){IrqHandlerStream2();});
	} else if (dma_num_ == 2 && stream == 3) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream3_IRQn, priority, [=](void){IrqHandlerStream3();});
	} else if (dma_num_ == 2 && stream == 4) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream4_IRQn, priority, [=](void){IrqHandlerStream4();});
	} else if (dma_num_ == 2 && stream == 5) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream5_IRQn, priority, [=](void){IrqHandlerStream5();});
	} else if (dma_num_ == 2 && stream == 6) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream6_IRQn, priority, [=](void){IrqHandlerStream6();});
	} else if (dma_num_ == 2 && stream == 7) {
		InterruptManager::instance().Callback_EnableIRQ(DMA2_Stream7_IRQn, priority, [=](void){IrqHandlerStream7();});
	}
}

void Dma::DisableIrq()
{
	uint32_t stream = GetStream();

	if (dma_num_ == 1 && stream == 0) {
		NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream0_IRQn);
	} else if (dma_num_ == 1 && stream == 1) {
		NVIC_SetPriority(DMA1_Stream1_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	} else if (dma_num_ == 1 && stream == 2) {
		NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream2_IRQn);
	} else if (dma_num_ == 1 && stream == 3) {
		NVIC_SetPriority(DMA1_Stream3_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	} else if (dma_num_ == 1 && stream == 4) {
		NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream4_IRQn);
	} else if (dma_num_ == 1 && stream == 5) {
		NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	} else if (dma_num_ == 1 && stream == 6) {
		NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream6_IRQn);
	} else if (dma_num_ == 1 && stream == 7) {
		NVIC_SetPriority(DMA1_Stream7_IRQn, 0);
		NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	} else if (dma_num_ == 2 && stream == 0) {
		NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	} else if (dma_num_ == 2 && stream == 1) {
		NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	} else if (dma_num_ == 2 && stream == 2) {
		NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	} else if (dma_num_ == 2 && stream == 3) {
		NVIC_SetPriority(DMA2_Stream3_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream3_IRQn);
	} else if (dma_num_ == 2 && stream == 4) {
		NVIC_SetPriority(DMA2_Stream4_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream4_IRQn);
	} else if (dma_num_ == 2 && stream == 5) {
		NVIC_SetPriority(DMA2_Stream5_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream5_IRQn);
	} else if (dma_num_ == 2 && stream == 6) {
		NVIC_SetPriority(DMA2_Stream6_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream6_IRQn);
	} else if (dma_num_ == 2 && stream == 7) {
		NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
		NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	}
}
