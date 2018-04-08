#include "bldc-dma.h"

void bldc_dma_init(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config)
{
	LL_DMA_ConfigTransfer(DMAx, stream, config);
	LL_DMA_SetChannelSelection(DMAx, stream, channel);
}
