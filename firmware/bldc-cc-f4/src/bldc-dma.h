#ifndef _BLDC_DMA_H_
#define _BLDC_DMA_H_

#include "bldc-include.h"


#ifdef __cplusplus
extern "C" {
#endif

void bldc_dma_init(DMA_TypeDef* DMAx, uint32_t stream, uint32_t channel, uint32_t config);


#ifdef __cplusplus
}
#endif

#endif
