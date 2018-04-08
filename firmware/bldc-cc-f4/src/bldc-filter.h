#ifndef _BLDC_FILTER_H_
#define _BLDC_FILTER_H_

#include "bldc-include.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct BldcFilter_ {
	int32_t alpha;		/* in 1/1000 of a unit */
	int32_t beta;		/* in 1/1000 of a unit */
						/* alpha + beta = 1000 */
	int32_t value;		/* holds the value multiplied by 1000 */
} BldcFilter;


void bldc_filter_init(BldcFilter *filter, int32_t alpha);
void bldc_filter_reset(BldcFilter *filter, int32_t value);
void bldc_filter_add_value(BldcFilter *filter, int32_t value);
int32_t bldc_filter_get(BldcFilter *filter);

#ifdef __cplusplus
}
#endif

#endif
