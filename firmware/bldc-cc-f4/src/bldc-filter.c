#include <string.h>
#include "bldc-filter.h"

void bldc_filter_init(BldcFilter *filter, int32_t alpha)
{
	memset(filter, 0, sizeof(*filter));
	filter->alpha = alpha;
	filter->beta = 1000 - alpha;
}

void bldc_filter_reset(BldcFilter *filter, int32_t value)
{
	filter->value = value;
}

void bldc_filter_add_value(BldcFilter *filter, int32_t value)
{
	filter->value = (filter->alpha * filter->value + filter->beta * value) / 1000;
}

int32_t bldc_filter_get(BldcFilter *filter)
{
	return filter->value;
}
