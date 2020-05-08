#ifndef _ADCDATA_H_
#define _ADCDATA_H_

#include "adc.h"

extern Adc adc1;
extern Adc adc2;
extern Adc adc3;


/** AdcData class provides some level of abstraction over the ADC
 *
 * The individual Adc objects  * are responsible for reading data from physical ADC devices.
 * By introducing the AdcData  * we can abstract the individual ADC devices, which may vary
 * for different boards and configurations.
 *
 * This class returns the ADC data without any conversion or calculations. The only conversion
 * applied to the data is the size of the integral type and converted to signed value.
 *
 * One reason we might want to use signed values is that the ADC outputs are normally biased,
 * so if we try to remove the bias from the output we might get data varying around 0 and go
 * slightly negative.
 */
class AdcData {
public:
    static void ReadPhaseCurrent(int32_t *data, size_t size);
    static int32_t ReadBusVoltage();
};


#endif /* _ADCDATA_H_*/
