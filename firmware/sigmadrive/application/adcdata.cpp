#include "adcdata.h"


/** Read the phase currents
 *
 * @param data the output buffer where the phase current readings will be stored. The buffer
 * should be large enough to accommodate the 3 phases.
 * @param size the size of the buffer, this should be set to 3
 */
void AdcData::ReadPhaseCurrent(int32_t *data, size_t size)
{
    if (size > 0) {
        data[0] = (int32_t) adc1.InjReadConversionData(LL_ADC_INJ_RANK_1);
    }
    if (size > 1) {
        data[1] = (int32_t) adc1.InjReadConversionData(LL_ADC_INJ_RANK_2);
//        data[1] = (int32_t) adc2.InjReadConversionData(LL_ADC_INJ_RANK_1);
    }
    if (size > 2) {
        data[2] = (int32_t) adc1.InjReadConversionData(LL_ADC_INJ_RANK_3);
//        data[2] = (int32_t) adc3.InjReadConversionData(LL_ADC_INJ_RANK_1);
    }
}


/** Read the Vbus voltage ADC data
 *
 * @return Vbus ADC data
 */
int32_t AdcData::ReadBusVoltage()
{
    return (int32_t) adc1.InjReadConversionData(LL_ADC_INJ_RANK_4);
//    return (int32_t) adc1.RegReadConversionData(13);
}
