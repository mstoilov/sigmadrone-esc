#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <type_traits>
#include <algorithm>
#include <iterator>
#include <assert.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "main.h"
#include "appmain.h"
#include "command_task.h"


#include "adc.h"
#include "uart.h"
#include "spimaster.h"
#include "drv8323.h"
#include "exti.h"
#include "pwm_generator.h"
#include "quadrature_encoder.h"
#include "servo_drive.h"
#include "ring.h"
#include "cdc_iface.h"
#include "property.h"

#define MOTOR_POLE_PAIRS 7

osThreadId commandTaskHandle;
Adc adc1;
Adc adc2;
Adc adc3;
Uart uart1;
SPIMaster spi3;
PwmGenerator tim1;
CdcIface usb_cdc;
QuadratureEncoder tim4(0x2000, MOTOR_POLE_PAIRS);
Drv8323 drv1(spi3, GPIOC, GPIO_PIN_13);
Drv8323 drv2(spi3, GPIOC, GPIO_PIN_14);
Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});
ServoDrive servo(&tim4, &tim1, SYSTEM_CORE_CLOCK / TIM1_PERIOD_CLOCKS / (TIM1_RCR + 1));
std::vector<IServoDrive*> g_motors{&servo};

rexjson::property props =
		rexjson::property_map {
			{"clock_hz", rexjson::property(&SystemCoreClock, rexjson::property_access::readonly)},
			{"servo", rexjson::property({servo.props_})},
		};
rexjson::property* g_properties = &props;


extern "C"
void SD_TIM1_IRQHandler(TIM_HandleTypeDef* htim)
{
	TIM_TypeDef* TIMx = htim->Instance;

	if (LL_TIM_IsActiveFlag_UPDATE(TIMx)) {
		LL_TIM_ClearFlag_UPDATE(TIMx);
//		if (TIMx == TIM1) {
//			fprintf(stderr, "TIM1_IRQHandler:\n");
//			servo.SignalThreadUpdate();
//		}
	}
//	servo.PeriodElapsedCallback();
}

extern "C"
void SD_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
	ADC_TypeDef* ADCx = hadc->Instance;

	if (LL_ADC_IsActiveFlag_JEOS(ADCx) && LL_ADC_IsEnabledIT_JEOS(ADCx)) {
		LL_ADC_ClearFlag_JEOS(ADCx);
		if (hadc == &hadc1) {
			adc1.InjectedConvCpltCallback();
//			fprintf(stderr, "SD_ADC_IRQHandler:\n");
			servo.SignalThreadUpdate();
		}
	}
	if (LL_ADC_IsActiveFlag_EOCS(ADCx) && LL_ADC_IsEnabledIT_EOCS(ADCx)) {
		LL_ADC_ClearFlag_EOCS(ADCx);
	}
	if (LL_ADC_IsActiveFlag_OVR(ADCx) && LL_ADC_IsEnabledIT_OVR(ADCx)) {
		LL_ADC_ClearFlag_OVR(ADCx);
	}
	if (LL_ADC_IsActiveFlag_AWD1(ADCx) && LL_ADC_IsEnabledIT_AWD1(ADCx)) {
		LL_ADC_ClearFlag_AWD1(ADCx);
	}

}


void AdcBiasSetup()
{
	drv1.EnableCalibration();
	for (size_t i = 0; i < Adc::ADC_HISTORY_SIZE; i++) {
		vTaskDelay(10 / portTICK_RATE_MS);
		adc1.InjectedSwTrig();
	}
	drv1.DisableCalibration();

	memset(adc1.bias_, 0, sizeof(adc1.bias_));
	for (size_t i = 0; i < Adc::ADC_HISTORY_SIZE; i++) {
		adc1.bias_[0] += adc1.injhistory_[i][0];
		adc1.bias_[1] += adc1.injhistory_[i][1];
		adc1.bias_[2] += adc1.injhistory_[i][2];
	}
	adc1.bias_[0] = adc1.bias_[0]/Adc::ADC_HISTORY_SIZE;
	adc1.bias_[1] = adc1.bias_[1]/Adc::ADC_HISTORY_SIZE;
	adc1.bias_[2] = adc1.bias_[2]/Adc::ADC_HISTORY_SIZE;
}

extern "C"
int application_main()
{
	*_impure_ptr = *_impure_data_ptr;

	Exti exti_usr_button(USER_BTN_Pin, []()->void{HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);});

	/*
	 * Attach the HAL handles to the
	 * C++ wrapper objects. At this point the HAL handles
	 * should be fully initialized.
	 */
	adc1.Attach(&hadc1);
	adc2.Attach(&hadc2);
	adc3.Attach(&hadc3);
	uart1.Attach(&huart1);
	spi3.Attach(&hspi3);
	tim4.Attach(&htim4);
	tim1.Attach(&htim1);
//	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
	usb_cdc.Attach(&hUsbDeviceFS);

	drv1.WriteReg(2, 0x0);
	drv1.WriteReg(3, 0x0);
	drv1.WriteReg(4, 0x0);
	drv1.WriteReg(5, 0x0);
	drv1.WriteReg(6, 0x0);

	drv1.SetIDriveP_HS(Drv8323::IDRIVEP_370mA);
	drv1.SetIDriveN_HS(Drv8323::IDRIVEN_1360mA);
	drv1.SetIDriveP_LS(Drv8323::IDRIVEP_370mA);
	drv1.SetIDriveN_LS(Drv8323::IDRIVEN_1360mA);
	drv1.SetTDrive(Drv8323::TDRIVE_4000ns);
	drv1.EnableCBC();
	drv1.DisableCPUV();
	drv1.EnableCPUV();
	drv1.DisableGDF();
	drv1.EnableGDF();
	drv1.EnableOTW();
	drv1.DisableOTW();
	drv1.SetPWMMode(Drv8323::PWM_MODE_6X);
	drv1.SetDeadTime(Drv8323::DEADTIME_100ns);
	drv1.SetOCPMode(Drv8323::OCP_RETRY_FAULT);
	drv1.SetOCPDeglitch(Drv8323::OCP_DEG_4us);
	drv1.SetVDSLevel(Drv8323::VDS_LVL_060V);
	drv1.EnableVREFDiv();
	drv1.SetCSAGain(Drv8323::CSA_GAIN_40VV);
	drv1.SetOCPSenseLevel(Drv8323::SEN_LVL_100V);

	fprintf(stdout, "\n\nmain_task 1\n");
	fprintf(stdout, "DRV1: \n");
	drv1.DumpRegs();
#if 1
	g_properties->enumerate_children("props", [](const std::string& path, rexjson::property& prop)->void{std::cout << path << " : " << prop.get_prop().to_string() << std::endl;});
#endif
	HAL_Delay(50);

	osThreadAttr_t commandTask_attributes;
	memset(&commandTask_attributes, 0, sizeof(commandTask_attributes));
	commandTask_attributes.name = "commandTask";
	commandTask_attributes.priority = (osPriority_t) osPriorityNormal;
	commandTask_attributes.stack_size = 16000;
	commandTaskHandle = osThreadNew(StartCommandTask, NULL, &commandTask_attributes);

	uint32_t old_counter = 0, new_counter = 0;


	servo.Attach();

	tim4.Start();

	for (;;) {
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

		new_counter = tim4.GetCounter();
		if (new_counter != old_counter) {
//			fprintf(stderr, "Counter: %lu\n", new_counter);
			old_counter = new_counter;
		}

		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}

	return 0;
}
