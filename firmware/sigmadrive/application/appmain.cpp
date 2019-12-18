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

#include "uartrpcserver.h"
#include "adc.h"
#include "uart.h"
#include "spimaster.h"
#include "drv8323.h"
#include "exti.h"
#include "pwm_generator.h"
#include "quadrature_encoder.h"
#include "ring.h"
#include "cdc_iface.h"
#include "motordrive.h"
#include "motorctrl_complexfoc.h"
#include "property.h"
#include "minasa4encoder.h"
#include "hrtimer.h"

#define MOTOR_POLE_PAIRS 7

UartRpcServer rpc_server;
osThreadId_t commandTaskHandle;
Adc adc1;
Adc adc2;
Adc adc3;
Uart uart1;
SPIMaster spi3;
PwmGenerator tim1;
CdcIface usb_cdc;
QuadratureEncoder tim4(0x2000);
MinasA4Encoder ma4_abs_encoder;
Drv8323 drv1(spi3, GPIOC, GPIO_PIN_13);
Drv8323 drv2(spi3, GPIOC, GPIO_PIN_14);
Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});
MotorDrive servo(&ma4_abs_encoder, &tim1, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorCtrlComplexFOC cfoc(&servo);
HRTimer hrtimer(SYSTEM_CORE_CLOCK/2);

bool debug_encoder = false;
std::string use_encoder = "quadrature";

void SetEncoder()
{
	if (use_encoder == "minas") {
		servo.SetEncoder(&ma4_abs_encoder);
		ma4_abs_encoder.Detect();
		servo.config_.reset_voltage_ = 3.4;
		servo.config_.spin_voltage_ = 3.4;
		servo.config_.pole_pairs = 4;
		cfoc.config_.spin_voltage_ = 3.4;
	} else if (use_encoder == "minas6") {
		servo.SetEncoder(&ma4_abs_encoder);
		ma4_abs_encoder.Detect();
		servo.config_.reset_voltage_ = 3.4;
		servo.config_.spin_voltage_ = 3.4;
		servo.config_.pole_pairs = 5;
		cfoc.config_.spin_voltage_ = 3.4;
	} else if (use_encoder == "quadrature") {
		servo.SetEncoder(&tim4);
		servo.config_.reset_voltage_ = 0.4;
		servo.config_.spin_voltage_ = 0.4;
		servo.config_.pole_pairs = 7;
		cfoc.config_.spin_voltage_ = 0.4;
	}
}


rexjson::property props =
		rexjson::property_map {
			{"clock_hz", rexjson::property(&SystemCoreClock, rexjson::property_access::readonly)},
			{"cfoc", rexjson::property({cfoc.props_})},
			{"use_encoder", rexjson::property(
					&use_encoder,
					rexjson::property_access::readwrite,
					[](const rexjson::value& v){if (v.get_str() != "minas" && v.get_str() != "minas6" && v.get_str() != "quadrature") throw std::range_error("Invalid value");},
					[&](void*)->void {
						SetEncoder();
					})},
			{"debug_encoder", &debug_encoder},
		};
rexjson::property* g_properties = &props;


extern "C"
void SD_TIM1_IRQHandler(TIM_HandleTypeDef* htim)
{
	TIM_TypeDef* TIMx = htim->Instance;

	if (LL_TIM_IsActiveFlag_UPDATE(TIMx)) {
		LL_TIM_ClearFlag_UPDATE(TIMx);
	}
}

extern "C"
void SD_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
	ADC_TypeDef* ADCx = hadc->Instance;

	if (LL_ADC_IsActiveFlag_JEOS(ADCx) && LL_ADC_IsEnabledIT_JEOS(ADCx)) {
		LL_ADC_ClearFlag_JEOS(ADCx);
		if (hadc == &hadc1) {
			servo.IrqUpdateCallback();
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
	hrtimer.Attach(&htim5);
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
	drv1.SetCSAGain(Drv8323::CSA_GAIN_10VV);
	drv1.SetOCPSenseLevel(Drv8323::SEN_LVL_100V);

	fprintf(stdout, "\n\nmain_task 1\n");
	fprintf(stdout, "DRV1: \n");
	drv1.DumpRegs();
#if 1
	g_properties->enumerate_children("props", [](const std::string& path, rexjson::property& prop)->void{std::cout << path << " : " << prop.get_prop().to_string() << std::endl;});
#endif
	HAL_Delay(50);


	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "CommandTask";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 12000;
	commandTaskHandle = osThreadNew(RunCommandTask, NULL, &task_attributes);

	tim4.Start();
	SetEncoder();
	servo.Attach();

	rpc_server.add("minas.resetF", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeF, "uint32_t MinasA4Encoder::ResetErrorCodeF()"));
	rpc_server.add("minas.resetB", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeB, "uint32_t MinasA4Encoder::ResetErrorCodeB()"));
	rpc_server.add("minas.resetE", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeE, "uint32_t MinasA4Encoder::ResetErrorCodeE()"));
	rpc_server.add("minas.reset9", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCode9, "uint32_t MinasA4Encoder::ResetErrorCode9()"));
	rpc_server.add("minas.reset_position", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetPosition, "void MinasA4Encoder::ResetPosition()"));

	ma4_abs_encoder.Attach(&huart3);
	uint32_t old_counter = 0, new_counter = 0;

	for (size_t i = 0; ; i++) {
		vTaskDelay(50 / portTICK_RATE_MS);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

		if (debug_encoder) {
			if (use_encoder == "quadrature") {
				new_counter = tim4.GetPosition();
				if (new_counter != old_counter) {
					fprintf(stderr, "Counter: %lu\n", new_counter);
					old_counter = new_counter;
				}
			} else {
				ma4_abs_encoder.Update(osThreadGetId());
				ma4_abs_encoder.WaitForUpdate();
				new_counter = ma4_abs_encoder.GetCounter();
				if (new_counter != old_counter || ma4_abs_encoder.status_) {
					fprintf(stderr, "Minas Enc(0x%x): %7.2f, Cnt: %7lu, Pos: %7lu, Rev: %7lu, Status: %2u (OS: %2u, FS: %2u, CE: %2u, OF: %2u, ME: %2u, SYD: %2u, BA: %2u ) (Time: %8lu uSec)\n",
							(int)ma4_abs_encoder.GetDeviceID(),
							ma4_abs_encoder.GetMechanicalPosition(new_counter) / M_PI * 180.0f,
							new_counter,
							ma4_abs_encoder.GetPosition(),
							ma4_abs_encoder.GetRevolutions(),
							ma4_abs_encoder.status_,
							ma4_abs_encoder.almc_.overspeed_,
							ma4_abs_encoder.almc_.full_abs_status_,
							ma4_abs_encoder.almc_.count_error_,
							ma4_abs_encoder.almc_.counter_overflow_,
							ma4_abs_encoder.almc_.multiple_revolution_error_,
							ma4_abs_encoder.almc_.system_down_,
							ma4_abs_encoder.almc_.battery_alarm_,
							ma4_abs_encoder.signal_time_ms_);
					old_counter = new_counter;
				}
			}
		}
		vTaskDelay(50 / portTICK_RATE_MS);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}

	return 0;
}
