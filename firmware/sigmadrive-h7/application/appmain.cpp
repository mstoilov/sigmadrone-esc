/*
 * Posible power supply:
 * JUNTEK DPH8920-RF
 */
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
#include "stm32f745xx.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "stm32f7xx_ll_dma.h"

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
#include "ring.h"
#include "cdc_iface.h"
#include "motordrive.h"
#include "motorctrl_foc.h"
#include "rexjson/rexjsonproperty.h"
#include "minasa4encoder.h"
#include "encoder_minas.h"
#include "dumbencoder.h"
#include "hrtimer.h"
#include "flashmemory.h"
#include "blinkled.h"

__attribute__((__section__(".flash_config"))) char flashregion[128 * 1024];

const unsigned boot_delay = 500;    /**< Wait for the encoder to boot up, before attempting to initialize */
FlashMemory flash_config(flashregion, sizeof(flashregion), FLASH_SECTOR_7, 1);
UartRpcServer rpc_server;
osThreadId_t commandTaskHandle;
Adc adc1;
Adc adc2;
Adc adc3;
Uart uart1;
Uart uart8;
Uart uart4;
Uart& rpc_uart = uart8;
SPIMaster spi2;
PwmGenerator tim1;
PwmGenerator tim8;
CdcIface usb_cdc;
// QuadratureEncoder tim4(0x2000);
// Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});
DumbEncoder dumb_encoder;
EncoderMinas ma4_abs_encoder1;
EncoderMinas ma4_abs_encoder2;
Drv8323 drv1(spi2, GPIOC, GPIO_PIN_13, GPIOE, GPIO_PIN_15);
Drv8323 drv2(spi2, GPIOC, GPIO_PIN_14, GPIOB, GPIO_PIN_2);
MotorDrive motor_drive1(1, &drv1, &adc1, &adc1, &ma4_abs_encoder1, &tim1, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorDrive motor_drive2(2, &drv2, &adc2, &adc1, &ma4_abs_encoder2, &tim8, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorCtrlFOC foc1(&motor_drive1, "axis1", &htim2);
MotorCtrlFOC foc2(&motor_drive2, "axis2", &htim5);

HRTimer hrtimer(SYSTEM_CORE_CLOCK/2, 0xFFFF);


rexjson::property g_props =
		rexjson::property_map {
			{"clock_hz", rexjson::property(&SystemCoreClock, rexjson::property_get<decltype(SystemCoreClock)>)},
			{"axis1", rexjson::property({foc1.GetPropertyMap()})},
			{"axis2", rexjson::property({foc2.GetPropertyMap()})},
		};
rexjson::property* g_properties = &g_props;


rexjson::property g_configprops =
		rexjson::property_map {
			{"axis1", rexjson::property({foc1.GetConfigPropertyMap()})},
			{"axis2", rexjson::property({foc2.GetConfigPropertyMap()})},
		};
rexjson::property* g_config_properties = &g_configprops;



extern "C"
void RunRpcTask(void *argument)
{
	std::string id;
	for (;;) {
		try {
			std::string reqstr = usb_cdc.GetLine();
			id = rexjson::read(reqstr)["id"].to_string();
			rexjson::value res = rpc_server.call(reqstr);
			std::string response = res.write(false, true, 0, 9);
			response += "\r\n";
			usb_cdc.Transmit(response);

			/*
			 * Log the Request/Response
			 */
			reqstr.erase(reqstr.find_last_not_of(" \n\r\t")+1);
			response.erase(response.find_last_not_of(" \n\r\t")+1);
			std::cerr << reqstr << "\r\n";
			std::cerr << response << "\r\n";


		} catch (std::exception& e) {
			rexjson::object ret;
			rexjson::object errobj;
			errobj["message"] = e.what();
			errobj["code"] = rexjson::RPC_MISC_ERROR;
			ret["id"] = id;
			ret["error"] = errobj;
			usb_cdc.Transmit(rexjson::value(ret).write(false, true) + "\r\n");
		}
	}
}

extern "C"
void SD_TIM2_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (!foc1.PulseCallback())
		__HAL_TIM_DISABLE(htim);
}

extern "C"
void SD_TIM5_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (!foc2.PulseCallback())
		__HAL_TIM_DISABLE(htim);
}

extern "C"
void SD_TIM1_IRQHandler(TIM_HandleTypeDef* htim)
{
	TIM_TypeDef* TIMx = htim->Instance;

	if (LL_TIM_IsActiveFlag_UPDATE(TIMx)) {
		LL_TIM_ClearFlag_UPDATE(TIMx);
	}
}

//#define NO_AXIS2_UPDATE

/** SD_ADC_IRQHandler is the interrupt handler that drives the motor.
 *
 * We use center aligned PWM. When the counter is going up the
 * current through the windings should be zero at that point
 * we update the bias. When the counter is going down we update
 * the current.
 * ```
 *   |       _____|_____       |
 *   |      |     |     |      |
 *   |______|     |     |______|
 *   |            |            |
 *   /\     up    /\     down
 *    \            \
 *     \            \___ update current
 *      \
 *       \___update bias
 *
 * ```
 */

extern "C"
void SD_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
	ADC_TypeDef* ADCx = hadc->Instance;

	if (LL_ADC_IsActiveFlag_JEOS(ADCx) && LL_ADC_IsEnabledIT_JEOS(ADCx)) {
		LL_ADC_ClearFlag_JEOS(ADCx);
		if (hadc == &hadc1) {
			if (tim1.GetCounterDirection()) {
#ifndef NO_AXIS2_UPDATE
				motor_drive2.t_begin_ = hrtimer.GetCounter();
#endif
				motor_drive1.UpdateBias();
#ifndef NO_AXIS2_UPDATE
				motor_drive2.UpdateCurrent();
#endif
			} else {
				motor_drive1.t_begin_ = hrtimer.GetCounter();
#ifndef NO_AXIS2_UPDATE
				motor_drive2.UpdateBias();
#endif
				motor_drive1.UpdateCurrent();
			}
		}
	}
}


extern "C"
void SD_DMA1_Stream1_IRQHandler(void)
{
	DMA_TypeDef* DMAx = DMA1;
	if (LL_DMA_IsActiveFlag_TC1(DMAx)) {
		LL_DMA_ClearFlag_TC1(DMAx);
		ma4_abs_encoder1.ReceiveCompleteCallback();
	}
	if (LL_DMA_IsActiveFlag_DME1(DMAx)) {
		LL_DMA_ClearFlag_DME1(DMAx);

	}
	if (LL_DMA_IsActiveFlag_FE1(DMAx)) {
		LL_DMA_ClearFlag_FE1(DMAx);

	}
	if (LL_DMA_IsActiveFlag_HT1(DMAx)) {
		LL_DMA_ClearFlag_HT1(DMAx);

	}
	if (LL_DMA_IsActiveFlag_TE1(DMAx)) {
		LL_DMA_ClearFlag_TE1(DMAx);
	}
}


extern "C"
void SD_DMA1_Stream5_IRQHandler(void)
{
	DMA_TypeDef* DMAx = DMA1;
	if (LL_DMA_IsActiveFlag_TC5(DMAx)) {
		LL_DMA_ClearFlag_TC5(DMAx);
		ma4_abs_encoder2.ReceiveCompleteCallback();
	}
	if (LL_DMA_IsActiveFlag_DME5(DMAx)) {
		LL_DMA_ClearFlag_DME5(DMAx);

	}
	if (LL_DMA_IsActiveFlag_FE5(DMAx)) {
		LL_DMA_ClearFlag_FE5(DMAx);

	}
	if (LL_DMA_IsActiveFlag_HT5(DMAx)) {
		LL_DMA_ClearFlag_HT5(DMAx);

	}
	if (LL_DMA_IsActiveFlag_TE5(DMAx)) {
		LL_DMA_ClearFlag_TE5(DMAx);
	}
}


void SaveConfig()
{
	std::string ret;
	rexjson::value props = g_config_properties->to_json();
	ret = props.write(true, true, 4, 9);
	std::cout << ret << std::endl;
	flash_config.erase();
	flash_config.program(ret.c_str(), ret.size() + 1);
	if (ret != flashregion)
		throw std::runtime_error("Failed to save configuration!");
}

void LoadConfig()
{
	std::string configuration(flashregion);

	try {
		rexjson::value props = rexjson::read(configuration);
		g_config_properties->enumerate(*g_config_properties, "", [&](const std::string& path, rexjson::property& prop)->void {

			if (prop.access() & rexjson::property_access::access_write) {
				rexjson::object::const_iterator it = props.get_obj().find(path);
				if (it != props.get_obj().end()) {
					try {
						prop.set(it->second);
					} catch (std::exception& e) {
						std::cout << e.what() << ", Failed to set " << path << " : " << it->second.to_string() << "\r\n";
					}
				}
			}
		});
	} catch (std::exception& e) {
		throw std::runtime_error("Failed to read/apply configuration settings.");
	}
}
#if 0
static const char * dump = R"desc(
  1 : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  2 : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  3 : cccccccccccccccccccccccccccccccccccccccccccc
  4 : dddddddddddddddddddddddddddddddddddddddddddd
  5 : eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
  6 : ffffffffffffffffffffffffffffffffffffffffffff
  7 : gggggggggggggggggggggggggggggggggggggggggggg
  8 : hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
  9 : iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 10 : jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
 11 : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 12 : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
 13 : cccccccccccccccccccccccccccccccccccccccccccc
 14 : dddddddddddddddddddddddddddddddddddddddddddd
 15 : eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 16 : ffffffffffffffffffffffffffffffffffffffffffff
 17 : gggggggggggggggggggggggggggggggggggggggggggg
 18 : hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
 19 : iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 20 : jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
 21 : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 22 : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
 23 : cccccccccccccccccccccccccccccccccccccccccccc
 24 : dddddddddddddddddddddddddddddddddddddddddddd
 25 : eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 26 : ffffffffffffffffffffffffffffffffffffffffffff
 27 : gggggggggggggggggggggggggggggggggggggggggggg
 28 : hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
 29 : iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 30 : jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
 31 : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 32 : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
 33 : cccccccccccccccccccccccccccccccccccccccccccc
 34 : dddddddddddddddddddddddddddddddddddddddddddd
 35 : eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 36 : ffffffffffffffffffffffffffffffffffffffffffff
 37 : gggggggggggggggggggggggggggggggggggggggggggg
 38 : hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
 39 : iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 40 : jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
 41 : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 42 : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
 43 : cccccccccccccccccccccccccccccccccccccccccccc
 44 : dddddddddddddddddddddddddddddddddddddddddddd
 45 : eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
 46 : ffffffffffffffffffffffffffffffffffffffffffff
 47 : gggggggggggggggggggggggggggggggggggggggggggg
 48 : hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
 49 : iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
 50 : jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj


)desc";
#endif

std::string DumpTextDo()
{
	std::ostringstream oss;

	oss << "\r\n\r\n";
	for (size_t i = 0; i < 50; i++) {
		oss.width(5);
		oss << i;
		oss << " : " << std::string(20, 'a' + i % 10) << "\r\n";
	}

	HAL_UART_Transmit_DMA(&huart4, (uint8_t*)oss.str().c_str(), strlen(oss.str().c_str()));
//    HAL_UART_Transmit_DMA(uart1.huart_, (uint8_t*)dump, strlen(dump));

	osDelay(200);

	std::stringstream stream;
	stream << std::hex << huart4.Instance->ISR;
	std::string result( stream.str() );
	return result;
//    std::cout << oss.str();
}

std::string DumpText()
{
//    for (size_t i = 0; i < 5; i++)
		return DumpTextDo();
}

void AllGo()
{
	__disable_irq();
	foc1.Go();
	foc2.Go();
	__enable_irq();
}

void TrapezoidMoveXY(int64_t Xfin, int64_t Yfin, int64_t Vmax, int64_t Acc, int64_t Dec)
{
	int64_t Xin = foc1.GetTarget();
	int64_t Yin = foc2.GetTarget();
	std::vector<std::vector<std::vector<int64_t>>> pointsXY = CalculateTrapezoidPointsXY(Xin, Yin, Xfin, Yfin, Vmax, Acc, Dec, foc1.drive_->GetUpdateFrequency());
	const std::vector<std::vector<int64_t>>& pointsX = pointsXY[0];
	const std::vector<std::vector<int64_t>>& pointsY = pointsXY[1];
	for (const auto& v : pointsX) {
		foc1.PushStreamPointV(v);
	}
	for (const auto& v : pointsY) {
		foc2.PushStreamPointV(v);
	}
	foc1.SetTarget(Xfin);
	foc2.SetTarget(Yfin);
}

void GoTrapezoidMoveXY(int64_t Xfin, int64_t Yfin, int64_t Vmax, int64_t Acc, int64_t Dec)
{
	TrapezoidMoveXY(Xfin, Yfin, Vmax, Acc, Dec);
	AllGo();
}

void TrapezoidMovePolar(int64_t D, float Angle, int64_t Vmax, int64_t Acc, int64_t Dec)
{
	int64_t Xin = foc1.GetTarget();
	int64_t Yin = foc2.GetTarget();
	int64_t Xfin = Xin + D * std::cos(Angle);
	int64_t Yfin = Yin + D * std::sin(Angle);

	std::vector<std::vector<std::vector<int64_t>>> pointsXY = CalculateTrapezoidPointsXY(Xin, Yin, Xfin, Yfin, Vmax, Acc, Dec, foc1.drive_->GetUpdateFrequency());
	const std::vector<std::vector<int64_t>>& pointsX = pointsXY[0];
	const std::vector<std::vector<int64_t>>& pointsY = pointsXY[1];
	for (const auto& v : pointsX) {
		foc1.PushStreamPointV(v);
	}
	for (const auto& v : pointsY) {
		foc2.PushStreamPointV(v);
	}
	foc1.SetTarget(Xfin);
	foc2.SetTarget(Yfin);
}

void GoTrapezoidMovePolar(int64_t D, float Angle, int64_t Vmax, int64_t Acc, int64_t Dec)
{
	TrapezoidMovePolar(D, Angle, Vmax, Acc, Dec);
	AllGo();
}

void AllMoveToPosition(uint64_t pos_a1, uint64_t pos_a2)
{
	foc1.MoveToPosition(pos_a1);
	foc2.MoveToPosition(pos_a2);
}

void AllMoveRelative(int64_t offset_a1, int64_t offset_a2)
{
	foc1.MoveRelative(offset_a1);
	foc2.MoveRelative(offset_a2);
}

void AllModeClp()
{
	foc1.ModeClosedLoopPositionTrajectory();
	foc2.ModeClosedLoopPositionTrajectory();
}

void AllModeClv()
{
	foc1.ModeClosedLoopVelocity();
	foc2.ModeClosedLoopVelocity();
}

void AllModeStop()
{
	foc1.Stop();
	foc2.Stop();
}


void RegisterRpcMethods()
{
	rpc_server.add("dumptext", rexjson::make_rpc_wrapper(DumpText, "void DumpText()"));
	rpc_server.add("LoadConfig", rexjson::make_rpc_wrapper(LoadConfig, "void LoadConfig()"));
	rpc_server.add("SaveConfig", rexjson::make_rpc_wrapper(SaveConfig, "void SaveConfig()"));

	rpc_server.add("drv1.ChipSelect", rexjson::make_rpc_wrapper(&drv1, &Drv8323::ChipSelect, "void Drv8323::ChipSelect()"));
	rpc_server.add("drv1.EnableVREFDiv", rexjson::make_rpc_wrapper(&drv1, &Drv8323::EnableVREFDiv, "void Drv8323::EnableVREFDiv()"));
	rpc_server.add("drv1.DisableVREFDiv", rexjson::make_rpc_wrapper(&drv1, &Drv8323::DisableVREFDiv, "void Drv8323::DisableVREFDiv()"));
	rpc_server.add("drv1.DumpRegs", rexjson::make_rpc_wrapper(&drv1, &Drv8323::DumpRegs, "void Drv8323::DumpRegs()"));
	rpc_server.add("drv1.WriteReg", rexjson::make_rpc_wrapper(&drv1, &Drv8323::WriteReg, "void Drv8323::WriteReg()"));
	rpc_server.add("drv1.ReadReg", rexjson::make_rpc_wrapper(&drv1, &Drv8323::ReadReg, "void Drv8323::ReadReg()"));
	rpc_server.add("drv1.EnableDriver", rexjson::make_rpc_wrapper(&drv1, &Drv8323::EnableDriver, "void Drv8323::EnableDriver()"));
	rpc_server.add("drv1.DisableDriver", rexjson::make_rpc_wrapper(&drv1, &Drv8323::DisableDriver, "void Drv8323::DisableDriver()"));
	rpc_server.add("drv1.InitializeDefaults", rexjson::make_rpc_wrapper(&drv1, &Drv8323::InitializeDefaults, "void Drv8323::InitializeDefaults()"));

	rpc_server.add("drv2.ChipSelect", rexjson::make_rpc_wrapper(&drv2, &Drv8323::ChipSelect, "void Drv8323::ChipSelect()"));
	rpc_server.add("drv2.EnableVREFDiv", rexjson::make_rpc_wrapper(&drv2, &Drv8323::EnableVREFDiv, "void Drv8323::EnableVREFDiv()"));
	rpc_server.add("drv2.DisableVREFDiv", rexjson::make_rpc_wrapper(&drv2, &Drv8323::DisableVREFDiv, "void Drv8323::DisableVREFDiv()"));
	rpc_server.add("drv2.DumpRegs", rexjson::make_rpc_wrapper(&drv2, &Drv8323::DumpRegs, "void Drv8323::DumpRegs()"));
	rpc_server.add("drv2.WriteReg", rexjson::make_rpc_wrapper(&drv2, &Drv8323::WriteReg, "void Drv8323::WriteReg()"));
	rpc_server.add("drv2.ReadReg", rexjson::make_rpc_wrapper(&drv2, &Drv8323::ReadReg, "void Drv8323::ReadReg()"));
	rpc_server.add("drv2.EnableDriver", rexjson::make_rpc_wrapper(&drv2, &Drv8323::EnableDriver, "void Drv8323::EnableDriver()"));
	rpc_server.add("drv2.DisableDriver", rexjson::make_rpc_wrapper(&drv2, &Drv8323::DisableDriver, "void Drv8323::DisableDriver()"));
	rpc_server.add("drv2.InitializeDefaults", rexjson::make_rpc_wrapper(&drv2, &Drv8323::InitializeDefaults, "void Drv8323::InitializeDefaults()"));

	rpc_server.add("modeclv", rexjson::make_rpc_wrapper(AllModeClv, "void AllModeClv()"));
	rpc_server.add("modeclp", rexjson::make_rpc_wrapper(AllModeClp, "void AllModeClp()"));
	rpc_server.add("stop", rexjson::make_rpc_wrapper(AllModeStop, "void AllModeStop()"));
	rpc_server.add("go", rexjson::make_rpc_wrapper(AllGo, "void AllGo()"));
	rpc_server.add("mvxy", rexjson::make_rpc_wrapper(TrapezoidMoveXY, "void TrapezoidMoveXY(int64_t Xfin, int64_t Yfin, int64_t Vmax, int64_t Acc, int64_t Dec)"));
	rpc_server.add("gomvxy", rexjson::make_rpc_wrapper(GoTrapezoidMoveXY, "void GoTrapezoidMoveXY(int64_t Xfin, int64_t Yfin, int64_t Vmax, int64_t Acc, int64_t Dec)"));
	rpc_server.add("mvpolar", rexjson::make_rpc_wrapper(TrapezoidMovePolar, "void TrapezoidMovePolar(int64_t D, float Angle, int64_t Vmax, int64_t Acc, int64_t Dec)"));
	rpc_server.add("gomvpolar", rexjson::make_rpc_wrapper(GoTrapezoidMovePolar, "void GoTrapezoidMovePolar(int64_t D, float Angle, int64_t Vmax, int64_t Acc, int64_t Dec)"));
}


void StartRpcThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "RpcTask";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 12000;
	commandTaskHandle = osThreadNew(RunRpcTask, NULL, &task_attributes);
}


void StartCommandThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "CommandTask";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 16000;
	commandTaskHandle = osThreadNew(RunCommandTask, NULL, &task_attributes);
}

void DisplayPropertiesInfo()
{
	g_properties->enumerate(*g_properties, "", [](const std::string& path, rexjson::property& prop)->void{std::cout << path << " : " << prop.get().to_string() << "\r\n";});
}

void DisplayDrvRegs()
{
	fprintf(stdout, "DRV1: \r\n");
	drv1.DumpRegs();

	fprintf(stdout, "\r\n\r\nDRV2: \r\n");
	drv2.DumpRegs();

}

void EnterMainLoop()
{
	BlinkLed warnblinker(LED_WARN_GPIO_Port, LED_WARN_Pin, 150, 150);

	for (;;) {
		/*
		 * Blink the WARN LED
		 */
		warnblinker.Blink();

	}
}

extern "C"
int application_main()
{
	*_impure_ptr = *_impure_data_ptr;

//	Exti exti_usr_button(USER_BTN_Pin, []()->void{HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);});

	/*
	 * Attach the HAL handles to the
	 * C++ wrapper objects. At this point the HAL handles
	 * should be fully initialized.
	 */
	osDelay(boot_delay);
	hrtimer.Attach(&htim12);
	ma4_abs_encoder2.Attach(&huart2, DMA1, LL_DMA_STREAM_5, LL_DMA_STREAM_6);
	ma4_abs_encoder1.Attach(&huart3, DMA1, LL_DMA_STREAM_1, LL_DMA_STREAM_3);

	/*
	 * Set up RPC properties/methods for the encoders.
	 */
	g_props.insert("enc1", ma4_abs_encoder1.GetPropertyMap());
	ma4_abs_encoder1.RegisterRpcMethods("enc1.");

	g_props.insert("enc2", ma4_abs_encoder2.GetPropertyMap());
	ma4_abs_encoder2.RegisterRpcMethods("enc2.");

	adc1.Attach(&hadc1, 6, true);
	adc2.Attach(&hadc2, 3, false);
//    adc3.Attach(&hadc3, 1, false);
	uart1.Attach(&huart1);
	uart8.Attach(&huart8);
	uart4.Attach(&huart4);
	spi2.Attach(&hspi2);
	LL_TIM_SetCounter(TIM8, TIM1_PERIOD_CLOCKS - 1);
	LL_TIM_SetCounter(TIM1, 0);
	tim8.Attach(&htim8);
	tim1.Attach(&htim1);
	usb_cdc.Attach(&hUsbDeviceFS, true);
	drv1.EnableDriver();
	drv2.EnableDriver();
	osDelay(2);
	drv1.InitializeDefaults();
	drv2.InitializeDefaults();

	DisplayDrvRegs();
	DisplayPropertiesInfo();
	motor_drive1.Attach();
	motor_drive2.Attach();

	/*
	 * Start the motor timers.
	 */
	tim1.EnableCounter(true);

	/*
	 * Reconfigure pole_pairs for panasonic motors
	 */
	if (motor_drive1.GetEncoder()->GetResolutionBits() == 17) {
		motor_drive1.config_.pole_pairs_ = 4;
	} else if (motor_drive1.GetEncoder()->GetResolutionBits() == 23) {
		motor_drive1.config_.pole_pairs_ = 5;
	}

	/*
	 * Reconfigure pole_pairs for panasonic motors
	 */
	if (motor_drive2.GetEncoder()->GetResolutionBits() == 17) {
		motor_drive2.config_.pole_pairs_ = 4;
	} else if (motor_drive2.GetEncoder()->GetResolutionBits() == 23) {
		motor_drive2.config_.pole_pairs_ = 5;
	}

	RegisterRpcMethods();

	/*
	 * Start Helper Tasks
	 */
	StartCommandThread();

	/*
	 * Run the RPC thread
	 */
	StartRpcThread();

	/*
	 * We should never exit from the this method.
	 */
	EnterMainLoop();

	return 0;
}
