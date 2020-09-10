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
#include "rexjsonrpc/property.h"
#include "minasa4encoder.h"
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
Uart uart2;
SPIMaster spi2;
PwmGenerator tim1;
PwmGenerator tim8;
CdcIface usb_cdc;
// QuadratureEncoder tim4(0x2000);
// Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});
DumbEncoder dumb_encoder;
MinasA4Encoder ma4_abs_encoder1;
MinasA4Encoder ma4_abs_encoder2;
Drv8323 drv1(spi2, GPIOC, GPIO_PIN_13, GPIOE, GPIO_PIN_15);
Drv8323 drv2(spi2, GPIOC, GPIO_PIN_14, GPIOB, GPIO_PIN_2);
MotorDrive motor_drive1(1, &drv1, &adc1, &adc1, &ma4_abs_encoder2, &tim1, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorDrive motor_drive2(2, &drv2, &adc2, &adc1, &ma4_abs_encoder1, &tim8, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorCtrlFOC foc1(&motor_drive1, "axis1");
MotorCtrlFOC foc2(&motor_drive2, "axis2");
HRTimer hrtimer(SYSTEM_CORE_CLOCK/2, 0xFFFF);


rexjson::property g_props =
        rexjson::property_map {
            {"clock_hz", rexjson::property(&SystemCoreClock, rexjson::property_access::readonly)},
            {"axis1", rexjson::property({foc1.GetPropertyMap()})},
            {"axis2", rexjson::property({foc2.GetPropertyMap()})},
        };
rexjson::property* g_properties = &g_props;


extern "C"
void RunRpcTask(void *argument)
{
    for (;;) {
        try {
            std::string req = uart2.GetLine();
            rexjson::value res = rpc_server.call(req);
            std::string response = res.write(false, false, 0, 9);
            response += "\n";
            uart2.Transmit(response);
//			usb_cdc.Transmit(req);
//			usb_cdc.Transmit(response);
        } catch (std::exception& e) {

        }
    }
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
        ma4_abs_encoder2.ReceiveCompleteCallback();
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
        ma4_abs_encoder1.ReceiveCompleteCallback();
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
    rexjson::value props = g_properties->to_json();
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
    rexjson::value props = rexjson::read(configuration);
    g_properties->enumerate_children("", [&](const std::string& path, rexjson::property& prop)->void {

        if (prop.access() & rexjson::property_access::writeonly) {
            rexjson::object::const_iterator it = props.get_obj().find(path);
            if (it != props.get_obj().end()) {
                try {
                    prop.set_prop(it->second);
                } catch (std::exception& e) {
                    std::cout << e.what() << ", Failed to set " << path << " : " << it->second.to_string() << std::endl;
                }
            }
        }
    });
}

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


void DumpTextDo()
{
    std::ostringstream oss;

    oss << "\r\n\r\n";
    for (size_t i = 0; i < 50; i++) {
        oss.width(5);
        oss << i;
        oss << " : " << std::string(20, 'a' + i % 10) << "\r\n";
    }

    HAL_UART_Transmit_DMA(uart1.huart_, (uint8_t*)oss.str().c_str(), strlen(oss.str().c_str()));
//    HAL_UART_Transmit_DMA(uart1.huart_, (uint8_t*)dump, strlen(dump));
    osDelay(1000);

//    std::cout << oss.str();
}

void DumpText()
{
    for (size_t i = 0; i < 5; i++)
        DumpTextDo();
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

void AllModeCltr()
{
    foc1.ModeClosedLoopTrajectory();
    foc2.ModeClosedLoopTrajectory();
}

void AllModeClv()
{
    foc1.ModeClosedLoopVelocity();
    foc2.ModeClosedLoopVelocity();
}

void AllModeStream()
{
    foc1.ModeClosedLoopStream();
    foc2.ModeClosedLoopStream();
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

    rpc_server.add("all.mvp", rexjson::make_rpc_wrapper(AllMoveToPosition, "AllMoveToPosition(uint64_t pos_a1, uint64_t pos_a2)"));
    rpc_server.add("all.mvr", rexjson::make_rpc_wrapper(AllMoveRelative, "void AllMoveRelative(int64_t offset_a1, int64_t offset_a2)"));
    rpc_server.add("all.modecltr", rexjson::make_rpc_wrapper(AllModeCltr, "void AllModeCltr()"));
    rpc_server.add("all.modeclv", rexjson::make_rpc_wrapper(AllModeClv, "void AllModeClv()"));
    rpc_server.add("all.modecls", rexjson::make_rpc_wrapper(AllModeStream, "void AllModeStream()"));
    rpc_server.add("all.stop", rexjson::make_rpc_wrapper(AllModeStop, "void AllModeStop()"));
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
    g_properties->enumerate_children("", [](const std::string& path, rexjson::property& prop)->void{std::cout << path << " : " << prop.get_prop().to_string() << std::endl;});
}

void DisplayDrvRegs()
{
    fprintf(stdout, "DRV1: \n");
    drv1.DumpRegs();

    fprintf(stdout, "\n\nDRV2: \n");
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
    ma4_abs_encoder1.Attach(&huart2, DMA1, LL_DMA_STREAM_5, LL_DMA_STREAM_6);
    ma4_abs_encoder2.Attach(&huart3, DMA1, LL_DMA_STREAM_1, LL_DMA_STREAM_3);

    /*
     * Set up RPC properties/methods for the encoders.
     */
    if (ma4_abs_encoder1.GetDeviceId() != 0) {
        g_props.insert("enc1", ma4_abs_encoder1.GetPropertyMap());
        ma4_abs_encoder1.RegisterRpcMethods("enc1.");
    }

    if (ma4_abs_encoder2.GetDeviceId() != 0) {
        g_props.insert("enc2", ma4_abs_encoder2.GetPropertyMap());
        ma4_abs_encoder1.RegisterRpcMethods("enc2.");
    }

    adc1.Attach(&hadc1, 6, true);
    adc2.Attach(&hadc2, 3, false);
//    adc3.Attach(&hadc3, 1, false);
    uart1.Attach(&huart1);
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
    if (motor_drive1.encoder_->GetResolutionBits() == 17) {
        motor_drive1.config_.pole_pairs = 4;
    } else if (motor_drive1.encoder_->GetResolutionBits() == 23) {
        motor_drive1.config_.pole_pairs = 5;
    }

    /*
     * Reconfigure pole_pairs for panasonic motors
     */
    if (motor_drive2.encoder_->GetResolutionBits() == 17) {
        motor_drive2.config_.pole_pairs = 4;
    } else if (motor_drive2.encoder_->GetResolutionBits() == 23) {
        motor_drive2.config_.pole_pairs = 5;
    }

    RegisterRpcMethods();

    /*
     * Start Helper Tasks
     */
    StartCommandThread();

    /*
     * Run the RPC thread
     */
//    StartRpcThread();

    /*
     * We should never exit from the this method.
     */
    EnterMainLoop();

    return 0;
}
