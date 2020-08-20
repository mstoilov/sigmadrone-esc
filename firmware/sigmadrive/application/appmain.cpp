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
SPIMaster spi3;
PwmGenerator tim1;
PwmGenerator tim8;
CdcIface usb_cdc;
// QuadratureEncoder tim4(0x2000);
// Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});
DumbEncoder dumb_encoder;
MinasA4Encoder ma4_abs_encoder;
Drv8323 drv1(spi3, GPIOC, GPIO_PIN_13);
Drv8323 drv2(spi3, GPIOC, GPIO_PIN_14);
MotorDrive motor_drive1(1, &drv1, &adc1, &dumb_encoder, &tim1, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorDrive motor_drive2(2, &drv2, &adc2, &dumb_encoder, &tim8, SYSTEM_CORE_CLOCK / (2 * TIM1_PERIOD_CLOCKS * (TIM1_RCR + 1)));
MotorCtrlFOC foc1(&motor_drive1, "axis1");
MotorCtrlFOC foc2(&motor_drive2, "axis2");
HRTimer hrtimer(SYSTEM_CORE_CLOCK/2, 0xFFFF);

bool debug_encoder = false;

void DetectMinasEncoder();

rexjson::property props =
        rexjson::property_map {
            {"clock_hz", rexjson::property(&SystemCoreClock, rexjson::property_access::readonly)},
            {"debug_encoder", &debug_encoder},
            {"axis1", rexjson::property({foc1.GetPropertyMap()})},
            {"axis2", rexjson::property({foc2.GetPropertyMap()})},
        };
rexjson::property* g_properties = &props;


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

extern "C"
void SD_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
    ADC_TypeDef* ADCx = hadc->Instance;

    if (LL_ADC_IsActiveFlag_JEOS(ADCx) && LL_ADC_IsEnabledIT_JEOS(ADCx)) {
        LL_ADC_ClearFlag_JEOS(ADCx);
        if (hadc == &hadc1) {
            if (tim1.GetCounterDirection()) {
                motor_drive2.t_begin_ = hrtimer.GetCounter();
                motor_drive1.UpdateBias();
                motor_drive2.UpdateCurrent();
            } else {
                motor_drive1.t_begin_ = hrtimer.GetCounter();
                motor_drive2.UpdateBias();
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
        ma4_abs_encoder.ReceiveCompleteCallback();
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

void SetEncoder()
{
    try {
        motor_drive1.SetEncoder(&ma4_abs_encoder);
        if (motor_drive1.encoder_->GetResolutionBits() == 17) {
            motor_drive1.config_.pole_pairs = 4;
        } else if (motor_drive1.encoder_->GetResolutionBits() == 23) {
            motor_drive1.config_.pole_pairs = 5;
        }
    } catch (std::exception& e) {
        fprintf(stdout, "Error: %s\n", e.what());
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

void RegisterRpcMethods()
{
    rpc_server.add("dumptext", rexjson::make_rpc_wrapper(DumpText, "void DumpText()"));
    rpc_server.add("LoadConfig", rexjson::make_rpc_wrapper(LoadConfig, "void LoadConfig()"));
    rpc_server.add("SaveConfig", rexjson::make_rpc_wrapper(SaveConfig, "void SaveConfig()"));
    rpc_server.add("drv1.EnableVREFDiv", rexjson::make_rpc_wrapper(&drv1, &Drv8323::EnableVREFDiv, "void Drv8323::EnableVREFDiv()"));
    rpc_server.add("drv1.DisableVREFDiv", rexjson::make_rpc_wrapper(&drv1, &Drv8323::DisableVREFDiv, "void Drv8323::DisableVREFDiv()"));
    rpc_server.add("drv1.DumpRegs", rexjson::make_rpc_wrapper(&drv1, &Drv8323::DumpRegs, "void Drv8323::DumpRegs()"));
    rpc_server.add("minas.resetF", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeF, "uint32_t MinasA4Encoder::ResetErrorCodeF()"));
    rpc_server.add("minas.resetB", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeB, "uint32_t MinasA4Encoder::ResetErrorCodeB()"));
    rpc_server.add("minas.resetE", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCodeE, "uint32_t MinasA4Encoder::ResetErrorCodeE()"));
    rpc_server.add("minas.reset9", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetErrorCode9, "uint32_t MinasA4Encoder::ResetErrorCode9()"));
    rpc_server.add("minas.reset_counter", rexjson::make_rpc_wrapper(&ma4_abs_encoder, &MinasA4Encoder::ResetPosition, "void MinasA4Encoder::ResetPosition()"));
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

void DisplayEncoderDebugInfo()
{
    static uint32_t old_counter = 0, new_counter = 0;

    new_counter = motor_drive1.GetEncoderPosition() & motor_drive1.enc_resolution_mask_;
    MA4Almc almc;
    almc.as_byte_ = ma4_abs_encoder.GetLastError();
    if (new_counter != old_counter || ma4_abs_encoder.status_) {
        fprintf(stderr, "Minas(0x%x): %7.2f, Cnt: 0x%llx, Rev: 0x%llx, Pos: 0x%llx, Status: %2lu (OS: %2u, FS: %2u, CE: %2u, OF: %2u, ME: %2u, SYD: %2u, BA: %2u ) (UpdT: %5lu, t1_to_t1: %5lu)\n",
                (int)ma4_abs_encoder.GetDeviceID(),
                new_counter * 360.0f / motor_drive1.enc_cpr_,
                motor_drive1.GetEncoderPosition() & motor_drive1.enc_resolution_mask_,
                motor_drive1.GetEncoderPosition() >> motor_drive1.enc_resolution_bits_,
                motor_drive1.GetEncoderPosition(),
                ma4_abs_encoder.status_,
                almc.overspeed_,
                almc.full_abs_status_,
                almc.count_error_,
                almc.counter_overflow_,
                almc.multiple_revolution_error_,
                almc.system_down_,
                almc.battery_alarm_,
                ma4_abs_encoder.update_time_ms_,
                ma4_abs_encoder.t1_to_t1_);
        old_counter = new_counter;
    }
}

void DisplayDrvRegs()
{
    fprintf(stdout, "DRV1: \n");
    drv1.DumpRegs();
}

void EnterMainLoop()
{
    BlinkLed warnblinker(LED_WARN_GPIO_Port, LED_WARN_Pin, 150, 150);

    for (;;) {
        /*
         * Blink the WARN LED
         */
        warnblinker.Blink();

        /*
         * Dump Encoder Info
         */
        if (debug_encoder)
            DisplayEncoderDebugInfo();
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
    ma4_abs_encoder.Attach(&huart3);
    adc1.Attach(&hadc1);
    adc2.Attach(&hadc2);
    adc3.Attach(&hadc3);
    uart1.Attach(&huart1);
    uart2.Attach(&huart2);
    spi3.Attach(&hspi3);
    LL_TIM_SetCounter(TIM8, TIM1_PERIOD_CLOCKS - 1);
    LL_TIM_SetCounter(TIM1, 0);
    tim8.Attach(&htim8);
    tim1.Attach(&htim1);
//    tim8.EnableCounter(true);
    tim1.EnableCounter(true);
//	tim4.Attach(&htim4);
//	tim4.Start();
//	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
    usb_cdc.Attach(&hUsbDeviceFS, true);
    drv1.InitializeDefaults();
    drv2.InitializeDefaults();

    DisplayDrvRegs();
    DisplayPropertiesInfo();
    SetEncoder();
    motor_drive2.SetEncoder(&dumb_encoder);
    motor_drive1.Attach();
    motor_drive2.Attach();
    RegisterRpcMethods();

    /*
     * Start Helper Tasks
     */
    StartCommandThread();
    StartRpcThread();

    /*
     * We should never exit from the this method.
     */
    EnterMainLoop();

    return 0;
}
