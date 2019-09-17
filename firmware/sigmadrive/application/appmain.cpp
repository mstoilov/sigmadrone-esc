#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include "hello.h"
#include "main.h"
#include "appmain.h"
#include "uart.h"
#include "spimaster.h"
#include "drv8323.h"
#include "exti.h"
#include "quadrature_encoder.h"
#include "ring.h"
#include "cdc_iface.h"

Uart uart1;
SPIMaster spi3;
CdcIface usb_cdc;
QuadratureEncoder tim4(0x2000);
Drv8323 drv1(spi3, GPIOC, GPIO_PIN_13);
Drv8323 drv2(spi3, GPIOC, GPIO_PIN_14);
Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});


extern "C"
int application_main()
{
	Exti exti_usr_button(USER_BTN_Pin, []()->void{HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);});

	/*
	 * Attach the HAL handles to the
	 * C++ wrapper objects. At this point the HAL handles
	 * should be fully initialized.
	 */
	uart1.Attach(&huart1);
	spi3.Attach(&hspi3);
	tim4.Attach(&htim4);
	usb_cdc.Attach(&hUsbDeviceFS);

	Hello h("World, hello: This is a message from the sigmadrive UART console.");
	h.print();

	drv1.WriteReg(2, 0x0);
	drv1.WriteReg(3, 0x0);
	drv1.WriteReg(4, 0x0);
	drv1.WriteReg(5, 0x0);
	drv1.WriteReg(6, 0x0);

	printf("main_task 1\n");
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

	printf("DRV1: \n");
	drv1.DumpRegs();

	char buffer[120];
	uint32_t old_counter = 0, new_counter = 0;

	tim4.Start();

	for (;;) {
		ssize_t siz = usb_cdc.Receive(buffer, sizeof(buffer));
		size_t offset = 0;
		while (siz) {
			size_t ret = usb_cdc.Transmit(buffer + offset, siz);
			siz -= ret;
			offset += ret;
		}
	}

	for (;;) {
//		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

		std::string *str = new std::string("Counter");

		new_counter = tim4.GetPosition();
		if (new_counter != old_counter) {
			printf("%s: %d\n", str->c_str(), new_counter);
			old_counter = new_counter;
		}

		delete str;
//		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}

	return 0;
}
