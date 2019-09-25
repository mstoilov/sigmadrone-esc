#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <string>
#include <assert.h>
#include "hello.h"
#include "main.h"
#include "appmain.h"

#include "ClEditLine.h"
#include "ClHistory.h"
#include "ClPort.h"
#include "uart.h"
#include "spimaster.h"
#include "drv8323.h"
#include "exti.h"
#include "quadrature_encoder.h"
#include "ring.h"
#include "cdc_iface.h"

#include "rexjson++.h"
#include "linenoise.h"


Uart uart1;
SPIMaster spi3;
CdcIface usb_cdc;
QuadratureEncoder tim4(0x2000);
Drv8323 drv1(spi3, GPIOC, GPIO_PIN_13);
Drv8323 drv2(spi3, GPIOC, GPIO_PIN_14);
Exti encoder_z(ENCODER_Z_Pin, []()->void{tim4.CallbackIndex();});



#include <iostream>
#include <streambuf>
#include <locale>
#include <cstdio>

class cdcbuf: public std::streambuf {
public:
	char buffer_[128];
	const size_t put_back_ = 16;

	cdcbuf() {
		setg(buffer_ + sizeof(buffer_), buffer_ + sizeof(buffer_), buffer_ + sizeof(buffer_));
	}
protected:
	/*
	 * central output function
	 */
	virtual int_type overflow(int_type c)
	{
		if (c != EOF) {
			// convert lowercase to uppercase
			char txc = c;
			send(&txc, sizeof(txc));
		}
		return c;
	}

	virtual std::streamsize xsputn( const char_type* s, std::streamsize count)
	{
		return send(s, count);
	}

	virtual std::streamsize send( const char_type* s, std::streamsize count)
	{
		std::streamsize offset = 0;
		std::streamsize siz = count;

		while (siz) {
			size_t ret = usb_cdc.Transmit(s + offset, siz);
			siz -= ret;
			offset += ret;
		}
		return count;
	}

	virtual int_type underflow()
	{
		char_type* begin = gptr();
		char_type* end = egptr();
		char_type* back = eback();

	    if (gptr() < egptr()) // buffer not exhausted
	        return traits_type::to_int_type(*gptr());

	    char *base = &buffer_[0];
	    char *start = base;

	    if (eback() == base) // true when this isn't the first fill
	    {
	        // Make arrangements for putback characters
	        std::memmove(base, egptr() - put_back_, put_back_);
	        start += put_back_;
	    }

	    // start is now the start of the buffer, proper.
	    // Read from fptr_ in to the provided buffer
	    std::streamsize n = recv(start, sizeof(buffer_) - (start - base));
	    if (n == 0)
	        return traits_type::eof();

	    // Set buffer pointers
	    setg(base, start, start + n);

	    return traits_type::to_int_type(*gptr());
	}

	virtual std::streamsize xsgetn( char_type* s, std::streamsize count )
	{
		return recv(s, count);
	}

	virtual std::streamsize recv( char_type* s, std::streamsize count)
	{
		std::streamsize offset = 0;
		std::streamsize ret = 0;
		while ((ret = usb_cdc.Receive(s, count)) == 0)
			;
		count -= ret;
		offset += ret;
		if (count)
			ret += usb_cdc.Receive(s + offset, count);
		return ret;
	}

};

char cl_heap[4096];

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

	printf("main_task 1\r\n");
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

	printf("DRV1: \r\n");
	drv1.DumpRegs();

	char buffer[120];
	uint32_t old_counter = 0, new_counter = 0;

	tim4.Start();

	cdcbuf ob;
	cdcbuf ib;
	std::ostream cdc_out(&ob);
	std::istream cdc_in(&ib);

#if 0
	while (1) {
		std::cout << std::string(1,cdc_in.get());
		std::cout << "*** text1 ***\n" << rexjson::read(text1).write(false) << std::endl << std::endl;
	}
#endif

	cl_mem_init(cl_heap, sizeof(cl_heap), 100);
	cl_history_init();
	char szBuffer[1024];
	int elret;
	while (1) {
		if ((elret = cl_editline("# ", szBuffer, sizeof(szBuffer), 5)) > 0) {
			assert(elret == (int)strlen(szBuffer));
			printf("\r\nYou wrote: %s", szBuffer);
		}
		printf("\r\n");
	}


	char *line;
	linenoiseSetMultiLine(1);
	linenoiseHistorySetMaxLen(3);
	while((line = linenoise("hello> ")) != NULL) {
	    printf("You wrote: %s\r\n", line);
	    linenoiseHistoryAdd(line);
	    linenoiseFree(line); /* Or just free(line) if you use libc malloc. */
	}

	while (1) {
		try {
			std::string str;
//			std::getline(cdc_in, str);
			char c = 0;
			while ((c = std::cin.get())) {
				if (c == '\r') {
					std::cout << "\r\n";
					std::cout << str;
					str.clear();
				} else {
					str += c;
				}
				std::string temp;
				temp = c;
				std::cout << temp;
			}

			rexjson::value v = rexjson::read(str);
			std::cout << v.write(false) << std::endl;
		} catch (std::runtime_error& e) {
			std::cout << e.what() << std::endl;
		}
	}


	for (;;) {
		ssize_t siz;
		std::string in;
		while ((siz = usb_cdc.Receive(buffer, sizeof(buffer))) > 0) {
			in += std::string(buffer, siz);
		}

		size_t offset = 0;
		siz = in.size();
		while (siz) {
			size_t ret = usb_cdc.Transmit(in.c_str() + offset, siz);
			siz -= ret;
			offset += ret;
		}
	}

	for (;;) {
//		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

		std::string *str = new std::string("Counter");

		std::cout << "help" << std::endl;
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
