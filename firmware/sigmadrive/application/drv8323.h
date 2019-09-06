/*
 * drv8323.h
 *
 *  Created on: Mar 16, 2019
 *      Author: mstoilov
 */

#ifndef DRV8323_H_
#define DRV8323_H_

#include "spimaster.h"

class Drv8323 {
public:
	/* Reg 0x02 */
	static constexpr uint32_t DIS_CPUV = (0x1 << 9);
	static constexpr uint32_t DIS_GDF = (0x1 << 8);
	static constexpr uint32_t OTW_REP = (0x1 << 7);
	static constexpr uint32_t PWM_MODE_MASK = (0x3);
	static constexpr uint32_t PWM_MODE_6X = (0x0);
	static constexpr uint32_t PWM_MODE_3X = (0x1);
	static constexpr uint32_t PWM_MODE_1X = (0x2);
	static constexpr uint32_t PWM_MODE_IN = (0x3);
	static constexpr uint32_t COAST       = (0x1 << 2);
	static constexpr uint32_t BREAK       = (0x1 << 1);
	static constexpr uint32_t CLR_FLT     = (0x1 << 0);

	/* Reg 0x03 */
	static constexpr uint32_t LOCK = (0x6 << 8);
	static constexpr uint32_t UNLOCK = (0x3 << 8);
	static constexpr uint32_t LOCK_MASK = (0x7 << 8);


	/* IDRIVEP (HS and LS) */
	static constexpr uint32_t IDRIVEP_10mA   = (0x0 << 0);
	static constexpr uint32_t IDRIVEP_30mA   = (0x1 << 0);
	static constexpr uint32_t IDRIVEP_60mA   = (0x2 << 0);
	static constexpr uint32_t IDRIVEP_80mA   = (0x3 << 0);
	static constexpr uint32_t IDRIVEP_120mA  = (0x4 << 0);
	static constexpr uint32_t IDRIVEP_140mA  = (0x5 << 0);
	static constexpr uint32_t IDRIVEP_170mA  = (0x6 << 0);
	static constexpr uint32_t IDRIVEP_190mA  = (0x7 << 0);
	static constexpr uint32_t IDRIVEP_260mA  = (0x8 << 0);
	static constexpr uint32_t IDRIVEP_330mA  = (0x9 << 0);
	static constexpr uint32_t IDRIVEP_370mA  = (0xa << 0);
	static constexpr uint32_t IDRIVEP_440mA  = (0xb << 0);
	static constexpr uint32_t IDRIVEP_570mA  = (0xc << 0);
	static constexpr uint32_t IDRIVEP_680mA  = (0xd << 0);
	static constexpr uint32_t IDRIVEP_820mA  = (0xe << 0);
	static constexpr uint32_t IDRIVEP_1000mA = (0xf << 0);
	static constexpr uint32_t IDRIVEP_MASK   = (0xf << 0);

	/* IDRIVEN (HS and LS) */
	static constexpr uint32_t IDRIVEN_20mA   = (0x0 << 0);
	static constexpr uint32_t IDRIVEN_60mA   = (0x1 << 0);
	static constexpr uint32_t IDRIVEN_120mA  = (0x2 << 0);
	static constexpr uint32_t IDRIVEN_160mA  = (0x3 << 0);
	static constexpr uint32_t IDRIVEN_240mA  = (0x4 << 0);
	static constexpr uint32_t IDRIVEN_280mA  = (0x5 << 0);
	static constexpr uint32_t IDRIVEN_340mA  = (0x6 << 0);
	static constexpr uint32_t IDRIVEN_380mA  = (0x7 << 0);
	static constexpr uint32_t IDRIVEN_520mA  = (0x8 << 0);
	static constexpr uint32_t IDRIVEN_660mA  = (0x9 << 0);
	static constexpr uint32_t IDRIVEN_740mA  = (0xa << 0);
	static constexpr uint32_t IDRIVEN_880mA  = (0xb << 0);
	static constexpr uint32_t IDRIVEN_1140mA = (0xc << 0);
	static constexpr uint32_t IDRIVEN_1360mA = (0xd << 0);
	static constexpr uint32_t IDRIVEN_1640mA = (0xe << 0);
	static constexpr uint32_t IDRIVEN_2000mA = (0xf << 0);
	static constexpr uint32_t IDRIVEN_MASK   = (0xf << 0);

	/* Reg 0x4 */
	static constexpr uint32_t CBC           = (0x1 << 10);
	static constexpr uint32_t TDRIVE_500ns  = 0x0;
	static constexpr uint32_t TDRIVE_1000ns = 0x1;
	static constexpr uint32_t TDRIVE_2000ns = 0x2;
	static constexpr uint32_t TDRIVE_4000ns = 0x3;
	static constexpr uint32_t TDRIVE_MASK   = 0x3;

	/* Reg 0x5 */
	static constexpr uint32_t TRETRY         = (0x1 << 10);
	static constexpr uint32_t DEADTIME_50ns  = 0x0;
	static constexpr uint32_t DEADTIME_100ns = 0x1;
	static constexpr uint32_t DEADTIME_200ns = 0x2;
	static constexpr uint32_t DEADTIME_400ns = 0x3;
	static constexpr uint32_t DEADTIME_MASK  = 0x3;

	static constexpr uint32_t OCP_LATCHED_FAULT = 0x0;
	static constexpr uint32_t OCP_RETRY_FAULT   = 0x1;
	static constexpr uint32_t OCP_REPORT_ONLY   = 0x2;
	static constexpr uint32_t OCP_NO_ACTION     = 0x3;
	static constexpr uint32_t OCP_MODE_MASK     = 0x3;

	static constexpr uint32_t OCP_DEG_2us  = 0x0;
	static constexpr uint32_t OCP_DEG_4us  = 0x1;
	static constexpr uint32_t OCP_DEG_6us  = 0x2;
	static constexpr uint32_t OCP_DEG_8us  = 0x3;
	static constexpr uint32_t OCP_DEG_MASK = 0x3;

	static constexpr uint32_t VDS_LVL_006V = 0x0;
	static constexpr uint32_t VDS_LVL_013V = 0x1;
	static constexpr uint32_t VDS_LVL_020V = 0x2;
	static constexpr uint32_t VDS_LVL_026V = 0x3;
	static constexpr uint32_t VDS_LVL_031V = 0x4;
	static constexpr uint32_t VDS_LVL_045V = 0x5;
	static constexpr uint32_t VDS_LVL_053V = 0x6;
	static constexpr uint32_t VDS_LVL_060V = 0x7;
	static constexpr uint32_t VDS_LVL_068V = 0x8;
	static constexpr uint32_t VDS_LVL_075V = 0x9;
	static constexpr uint32_t VDS_LVL_094V = 0xa;
	static constexpr uint32_t VDS_LVL_113V = 0xb;
	static constexpr uint32_t VDS_LVL_130V = 0xc;
	static constexpr uint32_t VDS_LVL_150V = 0xd;
	static constexpr uint32_t VDS_LVL_170V = 0xe;
	static constexpr uint32_t VDS_LVL_188V = 0xf;
	static constexpr uint32_t VDS_LVL_MASK = 0xf;


	/* Reg 0x6 */
	static constexpr uint32_t CSA_FET       = (0x1 << 10);
	static constexpr uint32_t VREF_DIV      = (0x1 << 9);
	static constexpr uint32_t LS_REF        = (0x1 << 8);
	static constexpr uint32_t CSA_GAIN_5VV  = 0x0;
	static constexpr uint32_t CSA_GAIN_10VV = 0x1;
	static constexpr uint32_t CSA_GAIN_20VV = 0x2;
	static constexpr uint32_t CSA_GAIN_40VV = 0x3;
	static constexpr uint32_t CSA_GAIN_MASK = 0x3;
	static constexpr uint32_t CSA_SHIFT_BIT = 0x6;
	static constexpr uint32_t DIS_SEN       = (0x1 << 5);
	static constexpr uint32_t CSA_CAL_A     = (0x1 << 4);
	static constexpr uint32_t CSA_CAL_B     = (0x1 << 3);
	static constexpr uint32_t CSA_CAL_C     = (0x1 << 2);
	static constexpr uint32_t SEN_LVL_025V  = 0x0;
	static constexpr uint32_t SEN_LVL_050V  = 0x1;
	static constexpr uint32_t SEN_LVL_075V  = 0x2;
	static constexpr uint32_t SEN_LVL_100V  = 0x3;
	static constexpr uint32_t SEN_LVL_MASK  = 0x3;


	Drv8323(SPIMaster& spi, GPIO_TypeDef* NSS_GPIOx, uint16_t NSS_GPIO_Pin);
	virtual ~Drv8323();

	uint32_t GetFaultStatus1() { return ReadReg(0x0); }
	uint32_t GetFaultStatus2() { return ReadReg(0x1); }

	void LockRegisters();
	void UnlockRegisters();
	void SetIDriveP_HS(uint32_t idrivep);
	void SetIDriveN_HS(uint32_t idriven);
	void SetIDriveP_LS(uint32_t idrivep);
	void SetIDriveN_LS(uint32_t idriven);
	void SetTDrive(uint32_t tdrive);
	void SetPWMMode(uint32_t pwm_mode);
	void SetDeadTime(uint32_t dead_time);
	void SetOCPMode(uint32_t ocp_mode);
	void SetOCPDeglitch(uint32_t ocp_deg);
	void SetVDSLevel(uint32_t vds_lvl);
	void SetCSAGain(uint32_t csa_gain);
	void SetOCPSenseLevel(uint32_t sen_lvl);

	void EnableCPUV();
	void DisableCPUV();

	void EnableGDF();
	void DisableGDF();

	void EnableOTW();
	void DisableOTW();

	void EnableCoast();
	void DisableCoast();

	void EnableBreak();
	void DisableBreak();

	void ClearFault();

	void EnableCBC();
	void DisableCBC();

	void EnableTRETRY();
	void DisableTRETRY();

	void EnableVREFDiv();
	void DisableVREFDiv();

	void EnableLSRef();
	void DisableLSRef();

	void EnableSenseOvercurrent();
	void DisableSenseOvercurrent();

	uint32_t ReadReg(uint32_t addr);
	void WriteReg(uint32_t addr, uint32_t value);
	void ModifyReg(uint32_t addr, uint32_t clear_mask, uint32_t set_mask);
	void DumpRegs();

protected:
	SPIMaster& spi_;
	GPIO_TypeDef* NSS_GPIOx_;
	uint16_t NSS_GPIO_Pin_;
};

#endif /* DRV8323_H_ */
