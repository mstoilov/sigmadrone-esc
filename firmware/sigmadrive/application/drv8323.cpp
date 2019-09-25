/*
 * drv8323.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: mstoilov
 */

#include <stdio.h>
#include <assert.h>
#include "drv8323.h"

Drv8323::Drv8323(SPIMaster& spi, GPIO_TypeDef* NSS_GPIOx, uint16_t NSS_GPIO_Pin)
	: spi_(spi)
	, NSS_GPIOx_(NSS_GPIOx)
	, NSS_GPIO_Pin_(NSS_GPIO_Pin)
{

}

Drv8323::~Drv8323()
{

}

uint32_t Drv8323::ReadReg(uint32_t addr)
{
	uint16_t data = (0x1 << 15) | ((addr & 0xF) << 11);
	uint16_t ret = 0;

	assert(spi_.hspi_->Init.DataSize == SPI_DATASIZE_16BIT);

	spi_.ChipSelect(NSS_GPIOx_, NSS_GPIO_Pin_, true);
	spi_.TransmitReceive((uint8_t *)&data, (uint8_t *)&ret, 1);
	spi_.ChipSelect(NSS_GPIOx_, NSS_GPIO_Pin_, false);
	return ret & 0x7FF;
}

void Drv8323::WriteReg(uint32_t addr, uint32_t value)
{
	uint16_t data = (uint16_t)(((addr & 0xF) << 11) | (value & 0x7FF));
	uint16_t ret = 0;

	assert(spi_.hspi_->Init.DataSize == SPI_DATASIZE_16BIT);

	spi_.ChipSelect(NSS_GPIOx_, NSS_GPIO_Pin_, true);
	spi_.TransmitReceive((uint8_t *)&data, (uint8_t *)&ret, 1);
	spi_.ChipSelect(NSS_GPIOx_, NSS_GPIO_Pin_, false);
}

void Drv8323::ModifyReg(uint32_t addr, uint32_t clear_mask, uint32_t set_mask)
{
	WriteReg(addr, (ReadReg(addr) & ~clear_mask) | set_mask);
}

void Drv8323::SetIDriveP_HS(uint32_t idrivep)
{
	assert(idrivep <= IDRIVEP_1000mA);
	ModifyReg(0x3, IDRIVEP_MASK << 4 , idrivep << 4);
}

void Drv8323::SetIDriveN_HS(uint32_t idriven)
{
	assert(idriven <= IDRIVEN_2000mA);
	ModifyReg(0x3, IDRIVEN_MASK, idriven);
}

void Drv8323::SetIDriveP_LS(uint32_t idrivep)
{
	assert(idrivep <= IDRIVEP_1000mA);
	ModifyReg(0x4, IDRIVEP_MASK << 4, idrivep << 4);
}

void Drv8323::SetIDriveN_LS(uint32_t idriven)
{
	assert(idriven <= IDRIVEN_2000mA);
	ModifyReg(0x4, IDRIVEN_MASK, idriven);
}

void Drv8323::SetPWMMode(uint32_t pwm_mode)
{
	assert(pwm_mode <= PWM_MODE_IN);
	ModifyReg(0x2, PWM_MODE_MASK << 5, pwm_mode << 5);
}

void Drv8323::SetDeadTime(uint32_t dead_time)
{
	assert(dead_time <= PWM_MODE_IN);
	ModifyReg(0x5, PWM_MODE_MASK << 8, dead_time << 8);
}

void Drv8323::SetOCPMode(uint32_t ocp_mode)
{
	assert(ocp_mode <= OCP_NO_ACTION);
	ModifyReg(0x5, OCP_MODE_MASK << 6, ocp_mode << 6);
}

void Drv8323::SetOCPDeglitch(uint32_t ocp_deg)
{
	assert(ocp_deg <= OCP_DEG_8us);
	ModifyReg(0x5, OCP_DEG_MASK << 4, ocp_deg << 4);
}

void Drv8323::SetVDSLevel(uint32_t vds_lvl)
{
	assert(vds_lvl <= VDS_LVL_188V);
	ModifyReg(0x5, VDS_LVL_MASK, vds_lvl);
}

void Drv8323::SetCSAGain(uint32_t csa_gain)
{
	assert(csa_gain <= CSA_GAIN_40VV);
	ModifyReg(0x6, CSA_GAIN_MASK << CSA_SHIFT_BIT, csa_gain << CSA_SHIFT_BIT);
}

void Drv8323::SetOCPSenseLevel(uint32_t sen_lvl)
{
	assert(sen_lvl <= SEN_LVL_100V);
	ModifyReg(0x6, SEN_LVL_MASK, sen_lvl);
}

void Drv8323::LockRegisters()
{
	ModifyReg(0x3, LOCK_MASK, LOCK);
}

void Drv8323::UnlockRegisters()
{
	ModifyReg(0x3, LOCK_MASK, UNLOCK);
}

/*
 * Enable charge pump Under-voltage Lock Out (UVLO)
 */
void Drv8323::EnableCPUV()
{
	ModifyReg(0x2, DIS_CPUV, 0);
}

/*
 * Disable charge pump Under-voltage Lock Out (UVLO)
 */
void Drv8323::DisableCPUV()
{
	ModifyReg(0x2, DIS_CPUV, DIS_CPUV);
}

/*
 * Enable Gate Driver Fault (GDF)
 */
void Drv8323::EnableGDF()
{
	ModifyReg(0x2, DIS_GDF, 0);
}

/*
 * Disable Gate Driver Fault (GDF)
 */
void Drv8323::DisableGDF()
{
	ModifyReg(0x2, DIS_GDF, DIS_GDF);
}

/*
 * OTW is reported on nFAULT or the FAULT bit
 */
void Drv8323::EnableOTW()
{
	ModifyReg(0x2, OTW_REP, OTW_REP);
}

/*
 * OTW is not reported on nFAULT or the FAULT bit
 */
void Drv8323::DisableOTW()
{
	ModifyReg(0x2, OTW_REP, 0);
}

/*
 * put all MOSFETs in the Hi-Z state
 */
void Drv8323::EnableCoast()
{
	ModifyReg(0x2, COAST, COAST);
}

void Drv8323::DisableCoast()
{
	ModifyReg(0x2, COAST, 0);
}

/*
 * turn on all three low-side MOSFETs in 1xPWM mode.
 * This bit is ORed with the INLC (BRAKE) input
 */
void Drv8323::EnableBreak()
{
	ModifyReg(0x2, BREAK, BREAK);
}

void Drv8323::DisableBreak()
{
	ModifyReg(0x2, BREAK, 0);
}


/*
 * Clear latched fault bits. This bit automatically resets after being written.
 */
void Drv8323::ClearFault()
{
	ModifyReg(0x2, CLR_FLT, CLR_FLT);
}

void Drv8323::EnableCBC()
{
	ModifyReg(0x4, CBC, CBC);
}

void Drv8323::DisableCBC()
{
	ModifyReg(0x4, CBC, 0);
}

void Drv8323::SetTDrive(uint32_t tdrive)
{
	assert(tdrive <= TDRIVE_4000ns);
	ModifyReg(0x4, TDRIVE_MASK << 8, tdrive << 8);
}

/*
 * VDS_OCP and SEN_OCP retry time is 50 μs
 */
void Drv8323::EnableTRETRY()
{
	ModifyReg(0x5, TRETRY, TRETRY);
}

/*
 * VDS_OCP and SEN_OCP retry time is 4 ms
 */
void Drv8323::DisableTRETRY()
{
	ModifyReg(0x5, TRETRY, 0);
}


/*
 * Current sense amplifier reference voltage is VREF/2
 */
void Drv8323::EnableVREFDiv()
{
	ModifyReg(0x6, VREF_DIV, VREF_DIV);
}

/*
 * Current sense amplifier reference voltage is VREF
 */
void Drv8323::DisableVREFDiv()
{
	ModifyReg(0x6, VREF_DIV, 0);
}

/*
 * VDS_OCP for the low-side MOSFET is measured across
 * SHx to SNx
 */
void Drv8323::EnableLSRef()
{
	ModifyReg(0x6, LS_REF, LS_REF);
}

/*
 * VDS_OCP for the low-side MOSFET is measured
 * across SHx to SPx
 */
void Drv8323::DisableLSRef()
{
	ModifyReg(0x6, LS_REF, 0);
}

void Drv8323::EnableSenseOvercurrent()
{
	ModifyReg(0x6, DIS_SEN, 0);
}

void Drv8323::DisableSenseOvercurrent()
{
	ModifyReg(0x6, DIS_SEN, DIS_SEN);
}

void Drv8323::DumpRegs()
{
	for (int i = 0; i < 7; i++) {
		printf("DRV: Reg %d: 0x%x\r\n", i, (unsigned int) ReadReg(i));
	}
	printf("\r\n\r\n");
}
