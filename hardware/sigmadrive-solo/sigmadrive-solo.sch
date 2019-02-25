EESchema Schematic File Version 4
LIBS:sigmadrive-solo-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1800 3650 1700 2450
U 58BF599E
F0 "Phase Sensing" 100
F1 "phase_sensing.sch" 60
F2 "PHASE_A" I R 3500 3850 60 
F3 "PHASE_B" I R 3500 3950 60 
F4 "PHASE_C" I R 3500 4050 60 
F5 "SENSE_C" O R 3500 6000 60 
F6 "SENSE_B" O R 3500 5900 60 
F7 "SENSE_A" O R 3500 5800 60 
$EndSheet
$Sheet
S 5200 2100 1450 3050
U 58BF664D
F0 "MOSFETs" 100
F1 "mosfets_driving.sch" 60
F2 "PWM_AH" I R 6650 2300 60 
F3 "PWM_AL" I R 6650 2400 60 
F4 "PHASE_A" O L 5200 3850 60 
F5 "PWM_BH" I R 6650 2500 60 
F6 "PWM_BL" I R 6650 2600 60 
F7 "PHASE_B" O L 5200 3950 60 
F8 "PWM_CH" I R 6650 2700 60 
F9 "PWM_CL" I R 6650 2800 60 
F10 "PHASE_C" O L 5200 4050 60 
F11 "GATE_ENABLE" I R 6650 3200 60 
F12 "DRV_SDI" I R 6650 3300 60 
F13 "DRV_SDO" O R 6650 3400 60 
F14 "DRV_CLK" I R 6650 3500 60 
F15 "DRV1_CS" I R 6650 3600 60 
F16 "DRV_CAL" I R 6650 3800 60 
F17 "DRV_FAULT" O R 6650 3900 60 
F18 "DRV_SOC" O R 6650 4100 60 
F19 "DRV_SOB" O R 6650 4200 60 
F20 "DRV_SOA" O R 6650 4300 60 
F21 "EXT_5V" I R 6650 4550 60 
F22 "VBAT_ADC" O R 6650 4450 60 
F23 "DRV2_CS" I R 6650 3700 60 
F24 "PWM_RH" I R 6650 2900 60 
F25 "PWM_RL" I R 6650 3000 60 
$EndSheet
$Sheet
S 7700 800  1400 5300
U 58BE2779
F0 "MCU" 100
F1 "mcu.sch" 60
F2 "SENSE_A" I L 7700 5800 60 
F3 "SENSE_B" I L 7700 5900 60 
F4 "SENSE_C" I L 7700 6000 60 
F5 "PWM_AH" O L 7700 2300 60 
F6 "PWM_AL" O L 7700 2400 60 
F7 "PWM_BH" O L 7700 2500 60 
F8 "PWM_BL" O L 7700 2600 60 
F9 "PWM_CH" O L 7700 2700 60 
F10 "PWM_CL" O L 7700 2800 60 
F11 "IA_FB" I L 7700 4300 60 
F12 "IB_FB" I L 7700 4200 60 
F13 "IC_FB" I L 7700 4100 60 
F14 "AUX_L" O L 7700 3000 60 
F15 "AUX_H" O L 7700 2900 60 
F16 "EXT_5V" O L 7700 4550 60 
F17 "VBAT_ADC" I L 7700 4450 60 
F18 "DRV_SDI" O L 7700 3300 60 
F19 "DRV_SDO" I L 7700 3400 60 
F20 "DRV_CLK" O L 7700 3500 60 
F21 "DRV1_CS" O L 7700 3600 60 
F22 "DRV_CAL" O L 7700 3800 60 
F23 "DRV_FAULT" I L 7700 3900 60 
F24 "GATE_ENABLE" O L 7700 3200 60 
F25 "DRV2_CS" O L 7700 3700 60 
$EndSheet
Wire Wire Line
	3500 5800 7700 5800
Wire Wire Line
	3500 5900 7700 5900
Wire Wire Line
	3500 6000 7700 6000
Wire Wire Line
	6650 2300 7700 2300
Wire Wire Line
	6650 2400 7700 2400
Wire Wire Line
	6650 2500 7700 2500
Wire Wire Line
	6650 2600 7700 2600
Wire Wire Line
	6650 2700 7700 2700
Wire Wire Line
	6650 2800 7700 2800
Wire Wire Line
	6650 2900 7700 2900
Wire Wire Line
	6650 3000 7700 3000
Wire Wire Line
	3500 3850 5200 3850
Wire Wire Line
	3500 3950 5200 3950
Wire Wire Line
	3500 4050 5200 4050
Wire Wire Line
	6650 4100 7700 4100
Wire Wire Line
	6650 4200 7700 4200
Wire Wire Line
	6650 4300 7700 4300
Wire Wire Line
	6650 4450 7700 4450
Wire Wire Line
	6650 4550 7700 4550
Wire Wire Line
	6650 3200 7700 3200
Wire Wire Line
	6650 3300 7700 3300
Wire Wire Line
	6650 3400 7700 3400
Wire Wire Line
	6650 3500 7700 3500
Wire Wire Line
	6650 3600 7700 3600
Wire Wire Line
	6650 3800 7700 3800
Wire Wire Line
	6650 3900 7700 3900
Wire Wire Line
	6650 3700 7700 3700
$EndSCHEMATC