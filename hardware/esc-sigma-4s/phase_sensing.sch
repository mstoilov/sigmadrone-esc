EESchema Schematic File Version 4
LIBS:esc-sigma-4s-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
Title "ESC 4s 15A"
Date "2017-12-16"
Rev "5.1"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 6200 2600 2    60   Input ~ 0
PHASE_A
Text HLabel 6200 2900 2    60   Input ~ 0
PHASE_B
Text HLabel 6200 3200 2    60   Input ~ 0
PHASE_C
$Comp
L power:GND #PWR024
U 1 1 58BF6189
P 4300 4600
F 0 "#PWR024" H 4300 4350 50  0001 C CNN
F 1 "GND" H 4300 4450 50  0000 C CNN
F 2 "" H 4300 4600 50  0001 C CNN
F 3 "" H 4300 4600 50  0001 C CNN
	1    4300 4600
	1    0    0    -1  
$EndComp
Text HLabel 1650 2900 0    60   Output ~ 0
CENTER
Text HLabel 1650 2200 0    60   Output ~ 0
SENSE_C
Text HLabel 1650 1900 0    60   Output ~ 0
SENSE_B
Text HLabel 1650 1600 0    60   Output ~ 0
SENSE_A
$Comp
L device:R R44
U 1 1 58D1DF14
P 2250 2600
F 0 "R44" V 2330 2600 50  0000 C CNN
F 1 "47k" V 2250 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2180 2600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 2250 2600 50  0001 C CNN
F 4 "311-47KDCT-ND" V 2250 2600 60  0001 C CNN "Part"
F 5 "DigiKey" V 2250 2600 60  0001 C CNN "Provider"
	1    2250 2600
	0    -1   -1   0   
$EndComp
$Comp
L device:R R47
U 1 1 58D1DF78
P 2250 2900
F 0 "R47" V 2330 2900 50  0000 C CNN
F 1 "47k" V 2250 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2180 2900 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 2250 2900 50  0001 C CNN
F 4 "311-47KDCT-ND" V 2250 2900 60  0001 C CNN "Part"
F 5 "DigiKey" V 2250 2900 60  0001 C CNN "Provider"
	1    2250 2900
	0    -1   -1   0   
$EndComp
$Comp
L device:R R50
U 1 1 58D1DFA6
P 2250 3200
F 0 "R50" V 2330 3200 50  0000 C CNN
F 1 "47k" V 2250 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2180 3200 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 2250 3200 50  0001 C CNN
F 4 "311-47KDCT-ND" V 2250 3200 60  0001 C CNN "Part"
F 5 "DigiKey" V 2250 3200 60  0001 C CNN "Provider"
	1    2250 3200
	0    -1   -1   0   
$EndComp
$Comp
L device:R R45
U 1 1 58D1DFF1
P 5350 2600
F 0 "R45" V 5430 2600 50  0000 C CNN
F 1 "47k" V 5350 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5280 2600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5350 2600 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5350 2600 60  0001 C CNN "Part"
F 5 "DigiKey" V 5350 2600 60  0001 C CNN "Provider"
	1    5350 2600
	0    -1   -1   0   
$EndComp
$Comp
L device:R R48
U 1 1 58D1E075
P 5350 2900
F 0 "R48" V 5430 2900 50  0000 C CNN
F 1 "47k" V 5350 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5280 2900 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5350 2900 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5350 2900 60  0001 C CNN "Part"
F 5 "DigiKey" V 5350 2900 60  0001 C CNN "Provider"
	1    5350 2900
	0    -1   -1   0   
$EndComp
$Comp
L device:R R51
U 1 1 58D1E0AB
P 5350 3200
F 0 "R51" V 5430 3200 50  0000 C CNN
F 1 "47k" V 5350 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5280 3200 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5350 3200 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5350 3200 60  0001 C CNN "Part"
F 5 "DigiKey" V 5350 3200 60  0001 C CNN "Provider"
	1    5350 3200
	0    -1   -1   0   
$EndComp
$Comp
L device:R R52
U 1 1 58D1E348
P 4050 3700
F 0 "R52" V 4130 3700 50  0000 C CNN
F 1 "4.7k" V 4050 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3980 3700 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4050 3700 50  0001 C CNN
F 4 "311-2606-2-ND" V 4050 3700 60  0001 C CNN "Part"
F 5 "DigiKey" V 4050 3700 60  0001 C CNN "Provider"
	1    4050 3700
	-1   0    0    1   
$EndComp
$Comp
L device:R R49
U 1 1 58D1E394
P 4300 3700
F 0 "R49" V 4380 3700 50  0000 C CNN
F 1 "4.7k" V 4300 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4230 3700 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4300 3700 50  0001 C CNN
F 4 "311-2606-2-ND" V 4300 3700 60  0001 C CNN "Part"
F 5 "DigiKey" V 4300 3700 60  0001 C CNN "Provider"
	1    4300 3700
	-1   0    0    1   
$EndComp
$Comp
L device:R R46
U 1 1 58D1E3C0
P 4550 3700
F 0 "R46" V 4630 3700 50  0000 C CNN
F 1 "4.7k" V 4550 3700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4480 3700 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4550 3700 50  0001 C CNN
F 4 "311-2606-2-ND" V 4550 3700 60  0001 C CNN "Part"
F 5 "DigiKey" V 4550 3700 60  0001 C CNN "Provider"
	1    4550 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 2600 4550 2600
Wire Wire Line
	2400 2900 4300 2900
Wire Wire Line
	2400 3200 4050 3200
Wire Wire Line
	2100 2600 1850 2600
Wire Wire Line
	1850 2600 1850 2900
Wire Wire Line
	1850 3200 2100 3200
Wire Wire Line
	1650 2900 1850 2900
Connection ~ 1850 2900
Wire Wire Line
	4050 2200 4050 3200
Wire Wire Line
	4300 1900 4300 2900
Wire Wire Line
	4550 1600 4550 2600
Wire Wire Line
	5500 2600 6200 2600
Wire Wire Line
	5500 2900 6200 2900
Wire Wire Line
	5500 3200 6200 3200
Wire Wire Line
	4050 3850 4050 4450
Wire Wire Line
	4050 4450 4300 4450
Wire Wire Line
	4300 3850 4300 4450
Connection ~ 4300 4450
Wire Wire Line
	1650 2200 4050 2200
Wire Wire Line
	4300 1900 1650 1900
Wire Wire Line
	1650 1600 4550 1600
Wire Wire Line
	4550 4450 4550 3850
Connection ~ 4550 2600
Connection ~ 4300 2900
Connection ~ 4050 3200
Wire Wire Line
	1850 2900 1850 3200
Wire Wire Line
	1850 2900 2100 2900
Wire Wire Line
	4300 4450 4550 4450
Wire Wire Line
	4300 4450 4300 4600
Wire Wire Line
	4550 2600 5200 2600
Wire Wire Line
	4550 2600 4550 3550
Wire Wire Line
	4300 2900 5200 2900
Wire Wire Line
	4300 2900 4300 3550
Wire Wire Line
	4050 3200 5200 3200
Wire Wire Line
	4050 3200 4050 3550
$EndSCHEMATC
