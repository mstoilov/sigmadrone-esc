EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 2 4
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 9900 6300 2    60   Input ~ 0
PHASE_A
Text HLabel 9900 6000 2    60   Input ~ 0
PHASE_B
Text HLabel 9900 5700 2    60   Input ~ 0
PHASE_C
Text HLabel 5350 4700 0    60   Output ~ 0
SENSE_C
Text HLabel 5350 5000 0    60   Output ~ 0
SENSE_B
Text HLabel 5350 5300 0    60   Output ~ 0
SENSE_A
$Comp
L Device:R R45
U 1 1 58D1DFF1
P 9050 5700
F 0 "R45" V 9130 5700 50  0000 C CNN
F 1 "47k" V 9050 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8980 5700 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9050 5700 50  0001 C CNN
F 4 "311-47KDCT-ND" V 9050 5700 60  0001 C CNN "Part"
F 5 "DigiKey" V 9050 5700 60  0001 C CNN "Provider"
	1    9050 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R48
U 1 1 58D1E075
P 9050 6000
F 0 "R48" V 9130 6000 50  0000 C CNN
F 1 "47k" V 9050 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8980 6000 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9050 6000 50  0001 C CNN
F 4 "311-47KDCT-ND" V 9050 6000 60  0001 C CNN "Part"
F 5 "DigiKey" V 9050 6000 60  0001 C CNN "Provider"
	1    9050 6000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R51
U 1 1 58D1E0AB
P 9050 6300
F 0 "R51" V 9130 6300 50  0000 C CNN
F 1 "47k" V 9050 6300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8980 6300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9050 6300 50  0001 C CNN
F 4 "311-47KDCT-ND" V 9050 6300 60  0001 C CNN "Part"
F 5 "DigiKey" V 9050 6300 60  0001 C CNN "Provider"
	1    9050 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7750 5300 7750 6300
Wire Wire Line
	8000 5000 8000 6000
Wire Wire Line
	8250 4700 8250 5700
Wire Wire Line
	9200 5700 9900 5700
Wire Wire Line
	9200 6000 9900 6000
Wire Wire Line
	9200 6300 9900 6300
Wire Wire Line
	7750 6800 7750 7050
Wire Wire Line
	7750 7050 8000 7050
Wire Wire Line
	8000 6800 8000 7050
Connection ~ 8000 7050
Wire Wire Line
	5350 5300 7750 5300
Wire Wire Line
	8000 5000 5350 5000
Wire Wire Line
	5350 4700 8250 4700
Wire Wire Line
	8250 7050 8250 6800
Connection ~ 8250 5700
Connection ~ 8000 6000
Connection ~ 7750 6300
Wire Wire Line
	8000 7050 8250 7050
Wire Wire Line
	8000 7050 8000 7200
Wire Wire Line
	8250 5700 8900 5700
Wire Wire Line
	8250 5700 8250 6500
Wire Wire Line
	8000 6000 8900 6000
Wire Wire Line
	8000 6000 8000 6500
Wire Wire Line
	7750 6300 8900 6300
Wire Wire Line
	7750 6300 7750 6500
$Comp
L Device:R R52
U 1 1 5C838571
P 7750 6650
AR Path="/58BF599E/5C838571" Ref="R52"  Part="1" 
AR Path="/58BE27E6/5C838571" Ref="R?"  Part="1" 
AR Path="/58BF664D/5C838571" Ref="R?"  Part="1" 
F 0 "R52" V 7830 6650 50  0000 C CNN
F 1 "3.3k" V 7750 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7680 6650 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7750 6650 50  0001 C CNN
F 4 "311-3.30KHRCT-ND" V 7750 6650 60  0001 C CNN "Part"
F 5 "DigiKey" V 7750 6650 60  0001 C CNN "Provider"
	1    7750 6650
	-1   0    0    1   
$EndComp
$Comp
L Device:R R49
U 1 1 5C83877F
P 8000 6650
AR Path="/58BF599E/5C83877F" Ref="R49"  Part="1" 
AR Path="/58BE27E6/5C83877F" Ref="R?"  Part="1" 
AR Path="/58BF664D/5C83877F" Ref="R?"  Part="1" 
F 0 "R49" V 8080 6650 50  0000 C CNN
F 1 "3.3k" V 8000 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7930 6650 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 8000 6650 50  0001 C CNN
F 4 "311-3.30KHRCT-ND" V 8000 6650 60  0001 C CNN "Part"
F 5 "DigiKey" V 8000 6650 60  0001 C CNN "Provider"
	1    8000 6650
	-1   0    0    1   
$EndComp
$Comp
L Device:R R46
U 1 1 5C838806
P 8250 6650
AR Path="/58BF599E/5C838806" Ref="R46"  Part="1" 
AR Path="/58BE27E6/5C838806" Ref="R?"  Part="1" 
AR Path="/58BF664D/5C838806" Ref="R?"  Part="1" 
F 0 "R46" V 8330 6650 50  0000 C CNN
F 1 "3.3k" V 8250 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8180 6650 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 8250 6650 50  0001 C CNN
F 4 "311-3.30KHRCT-ND" V 8250 6650 60  0001 C CNN "Part"
F 5 "DigiKey" V 8250 6650 60  0001 C CNN "Provider"
	1    8250 6650
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 58BF6189
P 8000 7200
F 0 "#PWR024" H 8000 6950 50  0001 C CNN
F 1 "GND" H 8000 7050 50  0000 C CNN
F 2 "" H 8000 7200 50  0001 C CNN
F 3 "" H 8000 7200 50  0001 C CNN
	1    8000 7200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
