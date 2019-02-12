EESchema Schematic File Version 4
LIBS:servo-sigma-48v-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 2 5
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 6250 9400 2    60   Input ~ 0
PHASE_A
Text HLabel 6250 9700 2    60   Input ~ 0
PHASE_B
Text HLabel 6250 10000 2    60   Input ~ 0
PHASE_C
$Comp
L power:GND #PWR024
U 1 1 58BF6189
P 4350 10900
F 0 "#PWR024" H 4350 10650 50  0001 C CNN
F 1 "GND" H 4350 10750 50  0000 C CNN
F 2 "" H 4350 10900 50  0001 C CNN
F 3 "" H 4350 10900 50  0001 C CNN
	1    4350 10900
	1    0    0    -1  
$EndComp
Text HLabel 1700 9000 0    60   Output ~ 0
SENSE_C
Text HLabel 1700 8700 0    60   Output ~ 0
SENSE_B
Text HLabel 1700 8400 0    60   Output ~ 0
SENSE_A
$Comp
L servo-sigma-48v-rescue:R-device R45
U 1 1 58D1DFF1
P 5400 9400
F 0 "R45" V 5480 9400 50  0000 C CNN
F 1 "47k" V 5400 9400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 9400 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5400 9400 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5400 9400 60  0001 C CNN "Part"
F 5 "DigiKey" V 5400 9400 60  0001 C CNN "Provider"
	1    5400 9400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R48
U 1 1 58D1E075
P 5400 9700
F 0 "R48" V 5480 9700 50  0000 C CNN
F 1 "47k" V 5400 9700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 9700 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5400 9700 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5400 9700 60  0001 C CNN "Part"
F 5 "DigiKey" V 5400 9700 60  0001 C CNN "Provider"
	1    5400 9700
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R51
U 1 1 58D1E0AB
P 5400 10000
F 0 "R51" V 5480 10000 50  0000 C CNN
F 1 "47k" V 5400 10000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 10000 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 5400 10000 50  0001 C CNN
F 4 "311-47KDCT-ND" V 5400 10000 60  0001 C CNN "Part"
F 5 "DigiKey" V 5400 10000 60  0001 C CNN "Provider"
	1    5400 10000
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R52
U 1 1 58D1E348
P 4100 10350
F 0 "R52" V 4180 10350 50  0000 C CNN
F 1 "4.7k" V 4100 10350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4030 10350 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4100 10350 50  0001 C CNN
F 4 "311-2606-2-ND" V 4100 10350 60  0001 C CNN "Part"
F 5 "DigiKey" V 4100 10350 60  0001 C CNN "Provider"
	1    4100 10350
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R49
U 1 1 58D1E394
P 4350 10350
F 0 "R49" V 4430 10350 50  0000 C CNN
F 1 "4.7k" V 4350 10350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4280 10350 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4350 10350 50  0001 C CNN
F 4 "311-2606-2-ND" V 4350 10350 60  0001 C CNN "Part"
F 5 "DigiKey" V 4350 10350 60  0001 C CNN "Provider"
	1    4350 10350
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R46
U 1 1 58D1E3C0
P 4600 10350
F 0 "R46" V 4680 10350 50  0000 C CNN
F 1 "4.7k" V 4600 10350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4530 10350 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4600 10350 50  0001 C CNN
F 4 "311-2606-2-ND" V 4600 10350 60  0001 C CNN "Part"
F 5 "DigiKey" V 4600 10350 60  0001 C CNN "Provider"
	1    4600 10350
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 9000 4100 10000
Wire Wire Line
	4350 8700 4350 9700
Wire Wire Line
	4600 8400 4600 9400
Wire Wire Line
	5550 9400 6250 9400
Wire Wire Line
	5550 9700 6250 9700
Wire Wire Line
	5550 10000 6250 10000
Wire Wire Line
	4100 10500 4100 10750
Wire Wire Line
	4100 10750 4350 10750
Wire Wire Line
	4350 10500 4350 10750
Connection ~ 4350 10750
Wire Wire Line
	1700 9000 4100 9000
Wire Wire Line
	4350 8700 1700 8700
Wire Wire Line
	1700 8400 4600 8400
Wire Wire Line
	4600 10750 4600 10500
Connection ~ 4600 9400
Connection ~ 4350 9700
Connection ~ 4100 10000
Wire Wire Line
	4350 10750 4600 10750
Wire Wire Line
	4350 10750 4350 10900
Wire Wire Line
	4600 9400 5250 9400
Wire Wire Line
	4600 9400 4600 10200
Wire Wire Line
	4350 9700 5250 9700
Wire Wire Line
	4350 9700 4350 10200
Wire Wire Line
	4100 10000 5250 10000
Wire Wire Line
	4100 10000 4100 10200
$Comp
L power:GND #PWR040
U 1 1 5BC37EC6
P 3900 1950
F 0 "#PWR040" H 3900 1700 50  0001 C CNN
F 1 "GND" H 3900 1800 50  0000 C CNN
F 2 "" H 3900 1950 50  0001 C CNN
F 3 "" H 3900 1950 50  0001 C CNN
	1    3900 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 1950 3900 2050
Wire Wire Line
	3900 2650 3900 2800
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5BC43B8B
P 1600 6250
AR Path="/58BE2779/5BC43B8B" Ref="R?"  Part="1" 
AR Path="/58BF599E/5BC43B8B" Ref="R16"  Part="1" 
F 0 "R16" V 1680 6250 50  0000 C CNN
F 1 "10k" V 1600 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1530 6250 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1600 6250 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 1600 6250 60  0001 C CNN "Part"
F 5 "DigiKey" V 1600 6250 60  0001 C CNN "Provider"
	1    1600 6250
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5BC46A5D
P 1600 6900
AR Path="/58BE2779/5BC46A5D" Ref="R?"  Part="1" 
AR Path="/58BF599E/5BC46A5D" Ref="R21"  Part="1" 
F 0 "R21" V 1680 6900 50  0000 C CNN
F 1 "10k" V 1600 6900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1530 6900 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1600 6900 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 1600 6900 60  0001 C CNN "Part"
F 5 "DigiKey" V 1600 6900 60  0001 C CNN "Provider"
	1    1600 6900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5BC48865
P 1600 7300
F 0 "#PWR033" H 1600 7050 50  0001 C CNN
F 1 "GND" H 1600 7150 50  0000 C CNN
F 2 "" H 1600 7300 50  0001 C CNN
F 3 "" H 1600 7300 50  0001 C CNN
	1    1600 7300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR032
U 1 1 5BC488A0
P 1600 5650
F 0 "#PWR032" H 1600 5500 50  0001 C CNN
F 1 "+3V3" H 1615 5823 50  0000 C CNN
F 2 "" H 1600 5650 50  0001 C CNN
F 3 "" H 1600 5650 50  0001 C CNN
	1    1600 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 5650 1600 5750
Wire Wire Line
	1600 7050 1600 7300
Wire Wire Line
	8950 4600 8750 4600
Wire Wire Line
	8750 4600 8750 4000
Wire Wire Line
	8750 4000 9800 4000
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BC4A0CE
P 950 6500
AR Path="/58BE2779/5BC4A0CE" Ref="C?"  Part="1" 
AR Path="/58BF599E/5BC4A0CE" Ref="C43"  Part="1" 
F 0 "C43" H 800 6400 50  0000 L CNN
F 1 "100nF 50V" H 500 6600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 988 6350 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 950 6500 50  0001 C CNN
F 4 "490-4779-2-ND" H 950 6500 60  0001 C CNN "Part"
F 5 "DigiKey" H 950 6500 60  0001 C CNN "Provider"
	1    950  6500
	-1   0    0    1   
$EndComp
Wire Wire Line
	950  6350 950  5750
Wire Wire Line
	950  5750 1600 5750
Connection ~ 1600 5750
Wire Wire Line
	1600 5750 1600 6100
$Comp
L power:GND #PWR021
U 1 1 5BC4B0DC
P 950 7300
F 0 "#PWR021" H 950 7050 50  0001 C CNN
F 1 "GND" H 950 7150 50  0000 C CNN
F 2 "" H 950 7300 50  0001 C CNN
F 3 "" H 950 7300 50  0001 C CNN
	1    950  7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  6650 950  7300
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BC518FC
P 10200 5200
AR Path="/58BE27E6/5BC518FC" Ref="C?"  Part="1" 
AR Path="/58BF599E/5BC518FC" Ref="C44"  Part="1" 
F 0 "C44" H 10050 5100 50  0000 L CNN
F 1 "4.7uF 25V" H 9800 5300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10238 5050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 10200 5200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 10200 5200 60  0001 C CNN "Part"
F 5 "DigiKey" H 10200 5200 60  0001 C CNN "Provider"
	1    10200 5200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR039
U 1 1 5BC52269
P 10200 5550
F 0 "#PWR039" H 10200 5300 50  0001 C CNN
F 1 "GND" H 10200 5400 50  0000 C CNN
F 2 "" H 10200 5550 50  0001 C CNN
F 3 "" H 10200 5550 50  0001 C CNN
	1    10200 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5050 10200 4700
Connection ~ 10200 4700
Wire Wire Line
	10200 4700 9800 4700
Wire Wire Line
	10200 5350 10200 5550
Text Notes 10300 5450 0    60   ~ 0
Originally 2.2uF
$Comp
L servo-sigma-48v-rescue:R-device R31
U 1 1 5BC55957
P 3150 2250
AR Path="/58BF599E/5BC55957" Ref="R31"  Part="1" 
AR Path="/58BE27E6/5BC55957" Ref="R?"  Part="1" 
F 0 "R31" V 3100 2075 50  0000 C CNN
F 1 "1.0k 0.1%" V 3225 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3080 2250 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3150 2250 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 3150 2250 60  0001 C CNN "Part"
F 5 "DigiKey" V 3150 2250 60  0001 C CNN "Provider"
	1    3150 2250
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R58
U 1 1 5BC55995
P 4100 1600
AR Path="/58BF599E/5BC55995" Ref="R58"  Part="1" 
AR Path="/58BE27E6/5BC55995" Ref="R?"  Part="1" 
F 0 "R58" V 4180 1600 50  0000 C CNN
F 1 "10k 0.1%" V 4000 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4030 1600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 4100 1600 50  0001 C CNN
F 4 "P10KDBCT-ND" V 4100 1600 60  0001 C CNN "Part"
F 5 "DigiKey" V 4100 1600 60  0001 C CNN "Provider"
	1    4100 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 2250 3500 2250
Wire Wire Line
	3950 1600 3500 1600
Wire Wire Line
	3500 1600 3500 2250
Connection ~ 3500 2250
Wire Wire Line
	3500 2250 3650 2250
Wire Wire Line
	4250 1600 4650 1600
Wire Wire Line
	4650 1600 4650 2350
Wire Wire Line
	4650 2350 4250 2350
Text HLabel 9450 2250 0    60   Input ~ 0
A_ISENSE_P
Text HLabel 9450 2450 0    60   Input ~ 0
A_ISENSE_N
Wire Wire Line
	2750 2250 3000 2250
Wire Wire Line
	2750 2450 3000 2450
Text HLabel 11800 2350 2    60   Output ~ 0
IA_FB
Wire Wire Line
	4850 2350 4650 2350
Connection ~ 4650 2350
$Comp
L power:GND #PWR062
U 1 1 5BC65AC5
P 7150 1950
F 0 "#PWR062" H 7150 1700 50  0001 C CNN
F 1 "GND" H 7150 1800 50  0000 C CNN
F 2 "" H 7150 1950 50  0001 C CNN
F 3 "" H 7150 1950 50  0001 C CNN
	1    7150 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7150 1950 7150 2050
Wire Wire Line
	7150 2650 7150 2800
Wire Wire Line
	6550 2250 6750 2250
Wire Wire Line
	7200 1600 6750 1600
Wire Wire Line
	6750 1600 6750 2250
Connection ~ 6750 2250
Wire Wire Line
	6750 2250 6900 2250
Wire Wire Line
	7500 1600 7900 1600
Wire Wire Line
	7900 1600 7900 2350
Wire Wire Line
	7900 2350 7500 2350
Text HLabel 6000 2250 0    60   Input ~ 0
B_ISENSE_P
Text HLabel 6000 2450 0    60   Input ~ 0
B_ISENSE_N
Wire Wire Line
	6000 2250 6250 2250
Wire Wire Line
	6000 2450 6250 2450
Text HLabel 8150 2350 2    60   Output ~ 0
IB_FB
Wire Wire Line
	8150 2350 7900 2350
Connection ~ 7900 2350
$Comp
L power:GND #PWR065
U 1 1 5BC672B4
P 14150 1950
F 0 "#PWR065" H 14150 1700 50  0001 C CNN
F 1 "GND" H 14150 1800 50  0000 C CNN
F 2 "" H 14150 1950 50  0001 C CNN
F 3 "" H 14150 1950 50  0001 C CNN
	1    14150 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	14150 1950 14150 2050
Wire Wire Line
	14150 2650 14150 2800
Wire Wire Line
	13550 2250 13750 2250
Wire Wire Line
	14200 1600 13750 1600
Wire Wire Line
	13750 1600 13750 2250
Connection ~ 13750 2250
Wire Wire Line
	13750 2250 13900 2250
Wire Wire Line
	14500 1600 14900 1600
Wire Wire Line
	14900 1600 14900 2350
Wire Wire Line
	14900 2350 14500 2350
Text HLabel 2750 2250 0    60   Input ~ 0
C_ISENSE_P
Text HLabel 2750 2450 0    60   Input ~ 0
C_ISENSE_N
Wire Wire Line
	13000 2250 13250 2250
Wire Wire Line
	13000 2450 13250 2450
Text HLabel 4850 2350 2    60   Output ~ 0
IC_FB
Wire Wire Line
	15100 2350 14900 2350
Connection ~ 14900 2350
$Comp
L power:GND #PWR060
U 1 1 5BC6FA4C
P 9200 4300
F 0 "#PWR060" H 9200 4050 50  0001 C CNN
F 1 "GND" H 9200 4150 50  0000 C CNN
F 2 "" H 9200 4300 50  0001 C CNN
F 3 "" H 9200 4300 50  0001 C CNN
	1    9200 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	9200 4300 9200 4400
Wire Wire Line
	9200 5000 9200 5150
Wire Wire Line
	3300 2450 3500 2450
Wire Wire Line
	13550 2450 13750 2450
Wire Wire Line
	6550 2450 6750 2450
$Comp
L servo-sigma-48v-rescue:R-device R57
U 1 1 5BC7DEE8
P 10200 3300
AR Path="/58BF599E/5BC7DEE8" Ref="R57"  Part="1" 
AR Path="/58BE27E6/5BC7DEE8" Ref="R?"  Part="1" 
F 0 "R57" V 10280 3300 50  0000 C CNN
F 1 "10k 0.1%" V 10100 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10130 3300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 10200 3300 50  0001 C CNN
F 4 "P10KDBCT-ND" V 10200 3300 60  0001 C CNN "Part"
F 5 "DigiKey" V 10200 3300 60  0001 C CNN "Provider"
	1    10200 3300
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R61
U 1 1 5BC7DFEB
P 13750 3300
AR Path="/58BF599E/5BC7DFEB" Ref="R61"  Part="1" 
AR Path="/58BE27E6/5BC7DFEB" Ref="R?"  Part="1" 
F 0 "R61" V 13830 3300 50  0000 C CNN
F 1 "10k 0.1%" V 13650 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13680 3300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 13750 3300 50  0001 C CNN
F 4 "P10KDBCT-ND" V 13750 3300 60  0001 C CNN "Part"
F 5 "DigiKey" V 13750 3300 60  0001 C CNN "Provider"
	1    13750 3300
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R65
U 1 1 5BC7E0BB
P 6750 3300
AR Path="/58BF599E/5BC7E0BB" Ref="R65"  Part="1" 
AR Path="/58BE27E6/5BC7E0BB" Ref="R?"  Part="1" 
F 0 "R65" V 6830 3300 50  0000 C CNN
F 1 "10k 0.1%" V 6650 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6680 3300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 6750 3300 50  0001 C CNN
F 4 "P10KDBCT-ND" V 6750 3300 60  0001 C CNN "Part"
F 5 "DigiKey" V 6750 3300 60  0001 C CNN "Provider"
	1    6750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 2450 13750 3150
Connection ~ 13750 2450
Wire Wire Line
	13750 2450 13900 2450
Wire Wire Line
	6750 3150 6750 2450
Connection ~ 6750 2450
Wire Wire Line
	6750 2450 6900 2450
Wire Wire Line
	3500 3150 3500 2450
Connection ~ 3500 2450
Wire Wire Line
	3500 2450 3650 2450
$Comp
L servo-sigma-48v-rescue:R-device R62
U 1 1 5BC8A197
P 7350 1600
AR Path="/58BF599E/5BC8A197" Ref="R62"  Part="1" 
AR Path="/58BE27E6/5BC8A197" Ref="R?"  Part="1" 
F 0 "R62" V 7430 1600 50  0000 C CNN
F 1 "10k 0.1%" V 7250 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 1600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7350 1600 50  0001 C CNN
F 4 "P10KDBCT-ND" V 7350 1600 60  0001 C CNN "Part"
F 5 "DigiKey" V 7350 1600 60  0001 C CNN "Provider"
	1    7350 1600
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R66
U 1 1 5BC8A2BB
P 14350 1600
AR Path="/58BF599E/5BC8A2BB" Ref="R66"  Part="1" 
AR Path="/58BE27E6/5BC8A2BB" Ref="R?"  Part="1" 
F 0 "R66" V 14430 1600 50  0000 C CNN
F 1 "10k 0.1%" V 14250 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14280 1600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 14350 1600 50  0001 C CNN
F 4 "P10KDBCT-ND" V 14350 1600 60  0001 C CNN "Part"
F 5 "DigiKey" V 14350 1600 60  0001 C CNN "Provider"
	1    14350 1600
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R41
U 1 1 5BC8AC4F
P 3150 2450
AR Path="/58BF599E/5BC8AC4F" Ref="R41"  Part="1" 
AR Path="/58BE27E6/5BC8AC4F" Ref="R?"  Part="1" 
F 0 "R41" V 3100 2300 50  0000 C CNN
F 1 "1.0k 0.1%" V 3225 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3080 2450 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3150 2450 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 3150 2450 60  0001 C CNN "Part"
F 5 "DigiKey" V 3150 2450 60  0001 C CNN "Provider"
	1    3150 2450
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R59
U 1 1 5BC8B52F
P 6400 2250
AR Path="/58BF599E/5BC8B52F" Ref="R59"  Part="1" 
AR Path="/58BE27E6/5BC8B52F" Ref="R?"  Part="1" 
F 0 "R59" V 6350 2075 50  0000 C CNN
F 1 "1.0k 0.1%" V 6475 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6330 2250 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 6400 2250 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 6400 2250 60  0001 C CNN "Part"
F 5 "DigiKey" V 6400 2250 60  0001 C CNN "Provider"
	1    6400 2250
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R60
U 1 1 5BC8B67D
P 6400 2450
AR Path="/58BF599E/5BC8B67D" Ref="R60"  Part="1" 
AR Path="/58BE27E6/5BC8B67D" Ref="R?"  Part="1" 
F 0 "R60" V 6350 2275 50  0000 C CNN
F 1 "1.0k 0.1%" V 6475 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6330 2450 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 6400 2450 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 6400 2450 60  0001 C CNN "Part"
F 5 "DigiKey" V 6400 2450 60  0001 C CNN "Provider"
	1    6400 2450
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R63
U 1 1 5BC8B6CB
P 13400 2250
AR Path="/58BF599E/5BC8B6CB" Ref="R63"  Part="1" 
AR Path="/58BE27E6/5BC8B6CB" Ref="R?"  Part="1" 
F 0 "R63" V 13350 2075 50  0000 C CNN
F 1 "1.0k 0.1%" V 13475 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13330 2250 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 13400 2250 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 13400 2250 60  0001 C CNN "Part"
F 5 "DigiKey" V 13400 2250 60  0001 C CNN "Provider"
	1    13400 2250
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R64
U 1 1 5BC8B7FD
P 13400 2450
AR Path="/58BF599E/5BC8B7FD" Ref="R64"  Part="1" 
AR Path="/58BE27E6/5BC8B7FD" Ref="R?"  Part="1" 
F 0 "R64" V 13350 2275 50  0000 C CNN
F 1 "1.0k 0.1%" V 13475 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13330 2450 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 13400 2450 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 13400 2450 60  0001 C CNN "Part"
F 5 "DigiKey" V 13400 2450 60  0001 C CNN "Provider"
	1    13400 2450
	0    -1   -1   0   
$EndComp
$Comp
L sigmadrone:OPA4374 U9
U 1 1 5BBFC0D5
P 3950 2350
F 0 "U9" H 4050 2200 50  0000 C CNN
F 1 "OPA4374" H 4100 2500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 3850 2450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 3950 2550 50  0001 C CNN
F 4 "296-18201-ND" H 4050 2650 60  0001 C CNN "Part"
F 5 "DigiKey" H 4150 2750 60  0001 C CNN "Provider"
	1    3950 2350
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5BC0A168
P 10600 1950
F 0 "#PWR0111" H 10600 1700 50  0001 C CNN
F 1 "GND" H 10600 1800 50  0000 C CNN
F 2 "" H 10600 1950 50  0001 C CNN
F 3 "" H 10600 1950 50  0001 C CNN
	1    10600 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	10600 1950 10600 2050
Wire Wire Line
	10600 2650 10600 2800
Wire Wire Line
	10000 2250 10200 2250
Wire Wire Line
	10650 1600 10200 1600
Wire Wire Line
	10200 1600 10200 2250
Connection ~ 10200 2250
Wire Wire Line
	10200 2250 10350 2250
Wire Wire Line
	10950 1600 11350 1600
Wire Wire Line
	11350 1600 11350 2350
Wire Wire Line
	11350 2350 10950 2350
Wire Wire Line
	9450 2250 9700 2250
Wire Wire Line
	9450 2450 9700 2450
Text HLabel 15100 2350 2    60   Output ~ 0
IR_FB
Wire Wire Line
	11800 2350 11350 2350
Connection ~ 11350 2350
Wire Wire Line
	10000 2450 10200 2450
$Comp
L servo-sigma-48v-rescue:R-device R72
U 1 1 5BC0A189
P 3500 3300
AR Path="/58BF599E/5BC0A189" Ref="R72"  Part="1" 
AR Path="/58BE27E6/5BC0A189" Ref="R?"  Part="1" 
F 0 "R72" V 3580 3300 50  0000 C CNN
F 1 "10k 0.1%" V 3400 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 3300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3500 3300 50  0001 C CNN
F 4 "P10KDBCT-ND" V 3500 3300 60  0001 C CNN "Part"
F 5 "DigiKey" V 3500 3300 60  0001 C CNN "Provider"
	1    3500 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 2450 10200 3150
Connection ~ 10200 2450
Wire Wire Line
	10200 2450 10350 2450
$Comp
L servo-sigma-48v-rescue:R-device R73
U 1 1 5BC0A196
P 10800 1600
AR Path="/58BF599E/5BC0A196" Ref="R73"  Part="1" 
AR Path="/58BE27E6/5BC0A196" Ref="R?"  Part="1" 
F 0 "R73" V 10880 1600 50  0000 C CNN
F 1 "10k 0.1%" V 10700 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10730 1600 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 10800 1600 50  0001 C CNN
F 4 "P10KDBCT-ND" V 10800 1600 60  0001 C CNN "Part"
F 5 "DigiKey" V 10800 1600 60  0001 C CNN "Provider"
	1    10800 1600
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R70
U 1 1 5BC0A19F
P 9850 2250
AR Path="/58BF599E/5BC0A19F" Ref="R70"  Part="1" 
AR Path="/58BE27E6/5BC0A19F" Ref="R?"  Part="1" 
F 0 "R70" V 9800 2075 50  0000 C CNN
F 1 "1.0k 0.1%" V 9925 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9780 2250 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9850 2250 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 9850 2250 60  0001 C CNN "Part"
F 5 "DigiKey" V 9850 2250 60  0001 C CNN "Provider"
	1    9850 2250
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R71
U 1 1 5BC0A1A8
P 9850 2450
AR Path="/58BF599E/5BC0A1A8" Ref="R71"  Part="1" 
AR Path="/58BE27E6/5BC0A1A8" Ref="R?"  Part="1" 
F 0 "R71" V 9800 2275 50  0000 C CNN
F 1 "1.0k 0.1%" V 9925 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9780 2450 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9850 2450 50  0001 C CNN
F 4 "P1.0KDBCT-ND" V 9850 2450 60  0001 C CNN "Part"
F 5 "DigiKey" V 9850 2450 60  0001 C CNN "Provider"
	1    9850 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	12500 4600 12300 4600
Wire Wire Line
	12300 4600 12300 4000
Wire Wire Line
	12300 4000 13400 4000
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BC2C7CF
P 13750 5200
AR Path="/58BE27E6/5BC2C7CF" Ref="C?"  Part="1" 
AR Path="/58BF599E/5BC2C7CF" Ref="C11"  Part="1" 
F 0 "C11" H 13600 5100 50  0000 L CNN
F 1 "4.7uF 25V" H 13350 5300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 13788 5050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 13750 5200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 13750 5200 60  0001 C CNN "Part"
F 5 "DigiKey" H 13750 5200 60  0001 C CNN "Provider"
	1    13750 5200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5BC2C7D5
P 13750 5550
F 0 "#PWR0112" H 13750 5300 50  0001 C CNN
F 1 "GND" H 13750 5400 50  0000 C CNN
F 2 "" H 13750 5550 50  0001 C CNN
F 3 "" H 13750 5550 50  0001 C CNN
	1    13750 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 5050 13750 4700
Connection ~ 13750 4700
Wire Wire Line
	13750 4700 13400 4700
Wire Wire Line
	13750 5350 13750 5550
Text Notes 13850 5450 0    60   ~ 0
Originally 2.2uF
$Comp
L power:GND #PWR0113
U 1 1 5BC2C7E0
P 12750 4300
F 0 "#PWR0113" H 12750 4050 50  0001 C CNN
F 1 "GND" H 12750 4150 50  0000 C CNN
F 2 "" H 12750 4300 50  0001 C CNN
F 3 "" H 12750 4300 50  0001 C CNN
	1    12750 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	12750 4300 12750 4400
Wire Wire Line
	12750 5000 12750 5150
$Comp
L sigmadrone:OPA4374 U9
U 3 1 5BC2C7F0
P 14200 2350
F 0 "U9" H 14300 2200 50  0000 C CNN
F 1 "OPA4374" H 14350 2500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 14100 2450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 14200 2550 50  0001 C CNN
F 4 "296-18201-ND" H 14300 2650 60  0001 C CNN "Part"
F 5 "DigiKey" H 14400 2750 60  0001 C CNN "Provider"
	3    14200 2350
	1    0    0    1   
$EndComp
Wire Wire Line
	5500 4600 5300 4600
Wire Wire Line
	5300 4600 5300 4000
Wire Wire Line
	5300 4000 6350 4000
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BC322F9
P 6750 5200
AR Path="/58BE27E6/5BC322F9" Ref="C?"  Part="1" 
AR Path="/58BF599E/5BC322F9" Ref="C45"  Part="1" 
F 0 "C45" H 6600 5100 50  0000 L CNN
F 1 "4.7uF 25V" H 6350 5300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6788 5050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 6750 5200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 6750 5200 60  0001 C CNN "Part"
F 5 "DigiKey" H 6750 5200 60  0001 C CNN "Provider"
	1    6750 5200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5BC322FF
P 6750 5550
F 0 "#PWR0114" H 6750 5300 50  0001 C CNN
F 1 "GND" H 6750 5400 50  0000 C CNN
F 2 "" H 6750 5550 50  0001 C CNN
F 3 "" H 6750 5550 50  0001 C CNN
	1    6750 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 5050 6750 4700
Connection ~ 6750 4700
Wire Wire Line
	6750 4700 6350 4700
Wire Wire Line
	6750 5350 6750 5550
Text Notes 6850 5450 0    60   ~ 0
Originally 2.2uF
$Comp
L power:GND #PWR0117
U 1 1 5BC3230A
P 5750 4300
F 0 "#PWR0117" H 5750 4050 50  0001 C CNN
F 1 "GND" H 5750 4150 50  0000 C CNN
F 2 "" H 5750 4300 50  0001 C CNN
F 3 "" H 5750 4300 50  0001 C CNN
	1    5750 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	5750 4300 5750 4400
Wire Wire Line
	5750 5000 5750 5150
$Comp
L sigmadrone:OPA4374 U10
U 1 1 5BC3231A
P 9250 4700
F 0 "U10" H 9350 4550 50  0000 C CNN
F 1 "OPA4374" H 9400 4850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9150 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 9250 4900 50  0001 C CNN
F 4 "296-18201-ND" H 9350 5000 60  0001 C CNN "Part"
F 5 "DigiKey" H 9450 5100 60  0001 C CNN "Provider"
	1    9250 4700
	1    0    0    1   
$EndComp
Wire Wire Line
	2250 4600 2050 4600
Wire Wire Line
	2050 4600 2050 4000
Wire Wire Line
	2050 4000 3100 4000
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BC359FB
P 3500 5200
AR Path="/58BE27E6/5BC359FB" Ref="C?"  Part="1" 
AR Path="/58BF599E/5BC359FB" Ref="C46"  Part="1" 
F 0 "C46" H 3350 5100 50  0000 L CNN
F 1 "4.7uF 25V" H 3100 5300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3538 5050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 3500 5200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 3500 5200 60  0001 C CNN "Part"
F 5 "DigiKey" H 3500 5200 60  0001 C CNN "Provider"
	1    3500 5200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5BC35A01
P 3500 5550
F 0 "#PWR0118" H 3500 5300 50  0001 C CNN
F 1 "GND" H 3500 5400 50  0000 C CNN
F 2 "" H 3500 5550 50  0001 C CNN
F 3 "" H 3500 5550 50  0001 C CNN
	1    3500 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5050 3500 4700
Connection ~ 3500 4700
Wire Wire Line
	3500 4700 3100 4700
Wire Wire Line
	3500 5350 3500 5550
Text Notes 3600 5450 0    60   ~ 0
Originally 2.2uF
$Comp
L power:GND #PWR0119
U 1 1 5BC35A0C
P 2500 4300
F 0 "#PWR0119" H 2500 4050 50  0001 C CNN
F 1 "GND" H 2500 4150 50  0000 C CNN
F 2 "" H 2500 4300 50  0001 C CNN
F 3 "" H 2500 4300 50  0001 C CNN
	1    2500 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2500 4300 2500 4400
Wire Wire Line
	2500 5000 2500 5150
$Comp
L sigmadrone:OPA4374 U10
U 3 1 5BC35A1C
P 5800 4700
F 0 "U10" H 5900 4550 50  0000 C CNN
F 1 "OPA4374" H 5950 4850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5700 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 5800 4900 50  0001 C CNN
F 4 "296-18201-ND" H 5900 5000 60  0001 C CNN "Part"
F 5 "DigiKey" H 6000 5100 60  0001 C CNN "Provider"
	3    5800 4700
	1    0    0    1   
$EndComp
Wire Wire Line
	1600 6400 1600 6550
Wire Wire Line
	8750 6550 8750 4800
Wire Wire Line
	8750 4800 8950 4800
Connection ~ 1600 6550
Wire Wire Line
	1600 6550 1600 6750
Wire Wire Line
	12300 6550 12300 4800
Wire Wire Line
	12300 4800 12500 4800
Wire Wire Line
	5300 6550 5300 4800
Wire Wire Line
	5300 4800 5500 4800
Wire Wire Line
	2050 6550 2050 4800
Wire Wire Line
	2050 4800 2250 4800
$Comp
L power:+3V3 #PWR0120
U 1 1 5BC5B84C
P 9200 5150
F 0 "#PWR0120" H 9200 5000 50  0001 C CNN
F 1 "+3V3" H 9215 5323 50  0000 C CNN
F 2 "" H 9200 5150 50  0001 C CNN
F 3 "" H 9200 5150 50  0001 C CNN
	1    9200 5150
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0121
U 1 1 5BC5B8EB
P 12750 5150
F 0 "#PWR0121" H 12750 5000 50  0001 C CNN
F 1 "+3V3" H 12765 5323 50  0000 C CNN
F 2 "" H 12750 5150 50  0001 C CNN
F 3 "" H 12750 5150 50  0001 C CNN
	1    12750 5150
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0122
U 1 1 5BC5B98A
P 5750 5150
F 0 "#PWR0122" H 5750 5000 50  0001 C CNN
F 1 "+3V3" H 5765 5323 50  0000 C CNN
F 2 "" H 5750 5150 50  0001 C CNN
F 3 "" H 5750 5150 50  0001 C CNN
	1    5750 5150
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0123
U 1 1 5BC5BA98
P 2500 5150
F 0 "#PWR0123" H 2500 5000 50  0001 C CNN
F 1 "+3V3" H 2515 5323 50  0000 C CNN
F 2 "" H 2500 5150 50  0001 C CNN
F 3 "" H 2500 5150 50  0001 C CNN
	1    2500 5150
	-1   0    0    1   
$EndComp
$Comp
L sigmadrone:OPA4374 U10
U 4 1 5BC5BB5E
P 2550 4700
F 0 "U10" H 2650 4550 50  0000 C CNN
F 1 "OPA4374" H 2700 4850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2450 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 2550 4900 50  0001 C CNN
F 4 "296-18201-ND" H 2650 5000 60  0001 C CNN "Part"
F 5 "DigiKey" H 2750 5100 60  0001 C CNN "Provider"
	4    2550 4700
	1    0    0    1   
$EndComp
$Comp
L sigmadrone:OPA4374 U10
U 2 1 5BC5BC86
P 12800 4700
F 0 "U10" H 12900 4550 50  0000 C CNN
F 1 "OPA4374" H 12950 4850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 12700 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 12800 4900 50  0001 C CNN
F 4 "296-18201-ND" H 12900 5000 60  0001 C CNN "Part"
F 5 "DigiKey" H 13000 5100 60  0001 C CNN "Provider"
	2    12800 4700
	1    0    0    1   
$EndComp
$Comp
L sigmadrone:OPA4374 U9
U 4 1 5BC5BD74
P 10650 2350
F 0 "U9" H 10750 2200 50  0000 C CNN
F 1 "OPA4374" H 10800 2500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 10550 2450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 10650 2550 50  0001 C CNN
F 4 "296-18201-ND" H 10750 2650 60  0001 C CNN "Part"
F 5 "DigiKey" H 10850 2750 60  0001 C CNN "Provider"
	4    10650 2350
	1    0    0    1   
$EndComp
$Comp
L sigmadrone:OPA4374 U9
U 2 1 5BC5BE62
P 7200 2350
F 0 "U9" H 7300 2200 50  0000 C CNN
F 1 "OPA4374" H 7350 2500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 7100 2450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa2373.pdf" H 7200 2550 50  0001 C CNN
F 4 "296-18201-ND" H 7300 2650 60  0001 C CNN "Part"
F 5 "DigiKey" H 7400 2750 60  0001 C CNN "Provider"
	2    7200 2350
	1    0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0124
U 1 1 5BC5C760
P 14150 2800
F 0 "#PWR0124" H 14150 2650 50  0001 C CNN
F 1 "+3V3" H 14165 2973 50  0000 C CNN
F 2 "" H 14150 2800 50  0001 C CNN
F 3 "" H 14150 2800 50  0001 C CNN
	1    14150 2800
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0125
U 1 1 5BC5C8B8
P 10600 2800
F 0 "#PWR0125" H 10600 2650 50  0001 C CNN
F 1 "+3V3" H 10615 2973 50  0000 C CNN
F 2 "" H 10600 2800 50  0001 C CNN
F 3 "" H 10600 2800 50  0001 C CNN
	1    10600 2800
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0126
U 1 1 5BC5CA10
P 7150 2800
F 0 "#PWR0126" H 7150 2650 50  0001 C CNN
F 1 "+3V3" H 7165 2973 50  0000 C CNN
F 2 "" H 7150 2800 50  0001 C CNN
F 3 "" H 7150 2800 50  0001 C CNN
	1    7150 2800
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR0127
U 1 1 5BC5CC46
P 3900 2800
F 0 "#PWR0127" H 3900 2650 50  0001 C CNN
F 1 "+3V3" H 3915 2973 50  0000 C CNN
F 2 "" H 3900 2800 50  0001 C CNN
F 3 "" H 3900 2800 50  0001 C CNN
	1    3900 2800
	-1   0    0    1   
$EndComp
Text HLabel 13000 2250 0    60   Input ~ 0
R_ISENSE_P
Text HLabel 13000 2450 0    60   Input ~ 0
R_ISENSE_N
Wire Wire Line
	9800 4000 9800 4700
Connection ~ 9800 4700
Wire Wire Line
	9800 4700 9550 4700
Wire Wire Line
	13400 4000 13400 4700
Connection ~ 13400 4700
Wire Wire Line
	13400 4700 13100 4700
Wire Wire Line
	6350 4000 6350 4700
Connection ~ 6350 4700
Wire Wire Line
	6350 4700 6100 4700
Wire Wire Line
	3100 4000 3100 4700
Connection ~ 3100 4700
Wire Wire Line
	3100 4700 2850 4700
Wire Wire Line
	3500 3450 3500 4700
Wire Wire Line
	6750 3450 6750 4700
Wire Wire Line
	13750 3450 13750 4700
Wire Wire Line
	10200 3450 10200 4700
Wire Wire Line
	1600 6550 2050 6550
Connection ~ 2050 6550
Wire Wire Line
	2050 6550 5300 6550
Connection ~ 5300 6550
Wire Wire Line
	5300 6550 8750 6550
Connection ~ 8750 6550
Wire Wire Line
	8750 6550 12300 6550
$EndSCHEMATC
