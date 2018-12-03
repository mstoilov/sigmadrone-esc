EESchema Schematic File Version 4
LIBS:esc-sigma-4s-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
Title "ESC 4s 15A"
Date "2017-12-16"
Rev "5.1"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	1950 3000 3550 3000
Wire Wire Line
	3950 3000 6550 3000
Wire Wire Line
	1950 3100 6550 3100
Wire Wire Line
	1950 3200 6550 3200
$Comp
L power:GND #PWR065
U 1 1 58F14037
P 6350 5000
F 0 "#PWR065" H 6350 4750 50  0001 C CNN
F 1 "GND" H 6350 4850 50  0000 C CNN
F 2 "" H 6350 5000 60  0000 C CNN
F 3 "" H 6350 5000 60  0000 C CNN
	1    6350 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4300 6350 4300
Wire Wire Line
	6350 4300 6350 5000
Wire Wire Line
	6550 4400 6350 4400
Connection ~ 6350 4400
Wire Wire Line
	6350 4500 6550 4500
Connection ~ 6350 4500
Wire Wire Line
	6550 4600 6350 4600
Connection ~ 6350 4600
Wire Wire Line
	6350 4700 6550 4700
Connection ~ 6350 4700
Wire Wire Line
	6550 4200 5950 4200
Wire Wire Line
	5950 4200 5950 4450
$Comp
L power:GND #PWR066
U 1 1 58F14038
P 5950 5000
F 0 "#PWR066" H 5950 4750 50  0001 C CNN
F 1 "GND" H 5950 4850 50  0000 C CNN
F 2 "" H 5950 5000 60  0000 C CNN
F 3 "" H 5950 5000 60  0000 C CNN
	1    5950 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4750 5950 5000
$Comp
L power:+3V3 #PWR067
U 1 1 58F14039
P 6350 2000
F 0 "#PWR067" H 6350 1850 50  0001 C CNN
F 1 "+3V3" H 6350 2140 50  0000 C CNN
F 2 "" H 6350 2000 60  0000 C CNN
F 3 "" H 6350 2000 60  0000 C CNN
	1    6350 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR068
U 1 1 58F1403A
P 5950 2000
F 0 "#PWR068" H 5950 1750 50  0001 C CNN
F 1 "GND" H 5950 1850 50  0000 C CNN
F 2 "" H 5950 2000 60  0000 C CNN
F 3 "" H 5950 2000 60  0000 C CNN
	1    5950 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5950 2000 5950 2250
Wire Wire Line
	5950 2550 5950 2900
$Comp
L power:GND #PWR069
U 1 1 58F1403B
P 5550 2000
F 0 "#PWR069" H 5550 1750 50  0001 C CNN
F 1 "GND" H 5550 1850 50  0000 C CNN
F 2 "" H 5550 2000 60  0000 C CNN
F 3 "" H 5550 2000 60  0000 C CNN
	1    5550 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 2000 5550 2250
Wire Wire Line
	5550 2550 5550 2900
Connection ~ 5950 2900
$Comp
L power:GND #PWR070
U 1 1 58F1403C
P 5150 2000
F 0 "#PWR070" H 5150 1750 50  0001 C CNN
F 1 "GND" H 5150 1850 50  0000 C CNN
F 2 "" H 5150 2000 60  0000 C CNN
F 3 "" H 5150 2000 60  0000 C CNN
	1    5150 2000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR071
U 1 1 58F1403D
P 2650 2000
F 0 "#PWR071" H 2650 1750 50  0001 C CNN
F 1 "GND" H 2650 1850 50  0000 C CNN
F 2 "" H 2650 2000 60  0000 C CNN
F 3 "" H 2650 2000 60  0000 C CNN
	1    2650 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 2000 5150 2250
Wire Wire Line
	5150 2550 5150 3000
Connection ~ 5150 3000
Wire Wire Line
	2650 2000 2650 2250
Wire Wire Line
	2650 2550 2650 3000
Connection ~ 2650 3000
$Comp
L power:GND #PWR072
U 1 1 58F1403E
P 2150 5000
F 0 "#PWR072" H 2150 4750 50  0001 C CNN
F 1 "GND" H 2150 4850 50  0000 C CNN
F 2 "" H 2150 5000 60  0000 C CNN
F 3 "" H 2150 5000 60  0000 C CNN
	1    2150 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3400 2150 5000
Wire Wire Line
	6350 2900 6350 2000
Wire Wire Line
	5550 2900 6550 2900
Connection ~ 6350 2900
Text HLabel 9500 2900 2    60   Output ~ 0
C_FTDI_TXD
Text HLabel 9500 3000 2    60   Input ~ 0
C_FTDI_RXD
$Comp
L sigmadrone:FB_SMT FB2
U 1 1 58F14041
P 3750 3000
F 0 "FB2" H 3750 3100 50  0000 C CNN
F 1 "BEAD/0.6OHM/0.2A/2.2 kOhm @ 100MHz" H 3750 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 3750 2900 30  0001 C CNN
F 3 "http://katalog.we-online.de/pbs/datasheet/742792093.pdf" H 3800 3000 60  0001 C CNN
F 4 "0805" H 3750 3000 25  0001 C CNN "SMT"
F 5 "732-1609-1-ND" H 3750 2850 30  0001 C CNN "Part"
F 6 "DigiKey" H 3750 2800 30  0001 C CNN "Provider"
	1    3750 3000
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG073
U 1 1 58F1404B
P 4350 2000
F 0 "#FLG073" H 4350 2095 50  0001 C CNN
F 1 "PWR_FLAG" H 4350 2224 50  0000 C CNN
F 2 "" H 4350 2000 50  0000 C CNN
F 3 "" H 4350 2000 50  0000 C CNN
	1    4350 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2000 4350 3000
Connection ~ 4350 3000
$Comp
L sigmadrone:629_105_150_921 CN1
U 1 1 58F1404D
P 1550 3500
F 0 "CN1" H 1700 4150 60  0000 C CNN
F 1 "629_105_150_921" V 1800 3350 60  0000 C CNN
F 2 "Sigmadrone:WURTH_629105150921_NO_OVALS" H 3100 4800 60  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/629105150921.pdf" H 3100 4800 60  0001 C CNN
F 4 "732-5961-1-ND" H 1550 3500 60  0001 C CNN "Part"
F 5 "DigiKey" H 1550 3500 60  0001 C CNN "Provider"
	1    1550 3500
	-1   0    0    -1  
$EndComp
NoConn ~ 1950 3300
Wire Wire Line
	1950 3500 2150 3500
Wire Wire Line
	1950 3600 2150 3600
Wire Wire Line
	1950 3700 2150 3700
Wire Wire Line
	1950 3800 2150 3800
Wire Wire Line
	1950 3900 2150 3900
Wire Wire Line
	1950 4000 2150 4000
Connection ~ 2150 3600
Connection ~ 2150 3700
Connection ~ 2150 3800
Connection ~ 2150 3900
Connection ~ 2150 4000
Connection ~ 2150 3500
Wire Wire Line
	1950 3400 2150 3400
$Comp
L power:PWR_FLAG #FLG074
U 1 1 58F1404E
P 2150 2000
F 0 "#FLG074" H 2150 2095 50  0001 C CNN
F 1 "PWR_FLAG" H 2150 2224 50  0000 C CNN
F 2 "" H 2150 2000 50  0000 C CNN
F 3 "" H 2150 2000 50  0000 C CNN
	1    2150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2000 2150 3000
Connection ~ 2150 3000
$Comp
L sigmadrone:FT232RQ U8
U 1 1 58F1404F
P 7250 3800
F 0 "U8" H 7600 4850 60  0000 C CNN
F 1 "FT232RQ" H 7000 2750 60  0000 C CNN
F 2 "Housings_DFN_QFN:QFN-32-1EP_5x5mm_Pitch0.5mm" H 7250 2675 60  0001 C CNN
F 3 "" H 7250 3800 60  0000 C CNN
F 4 "768-1008-1-ND" H 7250 2525 60  0001 C CNN "Part"
F 5 "DigiKey" H 7250 2600 60  0001 C CNN "Provider"
	1    7250 3800
	1    0    0    -1  
$EndComp
NoConn ~ 6550 3800
NoConn ~ 6550 4000
NoConn ~ 6550 4100
NoConn ~ 7950 3500
NoConn ~ 7950 3900
NoConn ~ 7950 4000
NoConn ~ 7950 4100
$Comp
L power:GND #PWR075
U 1 1 58F14050
P 8100 5000
F 0 "#PWR075" H 8100 4750 50  0001 C CNN
F 1 "GND" H 8100 4850 50  0000 C CNN
F 2 "" H 8100 5000 60  0000 C CNN
F 3 "" H 8100 5000 60  0000 C CNN
	1    8100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 4700 8100 4700
Wire Wire Line
	8100 4700 8100 5000
NoConn ~ 7950 3800
NoConn ~ 7950 3700
$Comp
L device:C C31
U 1 1 58F14C24
P 5550 2400
F 0 "C31" H 5400 2300 50  0000 L CNN
F 1 "10uF 35V X5R" V 5700 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5588 2250 50  0001 C CNN
F 3 "https://product.tdk.com/en/products/common/pdf/mlcc_partnumber_description.pdf" H 5550 2400 50  0001 C CNN
F 4 "445-14419-1-ND" H 5550 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 5550 2400 60  0001 C CNN "Provider"
	1    5550 2400
	-1   0    0    1   
$EndComp
$Comp
L device:C C29
U 1 1 58F14CDD
P 2650 2400
F 0 "C29" H 2500 2300 50  0000 L CNN
F 1 "10uF 35V X5R" V 2800 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2688 2250 50  0001 C CNN
F 3 "https://product.tdk.com/en/products/common/pdf/mlcc_partnumber_description.pdf" H 2650 2400 50  0001 C CNN
F 4 "445-14419-1-ND" H 2650 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 2650 2400 60  0001 C CNN "Provider"
	1    2650 2400
	-1   0    0    1   
$EndComp
$Comp
L device:C C30
U 1 1 58F224AF
P 5150 2400
F 0 "C30" H 5000 2300 50  0000 L CNN
F 1 "100nF 50V" V 5300 2200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5188 2250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 5150 2400 50  0001 C CNN
F 4 "490-4779-2-ND" H 5150 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 5150 2400 60  0001 C CNN "Provider"
	1    5150 2400
	-1   0    0    1   
$EndComp
$Comp
L device:C C32
U 1 1 58F22517
P 5950 2400
F 0 "C32" H 5800 2300 50  0000 L CNN
F 1 "100nF 50V" H 5500 2500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5988 2250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 5950 2400 50  0001 C CNN
F 4 "490-4779-2-ND" H 5950 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 5950 2400 60  0001 C CNN "Provider"
	1    5950 2400
	-1   0    0    1   
$EndComp
$Comp
L device:C C33
U 1 1 58F2256D
P 5950 4600
F 0 "C33" H 5800 4500 50  0000 L CNN
F 1 "100nF 50V" H 5500 4700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5988 4450 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 5950 4600 50  0001 C CNN
F 4 "490-4779-2-ND" H 5950 4600 60  0001 C CNN "Part"
F 5 "DigiKey" H 5950 4600 60  0001 C CNN "Provider"
	1    5950 4600
	-1   0    0    1   
$EndComp
Wire Wire Line
	7950 2900 9500 2900
Wire Wire Line
	7950 3000 9500 3000
Text HLabel 9500 2650 2    60   Input ~ 0
3V3
$Comp
L power:+3V3 #PWR076
U 1 1 58F217D7
P 9250 2500
F 0 "#PWR076" H 9250 2350 50  0001 C CNN
F 1 "+3V3" H 9250 2640 50  0000 C CNN
F 2 "" H 9250 2500 60  0000 C CNN
F 3 "" H 9250 2500 60  0000 C CNN
	1    9250 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2500 9250 2650
Wire Wire Line
	9250 2650 9500 2650
NoConn ~ 7950 3400
NoConn ~ 7950 3300
NoConn ~ 7950 3100
NoConn ~ 7950 3200
NoConn ~ 7950 3600
$EndSCHEMATC