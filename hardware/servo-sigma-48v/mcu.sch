EESchema Schematic File Version 4
LIBS:servo-sigma-48v-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 5 5
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 14650 3550 2    60   ~ 0
PWM_AH
Text Label 14650 5650 2    60   ~ 0
PWM_AL
Text Label 14650 3650 2    60   ~ 0
PWM_BH
Text Label 14650 5750 2    60   ~ 0
PWM_BL
Text Label 14650 3750 2    60   ~ 0
PWM_CH
Text Label 14650 5850 2    60   ~ 0
PWM_CL
Text Label 14650 4850 2    60   ~ 0
PWM
Text Label 3850 9000 3    60   ~ 0
LED_WARN
Text Label 3400 10900 1    60   ~ 0
LED_STATUS
$Comp
L servo-sigma-48v-rescue:LED-device D1
U 1 1 5AB8EC78
P 3400 9750
F 0 "D1" H 3400 9850 50  0000 C CNN
F 1 "GREEN" H 3400 9650 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3400 9750 50  0001 C CNN
F 3 "http://katalog.we-online.de/led/datasheet/150060GS75000.pdf" H 3400 9750 50  0001 C CNN
F 4 "732-4971-1-ND" H 3400 9750 60  0001 C CNN "Part"
F 5 "DigiKey" H 3400 9750 60  0001 C CNN "Provider"
	1    3400 9750
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:LED-device D4
U 1 1 5AB8EC79
P 3850 9750
F 0 "D4" H 3850 9850 50  0000 C CNN
F 1 "RED" H 3850 9650 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3850 9750 50  0001 C CNN
F 3 "http://katalog.we-online.de/led/datasheet/150060RS75000.pdf" H 3850 9750 50  0001 C CNN
F 4 "732-4978-1-ND" H 3850 9750 60  0001 C CNN "Part"
F 5 "DigiKey" H 3850 9750 60  0001 C CNN "Provider"
	1    3850 9750
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R4
U 1 1 5AB8EC7A
P 3850 10200
F 0 "R4" V 3930 10200 50  0000 C CNN
F 1 "2.2k" V 3850 10200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3780 10200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 3850 10200 50  0001 C CNN
F 4 "RMCF0603JT2K20CT-ND" V 3850 10200 60  0001 C CNN "Part"
F 5 "DigiKey" V 3850 10200 60  0001 C CNN "Provider"
	1    3850 10200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5AB8EC7B
P 3850 10750
F 0 "#PWR01" H 3850 10500 50  0001 C CNN
F 1 "GND" H 3850 10600 50  0000 C CNN
F 2 "" H 3850 10750 50  0001 C CNN
F 3 "" H 3850 10750 50  0001 C CNN
	1    3850 10750
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:CONN_02X05-conn J2
U 1 1 5AB8EC7C
P 2550 1300
F 0 "J2" H 2550 1600 50  0000 C CNN
F 1 "CONN_02X05" H 2550 950 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical" H 2550 100 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/6220xx21121.pdf" H 2550 100 50  0001 C CNN
F 4 "732-5374-ND" H 2550 1300 60  0001 C CNN "Part"
F 5 "DigiKey" H 2550 1300 60  0001 C CNN "Provider"
	1    2550 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 58D0728C
P 1100 1200
F 0 "#PWR02" H 1100 950 50  0001 C CNN
F 1 "GND" H 1100 1050 50  0000 C CNN
F 2 "" H 1100 1200 50  0001 C CNN
F 3 "" H 1100 1200 50  0001 C CNN
	1    1100 1200
	0    1    1    0   
$EndComp
Text Label 3450 1100 2    60   ~ 0
PA13
Text Label 3850 1100 2    60   ~ 0
SWDIO
Text Label 3450 1200 2    60   ~ 0
PA14
Text Label 3850 1200 2    60   ~ 0
SWCLK
Text Label 3850 1500 2    60   ~ 0
NRST
Text Label 3450 1300 2    60   ~ 0
PB3
Text Notes 1600 1800 0    60   ~ 0
SWD Interface
Text HLabel 14950 3550 2    60   Output ~ 0
PWM_AH
Text HLabel 14950 5650 2    60   Output ~ 0
PWM_AL
Text HLabel 14950 3650 2    60   Output ~ 0
PWM_BH
Text HLabel 14950 5750 2    60   Output ~ 0
PWM_BL
Text HLabel 14950 3750 2    60   Output ~ 0
PWM_CH
Text HLabel 14950 5850 2    60   Output ~ 0
PWM_CL
Text HLabel 14950 4850 2    60   Input ~ 0
PWM
$Comp
L servo-sigma-48v-rescue:R-device R1
U 1 1 5AB8EC7E
P 3400 10200
F 0 "R1" V 3480 10200 50  0000 C CNN
F 1 "2.2k" V 3400 10200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3330 10200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 3400 10200 50  0001 C CNN
F 4 "RMCF0603JT2K20CT-ND" V 3400 10200 60  0001 C CNN "Part"
F 5 "DigiKey" V 3400 10200 60  0001 C CNN "Provider"
	1    3400 10200
	-1   0    0    1   
$EndComp
Text Label 13350 4450 2    60   ~ 0
PB0
Text Label 13350 4550 2    60   ~ 0
PB1
Text Label 13350 4650 2    60   ~ 0
PB2
Text Label 13350 4750 2    60   ~ 0
PB3
Text Label 13350 4850 2    60   ~ 0
PB4
Text Label 13350 4950 2    60   ~ 0
PB5
Text Label 13350 5050 2    60   ~ 0
PB6
Text Label 13350 5150 2    60   ~ 0
PB7
Text Label 13350 5250 2    60   ~ 0
PB8
Text Label 13350 5350 2    60   ~ 0
PB9
Text Label 13350 5450 2    60   ~ 0
PB10
Text Label 13350 5550 2    60   ~ 0
PB12
Text Label 13350 5650 2    60   ~ 0
PB13
Text Label 13350 5750 2    60   ~ 0
PB14
Text Label 13350 5850 2    60   ~ 0
PB15
Text Label 13350 2750 2    60   ~ 0
PA0
Text Label 13350 2850 2    60   ~ 0
PA1
Text Label 13350 2950 2    60   ~ 0
PA2
Text Label 13350 3050 2    60   ~ 0
PA3
Text Label 13350 3150 2    60   ~ 0
PA4
Text Label 13350 3250 2    60   ~ 0
PA5
Text Label 13350 3350 2    60   ~ 0
PA6
Text Label 13350 3450 2    60   ~ 0
PA7
Text Label 13350 3550 2    60   ~ 0
PA8
Text Label 13350 3650 2    60   ~ 0
PA9
Text Label 13350 3750 2    60   ~ 0
PA10
Text Label 13350 3850 2    60   ~ 0
PA11
Text Label 13350 3950 2    60   ~ 0
PA12
Text Label 13350 4050 2    60   ~ 0
PA13
Text Label 13350 4150 2    60   ~ 0
PA14
Text Label 13350 4250 2    60   ~ 0
PA15
Text Label 6750 4350 0    60   ~ 0
PC0
Text Label 6750 4450 0    60   ~ 0
PC1
Text Label 6750 4550 0    60   ~ 0
PC2
Text Label 6750 4650 0    60   ~ 0
PC3
Text Label 6750 4750 0    60   ~ 0
PC4
Text Label 6750 4850 0    60   ~ 0
PC5
Text Label 6750 4950 0    60   ~ 0
PC6
Text Label 6750 5050 0    60   ~ 0
PC7
Text Label 6750 5150 0    60   ~ 0
PC8
Text Label 6750 5250 0    60   ~ 0
PC9
Text Label 6750 5350 0    60   ~ 0
PC10
Text Label 6750 5450 0    60   ~ 0
PC11
Text Label 6750 5550 0    60   ~ 0
PC12
Text Label 6750 5650 0    60   ~ 0
PC13
$Comp
L servo-sigma-48v-rescue:Crystal-device Y1
U 1 1 58ED7267
P 4700 4050
F 0 "Y1" H 4700 4200 50  0000 C CNN
F 1 "ABM3-8.000MHZ-D2Y-T" H 4700 3900 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_Abracon_ABM3-2Pin_5.0x3.2mm" H 4700 4050 50  0001 C CNN
F 3 "http://www.abracon.com/Resonators/abm3.pdf" H 4700 4050 50  0001 C CNN
F 4 "535-10630-1-ND" H 4700 4050 60  0001 C CNN "Part"
F 5 "DigiKey" H 4700 4050 60  0001 C CNN "Provider"
	1    4700 4050
	0    1    1    0   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C27
U 1 1 58EFF4C6
P 4200 3850
F 0 "C27" V 4150 3950 50  0000 C CNN
F 1 "27pF/50V/1%" V 4350 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 3700 50  0001 C CNN
F 3 "http://search.murata.co.jp/Ceramy/image/img/A01X/partnumbering_e_01.pdf" H 4200 3850 50  0001 C CNN
F 4 "490-9719-1-ND" V 4200 3850 60  0001 C CNN "Part"
F 5 "DigiKey" V 4200 3850 60  0001 C CNN "Provider"
	1    4200 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5AB8EC81
P 3600 3850
F 0 "#PWR04" H 3600 3600 50  0001 C CNN
F 1 "GND" H 3600 3700 50  0000 C CNN
F 2 "" H 3600 3850 50  0001 C CNN
F 3 "" H 3600 3850 50  0001 C CNN
	1    3600 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 58F00797
P 3600 4250
F 0 "#PWR05" H 3600 4000 50  0001 C CNN
F 1 "GND" H 3600 4100 50  0000 C CNN
F 2 "" H 3600 4250 50  0001 C CNN
F 3 "" H 3600 4250 50  0001 C CNN
	1    3600 4250
	0    1    1    0   
$EndComp
$Comp
L servo-sigma-48v-rescue:STM32F411RETx-stm32 U5
U 1 1 58F00E03
P 10050 4350
F 0 "U5" H 12650 6300 50  0000 C CNN
F 1 "STM32F411RETx" H 12550 2500 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 12850 6225 50  0001 R TNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b3/a5/46/3b/b4/e5/4c/85/DM00115249.pdf/files/DM00115249.pdf/jcr:content/translations/en.DM00115249.pdf" H 10050 4350 50  0001 C CNN
F 4 "497-14909-ND" H 10050 4350 60  0001 C CNN "Part"
F 5 "DigiKey" H 10050 4350 60  0001 C CNN "Provider"
	1    10050 4350
	1    0    0    -1  
$EndComp
Text Label 8000 1400 2    60   ~ 0
NRST
$Comp
L power:VDD #PWR08
U 1 1 58F11F73
P 1100 900
F 0 "#PWR08" H 1100 750 50  0001 C CNN
F 1 "VDD" H 1117 1073 50  0000 C CNN
F 2 "" H 1100 900 50  0001 C CNN
F 3 "" H 1100 900 50  0001 C CNN
	1    1100 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5AB8EC85
P 6300 2100
F 0 "#PWR010" H 6300 1850 50  0001 C CNN
F 1 "GND" H 6300 1950 50  0000 C CNN
F 2 "" H 6300 2100 50  0001 C CNN
F 3 "" H 6300 2100 50  0001 C CNN
	1    6300 2100
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R19
U 1 1 58F146B9
P 6300 2500
F 0 "R19" V 6380 2500 50  0000 C CNN
F 1 "10k" V 6300 2500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6230 2500 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 6300 2500 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 6300 2500 60  0001 C CNN "Part"
F 5 "DigiKey" V 6300 2500 60  0001 C CNN "Provider"
	1    6300 2500
	-1   0    0    1   
$EndComp
$Comp
L sigmadrone:FB_SMT FB1
U 1 1 5AB8EC87
P 4350 3250
F 0 "FB1" H 4350 3350 50  0000 C CNN
F 1 "BEAD/0.6OHM/0.2A/2.2 kOhm @ 100MHz" H 4350 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4350 3150 30  0001 C CNN
F 3 "http://katalog.we-online.de/pbs/datasheet/742792093.pdf" H 4400 3250 60  0001 C CNN
F 4 "0805" H 4350 3250 25  0001 C CNN "SMT"
F 5 "732-1609-1-ND" H 4350 3100 30  0001 C CNN "Part"
F 6 "DigiKey" H 4350 3050 30  0001 C CNN "Provider"
	1    4350 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 58F18A4C
P 3600 3450
F 0 "#PWR013" H 3600 3200 50  0001 C CNN
F 1 "GND" H 3600 3300 50  0000 C CNN
F 2 "" H 3600 3450 50  0001 C CNN
F 3 "" H 3600 3450 50  0001 C CNN
	1    3600 3450
	0    1    -1   0   
$EndComp
$Comp
L power:VDD #PWR014
U 1 1 58F18E09
P 3600 3250
F 0 "#PWR014" H 3600 3100 50  0001 C CNN
F 1 "VDD" H 3617 3423 50  0000 C CNN
F 2 "" H 3600 3250 50  0001 C CNN
F 3 "" H 3600 3250 50  0001 C CNN
	1    3600 3250
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR015
U 1 1 58F19218
P 5850 2100
F 0 "#PWR015" H 5850 1950 50  0001 C CNN
F 1 "VDD" H 5867 2273 50  0000 C CNN
F 2 "" H 5850 2100 50  0001 C CNN
F 3 "" H 5850 2100 50  0001 C CNN
	1    5850 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5AB8EC8B
P 5450 2100
F 0 "#PWR016" H 5450 1850 50  0001 C CNN
F 1 "GND" H 5450 1950 50  0000 C CNN
F 2 "" H 5450 2100 50  0001 C CNN
F 3 "" H 5450 2100 50  0001 C CNN
	1    5450 2100
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C25
U 1 1 5AB8EC8C
P 11000 1400
F 0 "C25" H 10850 1300 50  0000 L CNN
F 1 "100nF 50V" H 10550 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11038 1250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 11000 1400 50  0001 C CNN
F 4 "490-4779-2-ND" H 11000 1400 60  0001 C CNN "Part"
F 5 "DigiKey" H 11000 1400 60  0001 C CNN "Provider"
	1    11000 1400
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C24
U 1 1 58F1C379
P 11550 1400
F 0 "C24" H 11400 1300 50  0000 L CNN
F 1 "100nF 50V" H 11100 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11588 1250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 11550 1400 50  0001 C CNN
F 4 "490-4779-2-ND" H 11550 1400 60  0001 C CNN "Part"
F 5 "DigiKey" H 11550 1400 60  0001 C CNN "Provider"
	1    11550 1400
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C19
U 1 1 58F1C405
P 12100 1400
F 0 "C19" H 11950 1300 50  0000 L CNN
F 1 "100nF 50V" H 11650 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12138 1250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 12100 1400 50  0001 C CNN
F 4 "490-4779-2-ND" H 12100 1400 60  0001 C CNN "Part"
F 5 "DigiKey" H 12100 1400 60  0001 C CNN "Provider"
	1    12100 1400
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C14
U 1 1 5AB8EC8F
P 12700 1400
F 0 "C14" H 12550 1300 50  0000 L CNN
F 1 "100nF 50V" H 12250 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12738 1250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 12700 1400 50  0001 C CNN
F 4 "490-4779-2-ND" H 12700 1400 60  0001 C CNN "Part"
F 5 "DigiKey" H 12700 1400 60  0001 C CNN "Provider"
	1    12700 1400
	-1   0    0    1   
$EndComp
$Comp
L power:VDD #PWR017
U 1 1 5AB8EC90
P 9850 2000
F 0 "#PWR017" H 9850 1850 50  0001 C CNN
F 1 "VDD" H 9867 2173 50  0000 C CNN
F 2 "" H 9850 2000 50  0001 C CNN
F 3 "" H 9850 2000 50  0001 C CNN
	1    9850 2000
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR018
U 1 1 5AB8EC91
P 11550 750
F 0 "#PWR018" H 11550 600 50  0001 C CNN
F 1 "VDD" H 11567 923 50  0000 C CNN
F 2 "" H 11550 750 50  0001 C CNN
F 3 "" H 11550 750 50  0001 C CNN
	1    11550 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5AB8EC92
P 9850 6500
F 0 "#PWR019" H 9850 6250 50  0001 C CNN
F 1 "GND" H 9850 6350 50  0000 C CNN
F 2 "" H 9850 6500 50  0001 C CNN
F 3 "" H 9850 6500 50  0001 C CNN
	1    9850 6500
	1    0    0    -1  
$EndComp
Text Label 6750 4150 0    60   ~ 0
PD2
$Comp
L power:PWR_FLAG #FLG020
U 1 1 58F21458
P 6000 3550
F 0 "#FLG020" H 6000 3625 50  0001 C CNN
F 1 "PWR_FLAG" H 6000 3724 50  0000 C CNN
F 2 "" H 6000 3550 50  0001 C CNN
F 3 "" H 6000 3550 50  0001 C CNN
	1    6000 3550
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG021
U 1 1 58F217D5
P 7000 3550
F 0 "#FLG021" H 7000 3625 50  0001 C CNN
F 1 "PWR_FLAG" H 7000 3724 50  0000 C CNN
F 2 "" H 7000 3550 50  0001 C CNN
F 3 "" H 7000 3550 50  0001 C CNN
	1    7000 3550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 58F52F40
P 12100 2000
F 0 "#PWR022" H 12100 1750 50  0001 C CNN
F 1 "GND" H 12100 1850 50  0000 C CNN
F 2 "" H 12100 2000 50  0001 C CNN
F 3 "" H 12100 2000 50  0001 C CNN
	1    12100 2000
	1    0    0    -1  
$EndComp
NoConn ~ 7150 4150
NoConn ~ 7150 5350
NoConn ~ 7150 5450
NoConn ~ 7150 5550
Wire Wire Line
	3400 9900 3400 10050
Wire Wire Line
	3850 10750 3850 10350
Wire Wire Line
	3850 10050 3850 9900
Wire Wire Line
	3400 10350 3400 10900
Wire Wire Line
	3850 9000 3850 9600
Wire Wire Line
	3400 9200 3400 9600
Wire Wire Line
	2300 1200 1100 1200
Wire Wire Line
	2300 1300 2200 1300
Wire Wire Line
	1100 1500 1900 1500
Wire Wire Line
	1100 1100 2300 1100
Wire Wire Line
	2800 1100 3850 1100
Wire Wire Line
	2800 1200 3850 1200
Wire Wire Line
	2800 1300 2900 1300
Wire Wire Line
	2800 1500 3850 1500
Wire Wire Line
	13350 4050 12950 4050
Wire Wire Line
	13350 4150 12950 4150
Wire Wire Line
	7150 5150 6750 5150
Wire Wire Line
	7150 5250 6750 5250
Wire Wire Line
	7150 5350 6750 5350
Wire Wire Line
	7150 5450 6750 5450
Wire Wire Line
	7150 5550 6750 5550
Wire Wire Line
	7150 5650 6750 5650
Wire Wire Line
	4350 3850 4700 3850
Wire Wire Line
	4700 4200 4700 4250
Connection ~ 4700 3850
Connection ~ 4700 4250
Wire Wire Line
	3600 3850 4050 3850
Wire Wire Line
	3600 4250 4050 4250
Wire Wire Line
	6750 2750 7150 2750
Wire Wire Line
	1100 900  1100 1100
Wire Wire Line
	6300 2950 7150 2950
Wire Wire Line
	6300 2650 6300 2950
Wire Wire Line
	6300 2350 6300 2100
Wire Wire Line
	4350 4250 4700 4250
Wire Wire Line
	5100 4250 5100 3950
Wire Wire Line
	5100 3950 7150 3950
Wire Wire Line
	4550 3250 4850 3250
Wire Wire Line
	5150 3450 6000 3450
Wire Wire Line
	6000 3350 6000 3450
Wire Wire Line
	6000 3350 7150 3350
Wire Wire Line
	3600 3450 4850 3450
Wire Wire Line
	3600 3250 4150 3250
Wire Wire Line
	5850 2100 5850 2800
Wire Wire Line
	5450 2350 5450 2100
Wire Wire Line
	5450 2650 5450 2800
Wire Wire Line
	5850 3150 7150 3150
Wire Wire Line
	11000 1550 11000 1800
Wire Wire Line
	10500 1800 11000 1800
Wire Wire Line
	12700 1800 12700 1550
Wire Wire Line
	11550 1550 11550 1800
Connection ~ 11550 1800
Wire Wire Line
	12100 1550 12100 1800
Connection ~ 12100 1800
Wire Wire Line
	11000 1250 11000 950 
Wire Wire Line
	10500 950  11000 950 
Wire Wire Line
	12700 950  12700 1250
Wire Wire Line
	11550 750  11550 950 
Connection ~ 11550 950 
Wire Wire Line
	12100 1250 12100 950 
Connection ~ 12100 950 
Wire Wire Line
	9850 2000 9850 2200
Wire Wire Line
	9850 2200 9950 2200
Wire Wire Line
	10150 2200 10150 2350
Connection ~ 9850 2200
Wire Wire Line
	9950 2350 9950 2200
Connection ~ 9950 2200
Wire Wire Line
	10050 2350 10050 2200
Connection ~ 10050 2200
Wire Wire Line
	9850 6250 9850 6400
Wire Wire Line
	9950 6250 9950 6400
Wire Wire Line
	9850 6400 9950 6400
Connection ~ 9850 6400
Wire Wire Line
	10050 6400 10050 6250
Connection ~ 9950 6400
Wire Wire Line
	10150 6400 10150 6250
Connection ~ 10050 6400
Wire Wire Line
	10250 6400 10250 6250
Connection ~ 10150 6400
Wire Wire Line
	6750 4150 7150 4150
Connection ~ 6000 3450
Wire Wire Line
	7000 3550 7000 3250
Connection ~ 7000 3250
$Comp
L power:VDD #PWR023
U 1 1 58F73DF4
P 3400 9200
F 0 "#PWR023" H 3400 9050 50  0001 C CNN
F 1 "VDD" H 3417 9373 50  0000 C CNN
F 2 "" H 3400 9200 50  0001 C CNN
F 3 "" H 3400 9200 50  0001 C CNN
	1    3400 9200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 1250 10500 950 
Connection ~ 11000 950 
Wire Wire Line
	10500 1550 10500 1800
Connection ~ 11000 1800
Text Label 6750 5750 0    60   ~ 0
PC14
Text Label 6750 5850 0    60   ~ 0
PC15
Wire Wire Line
	4700 3900 4700 3850
$Comp
L servo-sigma-48v-rescue:C-device C30
U 1 1 5AB8EC97
P 4850 2500
F 0 "C30" H 4700 2400 50  0000 L CNN
F 1 "100nF 50V" H 4900 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4888 2350 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 4850 2500 50  0001 C CNN
F 4 "490-4779-2-ND" H 4850 2500 60  0001 C CNN "Part"
F 5 "DigiKey" H 4850 2500 60  0001 C CNN "Provider"
	1    4850 2500
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C31
U 1 1 5AB8EC98
P 4150 2500
F 0 "C31" H 4000 2400 50  0000 L CNN
F 1 "1uF 100V" H 4200 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 2350 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 4150 2500 50  0001 C CNN
F 4 "445-8887-1-ND" H 4150 2500 60  0001 C CNN "Part"
F 5 "DigiKey" H 4150 2500 60  0001 C CNN "Provider"
	1    4150 2500
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 2650 4150 2800
Wire Wire Line
	4150 2800 4850 2800
Wire Wire Line
	4850 2650 4850 2800
Connection ~ 4850 3250
Connection ~ 4850 2800
Wire Wire Line
	4150 2350 4150 2250
Wire Wire Line
	4150 2250 4850 2250
Wire Wire Line
	4850 2100 4850 2250
$Comp
L power:GND #PWR03
U 1 1 5AB8EC99
P 4850 2100
F 0 "#PWR03" H 4850 1850 50  0001 C CNN
F 1 "GND" H 4850 1950 50  0000 C CNN
F 2 "" H 4850 2100 50  0001 C CNN
F 3 "" H 4850 2100 50  0001 C CNN
	1    4850 2100
	1    0    0    1   
$EndComp
Connection ~ 4850 2250
Wire Wire Line
	5450 2800 5850 2800
Connection ~ 5850 2800
$Comp
L servo-sigma-48v-rescue:C-device C32
U 1 1 5AB8ECA0
P 4200 4250
F 0 "C32" V 4150 4350 50  0000 C CNN
F 1 "27pF/50V/1%" V 4350 4250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 4100 50  0001 C CNN
F 3 "http://search.murata.co.jp/Ceramy/image/img/A01X/partnumbering_e_01.pdf" H 4200 4250 50  0001 C CNN
F 4 "490-9719-1-ND" V 4200 4250 60  0001 C CNN "Part"
F 5 "DigiKey" V 4200 4250 60  0001 C CNN "Provider"
	1    4200 4250
	0    1    1    0   
$EndComp
NoConn ~ 7150 5150
NoConn ~ 7150 5250
$Comp
L servo-sigma-48v-rescue:C-device C15
U 1 1 5A3A53E5
P 5000 3450
F 0 "C15" V 4950 3250 50  0000 L CNN
F 1 "4.7uF 25V" V 4950 3550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5038 3300 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 5000 3450 50  0001 C CNN
F 4 "1276-2415-1-ND" H 5000 3450 60  0001 C CNN "Part"
F 5 "DigiKey" H 5000 3450 60  0001 C CNN "Provider"
	1    5000 3450
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C29
U 1 1 5A3B64F8
P 10500 1400
F 0 "C29" H 10400 1300 50  0000 L CNN
F 1 "4.7uF 25V" H 10100 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10538 1250 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 10500 1400 50  0001 C CNN
F 4 "1276-2415-1-ND" H 10500 1400 60  0001 C CNN "Part"
F 5 "DigiKey" H 10500 1400 60  0001 C CNN "Provider"
	1    10500 1400
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C28
U 1 1 5A540099
P 5450 2500
F 0 "C28" H 5300 2400 50  0000 L CNN
F 1 "1uF 100V" H 5500 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5488 2350 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 5450 2500 50  0001 C CNN
F 4 "445-8887-1-ND" H 5450 2500 60  0001 C CNN "Part"
F 5 "DigiKey" H 5450 2500 60  0001 C CNN "Provider"
	1    5450 2500
	-1   0    0    1   
$EndComp
Wire Wire Line
	4700 3850 7150 3850
Wire Wire Line
	4700 4250 5100 4250
Wire Wire Line
	11550 1800 12100 1800
Wire Wire Line
	12100 1800 12100 2000
Wire Wire Line
	12100 1800 12700 1800
Wire Wire Line
	11550 950  12100 950 
Wire Wire Line
	11550 950  11550 1250
Wire Wire Line
	12100 950  12700 950 
Wire Wire Line
	9850 2200 9850 2350
Wire Wire Line
	9950 2200 10050 2200
Wire Wire Line
	10050 2200 10150 2200
Wire Wire Line
	9850 6400 9850 6500
Wire Wire Line
	9950 6400 10050 6400
Wire Wire Line
	10050 6400 10150 6400
Wire Wire Line
	10150 6400 10250 6400
Wire Wire Line
	6000 3450 6000 3550
Wire Wire Line
	7000 3250 7150 3250
Wire Wire Line
	11000 950  11550 950 
Wire Wire Line
	11000 1800 11550 1800
Wire Wire Line
	4850 3250 7000 3250
Wire Wire Line
	4850 2800 4850 3250
Wire Wire Line
	4850 2250 4850 2350
Wire Wire Line
	5850 2800 5850 3150
Text Label 1100 1500 0    60   ~ 0
USART_TX
Wire Wire Line
	2300 1400 1100 1400
NoConn ~ 2800 1400
Text HLabel 1100 1400 0    60   Output ~ 0
EXT_5V
$Comp
L servo-sigma-48v-rescue:R-device R5
U 1 1 5AB437C6
P 7000 1400
F 0 "R5" V 7050 1550 50  0000 C CNN
F 1 "21" V 7000 1400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6930 1400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 7000 1400 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 7000 1400 60  0001 C CNN "Part"
F 5 "DigiKey" V 7000 1400 60  0001 C CNN "Provider"
	1    7000 1400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R8
U 1 1 5AB43AFD
P 7500 1100
F 0 "R8" V 7580 1100 50  0000 C CNN
F 1 "10k" V 7500 1100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7430 1100 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 7500 1100 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 7500 1100 60  0001 C CNN "Part"
F 5 "DigiKey" V 7500 1100 60  0001 C CNN "Provider"
	1    7500 1100
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C3
U 1 1 5AB43EE1
P 7500 1700
F 0 "C3" H 7350 1600 50  0000 L CNN
F 1 "100nF 50V" H 7050 1800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7538 1550 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 7500 1700 50  0001 C CNN
F 4 "490-4779-2-ND" H 7500 1700 60  0001 C CNN "Part"
F 5 "DigiKey" H 7500 1700 60  0001 C CNN "Provider"
	1    7500 1700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5AB442AF
P 7500 2000
F 0 "#PWR07" H 7500 1750 50  0001 C CNN
F 1 "GND" H 7500 1850 50  0000 C CNN
F 2 "" H 7500 2000 50  0001 C CNN
F 3 "" H 7500 2000 50  0001 C CNN
	1    7500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1400 7500 1400
Wire Wire Line
	7500 1250 7500 1400
Connection ~ 7500 1400
Wire Wire Line
	7500 1400 8000 1400
Wire Wire Line
	7500 1400 7500 1550
Wire Wire Line
	7500 1850 7500 2000
$Comp
L power:VDD #PWR06
U 1 1 5AB67E7B
P 7500 850
F 0 "#PWR06" H 7500 700 50  0001 C CNN
F 1 "VDD" H 7517 1023 50  0000 C CNN
F 2 "" H 7500 850 50  0001 C CNN
F 3 "" H 7500 850 50  0001 C CNN
	1    7500 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 950  7500 850 
Wire Wire Line
	6750 2750 6750 1400
Wire Wire Line
	6750 1400 6850 1400
$Comp
L power:VDD #PWR011
U 1 1 5AB8F23A
P 2350 2550
F 0 "#PWR011" H 2350 2400 50  0001 C CNN
F 1 "VDD" H 2367 2723 50  0000 C CNN
F 2 "" H 2350 2550 50  0001 C CNN
F 3 "" H 2350 2550 50  0001 C CNN
	1    2350 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5AB8F240
P 1200 3750
F 0 "#PWR09" H 1200 3500 50  0001 C CNN
F 1 "GND" H 1200 3600 50  0000 C CNN
F 2 "" H 1200 3750 50  0001 C CNN
F 3 "" H 1200 3750 50  0001 C CNN
	1    1200 3750
	1    0    0    -1  
$EndComp
Text Label 3000 3050 2    60   ~ 0
USER_BTN
$Comp
L servo-sigma-48v-rescue:R-device R9
U 1 1 5AB8F249
P 1750 3650
F 0 "R9" V 1830 3650 50  0000 C CNN
F 1 "100" V 1750 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1680 3650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1750 3650 50  0001 C CNN
F 4 "311-100HRCT-ND" V 1750 3650 60  0001 C CNN "Part"
F 5 "DigiKey" V 1750 3650 60  0001 C CNN "Provider"
	1    1750 3650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C4
U 1 1 5AB8F252
P 2350 3400
F 0 "C4" H 2200 3300 50  0000 L CNN
F 1 "100nF 50V" H 1900 3500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2388 3250 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2350 3400 50  0001 C CNN
F 4 "490-4779-2-ND" H 2350 3400 60  0001 C CNN "Part"
F 5 "DigiKey" H 2350 3400 60  0001 C CNN "Provider"
	1    2350 3400
	-1   0    0    1   
$EndComp
$Comp
L sigmadrone:SW_PUSH_TACKTILE2 USR1
U 1 1 5AB8F25B
P 1750 3050
F 0 "USR1" H 2100 3150 50  0000 R CNN
F 1 "WURTH_434153017835" H 2100 2850 50  0000 R CNN
F 2 "Sigmadrone:SW_SPST_WURTH_434153017835" H 1750 3050 60  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/434153017835.pdf" H 1750 3050 60  0001 C CNN
F 4 "732-10143-1-ND" H 1750 3050 60  0001 C CNN "Part"
F 5 "DigiKey" H 1750 3050 60  0001 C CNN "Provider"
	1    1750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 2550 2350 2650
Wire Wire Line
	2350 2950 2350 3050
Wire Wire Line
	1200 3050 1200 3150
Wire Wire Line
	1200 3650 1600 3650
Wire Wire Line
	2050 3050 2350 3050
Connection ~ 2350 3050
Wire Wire Line
	2350 3550 2350 3650
Wire Wire Line
	2350 3650 1900 3650
Wire Wire Line
	1450 3050 1200 3050
Connection ~ 1200 3650
Wire Wire Line
	1450 3150 1200 3150
Connection ~ 1200 3150
Wire Wire Line
	2350 3150 2050 3150
Connection ~ 2350 3150
$Comp
L servo-sigma-48v-rescue:R-device R10
U 1 1 5AB8F272
P 2350 2800
F 0 "R10" V 2430 2800 50  0000 C CNN
F 1 "10k" V 2350 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2280 2800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2350 2800 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2350 2800 60  0001 C CNN "Part"
F 5 "DigiKey" V 2350 2800 60  0001 C CNN "Provider"
	1    2350 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3050 2350 3150
Wire Wire Line
	2350 3050 3000 3050
Wire Wire Line
	1200 3650 1200 3750
Wire Wire Line
	1200 3150 1200 3650
Wire Wire Line
	2350 3150 2350 3250
Text Label 2300 4500 2    60   ~ 0
ENCODER_B_CON
Text Label 2300 4600 2    60   ~ 0
ENCODER_5V
Text Label 2300 4700 2    60   ~ 0
ENCODER_A_CON
Text Label 2300 4800 2    60   ~ 0
ENCODER_Z_CON
Text Label 2300 4900 2    60   ~ 0
ENCODER_GND
Wire Wire Line
	1400 4500 2300 4500
Wire Wire Line
	1400 4600 2300 4600
Wire Wire Line
	1400 4700 2300 4700
Wire Wire Line
	1400 4800 2300 4800
Wire Wire Line
	1400 4900 2300 4900
$Comp
L power:+5V #PWR?
U 1 1 5BB4B839
P 2300 4600
AR Path="/58BF599E/5BB4B839" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5BB4B839" Ref="#PWR0115"  Part="1" 
F 0 "#PWR0115" H 2300 4450 50  0001 C CNN
F 1 "+5V" H 2315 4773 50  0000 C CNN
F 2 "" H 2300 4600 50  0001 C CNN
F 3 "" H 2300 4600 50  0001 C CNN
	1    2300 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5BB57B1C
P 2300 4900
F 0 "#PWR0116" H 2300 4650 50  0001 C CNN
F 1 "GND" H 2300 4750 50  0000 C CNN
F 2 "" H 2300 4900 50  0001 C CNN
F 3 "" H 2300 4900 50  0001 C CNN
	1    2300 4900
	0    -1   -1   0   
$EndComp
Text Label 14650 2750 2    60   ~ 0
ENCODER_A
Text Label 14650 2850 2    60   ~ 0
ENCODER_B_DE
Wire Wire Line
	12950 4850 14950 4850
$Comp
L servo-sigma-48v-rescue:R-device R44
U 1 1 5BDC03FA
P 3050 1300
F 0 "R44" V 3100 1450 50  0000 C CNN
F 1 "21" V 3050 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2980 1300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 3050 1300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 3050 1300 60  0001 C CNN "Part"
F 5 "DigiKey" V 3050 1300 60  0001 C CNN "Provider"
	1    3050 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3200 1300 3850 1300
$Comp
L servo-sigma-48v-rescue:R-device R2
U 1 1 5BDC074C
P 2050 1300
F 0 "R2" V 2100 1450 50  0000 C CNN
F 1 "21" V 2050 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1980 1300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2050 1300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 2050 1300 60  0001 C CNN "Part"
F 5 "DigiKey" V 2050 1300 60  0001 C CNN "Provider"
	1    2050 1300
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R3
U 1 1 5BDE0108
P 2050 1500
F 0 "R3" V 2100 1650 50  0000 C CNN
F 1 "21" V 2050 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1980 1500 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2050 1500 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 2050 1500 60  0001 C CNN "Part"
F 5 "DigiKey" V 2050 1500 60  0001 C CNN "Provider"
	1    2050 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 1500 2300 1500
Wire Wire Line
	12950 4250 13350 4250
Wire Wire Line
	12950 2750 14650 2750
Wire Wire Line
	12950 2850 14650 2850
Text Label 14650 4450 2    60   ~ 0
ENCODER_Z_CON
Text Label 1100 1300 0    60   ~ 0
USART_RX
Wire Wire Line
	1100 1300 1900 1300
Text Label 3850 1300 2    60   ~ 0
SWO
$Comp
L sigmadrone:629_105_150_921 J6
U 1 1 5BEA08E5
P 1300 9700
F 0 "J6" H 1200 10350 60  0000 C CNN
F 1 "629_105_150_921" H 1250 9050 60  0000 C CNN
F 2 "Sigmadrone:WURTH_629105150921_NO_OVALS" H 1194 10487 60  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/629105150921.pdf" H 1194 10381 60  0001 C CNN
F 4 "732-5961-1-ND" H 1300 9700 50  0001 C CNN "Part"
F 5 "DigiKey" H 1300 9700 50  0001 C CNN "Provider"
	1    1300 9700
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 5BEB5E6E
P 1950 10350
F 0 "#PWR041" H 1950 10100 50  0001 C CNN
F 1 "GND" H 1950 10200 50  0000 C CNN
F 2 "" H 1950 10350 50  0001 C CNN
F 3 "" H 1950 10350 50  0001 C CNN
	1    1950 10350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 9700 1950 9700
Wire Wire Line
	1950 9700 1950 9800
Wire Wire Line
	1700 9800 1950 9800
Connection ~ 1950 9800
Wire Wire Line
	1950 9800 1950 9900
Wire Wire Line
	1700 9900 1950 9900
Connection ~ 1950 9900
Wire Wire Line
	1950 9900 1950 10000
Wire Wire Line
	1700 10000 1950 10000
Connection ~ 1950 10000
Wire Wire Line
	1950 10000 1950 10100
Wire Wire Line
	1700 10100 1950 10100
Connection ~ 1950 10100
Wire Wire Line
	1950 10100 1950 10200
Wire Wire Line
	1700 10200 1950 10200
Connection ~ 1950 10200
Wire Wire Line
	1950 10200 1950 10350
Wire Wire Line
	1700 9600 1950 9600
Wire Wire Line
	1950 9600 1950 9700
Connection ~ 1950 9700
NoConn ~ 1700 9500
NoConn ~ 1700 9200
Text Label 2750 9300 2    60   ~ 0
USB_DM
Text Label 2750 9400 2    60   ~ 0
USB_DP
Wire Wire Line
	1700 9300 1900 9300
Wire Wire Line
	1700 9400 1900 9400
$Comp
L servo-sigma-48v-rescue:R-device R47
U 1 1 5BF37336
P 2050 9300
F 0 "R47" V 2100 9450 50  0000 C CNN
F 1 "21" V 2050 9300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1980 9300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2050 9300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 2050 9300 60  0001 C CNN "Part"
F 5 "DigiKey" V 2050 9300 60  0001 C CNN "Provider"
	1    2050 9300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 9300 2750 9300
$Comp
L servo-sigma-48v-rescue:R-device R50
U 1 1 5BF374B9
P 2050 9400
F 0 "R50" V 2100 9550 50  0000 C CNN
F 1 "21" V 2050 9400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1980 9400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2050 9400 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 2050 9400 60  0001 C CNN "Part"
F 5 "DigiKey" V 2050 9400 60  0001 C CNN "Provider"
	1    2050 9400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 9400 2750 9400
Text Label 14650 3850 2    60   ~ 0
USB_DM
Text Label 14650 3950 2    60   ~ 0
USB_DP_USART1_RTS
Wire Wire Line
	12950 3850 14650 3850
Wire Wire Line
	12950 3950 14650 3950
Text HLabel 6450 5850 0    60   Output ~ 0
AUX_L
Text HLabel 6450 5750 0    60   Output ~ 0
AUX_H
Wire Wire Line
	12950 3550 14950 3550
Wire Wire Line
	12950 3650 14950 3650
Wire Wire Line
	12950 3750 14950 3750
Wire Wire Line
	12950 5650 14950 5650
Wire Wire Line
	12950 5750 14950 5750
Wire Wire Line
	12950 5850 14950 5850
Text Label 14650 5350 2    60   ~ 0
LED_WARN
Text Label 14650 5450 2    60   ~ 0
LED_STATUS
NoConn ~ 7150 5650
Wire Wire Line
	12950 5250 13350 5250
NoConn ~ 12950 5250
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 5C6213BC
P 4750 9750
F 0 "J8" H 4750 9950 50  0000 C CNN
F 1 "Conn_01x03" H 4750 9550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4750 9750 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 4750 9750 50  0001 C CNN
F 4 "WM4901-ND" H 4750 9750 50  0001 C CNN "Part"
F 5 "DigiKey" H 4750 9750 50  0001 C CNN "Provider"
	1    4750 9750
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C645EB0
P 5650 9750
AR Path="/58BF599E/5C645EB0" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5C645EB0" Ref="#PWR075"  Part="1" 
F 0 "#PWR075" H 5650 9600 50  0001 C CNN
F 1 "+5V" V 5650 9950 50  0000 C CNN
F 2 "" H 5650 9750 50  0001 C CNN
F 3 "" H 5650 9750 50  0001 C CNN
	1    5650 9750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR076
U 1 1 5C645EB6
P 5650 9850
F 0 "#PWR076" H 5650 9600 50  0001 C CNN
F 1 "GND" V 5650 9650 50  0000 C CNN
F 2 "" H 5650 9850 50  0001 C CNN
F 3 "" H 5650 9850 50  0001 C CNN
	1    5650 9850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4950 9850 5650 9850
Wire Wire Line
	4950 9750 5650 9750
Text Label 5900 9650 2    60   ~ 0
SW1
Wire Wire Line
	4950 9650 5350 9650
$Comp
L servo-sigma-48v-rescue:R-device R20
U 1 1 5C69BA3F
P 5350 10500
F 0 "R20" V 5430 10500 50  0000 C CNN
F 1 "100" V 5350 10500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5280 10500 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5350 10500 50  0001 C CNN
F 4 "311-100HRCT-ND" V 5350 10500 60  0001 C CNN "Part"
F 5 "DigiKey" V 5350 10500 60  0001 C CNN "Provider"
	1    5350 10500
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C51
U 1 1 5C69BA48
P 5350 10100
F 0 "C51" H 5200 10000 50  0000 L CNN
F 1 "100nF 50V" H 4900 10200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5388 9950 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 5350 10100 50  0001 C CNN
F 4 "490-4779-2-ND" H 5350 10100 60  0001 C CNN "Part"
F 5 "DigiKey" H 5350 10100 60  0001 C CNN "Provider"
	1    5350 10100
	-1   0    0    1   
$EndComp
Wire Wire Line
	5350 10250 5350 10350
Wire Wire Line
	5350 9950 5350 9650
Connection ~ 5350 9650
Wire Wire Line
	5350 9650 5900 9650
$Comp
L power:GND #PWR073
U 1 1 5C70ECD7
P 5350 10750
F 0 "#PWR073" H 5350 10500 50  0001 C CNN
F 1 "GND" H 5350 10600 50  0000 C CNN
F 2 "" H 5350 10750 50  0001 C CNN
F 3 "" H 5350 10750 50  0001 C CNN
	1    5350 10750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 10750 5350 10650
$Comp
L servo-sigma-48v-rescue:R-device R17
U 1 1 5C71BDB8
P 5350 9350
F 0 "R17" V 5430 9350 50  0000 C CNN
F 1 "10k" V 5350 9350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5280 9350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5350 9350 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 5350 9350 60  0001 C CNN "Part"
F 5 "DigiKey" V 5350 9350 60  0001 C CNN "Provider"
	1    5350 9350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C71BEBC
P 5350 9050
AR Path="/58BF599E/5C71BEBC" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5C71BEBC" Ref="#PWR072"  Part="1" 
F 0 "#PWR072" H 5350 8900 50  0001 C CNN
F 1 "+5V" V 5350 9250 50  0000 C CNN
F 2 "" H 5350 9050 50  0001 C CNN
F 3 "" H 5350 9050 50  0001 C CNN
	1    5350 9050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 9500 5350 9650
Wire Wire Line
	5350 9050 5350 9200
$Comp
L power:+5V #PWR?
U 1 1 5C769B40
P 7500 9750
AR Path="/58BF599E/5C769B40" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5C769B40" Ref="#PWR083"  Part="1" 
F 0 "#PWR083" H 7500 9600 50  0001 C CNN
F 1 "+5V" V 7500 9950 50  0000 C CNN
F 2 "" H 7500 9750 50  0001 C CNN
F 3 "" H 7500 9750 50  0001 C CNN
	1    7500 9750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR084
U 1 1 5C769B46
P 7500 9850
F 0 "#PWR084" H 7500 9600 50  0001 C CNN
F 1 "GND" V 7500 9650 50  0000 C CNN
F 2 "" H 7500 9850 50  0001 C CNN
F 3 "" H 7500 9850 50  0001 C CNN
	1    7500 9850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6800 9850 7500 9850
Wire Wire Line
	6800 9750 7500 9750
Text Label 7750 9650 2    60   ~ 0
SW2
Wire Wire Line
	6800 9650 7200 9650
$Comp
L servo-sigma-48v-rescue:R-device R30
U 1 1 5C769B52
P 7200 10500
F 0 "R30" V 7280 10500 50  0000 C CNN
F 1 "100" V 7200 10500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7130 10500 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 7200 10500 50  0001 C CNN
F 4 "311-100HRCT-ND" V 7200 10500 60  0001 C CNN "Part"
F 5 "DigiKey" V 7200 10500 60  0001 C CNN "Provider"
	1    7200 10500
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C52
U 1 1 5C769B5B
P 7200 10100
F 0 "C52" H 7050 10000 50  0000 L CNN
F 1 "100nF 50V" H 6750 10200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7238 9950 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 7200 10100 50  0001 C CNN
F 4 "490-4779-2-ND" H 7200 10100 60  0001 C CNN "Part"
F 5 "DigiKey" H 7200 10100 60  0001 C CNN "Provider"
	1    7200 10100
	-1   0    0    1   
$EndComp
Wire Wire Line
	7200 10250 7200 10350
Wire Wire Line
	7200 9950 7200 9650
Connection ~ 7200 9650
Wire Wire Line
	7200 9650 7750 9650
$Comp
L power:GND #PWR082
U 1 1 5C769B66
P 7200 10750
F 0 "#PWR082" H 7200 10500 50  0001 C CNN
F 1 "GND" H 7200 10600 50  0000 C CNN
F 2 "" H 7200 10750 50  0001 C CNN
F 3 "" H 7200 10750 50  0001 C CNN
	1    7200 10750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 10750 7200 10650
$Comp
L servo-sigma-48v-rescue:R-device R28
U 1 1 5C769B6F
P 7200 9350
F 0 "R28" V 7280 9350 50  0000 C CNN
F 1 "10k" V 7200 9350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7130 9350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 7200 9350 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 7200 9350 60  0001 C CNN "Part"
F 5 "DigiKey" V 7200 9350 60  0001 C CNN "Provider"
	1    7200 9350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C769B76
P 7200 9050
AR Path="/58BF599E/5C769B76" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5C769B76" Ref="#PWR081"  Part="1" 
F 0 "#PWR081" H 7200 8900 50  0001 C CNN
F 1 "+5V" V 7200 9250 50  0000 C CNN
F 2 "" H 7200 9050 50  0001 C CNN
F 3 "" H 7200 9050 50  0001 C CNN
	1    7200 9050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 9500 7200 9650
Wire Wire Line
	7200 9050 7200 9200
Wire Wire Line
	12950 4650 14650 4650
Wire Wire Line
	12950 4950 13350 4950
NoConn ~ 12950 4950
Text Label 14650 4550 2    60   ~ 0
SW1
Text Label 14650 4650 2    60   ~ 0
SW2
Wire Wire Line
	12950 4550 14650 4550
Wire Wire Line
	12950 4450 14650 4450
Wire Wire Line
	6450 5750 7150 5750
Wire Wire Line
	6450 5850 7150 5850
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 5C86EEED
P 6600 9750
F 0 "J9" H 6600 9950 50  0000 C CNN
F 1 "Conn_01x03" H 6600 9550 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 6600 9750 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 6600 9750 50  0001 C CNN
F 4 "WM4901-ND" H 6600 9750 50  0001 C CNN "Part"
F 5 "DigiKey" H 6600 9750 50  0001 C CNN "Provider"
	1    6600 9750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	12950 3250 13350 3250
Wire Wire Line
	12950 3150 13350 3150
Text HLabel 14950 3350 2    60   Input ~ 0
IR_FB
Wire Wire Line
	12950 3350 14950 3350
Text HLabel 5950 4850 0    60   Input ~ 0
IC_FB
Wire Wire Line
	5950 4850 7150 4850
Text HLabel 5950 4650 0    60   Input ~ 0
IA_FB
Text HLabel 5950 4750 0    60   Input ~ 0
IB_FB
Wire Wire Line
	5950 4750 7150 4750
Wire Wire Line
	5950 4650 7150 4650
Text HLabel 5950 4350 0    60   Input ~ 0
SENSE_A
Text HLabel 5950 4450 0    60   Input ~ 0
SENSE_B
Text HLabel 5950 4550 0    60   Input ~ 0
SENSE_C
Wire Wire Line
	5950 4350 7150 4350
Wire Wire Line
	5950 4450 7150 4450
Wire Wire Line
	5950 4550 7150 4550
Text Label 14650 5550 2    60   ~ 0
USER_BTN
Wire Wire Line
	12950 5350 14650 5350
Wire Wire Line
	12950 5450 14650 5450
Wire Wire Line
	12950 5550 14650 5550
Text HLabel 14950 3450 2    60   Input ~ 0
VBAT_ADC
Wire Wire Line
	12950 3450 14950 3450
Text Label 14650 2950 2    60   ~ 0
ENCODER_DI
Text Label 14650 3050 2    60   ~ 0
ENCODER_RO
Wire Wire Line
	12950 2950 14650 2950
Wire Wire Line
	12950 3050 14650 3050
Text Label 14650 5050 2    60   ~ 0
USART1_TX
NoConn ~ 12950 4250
Text Label 14650 5150 2    60   ~ 0
USART1_RX
Text Label 14650 4750 2    60   ~ 0
SWO
Wire Wire Line
	12950 4750 14650 4750
Wire Wire Line
	12950 5050 14650 5050
Wire Wire Line
	12950 5150 14650 5150
Text Label 8500 9200 0    60   ~ 0
ENCODER_B_CON
Text Label 8500 9100 0    60   ~ 0
ENCODER_DE
$Comp
L Interface_UART:MAX3485 U12
U 1 1 5CB68FA7
P 13000 7850
AR Path="/58BE2779/5CB68FA7" Ref="U12"  Part="1" 
AR Path="/5CA7E4B7/5CB68FA7" Ref="U?"  Part="1" 
F 0 "U12" H 13100 8300 50  0000 C CNN
F 1 "SN65HVD75DGKR" H 12600 7300 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 13000 7150 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX3483-MAX3491.pdf" H 13000 7900 50  0001 C CNN
F 4 "296-35070-1-ND" H 13000 7850 50  0001 C CNN "Part"
F 5 "DigiKey" H 13000 7850 50  0001 C CNN "Provider"
	1    13000 7850
	1    0    0    -1  
$EndComp
Text Label 11150 10350 2    60   ~ 0
USART_TX
Text Label 11150 10450 2    60   ~ 0
RS485_DI
Text Label 11150 10550 2    60   ~ 0
USART_RX
Text Label 11150 10650 2    60   ~ 0
RS485_RO
Text Label 11150 10750 2    60   ~ 0
RS485_DE
Text Label 11150 10850 2    60   ~ 0
USB_DP
Wire Wire Line
	10250 10350 11150 10350
Wire Wire Line
	10250 10450 11150 10450
Wire Wire Line
	10250 10550 11150 10550
Wire Wire Line
	10250 10650 11150 10650
Wire Wire Line
	10250 10750 11150 10750
Wire Wire Line
	10250 10850 11150 10850
Text Label 8500 10350 0    60   ~ 0
USART1_TX
Wire Wire Line
	8500 10350 9550 10350
Wire Wire Line
	9650 10450 9550 10450
Wire Wire Line
	9550 10450 9550 10350
Connection ~ 9550 10350
Wire Wire Line
	9550 10350 9650 10350
Text Label 8500 10550 0    60   ~ 0
USART1_RX
Wire Wire Line
	8500 10550 9550 10550
Wire Wire Line
	9550 10550 9550 10650
Wire Wire Line
	9550 10650 9650 10650
Connection ~ 9550 10550
Wire Wire Line
	9550 10550 9650 10550
Wire Wire Line
	8500 10750 9550 10750
Wire Wire Line
	9650 10850 9550 10850
Wire Wire Line
	9550 10850 9550 10750
Connection ~ 9550 10750
Wire Wire Line
	9550 10750 9650 10750
Text Label 8500 10750 0    60   ~ 0
USB_DP_USART1_RTS
$Comp
L power:GND #PWR087
U 1 1 5CCDC2CF
P 13000 8600
F 0 "#PWR087" H 13000 8350 50  0001 C CNN
F 1 "GND" H 13000 8450 50  0000 C CNN
F 2 "" H 13000 8600 50  0001 C CNN
F 3 "" H 13000 8600 50  0001 C CNN
	1    13000 8600
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR086
U 1 1 5CCDC5C6
P 13000 6850
F 0 "#PWR086" H 13000 6700 50  0001 C CNN
F 1 "VDD" H 13017 7023 50  0000 C CNN
F 2 "" H 13000 6850 50  0001 C CNN
F 3 "" H 13000 6850 50  0001 C CNN
	1    13000 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	13000 7350 13000 7000
Wire Wire Line
	13000 8450 13000 8600
Text Label 11450 7750 0    60   ~ 0
RS485_RO
Text Label 11450 8050 0    60   ~ 0
RS485_DI
Text Label 11450 7950 0    60   ~ 0
RS485_DE
Wire Wire Line
	11450 7750 12100 7750
Wire Wire Line
	11450 7950 12100 7950
Wire Wire Line
	11450 8050 12600 8050
Wire Wire Line
	12600 7850 12100 7850
Wire Wire Line
	12100 7850 12100 7950
Connection ~ 12100 7950
Wire Wire Line
	12100 7950 12600 7950
$Comp
L servo-sigma-48v-rescue:C-device C8
U 1 1 5CD3A126
P 12450 7250
F 0 "C8" H 12300 7150 50  0000 L CNN
F 1 "100nF 50V" H 12000 7350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12488 7100 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 12450 7250 50  0001 C CNN
F 4 "490-4779-2-ND" H 12450 7250 60  0001 C CNN "Part"
F 5 "DigiKey" H 12450 7250 60  0001 C CNN "Provider"
	1    12450 7250
	-1   0    0    1   
$EndComp
Wire Wire Line
	12450 7100 12450 7000
Wire Wire Line
	12450 7000 13000 7000
Connection ~ 13000 7000
Wire Wire Line
	13000 7000 13000 6850
$Comp
L power:GND #PWR085
U 1 1 5CD69F12
P 12450 7450
F 0 "#PWR085" H 12450 7200 50  0001 C CNN
F 1 "GND" H 12450 7300 50  0000 C CNN
F 2 "" H 12450 7450 50  0001 C CNN
F 3 "" H 12450 7450 50  0001 C CNN
	1    12450 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	12450 7450 12450 7400
$Comp
L servo-sigma-48v-rescue:R-device R84
U 1 1 5CD9AA3E
P 12100 7250
F 0 "R84" V 12180 7250 50  0000 C CNN
F 1 "10k" V 12100 7250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12030 7250 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12100 7250 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 12100 7250 60  0001 C CNN "Part"
F 5 "DigiKey" V 12100 7250 60  0001 C CNN "Provider"
	1    12100 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	12100 7100 12100 7000
Wire Wire Line
	12100 7000 12450 7000
Connection ~ 12450 7000
Wire Wire Line
	12100 7400 12100 7750
Connection ~ 12100 7750
Wire Wire Line
	12100 7750 12600 7750
$Comp
L servo-sigma-48v-rescue:R-device R85
U 1 1 5CDBBD56
P 12100 8350
F 0 "R85" V 12180 8350 50  0000 C CNN
F 1 "10k" V 12100 8350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12030 8350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12100 8350 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 12100 8350 60  0001 C CNN "Part"
F 5 "DigiKey" V 12100 8350 60  0001 C CNN "Provider"
	1    12100 8350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR063
U 1 1 5CDCC8E4
P 12100 8600
F 0 "#PWR063" H 12100 8350 50  0001 C CNN
F 1 "GND" H 12100 8450 50  0000 C CNN
F 2 "" H 12100 8600 50  0001 C CNN
F 3 "" H 12100 8600 50  0001 C CNN
	1    12100 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	12100 7950 12100 8200
Wire Wire Line
	12100 8500 12100 8600
Wire Wire Line
	13400 7750 13650 7750
Wire Wire Line
	13400 8050 13650 8050
$Comp
L servo-sigma-48v-rescue:R-device R88
U 1 1 5CEA24A4
P 14100 8550
F 0 "R88" V 14180 8550 50  0000 C CNN
F 1 "100" V 14100 8550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14030 8550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 14100 8550 50  0001 C CNN
F 4 "311-100HRCT-ND" V 14100 8550 60  0001 C CNN "Part"
F 5 "DigiKey" V 14100 8550 60  0001 C CNN "Provider"
	1    14100 8550
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R89
U 1 1 5CEA2A82
P 14100 9000
F 0 "R89" V 14150 9150 50  0000 C CNN
F 1 "21" V 14100 9000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14030 9000 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 14100 9000 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 14100 9000 60  0001 C CNN "Part"
F 5 "DigiKey" V 14100 9000 60  0001 C CNN "Provider"
	1    14100 9000
	1    0    0    -1  
$EndComp
Wire Wire Line
	14100 8700 14100 8850
Wire Wire Line
	14100 8400 14100 7750
Connection ~ 14100 7750
Wire Wire Line
	14100 9150 14100 9250
Wire Wire Line
	14100 9250 15000 9250
Text Label 15000 9250 2    60   ~ 0
B_TERM_R120
Text Label 15000 9050 2    60   ~ 0
A_TERM_R120
Wire Wire Line
	15000 9050 14300 9050
Wire Wire Line
	14300 9050 14300 8050
Connection ~ 14300 8050
Text Label 8500 10950 0    60   ~ 0
B_TERM_R120
Text Label 11150 10950 2    60   ~ 0
A_TERM_R120
Wire Wire Line
	8500 10950 9650 10950
Wire Wire Line
	10250 10950 11150 10950
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5CF4DCAA
P 13650 7400
AR Path="/58BF599E/5CF4DCAA" Ref="R?"  Part="1" 
AR Path="/58BE2779/5CF4DCAA" Ref="R86"  Part="1" 
F 0 "R86" V 13730 7400 50  0000 C CNN
F 1 "4.7k" V 13650 7400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13580 7400 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 13650 7400 50  0001 C CNN
F 4 "311-2606-2-ND" V 13650 7400 60  0001 C CNN "Part"
F 5 "DigiKey" V 13650 7400 60  0001 C CNN "Provider"
	1    13650 7400
	-1   0    0    1   
$EndComp
Wire Wire Line
	13000 7000 13650 7000
Wire Wire Line
	13650 7000 13650 7250
Wire Wire Line
	13650 7550 13650 7750
Connection ~ 13650 7750
Wire Wire Line
	13650 7750 14100 7750
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5CF86600
P 13650 8350
AR Path="/58BF599E/5CF86600" Ref="R?"  Part="1" 
AR Path="/58BE2779/5CF86600" Ref="R87"  Part="1" 
F 0 "R87" V 13730 8350 50  0000 C CNN
F 1 "4.7k" V 13650 8350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13580 8350 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 13650 8350 50  0001 C CNN
F 4 "311-2606-2-ND" V 13650 8350 60  0001 C CNN "Part"
F 5 "DigiKey" V 13650 8350 60  0001 C CNN "Provider"
	1    13650 8350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR088
U 1 1 5CF866A0
P 13650 8600
F 0 "#PWR088" H 13650 8350 50  0001 C CNN
F 1 "GND" H 13650 8450 50  0000 C CNN
F 2 "" H 13650 8600 50  0001 C CNN
F 3 "" H 13650 8600 50  0001 C CNN
	1    13650 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	13650 8200 13650 8050
Connection ~ 13650 8050
Wire Wire Line
	13650 8050 14300 8050
Wire Wire Line
	13650 8500 13650 8600
$Comp
L Connector_Generic:Conn_01x03 J10
U 1 1 5CFADD45
P 14900 8600
F 0 "J10" H 14900 8800 50  0000 C CNN
F 1 "Conn_01x03" H 14900 8400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 14900 8600 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 14900 8600 50  0001 C CNN
F 4 "WM4901-ND" H 14900 8600 50  0001 C CNN "Part"
F 5 "DigiKey" H 14900 8600 50  0001 C CNN "Provider"
	1    14900 8600
	0    -1   1    0   
$EndComp
Wire Wire Line
	14800 8400 14800 8050
Wire Wire Line
	14300 8050 14800 8050
Wire Wire Line
	14900 7750 14900 8400
Wire Wire Line
	14100 7750 14900 7750
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 5D05C4BC
P 15400 8600
F 0 "J11" H 15400 8800 50  0000 C CNN
F 1 "Conn_01x03" H 15400 8400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 15400 8600 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 15400 8600 50  0001 C CNN
F 4 "WM4901-ND" H 15400 8600 50  0001 C CNN "Part"
F 5 "DigiKey" H 15400 8600 50  0001 C CNN "Provider"
	1    15400 8600
	0    -1   1    0   
$EndComp
Wire Wire Line
	14800 8050 15300 8050
Wire Wire Line
	15300 8050 15300 8400
Connection ~ 14800 8050
Wire Wire Line
	14900 7750 15400 7750
Wire Wire Line
	15400 7750 15400 8400
Connection ~ 14900 7750
$Comp
L power:GND #PWR089
U 1 1 5D0BC76A
P 15000 8300
F 0 "#PWR089" H 15000 8050 50  0001 C CNN
F 1 "GND" H 15000 8150 50  0000 C CNN
F 2 "" H 15000 8300 50  0001 C CNN
F 3 "" H 15000 8300 50  0001 C CNN
	1    15000 8300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR090
U 1 1 5D0BC84D
P 15500 8300
F 0 "#PWR090" H 15500 8050 50  0001 C CNN
F 1 "GND" H 15500 8150 50  0000 C CNN
F 2 "" H 15500 8300 50  0001 C CNN
F 3 "" H 15500 8300 50  0001 C CNN
	1    15500 8300
	-1   0    0    1   
$EndComp
Wire Wire Line
	15000 8400 15000 8300
Wire Wire Line
	15500 8400 15500 8300
$Comp
L Interface_UART:MAX3485 U3
U 1 1 5D0E51D6
P 2850 7400
AR Path="/58BE2779/5D0E51D6" Ref="U3"  Part="1" 
AR Path="/5CA7E4B7/5D0E51D6" Ref="U?"  Part="1" 
F 0 "U3" H 2950 7850 50  0000 C CNN
F 1 "SN65HVD75DGKR" H 2450 6850 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2850 6700 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX3483-MAX3491.pdf" H 2850 7450 50  0001 C CNN
F 4 "296-35070-1-ND" H 2850 7400 50  0001 C CNN "Part"
F 5 "DigiKey" H 2850 7400 50  0001 C CNN "Provider"
	1    2850 7400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR050
U 1 1 5D0E51DD
P 2850 8150
F 0 "#PWR050" H 2850 7900 50  0001 C CNN
F 1 "GND" H 2850 8000 50  0000 C CNN
F 2 "" H 2850 8150 50  0001 C CNN
F 3 "" H 2850 8150 50  0001 C CNN
	1    2850 8150
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR049
U 1 1 5D0E51E3
P 2850 6400
F 0 "#PWR049" H 2850 6250 50  0001 C CNN
F 1 "VDD" H 2867 6573 50  0000 C CNN
F 2 "" H 2850 6400 50  0001 C CNN
F 3 "" H 2850 6400 50  0001 C CNN
	1    2850 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 6900 2850 6550
Wire Wire Line
	2850 8000 2850 8150
Wire Wire Line
	1300 7300 1950 7300
Wire Wire Line
	1300 7500 1950 7500
Wire Wire Line
	1300 7600 2450 7600
Wire Wire Line
	2450 7400 1950 7400
Wire Wire Line
	1950 7400 1950 7500
Connection ~ 1950 7500
Wire Wire Line
	1950 7500 2450 7500
$Comp
L servo-sigma-48v-rescue:C-device C5
U 1 1 5D0E51F7
P 2300 6800
F 0 "C5" H 2150 6700 50  0000 L CNN
F 1 "100nF 50V" H 1850 6900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2338 6650 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2300 6800 50  0001 C CNN
F 4 "490-4779-2-ND" H 2300 6800 60  0001 C CNN "Part"
F 5 "DigiKey" H 2300 6800 60  0001 C CNN "Provider"
	1    2300 6800
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 6650 2300 6550
Wire Wire Line
	2300 6550 2850 6550
Connection ~ 2850 6550
Wire Wire Line
	2850 6550 2850 6400
$Comp
L power:GND #PWR046
U 1 1 5D0E5202
P 2300 7000
F 0 "#PWR046" H 2300 6750 50  0001 C CNN
F 1 "GND" H 2300 6850 50  0000 C CNN
F 2 "" H 2300 7000 50  0001 C CNN
F 3 "" H 2300 7000 50  0001 C CNN
	1    2300 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 7000 2300 6950
$Comp
L servo-sigma-48v-rescue:R-device R14
U 1 1 5D0E520B
P 1950 6800
F 0 "R14" V 2030 6800 50  0000 C CNN
F 1 "10k" V 1950 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1880 6800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1950 6800 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 1950 6800 60  0001 C CNN "Part"
F 5 "DigiKey" V 1950 6800 60  0001 C CNN "Provider"
	1    1950 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 6650 1950 6550
Wire Wire Line
	1950 6550 2300 6550
Connection ~ 2300 6550
Wire Wire Line
	1950 6950 1950 7300
Connection ~ 1950 7300
Wire Wire Line
	1950 7300 2450 7300
$Comp
L servo-sigma-48v-rescue:R-device R15
U 1 1 5D0E521A
P 1950 7900
F 0 "R15" V 2030 7900 50  0000 C CNN
F 1 "10k" V 1950 7900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1880 7900 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1950 7900 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 1950 7900 60  0001 C CNN "Part"
F 5 "DigiKey" V 1950 7900 60  0001 C CNN "Provider"
	1    1950 7900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5D0E5221
P 1950 8150
F 0 "#PWR044" H 1950 7900 50  0001 C CNN
F 1 "GND" H 1950 8000 50  0000 C CNN
F 2 "" H 1950 8150 50  0001 C CNN
F 3 "" H 1950 8150 50  0001 C CNN
	1    1950 8150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 7500 1950 7750
Wire Wire Line
	1950 8050 1950 8150
Wire Wire Line
	3250 7300 3500 7300
Wire Wire Line
	3250 7600 3500 7600
$Comp
L servo-sigma-48v-rescue:R-device R81
U 1 1 5D0E522D
P 3950 7800
F 0 "R81" V 4030 7800 50  0000 C CNN
F 1 "100" V 3950 7800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 7800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 3950 7800 50  0001 C CNN
F 4 "311-100HRCT-ND" V 3950 7800 60  0001 C CNN "Part"
F 5 "DigiKey" V 3950 7800 60  0001 C CNN "Provider"
	1    3950 7800
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R83
U 1 1 5D0E5236
P 3950 8150
F 0 "R83" V 4000 8300 50  0000 C CNN
F 1 "21" V 3950 8150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 8150 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 3950 8150 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 3950 8150 60  0001 C CNN "Part"
F 5 "DigiKey" V 3950 8150 60  0001 C CNN "Provider"
	1    3950 8150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 7950 3950 8000
Wire Wire Line
	3950 7650 3950 7300
Connection ~ 3950 7300
Wire Wire Line
	4150 8350 4150 7600
Connection ~ 4150 7600
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5D0E5249
P 3500 6800
AR Path="/58BF599E/5D0E5249" Ref="R?"  Part="1" 
AR Path="/58BE2779/5D0E5249" Ref="R38"  Part="1" 
F 0 "R38" V 3580 6800 50  0000 C CNN
F 1 "4.7k" V 3500 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 6800 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3500 6800 50  0001 C CNN
F 4 "311-2606-2-ND" V 3500 6800 60  0001 C CNN "Part"
F 5 "DigiKey" V 3500 6800 60  0001 C CNN "Provider"
	1    3500 6800
	-1   0    0    1   
$EndComp
Wire Wire Line
	2850 6550 3500 6550
Wire Wire Line
	3500 6550 3500 6650
Wire Wire Line
	3500 6950 3500 7300
Connection ~ 3500 7300
Wire Wire Line
	3500 7300 3950 7300
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5D0E5257
P 3500 7850
AR Path="/58BF599E/5D0E5257" Ref="R?"  Part="1" 
AR Path="/58BE2779/5D0E5257" Ref="R40"  Part="1" 
F 0 "R40" V 3580 7850 50  0000 C CNN
F 1 "4.7k" V 3500 7850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 7850 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3500 7850 50  0001 C CNN
F 4 "311-2606-2-ND" V 3500 7850 60  0001 C CNN "Part"
F 5 "DigiKey" V 3500 7850 60  0001 C CNN "Provider"
	1    3500 7850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR052
U 1 1 5D0E525E
P 3500 8150
F 0 "#PWR052" H 3500 7900 50  0001 C CNN
F 1 "GND" H 3500 8000 50  0000 C CNN
F 2 "" H 3500 8150 50  0001 C CNN
F 3 "" H 3500 8150 50  0001 C CNN
	1    3500 8150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 7700 3500 7600
Connection ~ 3500 7600
Wire Wire Line
	3500 7600 4150 7600
Wire Wire Line
	3500 8000 3500 8150
Wire Wire Line
	3950 7300 5100 7300
Wire Wire Line
	4150 8350 3950 8350
Wire Wire Line
	3950 8300 3950 8350
$Comp
L Switch:SW_DIP_x08 SW1
U 1 1 5D20434C
P 9950 9500
F 0 "SW1" H 9950 10167 50  0000 C CNN
F 1 "SW_DIP_x08" H 9950 10076 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx08_Slide_Copal_CHS-08A_W5.08mm_P1.27mm_JPin" H 9950 9500 50  0001 C CNN
F 3 "http://www.grayhill.com/assets/1/7/DIP_Series_97.pdf" H 9950 9500 50  0001 C CNN
F 4 "GH7645CT-ND" H 9950 9500 50  0001 C CNN "Part"
F 5 "DigiKey" H 9950 9500 50  0001 C CNN "Provider"
	1    9950 9500
	1    0    0    -1  
$EndComp
Text Label 8500 9400 0    60   ~ 0
ENCODER_A_CON
Wire Wire Line
	8500 9400 9550 9400
Wire Wire Line
	9650 9500 9550 9500
Wire Wire Line
	9550 9500 9550 9400
Connection ~ 9550 9400
Wire Wire Line
	9550 9400 9650 9400
Text Label 1300 7600 0    60   ~ 0
ENCODER_DI
Text Label 1300 7300 0    60   ~ 0
ENCODER_RO
Text Label 1300 7500 0    60   ~ 0
ENCODER_DE
Text Label 11150 9200 2    60   ~ 0
ENCODER_B_DE
Wire Wire Line
	9650 9100 8500 9100
Wire Wire Line
	8500 9200 9550 9200
Text Label 11150 9400 2    60   ~ 0
ENCODER_A
Wire Wire Line
	10250 9400 11150 9400
Text Label 5100 7300 2    60   ~ 0
ENCODER_RS485_B
Text Label 5100 7600 2    60   ~ 0
ENCODER_RS485_A
Wire Wire Line
	4150 7600 5100 7600
Text Label 11150 9300 2    60   ~ 0
ENCODER_RS485_B
Text Label 11150 9500 2    60   ~ 0
ENCODER_RS485_A
Wire Wire Line
	10250 9500 11150 9500
Wire Wire Line
	10250 9300 11150 9300
$Comp
L Connector_Generic:Conn_01x05 J5
U 1 1 5D40B2A6
P 1200 4700
F 0 "J5" H 1200 5000 50  0000 C CNN
F 1 "Conn_01x05" H 1200 4400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Horizontal" H 1200 4700 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 1200 4700 50  0001 C CNN
F 4 "WM4903-ND" H 1200 4700 50  0001 C CNN "Part"
F 5 "DigiKey" H 1200 4700 50  0001 C CNN "Provider"
	1    1200 4700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6750 4950 7150 4950
Wire Wire Line
	6750 5050 7150 5050
NoConn ~ 7150 5050
NoConn ~ 7150 4950
Text Label 6750 2950 0    60   ~ 0
BOOT0
Text Label 8500 11050 0    60   ~ 0
BOOT0
$Comp
L power:VDD #PWR057
U 1 1 5D4B958F
P 11050 11050
F 0 "#PWR057" H 11050 10900 50  0001 C CNN
F 1 "VDD" H 11067 11223 50  0000 C CNN
F 2 "" H 11050 11050 50  0001 C CNN
F 3 "" H 11050 11050 50  0001 C CNN
	1    11050 11050
	0    1    1    0   
$EndComp
Wire Wire Line
	10250 11050 11050 11050
Wire Wire Line
	8500 11050 9650 11050
NoConn ~ 12950 3150
NoConn ~ 12950 3250
NoConn ~ 10250 9700
NoConn ~ 10250 9800
NoConn ~ 9650 9800
NoConn ~ 9650 9700
Wire Wire Line
	9650 9300 9550 9300
Wire Wire Line
	9550 9300 9550 9200
Connection ~ 9550 9200
Wire Wire Line
	9550 9200 9650 9200
Wire Wire Line
	10250 9200 10350 9200
Wire Wire Line
	10250 9100 10350 9100
Wire Wire Line
	10350 9100 10350 9200
Connection ~ 10350 9200
Wire Wire Line
	10350 9200 11150 9200
NoConn ~ 9650 9600
NoConn ~ 10250 9600
$Comp
L Switch:SW_DIP_x08 SW2
U 1 1 5C631581
P 9950 10750
F 0 "SW2" H 9950 11417 50  0000 C CNN
F 1 "SW_DIP_x08" H 9950 11326 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx08_Slide_Copal_CHS-08A_W5.08mm_P1.27mm_JPin" H 9950 10750 50  0001 C CNN
F 3 "http://www.grayhill.com/assets/1/7/DIP_Series_97.pdf" H 9950 10750 50  0001 C CNN
F 4 "GH7645CT-ND" H 9950 10750 50  0001 C CNN "Part"
F 5 "DigiKey" H 9950 10750 50  0001 C CNN "Provider"
	1    9950 10750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
