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
Text Label 7400 9000 0    60   ~ 0
VBAT_ADC
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
Text Label 5600 9050 3    60   ~ 0
LED_WARN
Text Label 5150 10950 1    60   ~ 0
LED_STATUS
$Comp
L servo-sigma-48v-rescue:LED-device D1
U 1 1 5AB8EC78
P 5150 9800
F 0 "D1" H 5150 9900 50  0000 C CNN
F 1 "GREEN" H 5150 9700 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5150 9800 50  0001 C CNN
F 3 "http://katalog.we-online.de/led/datasheet/150060GS75000.pdf" H 5150 9800 50  0001 C CNN
F 4 "732-4971-1-ND" H 5150 9800 60  0001 C CNN "Part"
F 5 "DigiKey" H 5150 9800 60  0001 C CNN "Provider"
	1    5150 9800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:LED-device D4
U 1 1 5AB8EC79
P 5600 9800
F 0 "D4" H 5600 9900 50  0000 C CNN
F 1 "RED" H 5600 9700 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5600 9800 50  0001 C CNN
F 3 "http://katalog.we-online.de/led/datasheet/150060RS75000.pdf" H 5600 9800 50  0001 C CNN
F 4 "732-4978-1-ND" H 5600 9800 60  0001 C CNN "Part"
F 5 "DigiKey" H 5600 9800 60  0001 C CNN "Provider"
	1    5600 9800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R4
U 1 1 5AB8EC7A
P 5600 10250
F 0 "R4" V 5680 10250 50  0000 C CNN
F 1 "2.2k" V 5600 10250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5530 10250 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 5600 10250 50  0001 C CNN
F 4 "RMCF0603JT2K20CT-ND" V 5600 10250 60  0001 C CNN "Part"
F 5 "DigiKey" V 5600 10250 60  0001 C CNN "Provider"
	1    5600 10250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5AB8EC7B
P 5600 10800
F 0 "#PWR01" H 5600 10550 50  0001 C CNN
F 1 "GND" H 5600 10650 50  0000 C CNN
F 2 "" H 5600 10800 50  0001 C CNN
F 3 "" H 5600 10800 50  0001 C CNN
	1    5600 10800
	1    0    0    -1  
$EndComp
Text Label 7400 9100 0    60   ~ 0
SENSE_A
Text Label 7400 9200 0    60   ~ 0
SENSE_B
Text Label 7400 9300 0    60   ~ 0
SENSE_C
Text Label 7400 8900 0    60   ~ 0
CURRENT_ADC
Text Label 8700 9100 2    60   ~ 0
PC0
Text Label 8700 9200 2    60   ~ 0
PC1
Text Label 8700 9300 2    60   ~ 0
PC2
Text Label 8700 9400 2    60   ~ 0
PC3
Text Label 8700 8900 2    60   ~ 0
PA0
$Comp
L servo-sigma-48v-rescue:CONN_02X05-conn J2
U 1 1 5AB8EC7C
P 2350 5050
F 0 "J2" H 2350 5350 50  0000 C CNN
F 1 "CONN_02X05" H 2350 4700 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical" H 2350 3850 50  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/6220xx21121.pdf" H 2350 3850 50  0001 C CNN
F 4 "732-5374-ND" H 2350 5050 60  0001 C CNN "Part"
F 5 "DigiKey" H 2350 5050 60  0001 C CNN "Provider"
	1    2350 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 58D0728C
P 900 4950
F 0 "#PWR02" H 900 4700 50  0001 C CNN
F 1 "GND" H 900 4800 50  0000 C CNN
F 2 "" H 900 4950 50  0001 C CNN
F 3 "" H 900 4950 50  0001 C CNN
	1    900  4950
	0    1    1    0   
$EndComp
Text Label 3250 4850 2    60   ~ 0
PA13
Text Label 3650 4850 2    60   ~ 0
SWDIO
Text Label 3250 4950 2    60   ~ 0
PA14
Text Label 3650 4950 2    60   ~ 0
SWCLK
Text Label 3650 5250 2    60   ~ 0
NRST
Text Label 3250 5050 2    60   ~ 0
PB3
Text Notes 1400 5550 0    60   ~ 0
SWD Interface
Text HLabel 7100 9000 0    60   Input ~ 0
VBAT_ADC
Text HLabel 7100 9100 0    60   Input ~ 0
SENSE_A
Text HLabel 7100 9200 0    60   Input ~ 0
SENSE_B
Text HLabel 7100 9300 0    60   Input ~ 0
SENSE_C
Text HLabel 7100 8900 0    60   Input ~ 0
CURRENT_ADC
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
Text HLabel 14950 2950 2    60   Input ~ 0
CURRENT_FAULT
$Comp
L servo-sigma-48v-rescue:R-device R1
U 1 1 5AB8EC7E
P 5150 10250
F 0 "R1" V 5230 10250 50  0000 C CNN
F 1 "2.2k" V 5150 10250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5080 10250 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 5150 10250 50  0001 C CNN
F 4 "RMCF0603JT2K20CT-ND" V 5150 10250 60  0001 C CNN "Part"
F 5 "DigiKey" V 5150 10250 60  0001 C CNN "Provider"
	1    5150 10250
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
P 5600 4050
F 0 "Y1" H 5600 4200 50  0000 C CNN
F 1 "ABM3-8.000MHZ-D2Y-T" H 5600 3900 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_Abracon_ABM3-2Pin_5.0x3.2mm" H 5600 4050 50  0001 C CNN
F 3 "http://www.abracon.com/Resonators/abm3.pdf" H 5600 4050 50  0001 C CNN
F 4 "535-10630-1-ND" H 5600 4050 60  0001 C CNN "Part"
F 5 "DigiKey" H 5600 4050 60  0001 C CNN "Provider"
	1    5600 4050
	0    1    1    0   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C27
U 1 1 58EFF4C6
P 5100 3850
F 0 "C27" V 5050 3950 50  0000 C CNN
F 1 "27pF/50V/1%" V 5250 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5138 3700 50  0001 C CNN
F 3 "http://search.murata.co.jp/Ceramy/image/img/A01X/partnumbering_e_01.pdf" H 5100 3850 50  0001 C CNN
F 4 "490-9719-1-ND" V 5100 3850 60  0001 C CNN "Part"
F 5 "DigiKey" V 5100 3850 60  0001 C CNN "Provider"
	1    5100 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5AB8EC81
P 4200 3850
F 0 "#PWR04" H 4200 3600 50  0001 C CNN
F 1 "GND" H 4200 3700 50  0000 C CNN
F 2 "" H 4200 3850 50  0001 C CNN
F 3 "" H 4200 3850 50  0001 C CNN
	1    4200 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 58F00797
P 4200 4250
F 0 "#PWR05" H 4200 4000 50  0001 C CNN
F 1 "GND" H 4200 4100 50  0000 C CNN
F 2 "" H 4200 4250 50  0001 C CNN
F 3 "" H 4200 4250 50  0001 C CNN
	1    4200 4250
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
P 900 4650
F 0 "#PWR08" H 900 4500 50  0001 C CNN
F 1 "VDD" H 917 4823 50  0000 C CNN
F 2 "" H 900 4650 50  0001 C CNN
F 3 "" H 900 4650 50  0001 C CNN
	1    900  4650
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
NoConn ~ 12950 4650
NoConn ~ 12950 5450
NoConn ~ 12950 5550
Wire Wire Line
	7100 9000 8700 9000
Wire Wire Line
	5150 9950 5150 10100
Wire Wire Line
	5600 10800 5600 10400
Wire Wire Line
	5600 10100 5600 9950
Wire Wire Line
	5150 10400 5150 10950
Wire Wire Line
	5600 9050 5600 9650
Wire Wire Line
	5150 9250 5150 9650
Wire Wire Line
	7100 9100 8700 9100
Wire Wire Line
	7100 9200 8700 9200
Wire Wire Line
	7100 9300 8700 9300
Wire Wire Line
	7100 8900 8700 8900
Wire Wire Line
	2100 4950 900  4950
Wire Wire Line
	2100 5050 2000 5050
Wire Wire Line
	900  5250 1700 5250
Wire Wire Line
	900  4850 2100 4850
Wire Wire Line
	2600 4850 3650 4850
Wire Wire Line
	2600 4950 3650 4950
Wire Wire Line
	2600 5050 2700 5050
Wire Wire Line
	2600 5250 3650 5250
Wire Wire Line
	12950 4650 13350 4650
Wire Wire Line
	12950 5450 13350 5450
Wire Wire Line
	12950 5550 13350 5550
Wire Wire Line
	13350 2750 12950 2750
Wire Wire Line
	13350 2850 12950 2850
Wire Wire Line
	13350 3050 12950 3050
Wire Wire Line
	13350 3150 12950 3150
Wire Wire Line
	13350 4050 12950 4050
Wire Wire Line
	13350 4150 12950 4150
Wire Wire Line
	6750 4350 7150 4350
Wire Wire Line
	6750 4450 7150 4450
Wire Wire Line
	6750 4550 7150 4550
Wire Wire Line
	6750 4650 7150 4650
Wire Wire Line
	6750 4750 7150 4750
Wire Wire Line
	6750 4850 7150 4850
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
	5250 3850 5600 3850
Wire Wire Line
	5600 4200 5600 4250
Connection ~ 5600 3850
Connection ~ 5600 4250
Wire Wire Line
	4200 3850 4950 3850
Wire Wire Line
	4200 4250 4950 4250
Wire Wire Line
	6750 2750 7150 2750
Wire Wire Line
	900  4650 900  4850
Wire Wire Line
	6300 2950 7150 2950
Wire Wire Line
	6300 2650 6300 2950
Wire Wire Line
	6300 2350 6300 2100
Wire Wire Line
	5250 4250 5600 4250
Wire Wire Line
	6000 4250 6000 3950
Wire Wire Line
	6000 3950 7150 3950
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
Text Label 8700 9000 2    60   ~ 0
PA1
$Comp
L power:VDD #PWR023
U 1 1 58F73DF4
P 5150 9250
F 0 "#PWR023" H 5150 9100 50  0001 C CNN
F 1 "VDD" H 5167 9423 50  0000 C CNN
F 2 "" H 5150 9250 50  0001 C CNN
F 3 "" H 5150 9250 50  0001 C CNN
	1    5150 9250
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
	5600 3900 5600 3850
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
P 5100 4250
F 0 "C32" V 5050 4350 50  0000 C CNN
F 1 "27pF/50V/1%" V 5250 4250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5138 4100 50  0001 C CNN
F 3 "http://search.murata.co.jp/Ceramy/image/img/A01X/partnumbering_e_01.pdf" H 5100 4250 50  0001 C CNN
F 4 "490-9719-1-ND" V 5100 4250 60  0001 C CNN "Part"
F 5 "DigiKey" V 5100 4250 60  0001 C CNN "Provider"
	1    5100 4250
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
	5600 3850 7150 3850
Wire Wire Line
	5600 4250 6000 4250
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
Text Label 14650 4250 2    60   ~ 0
USART_TX
Text Label 900  5250 0    60   ~ 0
USART_TX
Wire Wire Line
	2100 5150 900  5150
NoConn ~ 2600 5150
Text HLabel 900  5150 0    60   Output ~ 0
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
P 2300 5850
F 0 "#PWR011" H 2300 5700 50  0001 C CNN
F 1 "VDD" H 2317 6023 50  0000 C CNN
F 2 "" H 2300 5850 50  0001 C CNN
F 3 "" H 2300 5850 50  0001 C CNN
	1    2300 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5AB8F240
P 1150 7050
F 0 "#PWR09" H 1150 6800 50  0001 C CNN
F 1 "GND" H 1150 6900 50  0000 C CNN
F 2 "" H 1150 7050 50  0001 C CNN
F 3 "" H 1150 7050 50  0001 C CNN
	1    1150 7050
	1    0    0    -1  
$EndComp
Text Label 2950 6350 2    60   ~ 0
PA4
$Comp
L servo-sigma-48v-rescue:R-device R9
U 1 1 5AB8F249
P 1700 6950
F 0 "R9" V 1780 6950 50  0000 C CNN
F 1 "100" V 1700 6950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 6950 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1700 6950 50  0001 C CNN
F 4 "311-100HRCT-ND" V 1700 6950 60  0001 C CNN "Part"
F 5 "DigiKey" V 1700 6950 60  0001 C CNN "Provider"
	1    1700 6950
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C4
U 1 1 5AB8F252
P 2300 6700
F 0 "C4" H 2150 6600 50  0000 L CNN
F 1 "100nF 50V" H 1850 6800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2338 6550 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2300 6700 50  0001 C CNN
F 4 "490-4779-2-ND" H 2300 6700 60  0001 C CNN "Part"
F 5 "DigiKey" H 2300 6700 60  0001 C CNN "Provider"
	1    2300 6700
	-1   0    0    1   
$EndComp
$Comp
L sigmadrone:SW_PUSH_TACKTILE2 USR1
U 1 1 5AB8F25B
P 1700 6350
F 0 "USR1" H 2050 6450 50  0000 R CNN
F 1 "WURTH_434153017835" H 2050 6150 50  0000 R CNN
F 2 "Sigmadrone:SW_SPST_WURTH_434153017835" H 1700 6350 60  0001 C CNN
F 3 "http://katalog.we-online.de/em/datasheet/434153017835.pdf" H 1700 6350 60  0001 C CNN
F 4 "732-10143-1-ND" H 1700 6350 60  0001 C CNN "Part"
F 5 "DigiKey" H 1700 6350 60  0001 C CNN "Provider"
	1    1700 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 5850 2300 5950
Wire Wire Line
	2300 6250 2300 6350
Wire Wire Line
	1150 6350 1150 6450
Wire Wire Line
	1150 6950 1550 6950
Wire Wire Line
	2000 6350 2300 6350
Connection ~ 2300 6350
Wire Wire Line
	2300 6850 2300 6950
Wire Wire Line
	2300 6950 1850 6950
Wire Wire Line
	1400 6350 1150 6350
Connection ~ 1150 6950
Wire Wire Line
	1400 6450 1150 6450
Connection ~ 1150 6450
Wire Wire Line
	2300 6450 2000 6450
Connection ~ 2300 6450
$Comp
L servo-sigma-48v-rescue:R-device R10
U 1 1 5AB8F272
P 2300 6100
F 0 "R10" V 2380 6100 50  0000 C CNN
F 1 "10k" V 2300 6100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2230 6100 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2300 6100 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2300 6100 60  0001 C CNN "Part"
F 5 "DigiKey" V 2300 6100 60  0001 C CNN "Provider"
	1    2300 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 6350 2300 6450
Wire Wire Line
	2300 6350 2950 6350
Wire Wire Line
	1150 6950 1150 7050
Wire Wire Line
	1150 6450 1150 6950
Wire Wire Line
	2300 6450 2300 6550
Text Label 8700 9500 2    60   ~ 0
PC4
Text Label 8700 9600 2    60   ~ 0
PC5
Text HLabel 7100 9400 0    60   Input ~ 0
IA_FB
Text HLabel 7100 9500 0    60   Input ~ 0
IB_FB
Text HLabel 7100 9600 0    60   Input ~ 0
IC_FB
Wire Wire Line
	7100 9400 8700 9400
Wire Wire Line
	7100 9500 8700 9500
Wire Wire Line
	7100 9600 8700 9600
Text Label 7400 9400 0    60   ~ 0
IA_FB
Text Label 7400 9500 0    60   ~ 0
IB_FB
Text Label 7400 9600 0    60   ~ 0
IC_FB
Text Label 2100 7800 2    60   ~ 0
ENCODER_B
Text Label 2100 7900 2    60   ~ 0
ENCODER_5V
Text Label 2100 8000 2    60   ~ 0
ENCODER_A
Text Label 2100 8100 2    60   ~ 0
ENCODER_Z
Text Label 2100 8200 2    60   ~ 0
ENCODER_GND
Wire Wire Line
	1350 7800 2100 7800
Wire Wire Line
	1350 7900 2100 7900
Wire Wire Line
	1350 8000 2100 8000
Wire Wire Line
	1350 8100 2100 8100
Wire Wire Line
	1350 8200 2100 8200
$Comp
L power:+5V #PWR?
U 1 1 5BB4B839
P 2100 7900
AR Path="/58BF599E/5BB4B839" Ref="#PWR?"  Part="1" 
AR Path="/58BE2779/5BB4B839" Ref="#PWR0115"  Part="1" 
F 0 "#PWR0115" H 2100 7750 50  0001 C CNN
F 1 "+5V" H 2115 8073 50  0000 C CNN
F 2 "" H 2100 7900 50  0001 C CNN
F 3 "" H 2100 7900 50  0001 C CNN
	1    2100 7900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5BB57B1C
P 2100 8200
F 0 "#PWR0116" H 2100 7950 50  0001 C CNN
F 1 "GND" H 2100 8050 50  0000 C CNN
F 2 "" H 2100 8200 50  0001 C CNN
F 3 "" H 2100 8200 50  0001 C CNN
	1    2100 8200
	0    -1   -1   0   
$EndComp
Text Label 14650 5050 2    60   ~ 0
ENCODER_A
Text Label 14650 5150 2    60   ~ 0
ENCODER_B
Wire Wire Line
	12950 3450 13350 3450
Wire Wire Line
	12950 4450 13350 4450
Wire Wire Line
	12950 4550 13350 4550
Wire Wire Line
	12950 4850 14950 4850
NoConn ~ 12950 4550
NoConn ~ 12950 4450
NoConn ~ 12950 3450
Wire Wire Line
	12950 4750 13350 4750
$Comp
L servo-sigma-48v-rescue:R-device R44
U 1 1 5BDC03FA
P 2850 5050
F 0 "R44" V 2900 5200 50  0000 C CNN
F 1 "21" V 2850 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2780 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2850 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 2850 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 2850 5050 60  0001 C CNN "Provider"
	1    2850 5050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3000 5050 3650 5050
$Comp
L servo-sigma-48v-rescue:R-device R2
U 1 1 5BDC074C
P 1850 5050
F 0 "R2" V 1900 5200 50  0000 C CNN
F 1 "21" V 1850 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1780 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1850 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1850 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 1850 5050 60  0001 C CNN "Provider"
	1    1850 5050
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R3
U 1 1 5BDE0108
P 1850 5250
F 0 "R3" V 1900 5400 50  0000 C CNN
F 1 "21" V 1850 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1780 5250 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1850 5250 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1850 5250 60  0001 C CNN "Part"
F 5 "DigiKey" V 1850 5250 60  0001 C CNN "Provider"
	1    1850 5250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 5250 2100 5250
Wire Wire Line
	12950 4250 14650 4250
Wire Wire Line
	12950 5050 14650 5050
Wire Wire Line
	12950 5150 14650 5150
Text Label 14650 4950 2    60   ~ 0
ENCODER_Z
Wire Wire Line
	12950 4950 14650 4950
Wire Wire Line
	12950 2950 14950 2950
Text Label 900  5050 0    60   ~ 0
USART_RX
Wire Wire Line
	900  5050 1700 5050
Text Label 1600 5050 2    60   ~ 0
PB3
Text Label 3650 5050 2    60   ~ 0
SWO
Text Label 1600 5250 2    60   ~ 0
PA15
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
USB_DP
Wire Wire Line
	12950 3850 14650 3850
Wire Wire Line
	12950 3950 14650 3950
Text HLabel 6500 5050 0    60   Output ~ 0
AUX_L
Text HLabel 6500 4950 0    60   Output ~ 0
AUX_H
$Comp
L servo-sigma-48v-rescue:CONN_01X05-conn J5
U 1 1 5BCE27D8
P 1150 8000
F 0 "J5" H 1150 8300 50  0000 C CNN
F 1 "CONN_01X05" H 1150 7700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Horizontal" H 1150 8000 50  0001 C CNN
F 3 "https://www.molex.com/pdm_docs/sd/705530002_sd.pdf" H 1150 8000 50  0001 C CNN
F 4 "WM4903-ND" H 1150 8000 50  0001 C CNN "Part"
F 5 "DigiKey" H 1150 8000 50  0001 C CNN "Provider"
	1    1150 8000
	-1   0    0    -1  
$EndComp
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
Text Label 8700 9700 2    60   ~ 0
PA3
Text HLabel 7100 9700 0    60   Input ~ 0
IR_FB
Wire Wire Line
	7100 9700 8700 9700
Text Label 14650 3250 2    60   ~ 0
LED_WARN
Text Label 14650 3350 2    60   ~ 0
LED_STATUS
NoConn ~ 7150 5650
Wire Wire Line
	6500 4950 7150 4950
Wire Wire Line
	6500 5050 7150 5050
Wire Wire Line
	12950 5250 13350 5250
Wire Wire Line
	12950 5350 13350 5350
NoConn ~ 12950 5250
NoConn ~ 12950 5350
Wire Wire Line
	12950 3350 14650 3350
Wire Wire Line
	12950 3250 14650 3250
Wire Wire Line
	6750 5750 7150 5750
Wire Wire Line
	6750 5850 7150 5850
NoConn ~ 7150 5750
NoConn ~ 7150 5850
$EndSCHEMATC