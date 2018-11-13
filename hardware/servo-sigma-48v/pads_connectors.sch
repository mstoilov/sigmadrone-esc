EESchema Schematic File Version 4
LIBS:servo-sigma-48v-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 4 5
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:+BATT #PWR046
U 1 1 58BEF24C
P 3250 3050
F 0 "#PWR046" H 3250 2900 50  0001 C CNN
F 1 "+BATT" H 3250 3190 50  0000 C CNN
F 2 "" H 3250 3050 50  0001 C CNN
F 3 "" H 3250 3050 50  0001 C CNN
	1    3250 3050
	-1   0    0    -1  
$EndComp
$Comp
L sigmadrone:ACS711 U3
U 1 1 58CF4867
P 4200 3500
F 0 "U3" H 4450 3800 60  0000 C CNN
F 1 "ACS711" H 4300 3200 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4200 3500 60  0001 C CNN
F 3 "http://www.allegromicro.com/en/Products/Part_Numbers/0711/0711.pdf" H 4200 3500 60  0001 C CNN
F 4 "620-1373-1-ND" H 4200 3500 60  0001 C CNN "Part"
F 5 "DigiKey" H 4200 3500 60  0001 C CNN "Provider"
	1    4200 3500
	1    0    0    -1  
$EndComp
Text HLabel 7900 2450 2    60   Output ~ 0
CURRENT_ADC_FILTERED
Text HLabel 7900 4650 2    60   Output ~ 0
CURRENT_FAULT
$Comp
L power:GND #PWR049
U 1 1 58CF635C
P 4850 3750
F 0 "#PWR049" H 4850 3500 50  0001 C CNN
F 1 "GND" H 4850 3600 50  0000 C CNN
F 2 "" H 4850 3750 50  0001 C CNN
F 3 "" H 4850 3750 50  0001 C CNN
	1    4850 3750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR050
U 1 1 58CF656C
P 5200 4100
F 0 "#PWR050" H 5200 3850 50  0001 C CNN
F 1 "GND" H 5200 3950 50  0000 C CNN
F 2 "" H 5200 4100 50  0001 C CNN
F 3 "" H 5200 4100 50  0001 C CNN
	1    5200 4100
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR052
U 1 1 58CF92CD
P 7350 3350
F 0 "#PWR052" H 7350 3100 50  0001 C CNN
F 1 "GND" H 7350 3200 50  0000 C CNN
F 2 "" H 7350 3350 50  0001 C CNN
F 3 "" H 7350 3350 50  0001 C CNN
	1    7350 3350
	1    0    0    -1  
$EndComp
Text Notes 6500 2250 0    60   ~ 0
Low-pass filter, cutoff at 3.4 Hz
Text Notes 3600 5650 0    60   ~ 0
Power\nDecoupling\n
$Comp
L power:GND #PWR053
U 1 1 58CF9CC6
P 9950 5300
F 0 "#PWR053" H 9950 5050 50  0001 C CNN
F 1 "GND" H 9950 5150 50  0000 C CNN
F 2 "" H 9950 5300 50  0001 C CNN
F 3 "" H 9950 5300 50  0001 C CNN
	1    9950 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR054
U 1 1 58CF9E89
P 9950 4000
F 0 "#PWR054" H 9950 3850 50  0001 C CNN
F 1 "+BATT" H 9950 4140 50  0000 C CNN
F 2 "" H 9950 4000 50  0001 C CNN
F 3 "" H 9950 4000 50  0001 C CNN
	1    9950 4000
	-1   0    0    -1  
$EndComp
Text HLabel 10350 4650 2    60   Output ~ 0
VBAT_ADC
Text Notes 10300 4450 0    60   ~ 0
Voltage Sense
$Comp
L servo-sigma-48v-rescue:R-device R15
U 1 1 58D1E7D1
P 7050 2450
F 0 "R15" V 7130 2450 50  0000 C CNN
F 1 "47k" V 7050 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6980 2450 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7050 2450 50  0001 C CNN
F 4 "311-47KDCT-ND" V 7050 2450 60  0001 C CNN "Part"
F 5 "DigiKey" V 7050 2450 60  0001 C CNN "Provider"
	1    7050 2450
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR063
U 1 1 58F74240
P 5200 3050
F 0 "#PWR063" H 5200 2900 50  0001 C CNN
F 1 "VDD" H 5217 3223 50  0000 C CNN
F 2 "" H 5200 3050 50  0001 C CNN
F 3 "" H 5200 3050 50  0001 C CNN
	1    5200 3050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C8
U 1 1 5A08C553
P 5200 3850
F 0 "C8" H 5050 3750 50  0000 L CNN
F 1 "1uF 100V" H 4800 3950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5238 3700 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 5200 3850 50  0001 C CNN
F 4 "445-8887-1-ND" H 5200 3850 60  0001 C CNN "Part"
F 5 "DigiKey" H 5200 3850 60  0001 C CNN "Provider"
	1    5200 3850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5A34CA62
P 7350 5200
F 0 "#PWR044" H 7350 4950 50  0001 C CNN
F 1 "GND" H 7350 5050 50  0000 C CNN
F 2 "" H 7350 5200 50  0001 C CNN
F 3 "" H 7350 5200 50  0001 C CNN
	1    7350 5200
	-1   0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C10
U 1 1 5A36219E
P 7350 4950
F 0 "C10" H 7200 4850 50  0000 L CNN
F 1 "100nF 50V" H 6900 5050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7388 4800 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 7350 4950 50  0001 C CNN
F 4 "490-4779-2-ND" H 7350 4950 60  0001 C CNN "Part"
F 5 "DigiKey" H 7350 4950 60  0001 C CNN "Provider"
	1    7350 4950
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R14
U 1 1 5A39E32B
P 7350 4250
F 0 "R14" V 7430 4250 50  0000 C CNN
F 1 "10k" V 7350 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 4250 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 7350 4250 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 7350 4250 60  0001 C CNN "Part"
F 5 "DigiKey" V 7350 4250 60  0001 C CNN "Provider"
	1    7350 4250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 3450 3700 3450
Wire Wire Line
	3550 3550 3550 3650
Connection ~ 3550 3650
Wire Wire Line
	4700 3650 4850 3650
Wire Wire Line
	4850 3650 4850 3750
Wire Wire Line
	4700 3450 6400 3450
Wire Wire Line
	3550 3550 3700 3550
Wire Wire Line
	7200 2450 7350 2450
Wire Wire Line
	7350 3350 7350 3050
Wire Wire Line
	7350 2750 7350 2450
Connection ~ 7350 2450
Wire Wire Line
	9950 5300 9950 5050
Wire Wire Line
	9950 4500 9950 4650
Wire Wire Line
	9950 4000 9950 4200
Wire Wire Line
	9950 4650 10350 4650
Connection ~ 9950 4650
Wire Wire Line
	7350 4800 7350 4650
Wire Wire Line
	7350 5100 7350 5200
$Comp
L power:VDD #PWR057
U 1 1 5A3AB83D
P 7350 4000
F 0 "#PWR057" H 7350 3850 50  0001 C CNN
F 1 "VDD" H 7367 4173 50  0000 C CNN
F 2 "" H 7350 4000 50  0001 C CNN
F 3 "" H 7350 4000 50  0001 C CNN
	1    7350 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 4100 7350 4000
$Comp
L servo-sigma-48v-rescue:C-device C5
U 1 1 5A5404E1
P 7350 2900
F 0 "C5" H 7200 2800 50  0000 L CNN
F 1 "1uF 100V" H 7400 3000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7388 2750 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 7350 2900 50  0001 C CNN
F 4 "445-8887-1-ND" H 7350 2900 60  0001 C CNN "Part"
F 5 "DigiKey" H 7350 2900 60  0001 C CNN "Provider"
	1    7350 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 3350 3700 3350
Wire Wire Line
	3550 3350 3550 3450
Wire Wire Line
	3550 3650 3700 3650
Wire Wire Line
	7350 2450 7900 2450
Wire Wire Line
	9950 4650 9950 4750
$Comp
L sigmadrone:FIDUCIAL F3
U 1 1 5A8818B5
P 2100 10000
F 0 "F3" H 2278 10053 60  0000 L CNN
F 1 "FIDUCIAL" H 2278 9947 60  0000 L CNN
F 2 "Fiducials:Fiducial_0.5mm_Dia_1mm_Outer" H 2000 9700 60  0001 C CNN
F 3 "" H 2100 10000 60  0001 C CNN
	1    2100 10000
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:FIDUCIAL F2
U 1 1 5A8819A7
P 1300 10400
F 0 "F2" H 1478 10453 60  0000 L CNN
F 1 "FIDUCIAL" H 1478 10347 60  0000 L CNN
F 2 "Fiducials:Fiducial_0.5mm_Dia_1mm_Outer" H 1200 10100 60  0001 C CNN
F 3 "" H 1300 10400 60  0001 C CNN
	1    1300 10400
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:FIDUCIAL F1
U 1 1 5A881A0D
P 1300 10000
F 0 "F1" H 1478 10053 60  0000 L CNN
F 1 "FIDUCIAL" H 1478 9947 60  0000 L CNN
F 2 "Fiducials:Fiducial_0.5mm_Dia_1mm_Outer" H 1200 9700 60  0001 C CNN
F 3 "" H 1300 10000 60  0001 C CNN
	1    1300 10000
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:FIDUCIAL F4
U 1 1 5A87BC33
P 2100 10400
F 0 "F4" H 2278 10453 60  0000 L CNN
F 1 "FIDUCIAL" H 2278 10347 60  0000 L CNN
F 2 "Fiducials:Fiducial_0.5mm_Dia_1mm_Outer" H 2000 10100 60  0001 C CNN
F 3 "" H 2100 10400 60  0001 C CNN
	1    2100 10400
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R6
U 1 1 5A8A6AE2
P 9950 4350
F 0 "R6" H 10100 4350 50  0000 C CNN
F 1 "47k" V 9950 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9880 4350 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9950 4350 50  0001 C CNN
F 4 "311-47KDCT-ND" V 9950 4350 60  0001 C CNN "Part"
F 5 "DigiKey" V 9950 4350 60  0001 C CNN "Provider"
	1    9950 4350
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R7
U 1 1 5A8A6AEB
P 9950 4900
F 0 "R7" H 10100 4900 50  0000 C CNN
F 1 "4.7k" V 9950 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9880 4900 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 9950 4900 50  0001 C CNN
F 4 "311-2606-2-ND" V 9950 4900 60  0001 C CNN "Part"
F 5 "DigiKey" V 9950 4900 60  0001 C CNN "Provider"
	1    9950 4900
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:ST1S14 U1
U 1 1 5B96E901
P 5400 7750
F 0 "U1" H 5050 8350 60  0000 C CNN
F 1 "ST1S14" H 5150 8250 60  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.35x2.35mm" H 5150 8350 60  0001 C CNN
F 3 "https://www.st.com/content/ccc/resource/technical/document/datasheet/a3/35/21/f8/1a/9f/41/da/CD00285678.pdf/files/CD00285678.pdf/jcr:content/translations/en.CD00285678.pdf" H 5250 8450 60  0001 C CNN
F 4 "497-10816-1-ND" H 5350 8550 60  0001 C CNN "Part"
F 5 "DigiKey" H 5450 8650 60  0001 C CNN "Provider"
	1    5400 7750
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:CONN_01X02-conn J1
U 1 1 5B988CDF
P 1700 3400
F 0 "J1" H 1619 3665 50  0000 C CNN
F 1 "CONN_01X02" H 1619 3574 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_02x7.62mm_Vertical" H 1700 3400 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1868076.pdf" H 1700 3400 50  0001 C CNN
F 4 "277-5840-ND" H 1700 3400 50  0001 C CNN "Part"
F 5 "DigiKey" H 1700 3400 50  0001 C CNN "Provider"
	1    1700 3400
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:CP-device C1
U 1 1 5B98E0E3
P 3250 5150
F 0 "C1" H 3275 5250 50  0000 L CNN
F 1 "120uF" H 3275 5050 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 3288 5000 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDD0000/DMD0000COL92.pdf" H 3250 5150 50  0001 C CNN
F 4 "P19316CT-ND" H 3250 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 5150 60  0001 C CNN "Provider"
	1    3250 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3450 2300 3450
Wire Wire Line
	2300 3450 2300 4100
Wire Wire Line
	3250 5300 3250 5400
Connection ~ 3250 5400
Wire Wire Line
	3250 5400 3550 5400
Wire Wire Line
	3550 5300 3550 5400
Connection ~ 3550 5400
Wire Wire Line
	3550 5400 3850 5400
Wire Wire Line
	3850 5300 3850 5400
Wire Wire Line
	3250 5000 3250 4900
Connection ~ 3250 4900
Wire Wire Line
	3250 4900 3550 4900
Wire Wire Line
	3550 5000 3550 4900
Connection ~ 3550 4900
Wire Wire Line
	3550 4900 3850 4900
Wire Wire Line
	3850 5000 3850 4900
$Comp
L sigmadrone:VIN #PWR012
U 1 1 5B9BBEE4
P 2300 3050
F 0 "#PWR012" H 2300 2900 50  0001 C CNN
F 1 "VIN" H 2315 3223 50  0000 C CNN
F 2 "" H 2300 3050 50  0000 C CNN
F 3 "" H 2300 3050 50  0000 C CNN
	1    2300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4900 3250 4600
$Comp
L power:GND #PWR020
U 1 1 5B9C31BD
P 3250 5550
F 0 "#PWR020" H 3250 5300 50  0001 C CNN
F 1 "GND" H 3255 5377 50  0000 C CNN
F 2 "" H 3250 5550 50  0001 C CNN
F 3 "" H 3250 5550 50  0001 C CNN
	1    3250 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 5550 3250 5400
Text Notes 3700 7650 0    60   ~ 0
10V-48V
$Comp
L servo-sigma-48v-rescue:C-device C13
U 1 1 5B9CA8B3
P 5400 6950
F 0 "C13" V 5650 7000 50  0000 L CNN
F 1 "100nF 50V" V 5550 6700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5438 6800 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 5400 6950 50  0001 C CNN
F 4 "490-4779-2-ND" H 5400 6950 60  0001 C CNN "Part"
F 5 "DigiKey" H 5400 6950 60  0001 C CNN "Provider"
	1    5400 6950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 7450 4500 7450
Wire Wire Line
	4500 7450 4500 6950
Wire Wire Line
	4500 6950 5250 6950
Wire Wire Line
	5550 6950 6550 6950
Wire Wire Line
	6550 6950 6550 7450
Wire Wire Line
	6550 7450 5900 7450
Wire Wire Line
	4500 7650 4500 7850
Wire Wire Line
	4500 7850 4900 7850
Connection ~ 4500 7650
Wire Wire Line
	4500 7650 4900 7650
$Comp
L servo-sigma-48v-rescue:C-device C12
U 1 1 5B9D6834
P 4150 8000
F 0 "C12" H 4200 8100 50  0000 L CNN
F 1 "100nF 100V" V 4000 7750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4188 7850 50  0001 C CNN
F 3 "https://www.murata.com/~/media/webrenewal/support/library/catalog/products/capacitor/mlcc/c02e.ashx?la=en-us" H 4150 8000 50  0001 C CNN
F 4 "490-3285-1-ND" H 4150 8000 60  0001 C CNN "Part"
F 5 "DigiKey" H 4150 8000 60  0001 C CNN "Provider"
	1    4150 8000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 5B9D7507
P 5400 8750
F 0 "#PWR047" H 5400 8500 50  0001 C CNN
F 1 "GND" H 5400 8600 50  0000 C CNN
F 2 "" H 5400 8750 50  0001 C CNN
F 3 "" H 5400 8750 50  0001 C CNN
	1    5400 8750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4150 8150 4150 8650
Wire Wire Line
	4150 8650 4500 8650
Wire Wire Line
	6150 8650 6150 8050
Wire Wire Line
	6150 8050 5900 8050
Wire Wire Line
	5400 8750 5400 8650
Connection ~ 5400 8650
Wire Wire Line
	5400 8650 6150 8650
Wire Wire Line
	5400 8300 5400 8650
Wire Wire Line
	4900 8050 4500 8050
Wire Wire Line
	4500 8050 4500 8650
Connection ~ 4500 8650
Wire Wire Line
	4500 8650 5400 8650
Wire Wire Line
	4150 7850 4150 7650
Connection ~ 4150 7650
Wire Wire Line
	4150 7650 4500 7650
Text Notes 5550 8750 0    60   ~ 0
small signal
$Comp
L servo-sigma-48v-rescue:C-device C2
U 1 1 5B9F7968
P 3250 8000
F 0 "C2" H 3300 8100 50  0000 L CNN
F 1 "10uF 50V" V 3100 7750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3288 7850 50  0001 C CNN
F 3 "https://www.murata.com/~/media/webrenewal/support/library/catalog/products/capacitor/mlcc/c02e.ashx?la=en-us" H 3250 8000 50  0001 C CNN
F 4 "490-12456-1-ND" H 3250 8000 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 8000 60  0001 C CNN "Provider"
	1    3250 8000
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C7
U 1 1 5B9F7A85
P 3650 8000
F 0 "C7" H 3700 8100 50  0000 L CNN
F 1 "10uF 50V" V 3500 7750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3688 7850 50  0001 C CNN
F 3 "https://www.murata.com/~/media/webrenewal/support/library/catalog/products/capacitor/mlcc/c02e.ashx?la=en-us" H 3650 8000 50  0001 C CNN
F 4 "490-12456-1-ND" H 3650 8000 60  0001 C CNN "Part"
F 5 "DigiKey" H 3650 8000 60  0001 C CNN "Provider"
	1    3650 8000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR048
U 1 1 5B9F7B43
P 7800 9100
F 0 "#PWR048" H 7800 8850 50  0001 C CNN
F 1 "GND" H 7800 8950 50  0000 C CNN
F 2 "" H 7800 9100 50  0001 C CNN
F 3 "" H 7800 9100 50  0001 C CNN
	1    7800 9100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3250 8150 3250 9000
Wire Wire Line
	3250 9000 3650 9000
Wire Wire Line
	7800 9000 7800 9100
Wire Wire Line
	3250 7850 3250 7650
Wire Wire Line
	3650 7850 3650 7650
Wire Wire Line
	3250 7650 3650 7650
Connection ~ 3650 7650
Wire Wire Line
	3650 7650 4150 7650
Wire Wire Line
	3650 8150 3650 9000
Connection ~ 3650 9000
Wire Wire Line
	3650 9000 6550 9000
Text Notes 5650 9100 0    60   ~ 0
power plane
$Comp
L servo-sigma-48v-rescue:L-device L1
U 1 1 5BA12B73
P 7400 7450
F 0 "L1" V 7590 7450 50  0000 C CNN
F 1 "8.2uH" V 7499 7450 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_WE-TPC-3816" H 7400 7450 50  0001 C CNN
F 3 "https://katalog.we-online.de/pbs/datasheet/744042008.pdf" H 7400 7450 50  0001 C CNN
F 4 "732-1024-1-ND" V 7400 7450 50  0001 C CNN "Part"
F 5 "DigiKey" V 7400 7450 50  0001 C CNN "Provider"
	1    7400 7450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7250 7450 6550 7450
Connection ~ 6550 7450
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5BA27D2A
P 7400 7650
AR Path="/58BF599E/5BA27D2A" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5BA27D2A" Ref="R12"  Part="1" 
F 0 "R12" V 7480 7650 50  0000 C CNN
F 1 "47k" V 7400 7650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7330 7650 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7400 7650 50  0001 C CNN
F 4 "311-47KDCT-ND" V 7400 7650 60  0001 C CNN "Part"
F 5 "DigiKey" V 7400 7650 60  0001 C CNN "Provider"
	1    7400 7650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5BA2D520
P 7400 7850
AR Path="/58BF599E/5BA2D520" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5BA2D520" Ref="R13"  Part="1" 
F 0 "R13" V 7480 7850 50  0000 C CNN
F 1 "30k" V 7400 7850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7330 7850 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7400 7850 50  0001 C CNN
F 4 "311-30KDCT-ND" V 7400 7850 60  0001 C CNN "Part"
F 5 "DigiKey" V 7400 7850 60  0001 C CNN "Provider"
	1    7400 7850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5900 7650 7250 7650
Wire Wire Line
	5900 7850 7000 7850
Wire Wire Line
	7550 7450 7800 7450
Wire Wire Line
	7800 7450 7800 7650
Wire Wire Line
	7800 7850 7550 7850
Wire Wire Line
	7550 7650 7800 7650
Connection ~ 7800 7650
Wire Wire Line
	7800 7650 7800 7850
$Comp
L servo-sigma-48v-rescue:R-device R?
U 1 1 5BA4402E
P 7000 8400
AR Path="/58BF599E/5BA4402E" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5BA4402E" Ref="R11"  Part="1" 
F 0 "R11" V 7080 8400 50  0000 C CNN
F 1 "2.7k" V 7000 8400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6930 8400 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7000 8400 50  0001 C CNN
F 4 "311-2.7KDCT-ND" V 7000 8400 60  0001 C CNN "Part"
F 5 "DigiKey" V 7000 8400 60  0001 C CNN "Provider"
	1    7000 8400
	-1   0    0    1   
$EndComp
Wire Wire Line
	6150 8650 7000 8650
Wire Wire Line
	7000 8650 7000 8550
Connection ~ 6150 8650
Wire Wire Line
	7000 8250 7000 8100
Connection ~ 7000 7850
Wire Wire Line
	7000 7850 7250 7850
$Comp
L servo-sigma-48v-rescue:C-device C?
U 1 1 5BA6CF08
P 7400 8100
AR Path="/58BE2779/5BA6CF08" Ref="C?"  Part="1" 
AR Path="/58BE27E6/5BA6CF08" Ref="C37"  Part="1" 
F 0 "C37" V 7350 8200 50  0000 C CNN
F 1 "820pF/50V/10%" V 7550 8100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7438 7950 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2018/07/25/CL10B821KB8NNNC_1.pdf" H 7400 8100 50  0001 C CNN
F 4 "1276-2117-1-ND" V 7400 8100 60  0001 C CNN "Part"
F 5 "DigiKey" V 7400 8100 60  0001 C CNN "Provider"
	1    7400 8100
	0    1    1    0   
$EndComp
Wire Wire Line
	7250 8100 7000 8100
Connection ~ 7000 8100
Wire Wire Line
	7000 8100 7000 7850
Wire Wire Line
	7550 8100 7800 8100
Wire Wire Line
	7800 8100 7800 7850
Connection ~ 7800 7850
Wire Wire Line
	7000 8650 7800 8650
Wire Wire Line
	7800 8650 7800 9000
Connection ~ 7000 8650
Connection ~ 7800 9000
$Comp
L servo-sigma-48v-rescue:CP-device C38
U 1 1 5BA9F858
P 8200 8200
F 0 "C38" H 8225 8300 50  0000 L CNN
F 1 "100uF" H 8225 8100 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 8238 8050 50  0001 C CNN
F 3 "http://katalog.we-online.de/pbs/datasheet/865080545012.pdf" H 8200 8200 50  0001 C CNN
F 4 "732-8511-1-ND" H 8200 8200 60  0001 C CNN "Part"
F 5 "DigiKey" H 8200 8200 60  0001 C CNN "Provider"
	1    8200 8200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 7450 8200 7450
Wire Wire Line
	8200 7450 8200 8050
Connection ~ 7800 7450
Wire Wire Line
	7800 9000 8200 9000
Wire Wire Line
	8200 9000 8200 8350
$Comp
L servo-sigma-48v-rescue:CP-device C6
U 1 1 5BAAD8C5
P 3550 5150
F 0 "C6" H 3575 5250 50  0000 L CNN
F 1 "120uF" H 3575 5050 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 3588 5000 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDD0000/DMD0000COL92.pdf" H 3550 5150 50  0001 C CNN
F 4 "P19316CT-ND" H 3550 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 3550 5150 60  0001 C CNN "Provider"
	1    3550 5150
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:CP-device C9
U 1 1 5BAAD939
P 3850 5150
F 0 "C9" H 3875 5250 50  0000 L CNN
F 1 "120uF" H 3875 5050 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 3888 5000 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDD0000/DMD0000COL92.pdf" H 3850 5150 50  0001 C CNN
F 4 "P19316CT-ND" H 3850 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 3850 5150 60  0001 C CNN "Provider"
	1    3850 5150
	1    0    0    -1  
$EndComp
Connection ~ 8200 7450
Connection ~ 8200 9000
$Comp
L power:+15V #PWR051
U 1 1 5BAC2E35
P 8200 6950
F 0 "#PWR051" H 8200 6800 50  0001 C CNN
F 1 "+15V" H 8215 7123 50  0000 C CNN
F 2 "" H 8200 6950 50  0001 C CNN
F 3 "" H 8200 6950 50  0001 C CNN
	1    8200 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 7450 8200 6950
$Comp
L power:+5V #PWR055
U 1 1 5BAD8868
P 11150 6950
F 0 "#PWR055" H 11150 6800 50  0001 C CNN
F 1 "+5V" H 11165 7123 50  0000 C CNN
F 2 "" H 11150 6950 50  0001 C CNN
F 3 "" H 11150 6950 50  0001 C CNN
	1    11150 6950
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C40
U 1 1 5BAD953E
P 11150 8200
F 0 "C40" H 11000 8100 50  0000 L CNN
F 1 "4.7uF 25V" H 10750 8300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 11188 8050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 11150 8200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 11150 8200 60  0001 C CNN "Part"
F 5 "DigiKey" H 11150 8200 60  0001 C CNN "Provider"
	1    11150 8200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:TPS793333-EP-regul U2
U 1 1 5BAD9546
P 9500 7550
F 0 "U2" H 9500 7892 50  0000 C CNN
F 1 "TPS70950" H 9500 7801 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 9500 7875 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps709.pdf" H 9500 7600 50  0001 C CNN
F 4 "296-35483-1-ND" H 9500 7550 60  0001 C CNN "Part"
F 5 "DigiKey" H 9500 7550 60  0001 C CNN "Provider"
	1    9500 7550
	1    0    0    -1  
$EndComp
NoConn ~ 9800 7550
$Comp
L servo-sigma-48v-rescue:C-device C39
U 1 1 5BAD954F
P 8750 8200
F 0 "C39" H 8600 8100 50  0000 L CNN
F 1 "1uF 100V" H 8350 8300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8788 8050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 8750 8200 50  0001 C CNN
F 4 "445-8887-1-ND" H 8750 8200 60  0001 C CNN "Part"
F 5 "DigiKey" H 8750 8200 60  0001 C CNN "Provider"
	1    8750 8200
	-1   0    0    1   
$EndComp
Wire Wire Line
	8750 7450 8750 8050
Wire Wire Line
	11150 7450 11150 8050
Wire Wire Line
	8750 7450 9200 7450
NoConn ~ 9200 7550
Wire Wire Line
	9800 7450 11150 7450
Wire Wire Line
	8200 9000 8750 9000
Wire Wire Line
	8750 8350 8750 9000
Connection ~ 8750 9000
Wire Wire Line
	8750 9000 11150 9000
Wire Wire Line
	11150 8350 11150 8600
Wire Wire Line
	8200 7450 8750 7450
Connection ~ 8750 7450
Wire Wire Line
	9500 7850 9500 8600
Wire Wire Line
	9500 8600 11150 8600
Connection ~ 11150 8600
Wire Wire Line
	11150 8600 11150 9000
Wire Wire Line
	11150 6950 11150 7450
Connection ~ 11150 7450
Text Notes 13050 8250 0    60   ~ 0
TPS70933\n150 mA\n30V input\nInternal Enable pullup\n
$Comp
L power:+3V3 #PWR058
U 1 1 5BB77721
P 14100 6950
F 0 "#PWR058" H 14100 6800 50  0001 C CNN
F 1 "+3V3" H 14115 7123 50  0000 C CNN
F 2 "" H 14100 6950 50  0001 C CNN
F 3 "" H 14100 6950 50  0001 C CNN
	1    14100 6950
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR059
U 1 1 5BB77727
P 14450 6950
F 0 "#PWR059" H 14450 6800 50  0001 C CNN
F 1 "VDD" H 14467 7123 50  0000 C CNN
F 2 "" H 14450 6950 50  0001 C CNN
F 3 "" H 14450 6950 50  0001 C CNN
	1    14450 6950
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C42
U 1 1 5BB7772F
P 14450 8200
F 0 "C42" H 14300 8100 50  0000 L CNN
F 1 "4.7uF 25V" H 14050 8300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 14488 8050 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 14450 8200 50  0001 C CNN
F 4 "1276-2415-1-ND" H 14450 8200 60  0001 C CNN "Part"
F 5 "DigiKey" H 14450 8200 60  0001 C CNN "Provider"
	1    14450 8200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:TPS793333-EP-regul U8
U 1 1 5BB77737
P 12800 7550
F 0 "U8" H 12800 7892 50  0000 C CNN
F 1 "TPS70933" H 12800 7801 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 12800 7875 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps709.pdf" H 12800 7600 50  0001 C CNN
F 4 "296-35483-1-ND" H 12800 7550 60  0001 C CNN "Part"
F 5 "DigiKey" H 12800 7550 60  0001 C CNN "Provider"
	1    12800 7550
	1    0    0    -1  
$EndComp
NoConn ~ 13100 7550
$Comp
L servo-sigma-48v-rescue:C-device C41
U 1 1 5BB77740
P 12250 8200
F 0 "C41" H 12100 8100 50  0000 L CNN
F 1 "1uF 100V" H 11850 8300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 12288 8050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 12250 8200 50  0001 C CNN
F 4 "445-8887-1-ND" H 12250 8200 60  0001 C CNN "Part"
F 5 "DigiKey" H 12250 8200 60  0001 C CNN "Provider"
	1    12250 8200
	-1   0    0    1   
$EndComp
Wire Wire Line
	12250 7450 12250 8050
Wire Wire Line
	14450 7450 14450 8050
Wire Wire Line
	12250 7450 12500 7450
NoConn ~ 12500 7550
Wire Wire Line
	11150 9000 12250 9000
Wire Wire Line
	14450 8350 14450 8600
Connection ~ 11150 9000
Wire Wire Line
	12250 8350 12250 9000
Connection ~ 12250 9000
Wire Wire Line
	12250 9000 14450 9000
Wire Wire Line
	12800 8600 14450 8600
Wire Wire Line
	12800 7850 12800 8600
Connection ~ 14450 8600
Wire Wire Line
	14450 8600 14450 9000
Wire Wire Line
	13100 7450 14100 7450
Wire Wire Line
	14450 6950 14450 7450
Connection ~ 14450 7450
Wire Wire Line
	14100 6950 14100 7450
Connection ~ 14100 7450
Wire Wire Line
	14100 7450 14450 7450
$Comp
L power:+15V #PWR056
U 1 1 5BBF6A78
P 12250 6950
F 0 "#PWR056" H 12250 6800 50  0001 C CNN
F 1 "+15V" H 12265 7123 50  0000 C CNN
F 2 "" H 12250 6950 50  0001 C CNN
F 3 "" H 12250 6950 50  0001 C CNN
	1    12250 6950
	1    0    0    -1  
$EndComp
Connection ~ 12250 7450
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D3
U 1 1 5BC00502
P 12250 7200
F 0 "D3" H 12250 7300 50  0000 C CNN
F 1 "BAT30KFILM" H 12250 7100 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 12250 7200 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 12250 7200 50  0001 C CNN
F 4 "497-5552-1-ND" H 12250 7200 60  0001 C CNN "Part"
F 5 "DigiKey" H 12250 7200 60  0001 C CNN "Provider"
	1    12250 7200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	12250 7350 12250 7450
Wire Wire Line
	12250 6950 12250 7050
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D2
U 1 1 5BC2471B
P 11750 7200
F 0 "D2" H 11750 7300 50  0000 C CNN
F 1 "BAT30KFILM" H 11750 7100 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 11750 7200 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 11750 7200 50  0001 C CNN
F 4 "497-5552-1-ND" H 11750 7200 60  0001 C CNN "Part"
F 5 "DigiKey" H 11750 7200 60  0001 C CNN "Provider"
	1    11750 7200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11750 7350 11750 7450
Wire Wire Line
	11750 7450 12250 7450
Text HLabel 11750 6850 1    60   Input ~ 0
EXT_5V
Wire Wire Line
	11750 6850 11750 7050
Text Notes 9750 8250 0    60   ~ 0
TPS70950\n150 mA\n30V input\nInternal Enable pullup\n
Text HLabel 2700 1250 2    60   UnSpc ~ 0
PHASE_A
Text HLabel 2700 1150 2    60   UnSpc ~ 0
PHASE_B
Text HLabel 2700 1050 2    60   UnSpc ~ 0
PHASE_C
Wire Wire Line
	2700 1050 1900 1050
Wire Wire Line
	2700 1250 1900 1250
Wire Wire Line
	2700 1150 1900 1150
$Comp
L servo-sigma-48v-rescue:CONN_01X03-conn J3
U 1 1 5BC83965
P 1700 1150
F 0 "J3" H 1619 1465 50  0000 C CNN
F 1 "CONN_01X03" H 1619 1374 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_03x7.62mm_Vertical" H 1700 1150 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1704936.pdf" H 1700 1150 50  0001 C CNN
F 4 "277-5954-ND" H 1700 1150 50  0001 C CNN "Part"
F 5 "DigiKey" H 1700 1150 50  0001 C CNN "Provider"
	1    1700 1150
	-1   0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:CONN_01X03_MALE-conn J4
U 1 1 5BCC7E52
P 1750 2100
F 0 "J4" H 1856 2490 50  0000 C CNN
F 1 "CONN_01X03_MALE" H 1856 2399 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1750 2300 50  0001 C CNN
F 3 "" H 1750 2300 50  0001 C CNN
F 4 "732-5316-ND" H 1750 2100 50  0001 C CNN "Part"
F 5 "DigiKey" H 1750 2100 50  0001 C CNN "Provider"
	1    1750 2100
	1    0    0    -1  
$EndComp
Text Label 2900 1900 2    60   ~ 0
PWM_IN
Text Label 2900 2100 2    60   ~ 0
PWM_5V
Text Label 2900 2300 2    60   ~ 0
PWM_GND
Text HLabel 3000 1900 2    60   Output ~ 0
PWM
Wire Wire Line
	2050 1900 3000 1900
$Comp
L power:GND #PWR045
U 1 1 5BD194DC
P 3000 2300
F 0 "#PWR045" H 3000 2050 50  0001 C CNN
F 1 "GND" H 3000 2150 50  0000 C CNN
F 2 "" H 3000 2300 50  0001 C CNN
F 3 "" H 3000 2300 50  0001 C CNN
	1    3000 2300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 2300 3000 2300
$Comp
L power:+5V #PWR043
U 1 1 5BD23E44
P 3000 2100
F 0 "#PWR043" H 3000 1950 50  0001 C CNN
F 1 "+5V" H 3015 2273 50  0000 C CNN
F 2 "" H 3000 2100 50  0001 C CNN
F 3 "" H 3000 2100 50  0001 C CNN
	1    3000 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 2100 3000 2100
Wire Wire Line
	3250 6950 3250 7650
Connection ~ 3250 7650
Wire Wire Line
	4700 3350 5200 3350
Wire Wire Line
	5200 3350 5200 3700
Wire Wire Line
	5200 3050 5200 3350
Connection ~ 5200 3350
Wire Wire Line
	5200 4000 5200 4100
Wire Wire Line
	4700 3550 6400 3550
Wire Wire Line
	7350 4650 7900 4650
Wire Wire Line
	7350 4400 7350 4650
Connection ~ 7350 4650
Wire Wire Line
	3250 3050 3250 3650
Wire Wire Line
	3250 3650 3550 3650
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5BB0B70D
P 10400 3000
F 0 "#FLG0101" H 10400 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 10400 3173 50  0000 C CNN
F 2 "" H 10400 3000 50  0001 C CNN
F 3 "" H 10400 3000 50  0001 C CNN
	1    10400 3000
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR0101
U 1 1 5BB0B933
P 10400 2500
F 0 "#PWR0101" H 10400 2350 50  0001 C CNN
F 1 "+BATT" H 10400 2640 50  0000 C CNN
F 2 "" H 10400 2500 50  0001 C CNN
F 3 "" H 10400 2500 50  0001 C CNN
	1    10400 2500
	-1   0    0    -1  
$EndComp
$Comp
L power:+15V #PWR0102
U 1 1 5BB0BCCE
P 10800 2500
F 0 "#PWR0102" H 10800 2350 50  0001 C CNN
F 1 "+15V" H 10815 2673 50  0000 C CNN
F 2 "" H 10800 2500 50  0001 C CNN
F 3 "" H 10800 2500 50  0001 C CNN
	1    10800 2500
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5BB0C549
P 10800 3000
F 0 "#FLG0102" H 10800 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 10800 3173 50  0000 C CNN
F 2 "" H 10800 3000 50  0001 C CNN
F 3 "" H 10800 3000 50  0001 C CNN
	1    10800 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	10400 2500 10400 3000
Wire Wire Line
	10800 2500 10800 3000
$Comp
L power:GND #PWR0103
U 1 1 5BB2B601
P 11250 2500
F 0 "#PWR0103" H 11250 2250 50  0001 C CNN
F 1 "GND" H 11250 2350 50  0000 C CNN
F 2 "" H 11250 2500 50  0001 C CNN
F 3 "" H 11250 2500 50  0001 C CNN
	1    11250 2500
	1    0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5BB2B73C
P 11250 3000
F 0 "#FLG0103" H 11250 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 11250 3173 50  0000 C CNN
F 2 "" H 11250 3000 50  0001 C CNN
F 3 "" H 11250 3000 50  0001 C CNN
	1    11250 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	11250 2500 11250 3000
$Comp
L sigmadrone:VIN #PWR0104
U 1 1 5BB311C6
P 9950 2500
F 0 "#PWR0104" H 9950 2350 50  0001 C CNN
F 1 "VIN" H 9965 2673 50  0000 C CNN
F 2 "" H 9950 2500 50  0000 C CNN
F 3 "" H 9950 2500 50  0000 C CNN
	1    9950 2500
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5BB31327
P 9950 3000
F 0 "#FLG0104" H 9950 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 9950 3173 50  0000 C CNN
F 2 "" H 9950 3000 50  0001 C CNN
F 3 "" H 9950 3000 50  0001 C CNN
	1    9950 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	9950 2500 9950 3000
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 5BB36D89
P 11750 7600
F 0 "#FLG0105" H 11750 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 11750 7773 50  0000 C CNN
F 2 "" H 11750 7600 50  0001 C CNN
F 3 "" H 11750 7600 50  0001 C CNN
	1    11750 7600
	-1   0    0    1   
$EndComp
Wire Wire Line
	11750 7450 11750 7600
Connection ~ 11750 7450
Wire Wire Line
	6400 3450 6400 2450
Wire Wire Line
	6400 2450 6900 2450
Wire Wire Line
	6400 3550 6400 4650
Wire Wire Line
	6400 4650 7350 4650
Wire Wire Line
	1900 3350 2300 3350
Connection ~ 3550 3350
Wire Wire Line
	2300 3350 2300 3050
Connection ~ 2300 3350
Wire Wire Line
	2300 3350 3550 3350
$Comp
L power:GND #PWR0108
U 1 1 5BC053F5
P 2300 4100
F 0 "#PWR0108" H 2300 3850 50  0001 C CNN
F 1 "GND" H 2300 3950 50  0000 C CNN
F 2 "" H 2300 4100 50  0001 C CNN
F 3 "" H 2300 4100 50  0001 C CNN
	1    2300 4100
	-1   0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0109
U 1 1 5BC182A9
P 3250 4600
F 0 "#PWR0109" H 3250 4450 50  0001 C CNN
F 1 "+BATT" H 3250 4740 50  0000 C CNN
F 2 "" H 3250 4600 50  0001 C CNN
F 3 "" H 3250 4600 50  0001 C CNN
	1    3250 4600
	-1   0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0110
U 1 1 5BC1E3B1
P 3250 6950
F 0 "#PWR0110" H 3250 6800 50  0001 C CNN
F 1 "+BATT" H 3250 7090 50  0000 C CNN
F 2 "" H 3250 6950 50  0001 C CNN
F 3 "" H 3250 6950 50  0001 C CNN
	1    3250 6950
	-1   0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D?
U 1 1 5BBD07BC
P 6550 8400
AR Path="/58BF664D/5BBD07BC" Ref="D?"  Part="1" 
AR Path="/58BE27E6/5BBD07BC" Ref="D5"  Part="1" 
F 0 "D5" H 6550 8500 50  0000 C CNN
F 1 "STPS0560Z" H 6550 8300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 6550 8400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 6550 8400 50  0001 C CNN
F 4 "497-3787-1-ND" H 6550 8400 60  0001 C CNN "Part"
F 5 "DigiKey" H 6550 8400 60  0001 C CNN "Provider"
	1    6550 8400
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 8550 6550 9000
Connection ~ 6550 9000
Wire Wire Line
	6550 9000 7800 9000
Wire Wire Line
	6550 8250 6550 7450
$EndSCHEMATC
