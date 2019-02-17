EESchema Schematic File Version 4
LIBS:servo-sigma-48v-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 3 5
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
L power:GND #PWR025
U 1 1 58BF6CB7
P 1900 2750
F 0 "#PWR025" H 1900 2500 50  0001 C CNN
F 1 "GND" H 1900 2600 50  0000 C CNN
F 2 "" H 1900 2750 50  0001 C CNN
F 3 "" H 1900 2750 50  0001 C CNN
	1    1900 2750
	1    0    0    -1  
$EndComp
Text Label 3900 1950 2    60   ~ 0
PHASE_A
Text Label 3900 1850 2    60   ~ 0
GATE_AH
$Comp
L power:GND #PWR027
U 1 1 58BF987A
P 2750 6200
F 0 "#PWR027" H 2750 5950 50  0001 C CNN
F 1 "GND" H 2750 6050 50  0000 C CNN
F 2 "" H 2750 6200 50  0001 C CNN
F 3 "" H 2750 6200 50  0001 C CNN
	1    2750 6200
	1    0    0    -1  
$EndComp
Text HLabel 1000 1750 0    60   Input ~ 0
PWM_AL
Text Label 950  3800 0    60   ~ 0
GATE_AH
Text HLabel 3150 4300 2    60   Output ~ 0
PHASE_A
Text Label 7400 1950 2    60   ~ 0
PHASE_B
Text HLabel 4550 1750 0    60   Input ~ 0
PWM_BL
Text Label 7400 1850 2    60   ~ 0
GATE_BH
$Comp
L power:GND #PWR029
U 1 1 58C227F2
P 6100 6200
F 0 "#PWR029" H 6100 5950 50  0001 C CNN
F 1 "GND" H 6100 6050 50  0000 C CNN
F 2 "" H 6100 6200 50  0001 C CNN
F 3 "" H 6100 6200 50  0001 C CNN
	1    6100 6200
	1    0    0    -1  
$EndComp
Text HLabel 4550 1850 0    60   Input ~ 0
PWM_BH
Text Label 4150 3800 0    60   ~ 0
GATE_BH
Text HLabel 6500 4300 2    60   Output ~ 0
PHASE_B
$Comp
L power:GND #PWR031
U 1 1 58C22F8B
P 9600 6200
F 0 "#PWR031" H 9600 5950 50  0001 C CNN
F 1 "GND" H 9600 6050 50  0000 C CNN
F 2 "" H 9600 6200 50  0001 C CNN
F 3 "" H 9600 6200 50  0001 C CNN
	1    9600 6200
	1    0    0    -1  
$EndComp
Text Label 7650 3800 0    60   ~ 0
GATE_CH
Text HLabel 10000 4300 2    60   Output ~ 0
PHASE_C
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D14
U 1 1 58C323FC
P 2500 1350
F 0 "D14" H 2500 1450 50  0000 C CNN
F 1 "STPS0560Z" H 2500 1250 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 2500 1350 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 2500 1350 50  0001 C CNN
F 4 "497-3787-1-ND" H 2500 1350 60  0001 C CNN "Part"
F 5 "DigiKey" H 2500 1350 60  0001 C CNN "Provider"
	1    2500 1350
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C22
U 1 1 58D19DC3
P 6900 1650
F 0 "C22" V 6850 1700 50  0000 L CNN
F 1 "1uF 100V" V 6850 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6938 1500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 6900 1650 50  0001 C CNN
F 4 "445-8887-1-ND" H 6900 1650 60  0001 C CNN "Part"
F 5 "DigiKey" H 6900 1650 60  0001 C CNN "Provider"
	1    6900 1650
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C34
U 1 1 58D19E8B
P 3400 1650
F 0 "C34" V 3350 1750 50  0000 L CNN
F 1 "1uF 100V" V 3350 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3438 1500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3400 1650 50  0001 C CNN
F 4 "445-8887-1-ND" H 3400 1650 60  0001 C CNN "Part"
F 5 "DigiKey" H 3400 1650 60  0001 C CNN "Provider"
	1    3400 1650
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D11
U 1 1 58D1E9C4
P 6000 1350
F 0 "D11" H 6000 1450 50  0000 C CNN
F 1 "STPS0560Z" H 6000 1250 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 6000 1350 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 6000 1350 50  0001 C CNN
F 4 "497-3787-1-ND" H 6000 1350 60  0001 C CNN "Part"
F 5 "DigiKey" H 6000 1350 60  0001 C CNN "Provider"
	1    6000 1350
	-1   0    0    1   
$EndComp
Text Label 3900 2050 2    60   ~ 0
GATE_AL
Text Label 950  5050 0    60   ~ 0
GATE_AL
$Comp
L servo-sigma-48v-rescue:C-device C36
U 1 1 58EB6948
P 1150 2200
F 0 "C36" H 1000 2100 50  0000 L CNN
F 1 "1uF 100V" V 1200 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1188 2050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 1150 2200 50  0001 C CNN
F 4 "445-8887-1-ND" H 1150 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 1150 2200 60  0001 C CNN "Provider"
	1    1150 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR034
U 1 1 58EB6C09
P 1150 2750
F 0 "#PWR034" H 1150 2500 50  0001 C CNN
F 1 "GND" H 1150 2600 50  0000 C CNN
F 2 "" H 1150 2750 50  0001 C CNN
F 3 "" H 1150 2750 50  0001 C CNN
	1    1150 2750
	1    0    0    -1  
$EndComp
Text Label 7400 2050 2    60   ~ 0
GATE_BL
$Comp
L servo-sigma-48v-rescue:C-device C26
U 1 1 58EB8B61
P 4700 2200
F 0 "C26" H 4550 2100 50  0000 L CNN
F 1 "1uF 100V" V 4750 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4738 2050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 4700 2200 50  0001 C CNN
F 4 "445-8887-1-ND" H 4700 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 4700 2200 60  0001 C CNN "Provider"
	1    4700 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR035
U 1 1 58EB8B67
P 4700 2750
F 0 "#PWR035" H 4700 2500 50  0001 C CNN
F 1 "GND" H 4700 2600 50  0000 C CNN
F 2 "" H 4700 2750 50  0001 C CNN
F 3 "" H 4700 2750 50  0001 C CNN
	1    4700 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 58EB8D70
P 5450 2750
F 0 "#PWR036" H 5450 2500 50  0001 C CNN
F 1 "GND" H 5450 2600 50  0000 C CNN
F 2 "" H 5450 2750 50  0001 C CNN
F 3 "" H 5450 2750 50  0001 C CNN
	1    5450 2750
	1    0    0    -1  
$EndComp
Text Label 4150 5050 0    60   ~ 0
GATE_BL
Text Label 11100 1950 2    60   ~ 0
PHASE_C
Text HLabel 8100 1750 0    60   Input ~ 0
PWM_CL
Text Label 11100 1850 2    60   ~ 0
GATE_CH
Text HLabel 8100 1850 0    60   Input ~ 0
PWM_CH
$Comp
L servo-sigma-48v-rescue:C-device C17
U 1 1 58EB9997
P 10600 1650
F 0 "C17" V 10550 1700 50  0000 L CNN
F 1 "1uF 100V" V 10550 1200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10638 1500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 10600 1650 50  0001 C CNN
F 4 "445-8887-1-ND" H 10600 1650 60  0001 C CNN "Part"
F 5 "DigiKey" H 10600 1650 60  0001 C CNN "Provider"
	1    10600 1650
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D8
U 1 1 58EB999F
P 9700 1350
F 0 "D8" H 9700 1450 50  0000 C CNN
F 1 "STPS0560Z" H 9700 1250 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 9700 1350 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 9700 1350 50  0001 C CNN
F 4 "497-3787-1-ND" H 9700 1350 60  0001 C CNN "Part"
F 5 "DigiKey" H 9700 1350 60  0001 C CNN "Provider"
	1    9700 1350
	-1   0    0    1   
$EndComp
Text Label 11100 2050 2    60   ~ 0
GATE_CL
$Comp
L servo-sigma-48v-rescue:C-device C20
U 1 1 58EB99B4
P 8350 2200
F 0 "C20" H 8200 2100 50  0000 L CNN
F 1 "1uF 100V" V 8400 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8388 2050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 8350 2200 50  0001 C CNN
F 4 "445-8887-1-ND" H 8350 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 8350 2200 60  0001 C CNN "Provider"
	1    8350 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 58EB99BA
P 8350 2750
F 0 "#PWR037" H 8350 2500 50  0001 C CNN
F 1 "GND" H 8350 2600 50  0000 C CNN
F 2 "" H 8350 2750 50  0001 C CNN
F 3 "" H 8350 2750 50  0001 C CNN
	1    8350 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 58EB99D7
P 9100 2750
F 0 "#PWR038" H 9100 2500 50  0001 C CNN
F 1 "GND" H 9100 2600 50  0000 C CNN
F 2 "" H 9100 2750 50  0001 C CNN
F 3 "" H 9100 2750 50  0001 C CNN
	1    9100 2750
	1    0    0    -1  
$EndComp
Text Label 7650 5050 0    60   ~ 0
GATE_CL
$Comp
L sigmadrone:L6398 U7
U 1 1 58F12792
P 2550 1900
F 0 "U7" H 2750 2200 60  0000 C CNN
F 1 "L6398" H 2700 1600 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2550 1400 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 2550 1500 60  0001 C CNN
F 4 "497-16166-5-ND" H 2550 1900 60  0001 C CNN "Part"
F 5 "DigiKey" H 2550 1900 60  0001 C CNN "Provider"
	1    2550 1900
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:L6398 U6
U 1 1 58F12B9A
P 6050 1900
F 0 "U6" H 6250 2200 60  0000 C CNN
F 1 "L6398" H 6200 1600 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 6050 1400 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 6050 1500 60  0001 C CNN
F 4 "497-16166-5-ND" H 6050 1900 60  0001 C CNN "Part"
F 5 "DigiKey" H 6050 1900 60  0001 C CNN "Provider"
	1    6050 1900
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:L6398 U4
U 1 1 58F12D24
P 9700 1900
F 0 "U4" H 9900 2200 60  0000 C CNN
F 1 "L6398" H 9850 1600 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9700 1400 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 9700 1500 60  0001 C CNN
F 4 "497-16166-5-ND" H 9700 1900 60  0001 C CNN "Part"
F 5 "DigiKey" H 9700 1900 60  0001 C CNN "Provider"
	1    9700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3300 2750 3600
Wire Wire Line
	2750 4000 2750 4300
Wire Wire Line
	2750 5250 2750 5400
Wire Wire Line
	1000 1750 1650 1750
Wire Wire Line
	2350 4650 2350 5050
Connection ~ 2350 5050
Wire Wire Line
	2350 5450 2350 6100
Wire Wire Line
	2350 6100 2750 6100
Connection ~ 2750 6100
Wire Wire Line
	2350 4300 2750 4300
Connection ~ 2750 4300
Wire Wire Line
	6100 3300 6100 3600
Wire Wire Line
	6100 4000 6100 4300
Wire Wire Line
	6100 5250 6100 5350
Wire Wire Line
	5600 4650 5600 5050
Connection ~ 5600 5050
Wire Wire Line
	5600 5450 5600 6100
Wire Wire Line
	5600 6100 6100 6100
Connection ~ 6100 6100
Wire Wire Line
	5600 4300 6100 4300
Connection ~ 6100 4300
Wire Wire Line
	9600 3300 9600 3600
Wire Wire Line
	9600 4000 9600 4300
Wire Wire Line
	9600 5250 9600 5400
Wire Wire Line
	8600 3800 9100 3800
Wire Wire Line
	8600 5050 9100 5050
Wire Wire Line
	9100 4650 9100 5050
Connection ~ 9100 5050
Wire Wire Line
	9100 5450 9100 6100
Wire Wire Line
	9100 6100 9600 6100
Connection ~ 9600 6100
Wire Wire Line
	9100 4300 9600 4300
Connection ~ 9600 4300
Wire Wire Line
	3100 1950 3400 1950
Wire Wire Line
	3100 2050 3900 2050
Wire Wire Line
	2000 2050 1900 2050
Wire Wire Line
	1900 2050 1900 2750
Wire Wire Line
	2650 1350 3150 1350
Wire Wire Line
	3100 1850 3900 1850
Wire Wire Line
	1150 750  1150 1350
Wire Wire Line
	1150 1950 2000 1950
Connection ~ 1150 1350
Wire Wire Line
	1150 2350 1150 2750
Wire Wire Line
	6600 1750 6700 1750
Wire Wire Line
	6700 1750 6700 1350
Connection ~ 6700 1350
Wire Wire Line
	6600 1850 7400 1850
Wire Wire Line
	6600 1950 6900 1950
Wire Wire Line
	6600 2050 7400 2050
Wire Wire Line
	4550 1750 5250 1750
Wire Wire Line
	4550 1850 5000 1850
Wire Wire Line
	4700 850  4700 1350
Wire Wire Line
	4700 1950 5500 1950
Connection ~ 4700 1350
Wire Wire Line
	4700 2350 4700 2750
Wire Wire Line
	5450 2750 5450 2050
Wire Wire Line
	5450 2050 5500 2050
Wire Wire Line
	10250 1850 11100 1850
Wire Wire Line
	10250 1950 10600 1950
Wire Wire Line
	10250 2050 11100 2050
Wire Wire Line
	8100 1750 8850 1750
Wire Wire Line
	8100 1850 8600 1850
Wire Wire Line
	8350 850  8350 1350
Wire Wire Line
	8350 1950 9150 1950
Connection ~ 8350 1350
Wire Wire Line
	8350 2350 8350 2750
Wire Wire Line
	9100 2750 9100 2050
Wire Wire Line
	9100 2050 9150 2050
Wire Wire Line
	10250 1750 10350 1750
Wire Wire Line
	10350 1750 10350 1350
Connection ~ 10350 1350
Connection ~ 8350 1950
Connection ~ 4700 1950
Connection ~ 1150 1950
$Comp
L servo-sigma-48v-rescue:R-device R42
U 1 1 59D34BBD
P 2350 4050
F 0 "R42" V 2430 4050 50  0000 C CNN
F 1 "10k" V 2350 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2280 4050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2350 4050 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2350 4050 60  0001 C CNN "Part"
F 5 "DigiKey" V 2350 4050 60  0001 C CNN "Provider"
	1    2350 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4200 2350 4300
Wire Wire Line
	2350 3400 2350 3800
Connection ~ 2350 3800
Wire Wire Line
	5600 3400 5600 3800
Connection ~ 5600 3800
Wire Wire Line
	5600 4200 5600 4300
Wire Wire Line
	9100 4200 9100 4300
Wire Wire Line
	9100 3400 9100 3800
Connection ~ 9100 3800
$Comp
L servo-sigma-48v-rescue:R-device R32
U 1 1 59D368B1
P 5600 4050
F 0 "R32" V 5680 4050 50  0000 C CNN
F 1 "10k" V 5600 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5530 4050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5600 4050 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 5600 4050 60  0001 C CNN "Part"
F 5 "DigiKey" V 5600 4050 60  0001 C CNN "Provider"
	1    5600 4050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R22
U 1 1 59D36CCF
P 9100 4050
F 0 "R22" V 9180 4050 50  0000 C CNN
F 1 "10k" V 9100 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9030 4050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9100 4050 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 9100 4050 60  0001 C CNN "Part"
F 5 "DigiKey" V 9100 4050 60  0001 C CNN "Provider"
	1    9100 4050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R56
U 1 1 59D5BB5C
P 2350 5300
F 0 "R56" V 2430 5300 50  0000 C CNN
F 1 "10k" V 2350 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2280 5300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2350 5300 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2350 5300 60  0001 C CNN "Part"
F 5 "DigiKey" V 2350 5300 60  0001 C CNN "Provider"
	1    2350 5300
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R37
U 1 1 59D5BE2E
P 5600 5300
F 0 "R37" V 5680 5300 50  0000 C CNN
F 1 "10k" V 5600 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5530 5300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5600 5300 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 5600 5300 60  0001 C CNN "Part"
F 5 "DigiKey" V 5600 5300 60  0001 C CNN "Provider"
	1    5600 5300
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R27
U 1 1 59D5C1C0
P 9100 5300
F 0 "R27" V 9180 5300 50  0000 C CNN
F 1 "10k" V 9100 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9030 5300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9100 5300 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 9100 5300 60  0001 C CNN "Part"
F 5 "DigiKey" V 9100 5300 60  0001 C CNN "Provider"
	1    9100 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1750 3150 1750
Wire Wire Line
	3150 1750 3150 1350
Connection ~ 3150 1350
Wire Wire Line
	3400 1350 3400 1500
Wire Wire Line
	3400 1800 3400 1950
Connection ~ 3400 1950
$Comp
L servo-sigma-48v-rescue:C-device C21
U 1 1 59FAAC4B
P 5250 2200
F 0 "C21" H 5100 2100 50  0000 L CNN
F 1 "100pF 100V" V 5300 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5288 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 5250 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 5250 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 5250 2200 60  0001 C CNN "Provider"
	1    5250 2200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C23
U 1 1 59FAAC53
P 5000 2200
F 0 "C23" H 4850 2100 50  0000 L CNN
F 1 "100pF 100V" V 5050 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5038 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 5000 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 5000 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 5000 2200 60  0001 C CNN "Provider"
	1    5000 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR078
U 1 1 59FAAC59
P 5250 2750
F 0 "#PWR078" H 5250 2500 50  0001 C CNN
F 1 "GND" H 5250 2600 50  0000 C CNN
F 2 "" H 5250 2750 50  0001 C CNN
F 3 "" H 5250 2750 50  0001 C CNN
	1    5250 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR077
U 1 1 59FAAC5F
P 5000 2750
F 0 "#PWR077" H 5000 2500 50  0001 C CNN
F 1 "GND" H 5000 2600 50  0000 C CNN
F 2 "" H 5000 2750 50  0001 C CNN
F 3 "" H 5000 2750 50  0001 C CNN
	1    5000 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2750 5000 2350
Wire Wire Line
	5250 2750 5250 2350
Text HLabel 1000 1850 0    60   Input ~ 0
PWM_AH
Wire Wire Line
	1000 1850 1400 1850
$Comp
L servo-sigma-48v-rescue:C-device C33
U 1 1 59FAB71D
P 1650 2200
F 0 "C33" H 1500 2100 50  0000 L CNN
F 1 "100pF 100V" V 1700 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1688 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 1650 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 1650 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 1650 2200 60  0001 C CNN "Provider"
	1    1650 2200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C35
U 1 1 59FAB725
P 1400 2200
F 0 "C35" H 1250 2100 50  0000 L CNN
F 1 "100pF 100V" V 1450 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1438 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 1400 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 1400 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 1400 2200 60  0001 C CNN "Provider"
	1    1400 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR074
U 1 1 59FAB72B
P 1650 2750
F 0 "#PWR074" H 1650 2500 50  0001 C CNN
F 1 "GND" H 1650 2600 50  0000 C CNN
F 2 "" H 1650 2750 50  0001 C CNN
F 3 "" H 1650 2750 50  0001 C CNN
	1    1650 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR042
U 1 1 59FAB731
P 1400 2750
F 0 "#PWR042" H 1400 2500 50  0001 C CNN
F 1 "GND" H 1400 2600 50  0000 C CNN
F 2 "" H 1400 2750 50  0001 C CNN
F 3 "" H 1400 2750 50  0001 C CNN
	1    1400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2750 1400 2350
Wire Wire Line
	1650 2750 1650 2350
Wire Wire Line
	6150 1350 6700 1350
Wire Wire Line
	6900 1350 6900 1500
Wire Wire Line
	6900 1800 6900 1950
Connection ~ 6900 1950
$Comp
L servo-sigma-48v-rescue:C-device C16
U 1 1 59FACD6A
P 8850 2200
F 0 "C16" H 8700 2100 50  0000 L CNN
F 1 "100pF 100V" V 8900 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8888 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 8850 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 8850 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 8850 2200 60  0001 C CNN "Provider"
	1    8850 2200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C18
U 1 1 59FACD72
P 8600 2200
F 0 "C18" H 8450 2100 50  0000 L CNN
F 1 "100pF 100V" V 8650 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8638 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 8600 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 8600 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 8600 2200 60  0001 C CNN "Provider"
	1    8600 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR080
U 1 1 59FACD78
P 8850 2750
F 0 "#PWR080" H 8850 2500 50  0001 C CNN
F 1 "GND" H 8850 2600 50  0000 C CNN
F 2 "" H 8850 2750 50  0001 C CNN
F 3 "" H 8850 2750 50  0001 C CNN
	1    8850 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR079
U 1 1 59FACD7E
P 8600 2750
F 0 "#PWR079" H 8600 2500 50  0001 C CNN
F 1 "GND" H 8600 2600 50  0000 C CNN
F 2 "" H 8600 2750 50  0001 C CNN
F 3 "" H 8600 2750 50  0001 C CNN
	1    8600 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 2750 8600 2350
Wire Wire Line
	8850 2750 8850 2350
Wire Wire Line
	9850 1350 10350 1350
Wire Wire Line
	10600 1350 10600 1500
Wire Wire Line
	10600 1800 10600 1950
Connection ~ 10600 1950
Wire Wire Line
	1400 2050 1400 1850
Connection ~ 1400 1850
Wire Wire Line
	1650 2050 1650 1750
Connection ~ 1650 1750
Wire Wire Line
	5000 2050 5000 1850
Connection ~ 5000 1850
Wire Wire Line
	5250 2050 5250 1750
Connection ~ 5250 1750
Wire Wire Line
	8600 2050 8600 1850
Connection ~ 8600 1850
Wire Wire Line
	8850 2050 8850 1750
Connection ~ 8850 1750
Wire Wire Line
	1950 1350 2350 1350
Wire Wire Line
	950  5050 1450 5050
Wire Wire Line
	1950 5050 2350 5050
Wire Wire Line
	950  3800 1450 3800
Wire Wire Line
	1950 3800 2350 3800
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D16
U 1 1 5A191D28
P 2100 3400
F 0 "D16" H 2100 3500 50  0000 C CNN
F 1 "BAT30KFILM" H 2100 3300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 2100 3400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 2100 3400 50  0001 C CNN
F 4 "497-5552-1-ND" H 2100 3400 60  0001 C CNN "Part"
F 5 "DigiKey" H 2100 3400 60  0001 C CNN "Provider"
	1    2100 3400
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R55
U 1 1 5A19228E
P 1700 3400
F 0 "R55" V 1780 3400 50  0000 C CNN
F 1 "5" V 1700 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1700 3400 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1700 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 1700 3400 60  0001 C CNN "Provider"
	1    1700 3400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D15
U 1 1 5A192746
P 2100 4650
F 0 "D15" H 2100 4750 50  0000 C CNN
F 1 "BAT30KFILM" H 2100 4550 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 2100 4650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 2100 4650 50  0001 C CNN
F 4 "497-5552-1-ND" H 2100 4650 60  0001 C CNN "Part"
F 5 "DigiKey" H 2100 4650 60  0001 C CNN "Provider"
	1    2100 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4650 2350 4650
Wire Wire Line
	1850 4650 1950 4650
Wire Wire Line
	1550 4650 1450 4650
Wire Wire Line
	1450 4650 1450 5050
Connection ~ 1450 5050
Wire Wire Line
	2250 3400 2350 3400
Wire Wire Line
	1550 3400 1450 3400
Wire Wire Line
	1450 3400 1450 3800
Connection ~ 1450 3800
Wire Wire Line
	1850 3400 1950 3400
Wire Wire Line
	5450 1350 5850 1350
Wire Wire Line
	4150 3800 4650 3800
Wire Wire Line
	5100 3800 5600 3800
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D13
U 1 1 5A1F9CD5
P 5300 3400
F 0 "D13" H 5300 3500 50  0000 C CNN
F 1 "BAT30KFILM" H 5300 3300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 5300 3400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 5300 3400 50  0001 C CNN
F 4 "497-5552-1-ND" H 5300 3400 60  0001 C CNN "Part"
F 5 "DigiKey" H 5300 3400 60  0001 C CNN "Provider"
	1    5300 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3400 5600 3400
Wire Wire Line
	4750 3400 4650 3400
Wire Wire Line
	4650 3400 4650 3800
Connection ~ 4650 3800
Wire Wire Line
	5050 3400 5150 3400
Wire Wire Line
	4150 5050 4650 5050
Wire Wire Line
	5100 5050 5600 5050
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D12
U 1 1 5A2089BC
P 5300 4650
F 0 "D12" H 5300 4750 50  0000 C CNN
F 1 "BAT30KFILM" H 5300 4550 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 5300 4650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 5300 4650 50  0001 C CNN
F 4 "497-5552-1-ND" H 5300 4650 60  0001 C CNN "Part"
F 5 "DigiKey" H 5300 4650 60  0001 C CNN "Provider"
	1    5300 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 4650 5600 4650
Wire Wire Line
	4750 4650 4650 4650
Wire Wire Line
	5050 4650 5150 4650
Wire Wire Line
	4650 4650 4650 5050
Connection ~ 4650 5050
Wire Wire Line
	7650 3800 8150 3800
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D10
U 1 1 5A22B887
P 8800 3400
F 0 "D10" H 8800 3500 50  0000 C CNN
F 1 "BAT30KFILM" H 8800 3300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 8800 3400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 8800 3400 50  0001 C CNN
F 4 "497-5552-1-ND" H 8800 3400 60  0001 C CNN "Part"
F 5 "DigiKey" H 8800 3400 60  0001 C CNN "Provider"
	1    8800 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3400 9100 3400
Wire Wire Line
	8250 3400 8150 3400
Wire Wire Line
	8150 3400 8150 3800
Connection ~ 8150 3800
Wire Wire Line
	8550 3400 8650 3400
Wire Wire Line
	7650 5050 8150 5050
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D9
U 1 1 5A22BAC7
P 8800 4650
F 0 "D9" H 8800 4750 50  0000 C CNN
F 1 "BAT30KFILM" H 8800 4550 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 8800 4650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 8800 4650 50  0001 C CNN
F 4 "497-5552-1-ND" H 8800 4650 60  0001 C CNN "Part"
F 5 "DigiKey" H 8800 4650 60  0001 C CNN "Provider"
	1    8800 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 4650 9100 4650
Wire Wire Line
	8250 4650 8150 4650
Wire Wire Line
	8150 4650 8150 5050
Connection ~ 8150 5050
Wire Wire Line
	8550 4650 8650 4650
Wire Wire Line
	9150 1350 9550 1350
$Comp
L servo-sigma-48v-rescue:R-device R53
U 1 1 5A2D51A9
P 1800 3800
F 0 "R53" V 1880 3800 50  0000 C CNN
F 1 "21" V 1800 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1730 3800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 3800 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1800 3800 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 3800 60  0001 C CNN "Provider"
	1    1800 3800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R54
U 1 1 5A2D5E24
P 1700 4650
F 0 "R54" V 1780 4650 50  0000 C CNN
F 1 "5" V 1700 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1630 4650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1700 4650 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1700 4650 60  0001 C CNN "Part"
F 5 "DigiKey" V 1700 4650 60  0001 C CNN "Provider"
	1    1700 4650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R43
U 1 1 5A2D5ED2
P 1800 5050
F 0 "R43" V 1880 5050 50  0000 C CNN
F 1 "21" V 1800 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1730 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1800 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 5050 60  0001 C CNN "Provider"
	1    1800 5050
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R36
U 1 1 5A2D8C1D
P 4900 3400
F 0 "R36" V 4980 3400 50  0000 C CNN
F 1 "5" V 4900 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4830 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4900 3400 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 4900 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 4900 3400 60  0001 C CNN "Provider"
	1    4900 3400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R34
U 1 1 5A2D8C25
P 4950 3800
F 0 "R34" V 5030 3800 50  0000 C CNN
F 1 "21" V 4950 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4880 3800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4950 3800 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 4950 3800 60  0001 C CNN "Part"
F 5 "DigiKey" V 4950 3800 60  0001 C CNN "Provider"
	1    4950 3800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R35
U 1 1 5A2D8C2D
P 4900 4650
F 0 "R35" V 4980 4650 50  0000 C CNN
F 1 "5" V 4900 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4830 4650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4900 4650 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 4900 4650 60  0001 C CNN "Part"
F 5 "DigiKey" V 4900 4650 60  0001 C CNN "Provider"
	1    4900 4650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R33
U 1 1 5A2D8C35
P 4950 5050
F 0 "R33" V 5030 5050 50  0000 C CNN
F 1 "21" V 4950 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4880 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4950 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 4950 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 4950 5050 60  0001 C CNN "Provider"
	1    4950 5050
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R23
U 1 1 5A2D9A3D
P 8450 5050
F 0 "R23" V 8530 5050 50  0000 C CNN
F 1 "21" V 8450 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8380 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8450 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 8450 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 8450 5050 60  0001 C CNN "Provider"
	1    8450 5050
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R25
U 1 1 5A2D9A35
P 8400 4650
F 0 "R25" V 8480 4650 50  0000 C CNN
F 1 "5" V 8400 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8330 4650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8400 4650 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 8400 4650 60  0001 C CNN "Part"
F 5 "DigiKey" V 8400 4650 60  0001 C CNN "Provider"
	1    8400 4650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R24
U 1 1 5A2D9A2D
P 8450 3800
F 0 "R24" V 8530 3800 50  0000 C CNN
F 1 "21" V 8450 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8380 3800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8450 3800 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 8450 3800 60  0001 C CNN "Part"
F 5 "DigiKey" V 8450 3800 60  0001 C CNN "Provider"
	1    8450 3800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R26
U 1 1 5A2D9A25
P 8400 3400
F 0 "R26" V 8480 3400 50  0000 C CNN
F 1 "5" V 8400 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8330 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8400 3400 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 8400 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 8400 3400 60  0001 C CNN "Provider"
	1    8400 3400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R39
U 1 1 5A2FFED6
P 1800 1350
F 0 "R39" V 1880 1350 50  0000 C CNN
F 1 "5" V 1800 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1730 1350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 1350 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1800 1350 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 1350 60  0001 C CNN "Provider"
	1    1800 1350
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R29
U 1 1 5A3001D5
P 5300 1350
F 0 "R29" V 5380 1350 50  0000 C CNN
F 1 "5" V 5300 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5230 1350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5300 1350 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 5300 1350 60  0001 C CNN "Part"
F 5 "DigiKey" V 5300 1350 60  0001 C CNN "Provider"
	1    5300 1350
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R18
U 1 1 5A30065C
P 9000 1350
F 0 "R18" V 9080 1350 50  0000 C CNN
F 1 "5" V 9000 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8930 1350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9000 1350 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 9000 1350 60  0001 C CNN "Part"
F 5 "DigiKey" V 9000 1350 60  0001 C CNN "Provider"
	1    9000 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 5050 2350 5150
Wire Wire Line
	2350 5050 2450 5050
Wire Wire Line
	2750 6100 2750 6200
Wire Wire Line
	2750 4300 2750 4850
Wire Wire Line
	5600 5050 5600 5150
Wire Wire Line
	5600 5050 5800 5050
Wire Wire Line
	6100 6100 6100 6200
Wire Wire Line
	6100 4300 6100 4850
Wire Wire Line
	9100 5050 9300 5050
Wire Wire Line
	9100 5050 9100 5150
Wire Wire Line
	9600 6100 9600 6200
Wire Wire Line
	9600 4300 9600 4850
Wire Wire Line
	1150 1350 1150 1950
Wire Wire Line
	1150 1350 1650 1350
Wire Wire Line
	6700 1350 6900 1350
Wire Wire Line
	4700 1350 4700 1950
Wire Wire Line
	4700 1350 5150 1350
Wire Wire Line
	8350 1350 8350 1950
Wire Wire Line
	8350 1350 8850 1350
Wire Wire Line
	10350 1350 10600 1350
Wire Wire Line
	8350 1950 8350 2050
Wire Wire Line
	4700 1950 4700 2050
Wire Wire Line
	1150 1950 1150 2050
Wire Wire Line
	2350 3800 2350 3900
Wire Wire Line
	2350 3800 2450 3800
Wire Wire Line
	5600 3800 5600 3900
Wire Wire Line
	5600 3800 5800 3800
Wire Wire Line
	9100 3800 9300 3800
Wire Wire Line
	9100 3800 9100 3900
Wire Wire Line
	3150 1350 3400 1350
Wire Wire Line
	3400 1950 3900 1950
Wire Wire Line
	6900 1950 7400 1950
Wire Wire Line
	10600 1950 11100 1950
Wire Wire Line
	1400 1850 2000 1850
Wire Wire Line
	1650 1750 2000 1750
Wire Wire Line
	5000 1850 5500 1850
Wire Wire Line
	5250 1750 5500 1750
Wire Wire Line
	8600 1850 9150 1850
Wire Wire Line
	8850 1750 9150 1750
Wire Wire Line
	1450 5050 1650 5050
Wire Wire Line
	1450 3800 1650 3800
Wire Wire Line
	4650 3800 4800 3800
Wire Wire Line
	4650 5050 4800 5050
Wire Wire Line
	8150 3800 8300 3800
Wire Wire Line
	8150 5050 8300 5050
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q5
U 1 1 5A928DF8
P 2650 3800
F 0 "Q5" H 2850 3850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 2850 3750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 2850 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 2650 3800 50  0001 C CNN
F 4 "DigiKey" H 2650 3800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 2650 3800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 2650 3800 50  0001 C CNN "Value1"
	1    2650 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4300 6500 4300
Wire Wire Line
	2750 4300 3150 4300
Wire Wire Line
	9600 4300 10000 4300
$Comp
L power:+15V #PWR0105
U 1 1 5BB538F3
P 1150 750
F 0 "#PWR0105" H 1150 600 50  0001 C CNN
F 1 "+15V" H 1165 923 50  0000 C CNN
F 2 "" H 1150 750 50  0001 C CNN
F 3 "" H 1150 750 50  0001 C CNN
	1    1150 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR0106
U 1 1 5BB53A37
P 4700 850
F 0 "#PWR0106" H 4700 700 50  0001 C CNN
F 1 "+15V" H 4715 1023 50  0000 C CNN
F 2 "" H 4700 850 50  0001 C CNN
F 3 "" H 4700 850 50  0001 C CNN
	1    4700 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR0107
U 1 1 5BB53C40
P 8350 850
F 0 "#PWR0107" H 8350 700 50  0001 C CNN
F 1 "+15V" H 8365 1023 50  0000 C CNN
F 2 "" H 8350 850 50  0001 C CNN
F 3 "" H 8350 850 50  0001 C CNN
	1    8350 850 
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R67
U 1 1 5BC9D211
P 2750 5700
F 0 "R67" V 2830 5700 50  0000 C CNN
F 1 "0.010 1%" V 2650 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 2680 5700 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2750 5700 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 2750 5700 60  0001 C CNN "Part"
F 5 "DigiKey" V 2750 5700 60  0001 C CNN "Provider"
	1    2750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 5850 2750 6100
$Comp
L servo-sigma-48v-rescue:R-device R68
U 1 1 5BC9D51F
P 6100 5700
F 0 "R68" V 6180 5700 50  0000 C CNN
F 1 "0.010 1%" V 6000 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 6030 5700 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 6100 5700 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 6100 5700 60  0001 C CNN "Part"
F 5 "DigiKey" V 6100 5700 60  0001 C CNN "Provider"
	1    6100 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5850 6100 6100
$Comp
L servo-sigma-48v-rescue:R-device R69
U 1 1 5BC9D5F5
P 9600 5700
F 0 "R69" V 9680 5700 50  0000 C CNN
F 1 "0.010 1%" V 9500 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 9530 5700 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9600 5700 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 9600 5700 60  0001 C CNN "Part"
F 5 "DigiKey" V 9600 5700 60  0001 C CNN "Provider"
	1    9600 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 5850 9600 6100
Text HLabel 3150 5400 2    60   Output ~ 0
A_ISENSE_P
Text HLabel 3150 6100 2    60   Output ~ 0
A_ISENSE_N
Text HLabel 6500 5350 2    60   Output ~ 0
B_ISENSE_P
Text HLabel 6500 6100 2    60   Output ~ 0
B_ISENSE_N
Text HLabel 10000 5400 2    60   Output ~ 0
C_ISENSE_P
Text HLabel 10000 6100 2    60   Output ~ 0
C_ISENSE_N
Wire Wire Line
	10000 5400 9600 5400
Connection ~ 9600 5400
Wire Wire Line
	9600 5400 9600 5550
Wire Wire Line
	6500 6100 6100 6100
Wire Wire Line
	3150 5400 2750 5400
Connection ~ 2750 5400
Wire Wire Line
	2750 5400 2750 5550
Wire Wire Line
	6500 5350 6100 5350
Connection ~ 6100 5350
Wire Wire Line
	6100 5350 6100 5550
$Comp
L power:GND #PWR070
U 1 1 5BC7FFE6
P 13550 6200
F 0 "#PWR070" H 13550 5950 50  0001 C CNN
F 1 "GND" H 13550 6050 50  0000 C CNN
F 2 "" H 13550 6200 50  0001 C CNN
F 3 "" H 13550 6200 50  0001 C CNN
	1    13550 6200
	1    0    0    -1  
$EndComp
Text Label 11600 3800 0    60   ~ 0
GATE_RH
Text Label 15050 1950 2    60   ~ 0
PHASE_R
Text HLabel 12050 1750 0    60   Input ~ 0
AUX_L
Text Label 15050 1850 2    60   ~ 0
GATE_RH
Text HLabel 12050 1850 0    60   Input ~ 0
AUX_H
$Comp
L servo-sigma-48v-rescue:C-device C50
U 1 1 5BC7FFF4
P 14550 1650
F 0 "C50" V 14500 1700 50  0000 L CNN
F 1 "1uF 100V" V 14500 1200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 14588 1500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 14550 1650 50  0001 C CNN
F 4 "445-8887-1-ND" H 14550 1650 60  0001 C CNN "Part"
F 5 "DigiKey" H 14550 1650 60  0001 C CNN "Provider"
	1    14550 1650
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D17
U 1 1 5BC7FFFC
P 13650 1350
F 0 "D17" H 13650 1450 50  0000 C CNN
F 1 "STPS0560Z" H 13650 1250 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 13650 1350 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 13650 1350 50  0001 C CNN
F 4 "497-3787-1-ND" H 13650 1350 60  0001 C CNN "Part"
F 5 "DigiKey" H 13650 1350 60  0001 C CNN "Provider"
	1    13650 1350
	-1   0    0    1   
$EndComp
Text Label 15050 2050 2    60   ~ 0
GATE_RL
$Comp
L servo-sigma-48v-rescue:C-device C47
U 1 1 5BC80005
P 12300 2200
F 0 "C47" H 12150 2100 50  0000 L CNN
F 1 "1uF 100V" V 12350 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 12338 2050 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 12300 2200 50  0001 C CNN
F 4 "445-8887-1-ND" H 12300 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 12300 2200 60  0001 C CNN "Provider"
	1    12300 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR064
U 1 1 5BC8000B
P 12300 2750
F 0 "#PWR064" H 12300 2500 50  0001 C CNN
F 1 "GND" H 12300 2600 50  0000 C CNN
F 2 "" H 12300 2750 50  0001 C CNN
F 3 "" H 12300 2750 50  0001 C CNN
	1    12300 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR068
U 1 1 5BC80011
P 13050 2750
F 0 "#PWR068" H 13050 2500 50  0001 C CNN
F 1 "GND" H 13050 2600 50  0000 C CNN
F 2 "" H 13050 2750 50  0001 C CNN
F 3 "" H 13050 2750 50  0001 C CNN
	1    13050 2750
	1    0    0    -1  
$EndComp
Text Label 11600 5050 0    60   ~ 0
GATE_RL
$Comp
L sigmadrone:L6398 U11
U 1 1 5BC8001A
P 13650 1900
F 0 "U11" H 13850 2200 60  0000 C CNN
F 1 "L6398" H 13800 1600 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 13650 1400 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 13650 1500 60  0001 C CNN
F 4 "497-16166-5-ND" H 13650 1900 60  0001 C CNN "Part"
F 5 "DigiKey" H 13650 1900 60  0001 C CNN "Provider"
	1    13650 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	13550 3300 13550 3600
Wire Wire Line
	13550 4000 13550 4300
Wire Wire Line
	12550 3800 13050 3800
Wire Wire Line
	12550 5050 13050 5050
Wire Wire Line
	13050 4650 13050 5050
Connection ~ 13050 5050
Wire Wire Line
	13050 5450 13050 6100
Wire Wire Line
	13050 6100 13550 6100
Connection ~ 13550 6100
Wire Wire Line
	13050 4300 13550 4300
Connection ~ 13550 4300
Wire Wire Line
	14200 1850 15050 1850
Wire Wire Line
	14200 1950 14550 1950
Wire Wire Line
	14200 2050 15050 2050
Wire Wire Line
	12050 1750 12800 1750
Wire Wire Line
	12050 1850 12550 1850
Wire Wire Line
	12300 850  12300 1350
Wire Wire Line
	12300 1950 13100 1950
Connection ~ 12300 1350
Wire Wire Line
	12300 2350 12300 2750
Wire Wire Line
	13050 2750 13050 2050
Wire Wire Line
	13050 2050 13100 2050
Wire Wire Line
	14200 1750 14300 1750
Wire Wire Line
	14300 1750 14300 1350
Connection ~ 14300 1350
Connection ~ 12300 1950
Wire Wire Line
	13050 4200 13050 4300
Wire Wire Line
	13050 3400 13050 3800
Connection ~ 13050 3800
$Comp
L servo-sigma-48v-rescue:R-device R79
U 1 1 5BC80040
P 13050 4050
F 0 "R79" V 13130 4050 50  0000 C CNN
F 1 "10k" V 13050 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12980 4050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 13050 4050 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 13050 4050 60  0001 C CNN "Part"
F 5 "DigiKey" V 13050 4050 60  0001 C CNN "Provider"
	1    13050 4050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R80
U 1 1 5BC80048
P 13050 5300
F 0 "R80" V 13130 5300 50  0000 C CNN
F 1 "10k" V 13050 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12980 5300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 13050 5300 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 13050 5300 60  0001 C CNN "Part"
F 5 "DigiKey" V 13050 5300 60  0001 C CNN "Provider"
	1    13050 5300
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C49
U 1 1 5BC80050
P 12800 2200
F 0 "C49" H 12650 2100 50  0000 L CNN
F 1 "100pF 100V" V 12850 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12838 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 12800 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 12800 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 12800 2200 60  0001 C CNN "Provider"
	1    12800 2200
	-1   0    0    1   
$EndComp
$Comp
L servo-sigma-48v-rescue:C-device C48
U 1 1 5BC80058
P 12550 2200
F 0 "C48" H 12400 2100 50  0000 L CNN
F 1 "100pF 100V" V 12600 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 12588 2050 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 12550 2200 50  0001 C CNN
F 4 "399-7821-2-ND" H 12550 2200 60  0001 C CNN "Part"
F 5 "DigiKey" H 12550 2200 60  0001 C CNN "Provider"
	1    12550 2200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR067
U 1 1 5BC8005E
P 12800 2750
F 0 "#PWR067" H 12800 2500 50  0001 C CNN
F 1 "GND" H 12800 2600 50  0000 C CNN
F 2 "" H 12800 2750 50  0001 C CNN
F 3 "" H 12800 2750 50  0001 C CNN
	1    12800 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR066
U 1 1 5BC80064
P 12550 2750
F 0 "#PWR066" H 12550 2500 50  0001 C CNN
F 1 "GND" H 12550 2600 50  0000 C CNN
F 2 "" H 12550 2750 50  0001 C CNN
F 3 "" H 12550 2750 50  0001 C CNN
	1    12550 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	12550 2750 12550 2350
Wire Wire Line
	12800 2750 12800 2350
Wire Wire Line
	13800 1350 14300 1350
Wire Wire Line
	14550 1350 14550 1500
Wire Wire Line
	14550 1800 14550 1950
Connection ~ 14550 1950
Wire Wire Line
	12550 2050 12550 1850
Connection ~ 12550 1850
Wire Wire Line
	12800 2050 12800 1750
Connection ~ 12800 1750
Wire Wire Line
	11600 3800 12100 3800
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D6
U 1 1 5BC80077
P 12750 3400
F 0 "D6" H 12750 3500 50  0000 C CNN
F 1 "BAT30KFILM" H 12750 3300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 12750 3400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 12750 3400 50  0001 C CNN
F 4 "497-5552-1-ND" H 12750 3400 60  0001 C CNN "Part"
F 5 "DigiKey" H 12750 3400 60  0001 C CNN "Provider"
	1    12750 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	12900 3400 13050 3400
Wire Wire Line
	12200 3400 12100 3400
Wire Wire Line
	12100 3400 12100 3800
Connection ~ 12100 3800
Wire Wire Line
	12500 3400 12600 3400
Wire Wire Line
	11600 5050 12100 5050
$Comp
L servo-sigma-48v-rescue:D_Schottky-device D7
U 1 1 5BC80085
P 12750 4650
F 0 "D7" H 12750 4750 50  0000 C CNN
F 1 "BAT30KFILM" H 12750 4550 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 12750 4650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 12750 4650 50  0001 C CNN
F 4 "497-5552-1-ND" H 12750 4650 60  0001 C CNN "Part"
F 5 "DigiKey" H 12750 4650 60  0001 C CNN "Provider"
	1    12750 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	12900 4650 13050 4650
Wire Wire Line
	12200 4650 12100 4650
Wire Wire Line
	12100 4650 12100 5050
Connection ~ 12100 5050
Wire Wire Line
	12500 4650 12600 4650
Wire Wire Line
	13100 1350 13500 1350
$Comp
L servo-sigma-48v-rescue:R-device R77
U 1 1 5BC80093
P 12400 5050
F 0 "R77" V 12480 5050 50  0000 C CNN
F 1 "21" V 12400 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12330 5050 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12400 5050 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 12400 5050 60  0001 C CNN "Part"
F 5 "DigiKey" V 12400 5050 60  0001 C CNN "Provider"
	1    12400 5050
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R75
U 1 1 5BC8009B
P 12350 4650
F 0 "R75" V 12430 4650 50  0000 C CNN
F 1 "5" V 12350 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12280 4650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12350 4650 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 12350 4650 60  0001 C CNN "Part"
F 5 "DigiKey" V 12350 4650 60  0001 C CNN "Provider"
	1    12350 4650
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R76
U 1 1 5BC800A3
P 12400 3800
F 0 "R76" V 12480 3800 50  0000 C CNN
F 1 "21" V 12400 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12330 3800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12400 3800 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 12400 3800 60  0001 C CNN "Part"
F 5 "DigiKey" V 12400 3800 60  0001 C CNN "Provider"
	1    12400 3800
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R74
U 1 1 5BC800AB
P 12350 3400
F 0 "R74" V 12430 3400 50  0000 C CNN
F 1 "5" V 12350 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12280 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12350 3400 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 12350 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 12350 3400 60  0001 C CNN "Provider"
	1    12350 3400
	0    -1   -1   0   
$EndComp
$Comp
L servo-sigma-48v-rescue:R-device R78
U 1 1 5BC800B3
P 12950 1350
F 0 "R78" V 13030 1350 50  0000 C CNN
F 1 "5" V 12950 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12880 1350 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12950 1350 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 12950 1350 60  0001 C CNN "Part"
F 5 "DigiKey" V 12950 1350 60  0001 C CNN "Provider"
	1    12950 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13050 5050 13250 5050
Wire Wire Line
	13050 5050 13050 5150
Wire Wire Line
	13550 6100 13550 6200
Wire Wire Line
	13550 4300 13550 4850
Wire Wire Line
	12300 1350 12300 1950
Wire Wire Line
	12300 1350 12800 1350
Wire Wire Line
	14300 1350 14550 1350
Wire Wire Line
	12300 1950 12300 2050
Wire Wire Line
	13050 3800 13250 3800
Wire Wire Line
	13050 3800 13050 3900
Wire Wire Line
	14550 1950 15050 1950
Wire Wire Line
	12550 1850 13100 1850
Wire Wire Line
	12800 1750 13100 1750
Wire Wire Line
	12100 3800 12250 3800
Wire Wire Line
	12100 5050 12250 5050
Wire Wire Line
	13550 4300 14550 4300
$Comp
L power:+15V #PWR061
U 1 1 5BC800D9
P 12300 850
F 0 "#PWR061" H 12300 700 50  0001 C CNN
F 1 "+15V" H 12315 1023 50  0000 C CNN
F 2 "" H 12300 850 50  0001 C CNN
F 3 "" H 12300 850 50  0001 C CNN
	1    12300 850 
	1    0    0    -1  
$EndComp
Text Label 15050 4300 2    60   ~ 0
PHASE_R
$Comp
L power:GND #PWR071
U 1 1 5BCAB899
P 14550 6200
F 0 "#PWR071" H 14550 5950 50  0001 C CNN
F 1 "GND" H 14550 6050 50  0000 C CNN
F 2 "" H 14550 6200 50  0001 C CNN
F 3 "" H 14550 6200 50  0001 C CNN
	1    14550 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	13550 5250 13550 6100
Wire Wire Line
	14550 4650 14550 4300
Connection ~ 14550 4300
Wire Wire Line
	14550 4300 15050 4300
$Comp
L servo-sigma-48v-rescue:CONN_01X02-conn J?
U 1 1 5BC5C1F6
P 15100 4700
AR Path="/58BE27E6/5BC5C1F6" Ref="J?"  Part="1" 
AR Path="/58BF664D/5BC5C1F6" Ref="J7"  Part="1" 
AR Path="/5BC5C1F6" Ref="J7"  Part="1" 
F 0 "J7" H 15019 4965 50  0000 C CNN
F 1 "CONN_01X02" H 15019 4874 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_02x7.62mm_Vertical" H 15100 4700 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1868076.pdf" H 15100 4700 50  0001 C CNN
F 4 "277-5840-ND" H 15100 4700 50  0001 C CNN "Part"
F 5 "DigiKey" H 15100 4700 50  0001 C CNN "Provider"
	1    15100 4700
	1    0    0    1   
$EndComp
Wire Wire Line
	14550 4650 14900 4650
Wire Wire Line
	14550 4750 14900 4750
Text HLabel 14950 5400 2    60   Output ~ 0
R_ISENSE_P
Text HLabel 14950 6100 2    60   Output ~ 0
R_ISENSE_N
Wire Wire Line
	14550 4750 14550 5400
Wire Wire Line
	14550 5400 14550 5550
Connection ~ 14550 5400
Wire Wire Line
	14950 5400 14550 5400
$Comp
L servo-sigma-48v-rescue:R-device R82
U 1 1 5BCAB8A5
P 14550 5700
F 0 "R82" V 14630 5700 50  0000 C CNN
F 1 "0.010 1%" V 14450 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 14480 5700 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 14550 5700 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 14550 5700 60  0001 C CNN "Part"
F 5 "DigiKey" V 14550 5700 60  0001 C CNN "Provider"
	1    14550 5700
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D199B5E
P 13550 3300
AR Path="/58BE27E6/5D199B5E" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D199B5E" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0129" H 13550 3150 50  0001 C CNN
F 1 "VIN" H 13565 3473 50  0000 C CNN
F 2 "" H 13550 3300 50  0000 C CNN
F 3 "" H 13550 3300 50  0000 C CNN
	1    13550 3300
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1A8822
P 9600 3300
AR Path="/58BE27E6/5D1A8822" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1A8822" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 9600 3150 50  0001 C CNN
F 1 "VIN" H 9615 3473 50  0000 C CNN
F 2 "" H 9600 3300 50  0000 C CNN
F 3 "" H 9600 3300 50  0000 C CNN
	1    9600 3300
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1B74E6
P 6100 3300
AR Path="/58BE27E6/5D1B74E6" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1B74E6" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 6100 3150 50  0001 C CNN
F 1 "VIN" H 6115 3473 50  0000 C CNN
F 2 "" H 6100 3300 50  0000 C CNN
F 3 "" H 6100 3300 50  0000 C CNN
	1    6100 3300
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1C61AA
P 2750 3300
AR Path="/58BE27E6/5D1C61AA" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1C61AA" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0132" H 2750 3150 50  0001 C CNN
F 1 "VIN" H 2765 3473 50  0000 C CNN
F 2 "" H 2750 3300 50  0000 C CNN
F 3 "" H 2750 3300 50  0000 C CNN
	1    2750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	14550 6100 14550 6200
Wire Wire Line
	14550 5850 14550 6100
Connection ~ 14550 6100
Wire Wire Line
	14950 6100 14550 6100
Wire Wire Line
	10000 6100 9600 6100
Wire Wire Line
	3150 6100 2750 6100
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q6
U 1 1 5C792FA5
P 2650 5050
F 0 "Q6" H 2850 5100 50  0000 L CNN
F 1 "STL160NS3LLH7" H 2850 5000 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 2850 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 2650 5050 50  0001 C CNN
F 4 "DigiKey" H 2650 5050 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 2650 5050 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 2650 5050 50  0001 C CNN "Value1"
	1    2650 5050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q3
U 1 1 5C79432A
P 6000 3800
F 0 "Q3" H 6200 3850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 6200 3750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 6200 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 6000 3800 50  0001 C CNN
F 4 "DigiKey" H 6000 3800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 6000 3800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 6000 3800 50  0001 C CNN "Value1"
	1    6000 3800
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q4
U 1 1 5C794408
P 6000 5050
F 0 "Q4" H 6200 5100 50  0000 L CNN
F 1 "STL160NS3LLH7" H 6200 5000 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 6200 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 6000 5050 50  0001 C CNN
F 4 "DigiKey" H 6000 5050 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 6000 5050 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 6000 5050 50  0001 C CNN "Value1"
	1    6000 5050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q1
U 1 1 5C794DDD
P 9500 3800
F 0 "Q1" H 9700 3850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9700 3750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9700 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9500 3800 50  0001 C CNN
F 4 "DigiKey" H 9500 3800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9500 3800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9500 3800 50  0001 C CNN "Value1"
	1    9500 3800
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q2
U 1 1 5C794EBD
P 9500 5050
F 0 "Q2" H 9700 5100 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9700 5000 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9700 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9500 5050 50  0001 C CNN
F 4 "DigiKey" H 9500 5050 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9500 5050 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9500 5050 50  0001 C CNN "Value1"
	1    9500 5050
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q7
U 1 1 5C79539C
P 13450 3800
F 0 "Q7" H 13650 3850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 13650 3750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 13650 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 13450 3800 50  0001 C CNN
F 4 "DigiKey" H 13450 3800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 13450 3800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 13450 3800 50  0001 C CNN "Value1"
	1    13450 3800
	1    0    0    -1  
$EndComp
$Comp
L servo-sigma-48v-rescue:Q_NMOS_SGD-device Q8
U 1 1 5C795574
P 13450 5050
F 0 "Q8" H 13650 5100 50  0000 L CNN
F 1 "STL160NS3LLH7" H 13650 5000 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 13650 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 13450 5050 50  0001 C CNN
F 4 "DigiKey" H 13450 5050 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 13450 5050 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 13450 5050 50  0001 C CNN "Value1"
	1    13450 5050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
