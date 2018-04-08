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
$Comp
L power:GND #PWR025
U 1 1 58BF6CB7
P 1900 3250
F 0 "#PWR025" H 1900 3000 50  0001 C CNN
F 1 "GND" H 1900 3100 50  0000 C CNN
F 2 "" H 1900 3250 50  0001 C CNN
F 3 "" H 1900 3250 50  0001 C CNN
	1    1900 3250
	1    0    0    -1  
$EndComp
Text Label 3900 2450 2    60   ~ 0
PHASE_A
Text Label 3900 2350 2    60   ~ 0
GATE_AH
$Comp
L device:Q_NMOS_SGD Q3
U 1 1 58BF7288
P 6000 4300
F 0 "Q3" H 6200 4350 50  0000 L CNN
F 1 "STL160NS3LLH7" H 6200 4250 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 6200 4400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 6000 4300 50  0001 C CNN
F 4 "DigiKey" H 6000 4300 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 6000 4300 60  0001 C CNN "Part"
	1    6000 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR026
U 1 1 58BF9644
P 2750 3800
F 0 "#PWR026" H 2750 3650 50  0001 C CNN
F 1 "+BATT" H 2750 3940 50  0000 C CNN
F 2 "" H 2750 3800 50  0001 C CNN
F 3 "" H 2750 3800 50  0001 C CNN
	1    2750 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 58BF987A
P 2750 6150
F 0 "#PWR027" H 2750 5900 50  0001 C CNN
F 1 "GND" H 2750 6000 50  0000 C CNN
F 2 "" H 2750 6150 50  0001 C CNN
F 3 "" H 2750 6150 50  0001 C CNN
	1    2750 6150
	1    0    0    -1  
$EndComp
Text HLabel 1000 2250 0    60   Input ~ 0
PWM_AL
Text Label 950  4300 0    60   ~ 0
GATE_AH
Text HLabel 3700 4800 2    60   Output ~ 0
PHASE_A
Text Label 7400 2450 2    60   ~ 0
PHASE_B
Text HLabel 4550 2250 0    60   Input ~ 0
PWM_BL
Text Label 7400 2350 2    60   ~ 0
GATE_BH
$Comp
L power:+BATT #PWR028
U 1 1 58C227E1
P 6100 3800
F 0 "#PWR028" H 6100 3650 50  0001 C CNN
F 1 "+BATT" H 6100 3940 50  0000 C CNN
F 2 "" H 6100 3800 50  0001 C CNN
F 3 "" H 6100 3800 50  0001 C CNN
	1    6100 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 58C227F2
P 6100 6150
F 0 "#PWR029" H 6100 5900 50  0001 C CNN
F 1 "GND" H 6100 6000 50  0000 C CNN
F 2 "" H 6100 6150 50  0001 C CNN
F 3 "" H 6100 6150 50  0001 C CNN
	1    6100 6150
	1    0    0    -1  
$EndComp
Text HLabel 4550 2350 0    60   Input ~ 0
PWM_BH
Text Label 4150 4300 0    60   ~ 0
GATE_BH
Text HLabel 7050 4800 2    60   Output ~ 0
PHASE_B
$Comp
L power:+BATT #PWR030
U 1 1 58C22F7A
P 9600 3800
F 0 "#PWR030" H 9600 3650 50  0001 C CNN
F 1 "+BATT" H 9600 3940 50  0000 C CNN
F 2 "" H 9600 3800 50  0001 C CNN
F 3 "" H 9600 3800 50  0001 C CNN
	1    9600 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 58C22F8B
P 9600 6150
F 0 "#PWR031" H 9600 5900 50  0001 C CNN
F 1 "GND" H 9600 6000 50  0000 C CNN
F 2 "" H 9600 6150 50  0001 C CNN
F 3 "" H 9600 6150 50  0001 C CNN
	1    9600 6150
	1    0    0    -1  
$EndComp
Text Label 7650 4300 0    60   ~ 0
GATE_CH
Text HLabel 10600 4800 2    60   Output ~ 0
PHASE_C
$Comp
L device:D_Schottky D14
U 1 1 58C323FC
P 2500 1850
F 0 "D14" H 2500 1950 50  0000 C CNN
F 1 "STPS0560Z" H 2500 1750 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 2500 1850 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 2500 1850 50  0001 C CNN
F 4 "497-3787-1-ND" H 2500 1850 60  0001 C CNN "Part"
F 5 "DigiKey" H 2500 1850 60  0001 C CNN "Provider"
	1    2500 1850
	-1   0    0    1   
$EndComp
Text HLabel 1350 7300 0    60   UnSpc ~ 0
GND
$Comp
L power:GND #PWR032
U 1 1 58D0BA33
P 1600 7500
F 0 "#PWR032" H 1600 7250 50  0001 C CNN
F 1 "GND" H 1600 7350 50  0000 C CNN
F 2 "" H 1600 7500 50  0001 C CNN
F 3 "" H 1600 7500 50  0001 C CNN
	1    1600 7500
	1    0    0    -1  
$EndComp
$Comp
L device:C C22
U 1 1 58D19DC3
P 6900 2150
F 0 "C22" V 6850 2200 50  0000 L CNN
F 1 "1uF 100V" V 6850 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6938 2000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 6900 2150 50  0001 C CNN
F 4 "445-8887-1-ND" H 6900 2150 60  0001 C CNN "Part"
F 5 "DigiKey" H 6900 2150 60  0001 C CNN "Provider"
	1    6900 2150
	-1   0    0    1   
$EndComp
$Comp
L device:C C34
U 1 1 58D19E8B
P 3400 2150
F 0 "C34" V 3350 2250 50  0000 L CNN
F 1 "1uF 100V" V 3350 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3438 2000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3400 2150 50  0001 C CNN
F 4 "445-8887-1-ND" H 3400 2150 60  0001 C CNN "Part"
F 5 "DigiKey" H 3400 2150 60  0001 C CNN "Provider"
	1    3400 2150
	-1   0    0    1   
$EndComp
$Comp
L device:D_Schottky D11
U 1 1 58D1E9C4
P 6000 1850
F 0 "D11" H 6000 1950 50  0000 C CNN
F 1 "STPS0560Z" H 6000 1750 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 6000 1850 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 6000 1850 50  0001 C CNN
F 4 "497-3787-1-ND" H 6000 1850 60  0001 C CNN "Part"
F 5 "DigiKey" H 6000 1850 60  0001 C CNN "Provider"
	1    6000 1850
	-1   0    0    1   
$EndComp
Text Label 3900 2550 2    60   ~ 0
GATE_AL
Text HLabel 1350 7100 0    60   UnSpc ~ 0
VBAT
$Comp
L power:+BATT #PWR033
U 1 1 58EB25D0
P 1600 6950
F 0 "#PWR033" H 1600 6800 50  0001 C CNN
F 1 "+BATT" H 1600 7090 50  0000 C CNN
F 2 "" H 1600 6950 50  0001 C CNN
F 3 "" H 1600 6950 50  0001 C CNN
	1    1600 6950
	1    0    0    -1  
$EndComp
Text Label 950  5550 0    60   ~ 0
GATE_AL
$Comp
L device:C C36
U 1 1 58EB6948
P 1150 2700
F 0 "C36" H 1000 2600 50  0000 L CNN
F 1 "1uF 100V" V 1200 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1188 2550 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 1150 2700 50  0001 C CNN
F 4 "445-8887-1-ND" H 1150 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 1150 2700 60  0001 C CNN "Provider"
	1    1150 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR034
U 1 1 58EB6C09
P 1150 3250
F 0 "#PWR034" H 1150 3000 50  0001 C CNN
F 1 "GND" H 1150 3100 50  0000 C CNN
F 2 "" H 1150 3250 50  0001 C CNN
F 3 "" H 1150 3250 50  0001 C CNN
	1    1150 3250
	1    0    0    -1  
$EndComp
Text Label 7400 2550 2    60   ~ 0
GATE_BL
$Comp
L device:C C26
U 1 1 58EB8B61
P 4700 2700
F 0 "C26" H 4550 2600 50  0000 L CNN
F 1 "1uF 100V" V 4750 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4738 2550 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 4700 2700 50  0001 C CNN
F 4 "445-8887-1-ND" H 4700 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 4700 2700 60  0001 C CNN "Provider"
	1    4700 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR035
U 1 1 58EB8B67
P 4700 3250
F 0 "#PWR035" H 4700 3000 50  0001 C CNN
F 1 "GND" H 4700 3100 50  0000 C CNN
F 2 "" H 4700 3250 50  0001 C CNN
F 3 "" H 4700 3250 50  0001 C CNN
	1    4700 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 58EB8D70
P 5450 3250
F 0 "#PWR036" H 5450 3000 50  0001 C CNN
F 1 "GND" H 5450 3100 50  0000 C CNN
F 2 "" H 5450 3250 50  0001 C CNN
F 3 "" H 5450 3250 50  0001 C CNN
	1    5450 3250
	1    0    0    -1  
$EndComp
Text Label 4150 5550 0    60   ~ 0
GATE_BL
Text Label 11100 2450 2    60   ~ 0
PHASE_C
Text HLabel 8100 2250 0    60   Input ~ 0
PWM_CL
Text Label 11100 2350 2    60   ~ 0
GATE_CH
Text HLabel 8100 2350 0    60   Input ~ 0
PWM_CH
$Comp
L device:C C17
U 1 1 58EB9997
P 10600 2150
F 0 "C17" V 10550 2200 50  0000 L CNN
F 1 "1uF 100V" V 10550 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 10638 2000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 10600 2150 50  0001 C CNN
F 4 "445-8887-1-ND" H 10600 2150 60  0001 C CNN "Part"
F 5 "DigiKey" H 10600 2150 60  0001 C CNN "Provider"
	1    10600 2150
	-1   0    0    1   
$EndComp
$Comp
L device:D_Schottky D8
U 1 1 58EB999F
P 9700 1850
F 0 "D8" H 9700 1950 50  0000 C CNN
F 1 "STPS0560Z" H 9700 1750 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 9700 1850 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 9700 1850 50  0001 C CNN
F 4 "497-3787-1-ND" H 9700 1850 60  0001 C CNN "Part"
F 5 "DigiKey" H 9700 1850 60  0001 C CNN "Provider"
	1    9700 1850
	-1   0    0    1   
$EndComp
Text Label 11100 2550 2    60   ~ 0
GATE_CL
$Comp
L device:C C20
U 1 1 58EB99B4
P 8350 2700
F 0 "C20" H 8200 2600 50  0000 L CNN
F 1 "1uF 100V" V 8400 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 8388 2550 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 8350 2700 50  0001 C CNN
F 4 "445-8887-1-ND" H 8350 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 8350 2700 60  0001 C CNN "Provider"
	1    8350 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 58EB99BA
P 8350 3250
F 0 "#PWR037" H 8350 3000 50  0001 C CNN
F 1 "GND" H 8350 3100 50  0000 C CNN
F 2 "" H 8350 3250 50  0001 C CNN
F 3 "" H 8350 3250 50  0001 C CNN
	1    8350 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 58EB99D7
P 9100 3250
F 0 "#PWR038" H 9100 3000 50  0001 C CNN
F 1 "GND" H 9100 3100 50  0000 C CNN
F 2 "" H 9100 3250 50  0001 C CNN
F 3 "" H 9100 3250 50  0001 C CNN
	1    9100 3250
	1    0    0    -1  
$EndComp
Text Label 7650 5550 0    60   ~ 0
GATE_CL
$Comp
L power:+BATT #PWR039
U 1 1 58F114D2
P 1150 1250
F 0 "#PWR039" H 1150 1100 50  0001 C CNN
F 1 "+BATT" H 1150 1390 50  0000 C CNN
F 2 "" H 1150 1250 50  0001 C CNN
F 3 "" H 1150 1250 50  0001 C CNN
	1    1150 1250
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:L6398 U7
U 1 1 58F12792
P 2550 2400
F 0 "U7" H 2750 2700 60  0000 C CNN
F 1 "L6398" H 2700 2100 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2550 1900 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 2550 2000 60  0001 C CNN
F 4 "497-16166-5-ND" H 2550 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 2550 2400 60  0001 C CNN "Provider"
	1    2550 2400
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:L6398 U6
U 1 1 58F12B9A
P 6050 2400
F 0 "U6" H 6250 2700 60  0000 C CNN
F 1 "L6398" H 6200 2100 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6050 1900 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 6050 2000 60  0001 C CNN
F 4 "497-16166-5-ND" H 6050 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 6050 2400 60  0001 C CNN "Provider"
	1    6050 2400
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:L6398 U4
U 1 1 58F12D24
P 9700 2400
F 0 "U4" H 9900 2700 60  0000 C CNN
F 1 "L6398" H 9850 2100 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 9700 1900 60  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/d3/f4/04/52/68/e9/40/02/CD00290377.pdf/files/CD00290377.pdf/jcr:content/translations/en.CD00290377.pdf" H 9700 2000 60  0001 C CNN
F 4 "497-16166-5-ND" H 9700 2400 60  0001 C CNN "Part"
F 5 "DigiKey" H 9700 2400 60  0001 C CNN "Provider"
	1    9700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3800 2750 4100
Wire Wire Line
	2750 4500 2750 4800
Wire Wire Line
	2750 5750 2750 6050
Wire Wire Line
	1000 2250 1650 2250
Wire Wire Line
	2350 5150 2350 5550
Connection ~ 2350 5550
Wire Wire Line
	2350 5950 2350 6050
Wire Wire Line
	2350 6050 2750 6050
Connection ~ 2750 6050
Wire Wire Line
	2350 4800 2750 4800
Connection ~ 2750 4800
Wire Wire Line
	6100 3800 6100 4100
Wire Wire Line
	6100 4500 6100 4800
Wire Wire Line
	6100 5750 6100 6050
Wire Wire Line
	5600 5150 5600 5550
Connection ~ 5600 5550
Wire Wire Line
	5600 5950 5600 6050
Wire Wire Line
	5600 6050 6100 6050
Connection ~ 6100 6050
Wire Wire Line
	5600 4800 6100 4800
Connection ~ 6100 4800
Wire Wire Line
	9600 3800 9600 4100
Wire Wire Line
	9600 4500 9600 4800
Wire Wire Line
	9600 5750 9600 6050
Wire Wire Line
	8600 4300 9100 4300
Wire Wire Line
	8600 5550 9100 5550
Wire Wire Line
	9100 5150 9100 5550
Connection ~ 9100 5550
Wire Wire Line
	9100 5950 9100 6050
Wire Wire Line
	9100 6050 9600 6050
Connection ~ 9600 6050
Wire Wire Line
	9100 4800 9600 4800
Connection ~ 9600 4800
Wire Wire Line
	1350 7300 1600 7300
Wire Wire Line
	1600 7300 1600 7500
Wire Wire Line
	3100 2450 3400 2450
Wire Wire Line
	3100 2550 3900 2550
Wire Wire Line
	2000 2550 1900 2550
Wire Wire Line
	1900 2550 1900 3250
Wire Wire Line
	1600 6950 1600 7100
Wire Wire Line
	1600 7100 1350 7100
Wire Wire Line
	2650 1850 3150 1850
Wire Wire Line
	3100 2350 3900 2350
Wire Wire Line
	1150 1250 1150 1850
Wire Wire Line
	1150 2450 2000 2450
Connection ~ 1150 1850
Wire Wire Line
	1150 2850 1150 3250
Wire Wire Line
	6600 2250 6700 2250
Wire Wire Line
	6700 2250 6700 1850
Connection ~ 6700 1850
Wire Wire Line
	6600 2350 7400 2350
Wire Wire Line
	6600 2450 6900 2450
Wire Wire Line
	6600 2550 7400 2550
Wire Wire Line
	4550 2250 5250 2250
Wire Wire Line
	4550 2350 5000 2350
Wire Wire Line
	4700 1350 4700 1850
Wire Wire Line
	4700 2450 5500 2450
Connection ~ 4700 1850
Wire Wire Line
	4700 2850 4700 3250
Wire Wire Line
	5450 3250 5450 2550
Wire Wire Line
	5450 2550 5500 2550
Wire Wire Line
	10250 2350 11100 2350
Wire Wire Line
	10250 2450 10600 2450
Wire Wire Line
	10250 2550 11100 2550
Wire Wire Line
	8100 2250 8850 2250
Wire Wire Line
	8100 2350 8600 2350
Wire Wire Line
	8350 1350 8350 1850
Wire Wire Line
	8350 2450 9150 2450
Connection ~ 8350 1850
Wire Wire Line
	8350 2850 8350 3250
Wire Wire Line
	9100 3250 9100 2550
Wire Wire Line
	9100 2550 9150 2550
Wire Wire Line
	10250 2250 10350 2250
Wire Wire Line
	10350 2250 10350 1850
Connection ~ 10350 1850
Connection ~ 8350 2450
Connection ~ 4700 2450
Connection ~ 1150 2450
$Comp
L device:R R42
U 1 1 59D34BBD
P 2350 4550
F 0 "R42" V 2430 4550 50  0000 C CNN
F 1 "10k" V 2350 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2280 4550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2350 4550 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2350 4550 60  0001 C CNN "Part"
F 5 "DigiKey" V 2350 4550 60  0001 C CNN "Provider"
	1    2350 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4700 2350 4800
Wire Wire Line
	2350 3900 2350 4300
Connection ~ 2350 4300
Wire Wire Line
	5600 3900 5600 4300
Connection ~ 5600 4300
Wire Wire Line
	5600 4700 5600 4800
Wire Wire Line
	9100 4700 9100 4800
Wire Wire Line
	9100 3900 9100 4300
Connection ~ 9100 4300
$Comp
L device:R R32
U 1 1 59D368B1
P 5600 4550
F 0 "R32" V 5680 4550 50  0000 C CNN
F 1 "10k" V 5600 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5530 4550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5600 4550 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 5600 4550 60  0001 C CNN "Part"
F 5 "DigiKey" V 5600 4550 60  0001 C CNN "Provider"
	1    5600 4550
	1    0    0    -1  
$EndComp
$Comp
L device:R R22
U 1 1 59D36CCF
P 9100 4550
F 0 "R22" V 9180 4550 50  0000 C CNN
F 1 "10k" V 9100 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9030 4550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9100 4550 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 9100 4550 60  0001 C CNN "Part"
F 5 "DigiKey" V 9100 4550 60  0001 C CNN "Provider"
	1    9100 4550
	1    0    0    -1  
$EndComp
$Comp
L device:R R56
U 1 1 59D5BB5C
P 2350 5800
F 0 "R56" V 2430 5800 50  0000 C CNN
F 1 "10k" V 2350 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2280 5800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2350 5800 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2350 5800 60  0001 C CNN "Part"
F 5 "DigiKey" V 2350 5800 60  0001 C CNN "Provider"
	1    2350 5800
	1    0    0    -1  
$EndComp
$Comp
L device:R R37
U 1 1 59D5BE2E
P 5600 5800
F 0 "R37" V 5680 5800 50  0000 C CNN
F 1 "10k" V 5600 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5530 5800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5600 5800 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 5600 5800 60  0001 C CNN "Part"
F 5 "DigiKey" V 5600 5800 60  0001 C CNN "Provider"
	1    5600 5800
	1    0    0    -1  
$EndComp
$Comp
L device:R R27
U 1 1 59D5C1C0
P 9100 5800
F 0 "R27" V 9180 5800 50  0000 C CNN
F 1 "10k" V 9100 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9030 5800 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9100 5800 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 9100 5800 60  0001 C CNN "Part"
F 5 "DigiKey" V 9100 5800 60  0001 C CNN "Provider"
	1    9100 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2250 3150 2250
Wire Wire Line
	3150 2250 3150 1850
Connection ~ 3150 1850
Wire Wire Line
	3400 1850 3400 2000
Wire Wire Line
	3400 2300 3400 2450
Connection ~ 3400 2450
$Comp
L device:C C21
U 1 1 59FAAC4B
P 5250 2700
F 0 "C21" H 5100 2600 50  0000 L CNN
F 1 "100pF 100V" V 5300 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5288 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 5250 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 5250 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 5250 2700 60  0001 C CNN "Provider"
	1    5250 2700
	-1   0    0    1   
$EndComp
$Comp
L device:C C23
U 1 1 59FAAC53
P 5000 2700
F 0 "C23" H 4850 2600 50  0000 L CNN
F 1 "100pF 100V" V 5050 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5038 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 5000 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 5000 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 5000 2700 60  0001 C CNN "Provider"
	1    5000 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR078
U 1 1 59FAAC59
P 5250 3250
F 0 "#PWR078" H 5250 3000 50  0001 C CNN
F 1 "GND" H 5250 3100 50  0000 C CNN
F 2 "" H 5250 3250 50  0001 C CNN
F 3 "" H 5250 3250 50  0001 C CNN
	1    5250 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR077
U 1 1 59FAAC5F
P 5000 3250
F 0 "#PWR077" H 5000 3000 50  0001 C CNN
F 1 "GND" H 5000 3100 50  0000 C CNN
F 2 "" H 5000 3250 50  0001 C CNN
F 3 "" H 5000 3250 50  0001 C CNN
	1    5000 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3250 5000 2850
Wire Wire Line
	5250 3250 5250 2850
Text HLabel 1000 2350 0    60   Input ~ 0
PWM_AH
Wire Wire Line
	1000 2350 1400 2350
$Comp
L device:C C33
U 1 1 59FAB71D
P 1650 2700
F 0 "C33" H 1500 2600 50  0000 L CNN
F 1 "100pF 100V" V 1700 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1688 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 1650 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 1650 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 1650 2700 60  0001 C CNN "Provider"
	1    1650 2700
	-1   0    0    1   
$EndComp
$Comp
L device:C C35
U 1 1 59FAB725
P 1400 2700
F 0 "C35" H 1250 2600 50  0000 L CNN
F 1 "100pF 100V" V 1450 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1438 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 1400 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 1400 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 1400 2700 60  0001 C CNN "Provider"
	1    1400 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR074
U 1 1 59FAB72B
P 1650 3250
F 0 "#PWR074" H 1650 3000 50  0001 C CNN
F 1 "GND" H 1650 3100 50  0000 C CNN
F 2 "" H 1650 3250 50  0001 C CNN
F 3 "" H 1650 3250 50  0001 C CNN
	1    1650 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR042
U 1 1 59FAB731
P 1400 3250
F 0 "#PWR042" H 1400 3000 50  0001 C CNN
F 1 "GND" H 1400 3100 50  0000 C CNN
F 2 "" H 1400 3250 50  0001 C CNN
F 3 "" H 1400 3250 50  0001 C CNN
	1    1400 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3250 1400 2850
Wire Wire Line
	1650 3250 1650 2850
Wire Wire Line
	6150 1850 6700 1850
Wire Wire Line
	6900 1850 6900 2000
Wire Wire Line
	6900 2300 6900 2450
Connection ~ 6900 2450
$Comp
L device:C C16
U 1 1 59FACD6A
P 8850 2700
F 0 "C16" H 8700 2600 50  0000 L CNN
F 1 "100pF 100V" V 8900 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8888 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 8850 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 8850 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 8850 2700 60  0001 C CNN "Provider"
	1    8850 2700
	-1   0    0    1   
$EndComp
$Comp
L device:C C18
U 1 1 59FACD72
P 8600 2700
F 0 "C18" H 8450 2600 50  0000 L CNN
F 1 "100pF 100V" V 8650 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8638 2550 50  0001 C CNN
F 3 "https://search.kemet.com/component-edge/download/datasheet/C0603C101J1GACTU.pdf" H 8600 2700 50  0001 C CNN
F 4 "399-7821-2-ND" H 8600 2700 60  0001 C CNN "Part"
F 5 "DigiKey" H 8600 2700 60  0001 C CNN "Provider"
	1    8600 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR080
U 1 1 59FACD78
P 8850 3250
F 0 "#PWR080" H 8850 3000 50  0001 C CNN
F 1 "GND" H 8850 3100 50  0000 C CNN
F 2 "" H 8850 3250 50  0001 C CNN
F 3 "" H 8850 3250 50  0001 C CNN
	1    8850 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR079
U 1 1 59FACD7E
P 8600 3250
F 0 "#PWR079" H 8600 3000 50  0001 C CNN
F 1 "GND" H 8600 3100 50  0000 C CNN
F 2 "" H 8600 3250 50  0001 C CNN
F 3 "" H 8600 3250 50  0001 C CNN
	1    8600 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3250 8600 2850
Wire Wire Line
	8850 3250 8850 2850
Wire Wire Line
	9850 1850 10350 1850
Wire Wire Line
	10600 1850 10600 2000
Wire Wire Line
	10600 2300 10600 2450
Connection ~ 10600 2450
Wire Wire Line
	1400 2550 1400 2350
Connection ~ 1400 2350
Wire Wire Line
	1650 2550 1650 2250
Connection ~ 1650 2250
Wire Wire Line
	5000 2550 5000 2350
Connection ~ 5000 2350
Wire Wire Line
	5250 2550 5250 2250
Connection ~ 5250 2250
Wire Wire Line
	8600 2550 8600 2350
Connection ~ 8600 2350
Wire Wire Line
	8850 2550 8850 2250
Connection ~ 8850 2250
Wire Wire Line
	1950 1850 2350 1850
Wire Wire Line
	950  5550 1450 5550
Wire Wire Line
	1950 5550 2350 5550
Wire Wire Line
	950  4300 1450 4300
Wire Wire Line
	1950 4300 2350 4300
$Comp
L device:D_Schottky D16
U 1 1 5A191D28
P 2100 3900
F 0 "D16" H 2100 4000 50  0000 C CNN
F 1 "BAT30KFILM" H 2100 3800 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 2100 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 2100 3900 50  0001 C CNN
F 4 "497-5552-1-ND" H 2100 3900 60  0001 C CNN "Part"
F 5 "DigiKey" H 2100 3900 60  0001 C CNN "Provider"
	1    2100 3900
	1    0    0    -1  
$EndComp
$Comp
L device:R R55
U 1 1 5A19228E
P 1700 3900
F 0 "R55" V 1780 3900 50  0000 C CNN
F 1 "5" V 1700 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1630 3900 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1700 3900 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1700 3900 60  0001 C CNN "Part"
F 5 "DigiKey" V 1700 3900 60  0001 C CNN "Provider"
	1    1700 3900
	0    -1   -1   0   
$EndComp
$Comp
L device:D_Schottky D15
U 1 1 5A192746
P 2100 5150
F 0 "D15" H 2100 5250 50  0000 C CNN
F 1 "BAT30KFILM" H 2100 5050 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 2100 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 2100 5150 50  0001 C CNN
F 4 "497-5552-1-ND" H 2100 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 2100 5150 60  0001 C CNN "Provider"
	1    2100 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5150 2350 5150
Wire Wire Line
	1850 5150 1950 5150
Wire Wire Line
	1550 5150 1450 5150
Wire Wire Line
	1450 5150 1450 5550
Connection ~ 1450 5550
Wire Wire Line
	2250 3900 2350 3900
Wire Wire Line
	1550 3900 1450 3900
Wire Wire Line
	1450 3900 1450 4300
Connection ~ 1450 4300
Wire Wire Line
	1850 3900 1950 3900
Wire Wire Line
	5450 1850 5850 1850
Wire Wire Line
	4150 4300 4650 4300
Wire Wire Line
	5100 4300 5600 4300
$Comp
L device:D_Schottky D13
U 1 1 5A1F9CD5
P 5300 3900
F 0 "D13" H 5300 4000 50  0000 C CNN
F 1 "BAT30KFILM" H 5300 3800 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 5300 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 5300 3900 50  0001 C CNN
F 4 "497-5552-1-ND" H 5300 3900 60  0001 C CNN "Part"
F 5 "DigiKey" H 5300 3900 60  0001 C CNN "Provider"
	1    5300 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3900 5600 3900
Wire Wire Line
	4750 3900 4650 3900
Wire Wire Line
	4650 3900 4650 4300
Connection ~ 4650 4300
Wire Wire Line
	5050 3900 5150 3900
Wire Wire Line
	4150 5550 4650 5550
Wire Wire Line
	5100 5550 5600 5550
$Comp
L device:D_Schottky D12
U 1 1 5A2089BC
P 5300 5150
F 0 "D12" H 5300 5250 50  0000 C CNN
F 1 "BAT30KFILM" H 5300 5050 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 5300 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 5300 5150 50  0001 C CNN
F 4 "497-5552-1-ND" H 5300 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 5300 5150 60  0001 C CNN "Provider"
	1    5300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 5150 5600 5150
Wire Wire Line
	4750 5150 4650 5150
Wire Wire Line
	5050 5150 5150 5150
Wire Wire Line
	4650 5150 4650 5550
Connection ~ 4650 5550
Wire Wire Line
	7650 4300 8150 4300
$Comp
L device:D_Schottky D10
U 1 1 5A22B887
P 8800 3900
F 0 "D10" H 8800 4000 50  0000 C CNN
F 1 "BAT30KFILM" H 8800 3800 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 8800 3900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 8800 3900 50  0001 C CNN
F 4 "497-5552-1-ND" H 8800 3900 60  0001 C CNN "Part"
F 5 "DigiKey" H 8800 3900 60  0001 C CNN "Provider"
	1    8800 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3900 9100 3900
Wire Wire Line
	8250 3900 8150 3900
Wire Wire Line
	8150 3900 8150 4300
Connection ~ 8150 4300
Wire Wire Line
	8550 3900 8650 3900
Wire Wire Line
	7650 5550 8150 5550
$Comp
L device:D_Schottky D9
U 1 1 5A22BAC7
P 8800 5150
F 0 "D9" H 8800 5250 50  0000 C CNN
F 1 "BAT30KFILM" H 8800 5050 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-523" H 8800 5150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 8800 5150 50  0001 C CNN
F 4 "497-5552-1-ND" H 8800 5150 60  0001 C CNN "Part"
F 5 "DigiKey" H 8800 5150 60  0001 C CNN "Provider"
	1    8800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 5150 9100 5150
Wire Wire Line
	8250 5150 8150 5150
Wire Wire Line
	8150 5150 8150 5550
Connection ~ 8150 5550
Wire Wire Line
	8550 5150 8650 5150
Wire Wire Line
	9150 1850 9550 1850
$Comp
L power:+BATT #PWR040
U 1 1 5A1621EF
P 4700 1350
F 0 "#PWR040" H 4700 1200 50  0001 C CNN
F 1 "+BATT" H 4700 1490 50  0000 C CNN
F 2 "" H 4700 1350 50  0001 C CNN
F 3 "" H 4700 1350 50  0001 C CNN
	1    4700 1350
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR041
U 1 1 5A17ED38
P 8350 1350
F 0 "#PWR041" H 8350 1200 50  0001 C CNN
F 1 "+BATT" H 8350 1490 50  0000 C CNN
F 2 "" H 8350 1350 50  0001 C CNN
F 3 "" H 8350 1350 50  0001 C CNN
	1    8350 1350
	1    0    0    -1  
$EndComp
$Comp
L device:R R53
U 1 1 5A2D51A9
P 1800 4300
F 0 "R53" V 1880 4300 50  0000 C CNN
F 1 "21" V 1800 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1730 4300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 4300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1800 4300 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 4300 60  0001 C CNN "Provider"
	1    1800 4300
	0    -1   -1   0   
$EndComp
$Comp
L device:R R54
U 1 1 5A2D5E24
P 1700 5150
F 0 "R54" V 1780 5150 50  0000 C CNN
F 1 "5" V 1700 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1630 5150 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1700 5150 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1700 5150 60  0001 C CNN "Part"
F 5 "DigiKey" V 1700 5150 60  0001 C CNN "Provider"
	1    1700 5150
	0    -1   -1   0   
$EndComp
$Comp
L device:R R43
U 1 1 5A2D5ED2
P 1800 5550
F 0 "R43" V 1880 5550 50  0000 C CNN
F 1 "21" V 1800 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1730 5550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 5550 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 1800 5550 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 5550 60  0001 C CNN "Provider"
	1    1800 5550
	0    -1   -1   0   
$EndComp
$Comp
L device:R R36
U 1 1 5A2D8C1D
P 4900 3900
F 0 "R36" V 4980 3900 50  0000 C CNN
F 1 "5" V 4900 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4830 3900 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4900 3900 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 4900 3900 60  0001 C CNN "Part"
F 5 "DigiKey" V 4900 3900 60  0001 C CNN "Provider"
	1    4900 3900
	0    -1   -1   0   
$EndComp
$Comp
L device:R R34
U 1 1 5A2D8C25
P 4950 4300
F 0 "R34" V 5030 4300 50  0000 C CNN
F 1 "21" V 4950 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4880 4300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4950 4300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 4950 4300 60  0001 C CNN "Part"
F 5 "DigiKey" V 4950 4300 60  0001 C CNN "Provider"
	1    4950 4300
	0    -1   -1   0   
$EndComp
$Comp
L device:R R35
U 1 1 5A2D8C2D
P 4900 5150
F 0 "R35" V 4980 5150 50  0000 C CNN
F 1 "5" V 4900 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4830 5150 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4900 5150 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 4900 5150 60  0001 C CNN "Part"
F 5 "DigiKey" V 4900 5150 60  0001 C CNN "Provider"
	1    4900 5150
	0    -1   -1   0   
$EndComp
$Comp
L device:R R33
U 1 1 5A2D8C35
P 4950 5550
F 0 "R33" V 5030 5550 50  0000 C CNN
F 1 "21" V 4950 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4880 5550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 4950 5550 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 4950 5550 60  0001 C CNN "Part"
F 5 "DigiKey" V 4950 5550 60  0001 C CNN "Provider"
	1    4950 5550
	0    -1   -1   0   
$EndComp
$Comp
L device:R R23
U 1 1 5A2D9A3D
P 8450 5550
F 0 "R23" V 8530 5550 50  0000 C CNN
F 1 "21" V 8450 5550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8380 5550 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8450 5550 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 8450 5550 60  0001 C CNN "Part"
F 5 "DigiKey" V 8450 5550 60  0001 C CNN "Provider"
	1    8450 5550
	0    -1   -1   0   
$EndComp
$Comp
L device:R R25
U 1 1 5A2D9A35
P 8400 5150
F 0 "R25" V 8480 5150 50  0000 C CNN
F 1 "5" V 8400 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8330 5150 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8400 5150 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 8400 5150 60  0001 C CNN "Part"
F 5 "DigiKey" V 8400 5150 60  0001 C CNN "Provider"
	1    8400 5150
	0    -1   -1   0   
$EndComp
$Comp
L device:R R24
U 1 1 5A2D9A2D
P 8450 4300
F 0 "R24" V 8530 4300 50  0000 C CNN
F 1 "21" V 8450 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8380 4300 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8450 4300 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 8450 4300 60  0001 C CNN "Part"
F 5 "DigiKey" V 8450 4300 60  0001 C CNN "Provider"
	1    8450 4300
	0    -1   -1   0   
$EndComp
$Comp
L device:R R26
U 1 1 5A2D9A25
P 8400 3900
F 0 "R26" V 8480 3900 50  0000 C CNN
F 1 "5" V 8400 3900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8330 3900 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 8400 3900 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 8400 3900 60  0001 C CNN "Part"
F 5 "DigiKey" V 8400 3900 60  0001 C CNN "Provider"
	1    8400 3900
	0    -1   -1   0   
$EndComp
$Comp
L device:R R39
U 1 1 5A2FFED6
P 1800 1850
F 0 "R39" V 1880 1850 50  0000 C CNN
F 1 "5" V 1800 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1730 1850 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1800 1850 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 1800 1850 60  0001 C CNN "Part"
F 5 "DigiKey" V 1800 1850 60  0001 C CNN "Provider"
	1    1800 1850
	0    -1   -1   0   
$EndComp
$Comp
L device:R R29
U 1 1 5A3001D5
P 5300 1850
F 0 "R29" V 5380 1850 50  0000 C CNN
F 1 "5" V 5300 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 5230 1850 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 5300 1850 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 5300 1850 60  0001 C CNN "Part"
F 5 "DigiKey" V 5300 1850 60  0001 C CNN "Provider"
	1    5300 1850
	0    -1   -1   0   
$EndComp
$Comp
L device:R R18
U 1 1 5A30065C
P 9000 1850
F 0 "R18" V 9080 1850 50  0000 C CNN
F 1 "5" V 9000 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8930 1850 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9000 1850 50  0001 C CNN
F 4 "311-4.99HRCT-ND" V 9000 1850 60  0001 C CNN "Part"
F 5 "DigiKey" V 9000 1850 60  0001 C CNN "Provider"
	1    9000 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 5550 2350 5650
Wire Wire Line
	2350 5550 2450 5550
Wire Wire Line
	2750 6050 2750 6150
Wire Wire Line
	2750 4800 2750 5350
Wire Wire Line
	5600 5550 5600 5650
Wire Wire Line
	5600 5550 5800 5550
Wire Wire Line
	6100 6050 6100 6150
Wire Wire Line
	6100 4800 6100 5350
Wire Wire Line
	9100 5550 9300 5550
Wire Wire Line
	9100 5550 9100 5650
Wire Wire Line
	9600 6050 9600 6150
Wire Wire Line
	9600 4800 9600 5350
Wire Wire Line
	1150 1850 1150 2450
Wire Wire Line
	1150 1850 1650 1850
Wire Wire Line
	6700 1850 6900 1850
Wire Wire Line
	4700 1850 4700 2450
Wire Wire Line
	4700 1850 5150 1850
Wire Wire Line
	8350 1850 8350 2450
Wire Wire Line
	8350 1850 8850 1850
Wire Wire Line
	10350 1850 10600 1850
Wire Wire Line
	8350 2450 8350 2550
Wire Wire Line
	4700 2450 4700 2550
Wire Wire Line
	1150 2450 1150 2550
Wire Wire Line
	2350 4300 2350 4400
Wire Wire Line
	2350 4300 2450 4300
Wire Wire Line
	5600 4300 5600 4400
Wire Wire Line
	5600 4300 5800 4300
Wire Wire Line
	9100 4300 9300 4300
Wire Wire Line
	9100 4300 9100 4400
Wire Wire Line
	3150 1850 3400 1850
Wire Wire Line
	3400 2450 3900 2450
Wire Wire Line
	6900 2450 7400 2450
Wire Wire Line
	10600 2450 11100 2450
Wire Wire Line
	1400 2350 2000 2350
Wire Wire Line
	1650 2250 2000 2250
Wire Wire Line
	5000 2350 5500 2350
Wire Wire Line
	5250 2250 5500 2250
Wire Wire Line
	8600 2350 9150 2350
Wire Wire Line
	8850 2250 9150 2250
Wire Wire Line
	1450 5550 1650 5550
Wire Wire Line
	1450 4300 1650 4300
Wire Wire Line
	4650 4300 4800 4300
Wire Wire Line
	4650 5550 4800 5550
Wire Wire Line
	8150 4300 8300 4300
Wire Wire Line
	8150 5550 8300 5550
$Comp
L device:Q_NMOS_SGD Q6
U 1 1 5A8C12F1
P 2650 5550
F 0 "Q6" H 2850 5600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 2850 5500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 2850 5650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 2650 5550 50  0001 C CNN
F 4 "DigiKey" H 2650 5550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 2650 5550 60  0001 C CNN "Part"
	1    2650 5550
	1    0    0    -1  
$EndComp
$Comp
L device:Q_NMOS_SGD Q4
U 1 1 5A8D4DE7
P 6000 5550
F 0 "Q4" H 6200 5600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 6200 5500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 6200 5650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 6000 5550 50  0001 C CNN
F 4 "DigiKey" H 6000 5550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 6000 5550 60  0001 C CNN "Part"
	1    6000 5550
	1    0    0    -1  
$EndComp
$Comp
L device:Q_NMOS_SGD Q1
U 1 1 5A8D51CA
P 9500 4300
F 0 "Q1" H 9700 4350 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9700 4250 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9700 4400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9500 4300 50  0001 C CNN
F 4 "DigiKey" H 9500 4300 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9500 4300 60  0001 C CNN "Part"
	1    9500 4300
	1    0    0    -1  
$EndComp
$Comp
L device:Q_NMOS_SGD Q2
U 1 1 5A8D529E
P 9500 5550
F 0 "Q2" H 9700 5600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9700 5500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9700 5650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9500 5550 50  0001 C CNN
F 4 "DigiKey" H 9500 5550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9500 5550 60  0001 C CNN "Part"
	1    9500 5550
	1    0    0    -1  
$EndComp
$Comp
L device:Q_NMOS_SGD Q5
U 1 1 5A928DF8
P 2650 4300
F 0 "Q5" H 2850 4350 50  0000 L CNN
F 1 "STL160NS3LLH7" H 2850 4250 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 2850 4400 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 2650 4300 50  0001 C CNN
F 4 "DigiKey" H 2650 4300 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 2650 4300 60  0001 C CNN "Part"
	1    2650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4800 7050 4800
Wire Wire Line
	2750 4800 3700 4800
Wire Wire Line
	9600 4800 10600 4800
$EndSCHEMATC
