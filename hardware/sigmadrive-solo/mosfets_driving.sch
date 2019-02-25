EESchema Schematic File Version 4
LIBS:sigmadrive-solo-cache
EELAYER 26 0
EELAYER END
$Descr A2 23386 16535
encoding utf-8
Sheet 3 4
Title "Servo Driver 48V"
Date "2018-09-03"
Rev "1.0"
Comp "Sigmadrone"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 8400 1550 0    60   ~ 0
GHA
Text Label 11050 1550 0    60   ~ 0
GHB
Text Label 13650 1550 0    60   ~ 0
GHC
Text Label 8400 2800 0    60   ~ 0
GLA
Text Label 11050 2800 0    60   ~ 0
GLB
Text Label 13650 2800 0    60   ~ 0
GLC
Wire Wire Line
	9450 1050 9450 1150
Wire Wire Line
	9450 1750 9450 2050
Wire Wire Line
	9450 3000 9450 3150
Connection ~ 9450 3850
Connection ~ 9450 2050
Wire Wire Line
	12100 1750 12100 2050
Connection ~ 12100 3850
Connection ~ 12100 2050
Wire Wire Line
	14700 1050 14700 1150
Wire Wire Line
	14700 1750 14700 2050
Wire Wire Line
	14700 3000 14700 3150
Connection ~ 14700 3850
Connection ~ 14700 2050
Wire Wire Line
	9450 3850 9450 3950
Wire Wire Line
	9450 2050 9450 2600
Wire Wire Line
	12100 3850 12100 3950
Wire Wire Line
	12100 2050 12100 2600
Wire Wire Line
	14700 3850 14700 3950
Wire Wire Line
	14700 2050 14700 2600
$Comp
L Device:Q_NMOS_SGD Q5
U 1 1 5A928DF8
P 9350 1550
F 0 "Q5" H 9550 1600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9550 1500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9550 1650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9350 1550 50  0001 C CNN
F 4 "DigiKey" H 9350 1550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9350 1550 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9350 1550 50  0001 C CNN "Value1"
	1    9350 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	12100 2050 12600 2050
Wire Wire Line
	9450 2050 9950 2050
Wire Wire Line
	14700 2050 15200 2050
$Comp
L Device:R R67
U 1 1 5BC9D211
P 9450 3450
F 0 "R67" V 9530 3450 50  0000 C CNN
F 1 "0.010 1%" V 9350 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 9380 3450 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 9450 3450 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 9450 3450 60  0001 C CNN "Part"
F 5 "DigiKey" V 9450 3450 60  0001 C CNN "Provider"
	1    9450 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 3600 9450 3850
$Comp
L Device:R R68
U 1 1 5BC9D51F
P 12100 3450
F 0 "R68" V 12180 3450 50  0000 C CNN
F 1 "0.010 1%" V 12000 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 12030 3450 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 12100 3450 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 12100 3450 60  0001 C CNN "Part"
F 5 "DigiKey" V 12100 3450 60  0001 C CNN "Provider"
	1    12100 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	12100 3600 12100 3850
$Comp
L Device:R R69
U 1 1 5BC9D5F5
P 14700 3450
F 0 "R69" V 14780 3450 50  0000 C CNN
F 1 "0.010 1%" V 14600 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 14630 3450 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 14700 3450 50  0001 C CNN
F 4 "CRA2512-FZ-R010ELFCT-ND" V 14700 3450 60  0001 C CNN "Part"
F 5 "DigiKey" V 14700 3450 60  0001 C CNN "Provider"
	1    14700 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	14700 3600 14700 3850
Connection ~ 14700 3150
Wire Wire Line
	14700 3150 14700 3300
Connection ~ 9450 3150
Wire Wire Line
	9450 3150 9450 3300
Text Label 8700 10450 0    60   ~ 0
GHR
Text Label 8700 11700 0    60   ~ 0
GLR
Wire Wire Line
	9750 9950 9750 10050
Wire Wire Line
	9750 10650 9750 10950
Connection ~ 9750 10950
Wire Wire Line
	9750 10950 9750 11500
Wire Wire Line
	9750 10950 11300 10950
Text Label 11800 10950 2    60   ~ 0
PHASE_R
Wire Wire Line
	11300 11300 11300 10950
Connection ~ 11300 10950
Wire Wire Line
	11300 10950 11800 10950
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5BC5C1F6
P 11850 11400
AR Path="/58BE27E6/5BC5C1F6" Ref="J?"  Part="1" 
AR Path="/58BF664D/5BC5C1F6" Ref="J7"  Part="1" 
AR Path="/5BC5C1F6" Ref="J7"  Part="1" 
F 0 "J7" H 11769 11665 50  0000 C CNN
F 1 "CONN_01X02" H 11769 11574 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_02x7.62mm_Vertical" H 11850 11400 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1868076.pdf" H 11850 11400 50  0001 C CNN
F 4 "277-5840-ND" H 11850 11400 50  0001 C CNN "Part"
F 5 "DigiKey" H 11850 11400 50  0001 C CNN "Provider"
	1    11850 11400
	1    0    0    1   
$EndComp
Wire Wire Line
	11300 11300 11650 11300
Wire Wire Line
	11300 11400 11650 11400
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D199B5E
P 9750 9950
AR Path="/58BE27E6/5D199B5E" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D199B5E" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0129" H 9750 9800 50  0001 C CNN
F 1 "VIN" H 9765 10123 50  0000 C CNN
F 2 "" H 9750 9950 50  0000 C CNN
F 3 "" H 9750 9950 50  0000 C CNN
	1    9750 9950
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1A8822
P 14700 1050
AR Path="/58BE27E6/5D1A8822" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1A8822" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 14700 900 50  0001 C CNN
F 1 "VIN" H 14715 1223 50  0000 C CNN
F 2 "" H 14700 1050 50  0000 C CNN
F 3 "" H 14700 1050 50  0000 C CNN
	1    14700 1050
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1B74E6
P 12100 1050
AR Path="/58BE27E6/5D1B74E6" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1B74E6" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 12100 900 50  0001 C CNN
F 1 "VIN" H 12115 1223 50  0000 C CNN
F 2 "" H 12100 1050 50  0000 C CNN
F 3 "" H 12100 1050 50  0000 C CNN
	1    12100 1050
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D1C61AA
P 9450 1050
AR Path="/58BE27E6/5D1C61AA" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D1C61AA" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0132" H 9450 900 50  0001 C CNN
F 1 "VIN" H 9465 1223 50  0000 C CNN
F 2 "" H 9450 1050 50  0000 C CNN
F 3 "" H 9450 1050 50  0000 C CNN
	1    9450 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q6
U 1 1 5C792FA5
P 9350 2800
F 0 "Q6" H 9550 2850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9550 2750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9550 2900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9350 2800 50  0001 C CNN
F 4 "DigiKey" H 9350 2800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9350 2800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9350 2800 50  0001 C CNN "Value1"
	1    9350 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q3
U 1 1 5C79432A
P 12000 1550
F 0 "Q3" H 12200 1600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 12200 1500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 12200 1650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 12000 1550 50  0001 C CNN
F 4 "DigiKey" H 12000 1550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 12000 1550 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 12000 1550 50  0001 C CNN "Value1"
	1    12000 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q4
U 1 1 5C794408
P 12000 2800
F 0 "Q4" H 12200 2850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 12200 2750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 12200 2900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 12000 2800 50  0001 C CNN
F 4 "DigiKey" H 12000 2800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 12000 2800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 12000 2800 50  0001 C CNN "Value1"
	1    12000 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q1
U 1 1 5C794DDD
P 14600 1550
F 0 "Q1" H 14800 1600 50  0000 L CNN
F 1 "STL160NS3LLH7" H 14800 1500 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 14800 1650 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 14600 1550 50  0001 C CNN
F 4 "DigiKey" H 14600 1550 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 14600 1550 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 14600 1550 50  0001 C CNN "Value1"
	1    14600 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q2
U 1 1 5C794EBD
P 14600 2800
F 0 "Q2" H 14800 2850 50  0000 L CNN
F 1 "STL160NS3LLH7" H 14800 2750 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 14800 2900 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 14600 2800 50  0001 C CNN
F 4 "DigiKey" H 14600 2800 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 14600 2800 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 14600 2800 50  0001 C CNN "Value1"
	1    14600 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q7
U 1 1 5C79539C
P 9650 10450
F 0 "Q7" H 9850 10500 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9850 10400 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9850 10550 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9650 10450 50  0001 C CNN
F 4 "DigiKey" H 9650 10450 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9650 10450 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9650 10450 50  0001 C CNN "Value1"
	1    9650 10450
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q8
U 1 1 5C795574
P 9650 11700
F 0 "Q8" H 9850 11750 50  0000 L CNN
F 1 "STL160NS3LLH7" H 9850 11650 50  0000 L CNN
F 2 "Sigmadrone:PQFN_5x6_SGD" H 9850 11800 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/6c/2c/68/6d/40/eb/44/cd/DM00087549.pdf/files/DM00087549.pdf/jcr:content/translations/en.DM00087549.pdf" H 9650 11700 50  0001 C CNN
F 4 "DigiKey" H 9650 11700 60  0001 C CNN "Provider"
F 5 "497-14988-6-ND" H 9650 11700 60  0001 C CNN "Part"
F 6 "STL160N4F7" H 9650 11700 50  0001 C CNN "Value1"
	1    9650 11700
	1    0    0    -1  
$EndComp
$Comp
L Device:L L?
U 1 1 5C6E5DEC
P 6900 4550
AR Path="/58BE27E6/5C6E5DEC" Ref="L?"  Part="1" 
AR Path="/58BF664D/5C6E5DEC" Ref="L2"  Part="1" 
F 0 "L2" V 7090 4550 50  0000 C CNN
F 1 "22uH" V 6999 4550 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_WE-TPC-3816" H 6900 4550 50  0001 C CNN
F 3 "https://katalog.we-online.de/pbs/datasheet/744042150.pdf" H 6900 4550 50  0001 C CNN
F 4 "732-1107-1-ND" V 6900 4550 50  0001 C CNN "Part"
F 5 "DigiKey" V 6900 4550 50  0001 C CNN "Provider"
	1    6900 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5550 4750 6600 4750
$Comp
L Device:D_Shockley D3
U 1 1 5C6F6ADB
P 6600 5300
AR Path="/58BF664D/5C6F6ADB" Ref="D3"  Part="1" 
AR Path="/58BE27E6/5C6F6ADB" Ref="D?"  Part="1" 
F 0 "D3" H 6600 5400 50  0000 C CNN
F 1 "STPS0560Z" H 6600 5200 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 6600 5300 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 6600 5300 50  0001 C CNN
F 4 "497-3787-1-ND" H 6600 5300 60  0001 C CNN "Part"
F 5 "DigiKey" H 6600 5300 60  0001 C CNN "Provider"
	1    6600 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 5150 6600 4750
Connection ~ 6600 4750
$Comp
L Device:R R?
U 1 1 5C70F366
P 7650 4750
AR Path="/58BF599E/5C70F366" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5C70F366" Ref="R?"  Part="1" 
AR Path="/58BF664D/5C70F366" Ref="R18"  Part="1" 
F 0 "R18" V 7730 4750 50  0000 C CNN
F 1 "18k" V 7650 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7580 4750 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7650 4750 50  0001 C CNN
F 4 "311-18.0KHRCT-ND" V 7650 4750 60  0001 C CNN "Part"
F 5 "DigiKey" V 7650 4750 60  0001 C CNN "Provider"
	1    7650 4750
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5C70F372
P 7650 5300
AR Path="/58BF599E/5C70F372" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5C70F372" Ref="R?"  Part="1" 
AR Path="/58BF664D/5C70F372" Ref="R22"  Part="1" 
F 0 "R22" V 7730 5300 50  0000 C CNN
F 1 "3.3k" V 7650 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7580 5300 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7650 5300 50  0001 C CNN
F 4 "311-3.30KHRCT-ND" V 7650 5300 60  0001 C CNN "Part"
F 5 "DigiKey" V 7650 5300 60  0001 C CNN "Provider"
	1    7650 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	7650 5750 7650 5450
$Comp
L power:GND #PWR055
U 1 1 5C74C07F
P 7650 5750
F 0 "#PWR055" H 7650 5500 50  0001 C CNN
F 1 "GND" H 7650 5600 50  0000 C CNN
F 2 "" H 7650 5750 50  0001 C CNN
F 3 "" H 7650 5750 50  0001 C CNN
	1    7650 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR051
U 1 1 5C754A78
P 6600 5750
F 0 "#PWR051" H 6600 5500 50  0001 C CNN
F 1 "GND" H 6600 5600 50  0000 C CNN
F 2 "" H 6600 5750 50  0001 C CNN
F 3 "" H 6600 5750 50  0001 C CNN
	1    6600 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5750 6600 5450
$Comp
L Device:CP C?
U 1 1 5C78B9CA
P 7150 5300
AR Path="/58BE27E6/5C78B9CA" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C78B9CA" Ref="C26"  Part="1" 
F 0 "C26" H 7175 5400 50  0000 L CNN
F 1 "100uF" H 7175 5200 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 7188 5150 50  0001 C CNN
F 3 "http://katalog.we-online.de/pbs/datasheet/865080545012.pdf" H 7150 5300 50  0001 C CNN
F 4 "732-8511-1-ND" H 7150 5300 60  0001 C CNN "Part"
F 5 "DigiKey" H 7150 5300 60  0001 C CNN "Provider"
	1    7150 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR054
U 1 1 5C7A75EC
P 7150 5750
F 0 "#PWR054" H 7150 5500 50  0001 C CNN
F 1 "GND" H 7150 5600 50  0000 C CNN
F 2 "" H 7150 5750 50  0001 C CNN
F 3 "" H 7150 5750 50  0001 C CNN
	1    7150 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5750 7150 5450
$Comp
L Device:C C?
U 1 1 5C7D8909
P 6100 4550
AR Path="/58BE2779/5C7D8909" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C7D8909" Ref="C22"  Part="1" 
F 0 "C22" V 6150 4400 50  0000 L CNN
F 1 "100nF" V 6050 4250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6138 4400 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 6100 4550 50  0001 C CNN
F 4 "490-4779-2-ND" H 6100 4550 60  0001 C CNN "Part"
F 5 "DigiKey" H 6100 4550 60  0001 C CNN "Provider"
	1    6100 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5550 4550 5950 4550
Wire Wire Line
	6250 4550 6600 4550
Wire Wire Line
	6600 4550 6600 4750
Wire Wire Line
	6600 4550 6750 4550
Connection ~ 6600 4550
Wire Wire Line
	7050 4550 7150 4550
Wire Wire Line
	7150 4550 7150 5150
Wire Wire Line
	7150 4550 7650 4550
Wire Wire Line
	7650 4550 7650 4600
Connection ~ 7150 4550
Wire Wire Line
	7650 4900 7650 4950
Wire Wire Line
	5550 4950 7650 4950
Connection ~ 7650 4950
Wire Wire Line
	7650 4950 7650 5150
$Comp
L power:+5V #PWR?
U 1 1 5C8517F8
P 7150 4450
AR Path="/58BE27E6/5C8517F8" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5C8517F8" Ref="#PWR053"  Part="1" 
F 0 "#PWR053" H 7150 4300 50  0001 C CNN
F 1 "+5V" H 7165 4623 50  0000 C CNN
F 2 "" H 7150 4450 50  0001 C CNN
F 3 "" H 7150 4450 50  0001 C CNN
	1    7150 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4550 7150 4450
$Comp
L Device:C C?
U 1 1 5C865E6F
P 3250 1450
AR Path="/58BE27E6/5C865E6F" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C865E6F" Ref="C13"  Part="1" 
F 0 "C13" H 3250 1550 50  0000 L CNN
F 1 "1uF 16V" V 3100 1200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3288 1300 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3250 1450 50  0001 C CNN
F 4 "732-7971-1-ND" H 3250 1450 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 1450 60  0001 C CNN "Provider"
	1    3250 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C86FE85
P 2350 1450
AR Path="/58BE27E6/5C86FE85" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C86FE85" Ref="C6"  Part="1" 
F 0 "C6" H 2350 1550 50  0000 L CNN
F 1 "4.7uF 50V" V 2200 1200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2388 1300 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 2350 1450 50  0001 C CNN
F 4 "490-10751-2-ND" H 2350 1450 60  0001 C CNN "Part"
F 5 "DigiKey" H 2350 1450 60  0001 C CNN "Provider"
F 6 "4.7uF 25V" H 2350 1450 50  0001 C CNN "Value1"
	1    2350 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C879D46
P 2800 1450
AR Path="/58BE2779/5C879D46" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C879D46" Ref="C12"  Part="1" 
F 0 "C12" H 2800 1550 50  0000 L CNN
F 1 "100nF" V 2650 1350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2838 1300 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2800 1450 50  0001 C CNN
F 4 "490-4779-2-ND" H 2800 1450 60  0001 C CNN "Part"
F 5 "DigiKey" H 2800 1450 60  0001 C CNN "Provider"
	1    2800 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 1250 3250 1250
Wire Wire Line
	3250 1250 3250 1300
Wire Wire Line
	3250 1600 3250 1650
Wire Wire Line
	3250 1650 3950 1650
Wire Wire Line
	2350 1300 2350 1250
Wire Wire Line
	2350 1250 2800 1250
Connection ~ 3250 1250
Wire Wire Line
	2800 1300 2800 1250
Connection ~ 2800 1250
Wire Wire Line
	2800 1250 3250 1250
Wire Wire Line
	2350 1600 2350 1650
Wire Wire Line
	2350 1650 2800 1650
Wire Wire Line
	2800 1650 2800 1600
$Comp
L Device:C C?
U 1 1 5C971DF5
P 3350 5250
AR Path="/58BE27E6/5C971DF5" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C971DF5" Ref="C17"  Part="1" 
F 0 "C17" H 3350 5350 50  0000 L CNN
F 1 "4.7uF 50V" V 3200 5000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3388 5100 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 3350 5250 50  0001 C CNN
F 4 "490-10751-2-ND" H 3350 5250 60  0001 C CNN "Part"
F 5 "DigiKey" H 3350 5250 60  0001 C CNN "Provider"
F 6 "4.7uF 25V" H 3350 5250 50  0001 C CNN "Value1"
	1    3350 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4950 3800 4950
Wire Wire Line
	3350 4950 3350 5100
$Comp
L power:GND #PWR042
U 1 1 5C97DC3B
P 3350 5500
F 0 "#PWR042" H 3350 5250 50  0001 C CNN
F 1 "GND" H 3350 5350 50  0000 C CNN
F 2 "" H 3350 5500 50  0001 C CNN
F 3 "" H 3350 5500 50  0001 C CNN
	1    3350 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 5400 3350 5500
Wire Wire Line
	2350 1750 2350 1650
Connection ~ 2350 1650
$Comp
L power:GND #PWR047
U 1 1 5C9A2FA1
P 6250 5750
F 0 "#PWR047" H 6250 5500 50  0001 C CNN
F 1 "GND" H 6250 5600 50  0000 C CNN
F 2 "" H 6250 5750 50  0001 C CNN
F 3 "" H 6250 5750 50  0001 C CNN
	1    6250 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5150 6250 5150
Wire Wire Line
	6250 5150 6250 5250
Wire Wire Line
	5550 5250 6250 5250
Connection ~ 6250 5250
Wire Wire Line
	6250 5250 6250 5350
Wire Wire Line
	5550 5350 6250 5350
Connection ~ 6250 5350
Wire Wire Line
	6250 5350 6250 5450
Wire Wire Line
	5550 5450 6250 5450
Connection ~ 6250 5450
Wire Wire Line
	5550 5550 6000 5550
Wire Wire Line
	6000 5550 6000 5650
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5CA3B4A1
P 2350 1150
AR Path="/58BE27E6/5CA3B4A1" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5CA3B4A1" Ref="#PWR030"  Part="1" 
F 0 "#PWR030" H 2350 1000 50  0001 C CNN
F 1 "VIN" H 2365 1323 50  0000 C CNN
F 2 "" H 2350 1150 50  0000 C CNN
F 3 "" H 2350 1150 50  0000 C CNN
	1    2350 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1150 2350 1250
Connection ~ 2350 1250
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5CA64BF7
P 3350 4850
AR Path="/58BE27E6/5CA64BF7" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5CA64BF7" Ref="#PWR038"  Part="1" 
F 0 "#PWR038" H 3350 4700 50  0001 C CNN
F 1 "VIN" H 3350 5000 50  0000 C CNN
F 2 "" H 3350 4850 50  0000 C CNN
F 3 "" H 3350 4850 50  0000 C CNN
	1    3350 4850
	1    0    0    -1  
$EndComp
Connection ~ 3350 4950
Wire Wire Line
	3350 4850 3350 4950
$Comp
L sigmadrone:DRV8323RS U2
U 1 1 5CBE0032
P 4750 3450
F 0 "U2" H 4750 5800 60  0000 C CNN
F 1 "DRV8323RS" V 4750 3450 60  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.45x5.45mm_ThermalVias" H 4750 3100 60  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/drv8323r.pdf" H 4750 3100 60  0001 C CNN
F 4 "296-47769-1-ND" H 4750 3450 50  0001 C CNN "Part"
F 5 "DigiKey" H 4750 3450 50  0001 C CNN "Provider"
	1    4750 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5150 3800 5150
Wire Wire Line
	3800 5150 3800 4950
Connection ~ 3800 4950
Wire Wire Line
	3800 4950 3350 4950
$Comp
L Device:C C?
U 1 1 5CC4E978
P 3250 2250
AR Path="/58BE27E6/5CC4E978" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CC4E978" Ref="C16"  Part="1" 
F 0 "C16" H 3250 2350 50  0000 L CNN
F 1 "0.047uF 100V" V 3100 2000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3288 2100 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3250 2250 50  0001 C CNN
F 4 "490-4969-1-ND" H 3250 2250 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 2250 60  0001 C CNN "Provider"
	1    3250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2050 3250 2050
Wire Wire Line
	3250 2050 3250 2100
Wire Wire Line
	3250 2400 3250 2450
Wire Wire Line
	3250 2450 3950 2450
$Comp
L Device:C C?
U 1 1 5CC701F5
P 3500 2650
AR Path="/58BE27E6/5CC701F5" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CC701F5" Ref="C18"  Part="1" 
F 0 "C18" V 3450 2700 50  0000 L CNN
F 1 "1uF 16V" V 3550 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3538 2500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3500 2650 50  0001 C CNN
F 4 "732-7971-1-ND" H 3500 2650 60  0001 C CNN "Part"
F 5 "DigiKey" H 3500 2650 60  0001 C CNN "Provider"
	1    3500 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR037
U 1 1 5CC80FC4
P 3100 2650
F 0 "#PWR037" H 3100 2400 50  0001 C CNN
F 1 "GND" H 3100 2500 50  0000 C CNN
F 2 "" H 3100 2650 50  0001 C CNN
F 3 "" H 3100 2650 50  0001 C CNN
	1    3100 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 2650 3100 2650
Wire Wire Line
	3650 2650 3950 2650
Text HLabel 3600 2850 0    60   Input ~ 0
GATE_ENABLE
Wire Wire Line
	3600 2850 3950 2850
Text HLabel 3600 2950 0    60   Input ~ 0
PWM_AH
Text HLabel 3600 3150 0    60   Input ~ 0
PWM_BH
Text HLabel 1450 3750 0    60   Input ~ 0
DRV_SDI
Text HLabel 3600 3050 0    60   Input ~ 0
PWM_AL
Text HLabel 1450 3650 0    60   Output ~ 0
DRV_SDO
Text HLabel 1450 3850 0    60   Input ~ 0
DRV_CLK
Wire Wire Line
	3600 2950 3950 2950
Wire Wire Line
	3600 3050 3950 3050
Wire Wire Line
	3600 3150 3950 3150
Wire Wire Line
	3600 3250 3950 3250
Wire Wire Line
	3600 3350 3950 3350
Wire Wire Line
	3600 3450 3950 3450
Text HLabel 1450 3950 0    60   Input ~ 0
DRV1_CS
Text HLabel 3600 3250 0    60   Input ~ 0
PWM_BL
Text HLabel 3600 3350 0    60   Input ~ 0
PWM_CH
Text HLabel 3600 3450 0    60   Input ~ 0
PWM_CL
Wire Wire Line
	1450 3750 3950 3750
Wire Wire Line
	1450 3850 3950 3850
Wire Wire Line
	1450 3950 3950 3950
Text HLabel 1450 4050 0    60   Input ~ 0
DRV_CAL
Wire Wire Line
	1450 4050 3950 4050
Text HLabel 1450 4150 0    60   Output ~ 0
DRV_FAULT
$Comp
L power:+3.3V #PWR035
U 1 1 5CDEADE7
P 2550 3150
F 0 "#PWR035" H 2550 3000 50  0001 C CNN
F 1 "+3.3V" H 2565 3323 50  0000 C CNN
F 2 "" H 2550 3150 50  0001 C CNN
F 3 "" H 2550 3150 50  0001 C CNN
	1    2550 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3150 2550 4250
$Comp
L Device:C C?
U 1 1 5CDFEE38
P 2550 4900
AR Path="/58BE2779/5CDFEE38" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CDFEE38" Ref="C7"  Part="1" 
F 0 "C7" H 2550 5000 50  0000 L CNN
F 1 "100nF" V 2400 4800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2588 4750 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2550 4900 50  0001 C CNN
F 4 "490-4779-2-ND" H 2550 4900 60  0001 C CNN "Part"
F 5 "DigiKey" H 2550 4900 60  0001 C CNN "Provider"
	1    2550 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 5CDFF17A
P 2550 5150
F 0 "#PWR036" H 2550 4900 50  0001 C CNN
F 1 "GND" H 2550 5000 50  0000 C CNN
F 2 "" H 2550 5150 50  0001 C CNN
F 3 "" H 2550 5150 50  0001 C CNN
	1    2550 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4750 2550 4250
Text HLabel 1450 4350 0    60   Output ~ 0
DRV_SOC
Text HLabel 1450 4450 0    60   Output ~ 0
DRV_SOB
Text HLabel 1450 4550 0    60   Output ~ 0
DRV_SOA
Wire Wire Line
	1450 4350 3950 4350
Wire Wire Line
	1450 4450 3950 4450
Wire Wire Line
	1450 4550 3950 4550
Wire Wire Line
	2550 5050 2550 5150
Connection ~ 2550 4250
Wire Wire Line
	2550 4250 3950 4250
Text Label 6300 1250 2    60   ~ 0
VDRAIN
Wire Wire Line
	5550 1250 6300 1250
Text Label 11350 1150 0    60   ~ 0
VDRAIN
Wire Wire Line
	12100 1150 11350 1150
Text Label 6300 1450 2    60   ~ 0
GHA
Wire Wire Line
	5550 1450 6300 1450
Text Label 6300 1650 2    60   ~ 0
GLA
Wire Wire Line
	5550 1650 6300 1650
Text Label 6300 1850 2    60   ~ 0
GHB
Wire Wire Line
	5550 1850 6300 1850
Text Label 6300 2050 2    60   ~ 0
GLB
Wire Wire Line
	5550 2050 6300 2050
Text Label 6300 2300 2    60   ~ 0
GHC
Wire Wire Line
	5550 2300 6300 2300
Text Label 6300 2500 2    60   ~ 0
GLC
Wire Wire Line
	5550 2500 6300 2500
Wire Wire Line
	13650 1550 14400 1550
Wire Wire Line
	13650 2800 14400 2800
Wire Wire Line
	11050 1550 11800 1550
Wire Wire Line
	11050 2800 11800 2800
Wire Wire Line
	8400 1550 9150 1550
Wire Wire Line
	8400 2800 9150 2800
Text Label 8400 2050 0    60   ~ 0
SHA
Text Label 11050 2050 0    60   ~ 0
SHB
Text Label 13650 2050 0    60   ~ 0
SHC
Text Label 6300 1550 2    60   ~ 0
SHA
Wire Wire Line
	5550 1550 6300 1550
Text Label 6300 1950 2    60   ~ 0
SHB
Wire Wire Line
	5550 1950 6300 1950
Text Label 6300 2400 2    60   ~ 0
SHC
Wire Wire Line
	5550 2400 6300 2400
Text Label 8400 3150 0    60   ~ 0
A_ISENSE_P
Text Label 8400 3850 0    60   ~ 0
A_ISENSE_N
$Comp
L Device:Net-Tie_2 NT2
U 1 1 5D379876
P 9150 3850
F 0 "NT2" H 9150 4028 50  0000 C CNN
F 1 "Net-Tie_2" H 9150 3937 50  0000 C CNN
F 2 "NetTie:NetTie-2_SMD_Pad0.5mm" H 9150 3850 50  0001 C CNN
F 3 "~" H 9150 3850 50  0001 C CNN
	1    9150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 3150 9450 3150
Wire Wire Line
	8400 3850 9050 3850
Wire Wire Line
	9250 3850 9450 3850
Text Label 11050 3150 0    60   ~ 0
B_ISENSE_P
Text Label 11050 3850 0    60   ~ 0
B_ISENSE_N
$Comp
L Device:Net-Tie_2 NT4
U 1 1 5D3C069E
P 11800 3850
F 0 "NT4" H 11800 4028 50  0000 C CNN
F 1 "Net-Tie_2" H 11800 3937 50  0000 C CNN
F 2 "NetTie:NetTie-2_SMD_Pad0.5mm" H 11800 3850 50  0001 C CNN
F 3 "~" H 11800 3850 50  0001 C CNN
	1    11800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 3150 12100 3150
Wire Wire Line
	11050 3850 11700 3850
Wire Wire Line
	11900 3850 12100 3850
Text Label 13650 3150 0    60   ~ 0
C_ISENSE_P
Text Label 13650 3850 0    60   ~ 0
C_ISENSE_N
$Comp
L Device:Net-Tie_2 NT6
U 1 1 5D3CEBCB
P 14400 3850
F 0 "NT6" H 14400 4028 50  0000 C CNN
F 1 "Net-Tie_2" H 14400 3937 50  0000 C CNN
F 2 "NetTie:NetTie-2_SMD_Pad0.5mm" H 14400 3850 50  0001 C CNN
F 3 "~" H 14400 3850 50  0001 C CNN
	1    14400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	13650 3150 14700 3150
Wire Wire Line
	13650 3850 14300 3850
Wire Wire Line
	14500 3850 14700 3850
Wire Wire Line
	12100 3000 12100 3150
Connection ~ 12100 3150
Wire Wire Line
	12100 3150 12100 3300
Text Label 6200 3950 2    60   ~ 0
A_ISENSE_P
Text Label 6200 4350 2    60   ~ 0
A_ISENSE_N
Wire Wire Line
	6200 4350 5550 4350
Wire Wire Line
	5550 3950 6200 3950
Text Label 6200 3450 2    60   ~ 0
B_ISENSE_P
Text Label 6200 3850 2    60   ~ 0
B_ISENSE_N
Wire Wire Line
	6200 3850 5550 3850
Wire Wire Line
	5550 3450 6200 3450
Text Label 6200 2950 2    60   ~ 0
C_ISENSE_P
Text Label 6200 3350 2    60   ~ 0
C_ISENSE_N
Wire Wire Line
	6200 3350 5550 3350
Wire Wire Line
	5550 2950 6200 2950
Text Notes 2450 8300 0    60   ~ 0
Power\nDecoupling\n
$Comp
L power:GND #PWR?
U 1 1 5D5E9124
P 3500 8450
AR Path="/58BE27E6/5D5E9124" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D5E9124" Ref="#PWR077"  Part="1" 
F 0 "#PWR077" H 3500 8200 50  0001 C CNN
F 1 "GND" H 3500 8300 50  0000 C CNN
F 2 "" H 3500 8450 50  0001 C CNN
F 3 "" H 3500 8450 50  0001 C CNN
	1    3500 8450
	1    0    0    -1  
$EndComp
Text HLabel 3900 7800 2    60   Output ~ 0
VBAT_ADC
Text Notes 3850 7600 0    60   ~ 0
Voltage Sense
Wire Wire Line
	3500 8450 3500 8200
Wire Wire Line
	3500 7650 3500 7800
Wire Wire Line
	3500 7150 3500 7350
Wire Wire Line
	3500 7800 3900 7800
Connection ~ 3500 7800
Wire Wire Line
	3500 7800 3500 7900
$Comp
L Device:R R?
U 1 1 5D5E9150
P 3500 7500
AR Path="/58BE27E6/5D5E9150" Ref="R?"  Part="1" 
AR Path="/58BF664D/5D5E9150" Ref="R23"  Part="1" 
F 0 "R23" H 3650 7500 50  0000 C CNN
F 1 "47k" V 3500 7500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 7500 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3500 7500 50  0001 C CNN
F 4 "311-47KDCT-ND" V 3500 7500 60  0001 C CNN "Part"
F 5 "DigiKey" V 3500 7500 60  0001 C CNN "Provider"
	1    3500 7500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5D5E9159
P 3500 8050
AR Path="/58BE27E6/5D5E9159" Ref="R?"  Part="1" 
AR Path="/58BF664D/5D5E9159" Ref="R24"  Part="1" 
F 0 "R24" H 3650 8050 50  0000 C CNN
F 1 "4.7k" V 3500 8050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 8050 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 3500 8050 50  0001 C CNN
F 4 "311-2606-2-ND" V 3500 8050 60  0001 C CNN "Part"
F 5 "DigiKey" V 3500 8050 60  0001 C CNN "Provider"
	1    3500 8050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5D5E9162
P 1250 7650
AR Path="/58BE27E6/5D5E9162" Ref="J?"  Part="1" 
AR Path="/58BF664D/5D5E9162" Ref="J1"  Part="1" 
F 0 "J1" H 1169 7915 50  0000 C CNN
F 1 "CONN_01X02" H 1169 7824 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_02x7.62mm_Vertical" H 1250 7650 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1868076.pdf" H 1250 7650 50  0001 C CNN
F 4 "277-5840-ND" H 1250 7650 50  0001 C CNN "Part"
F 5 "DigiKey" H 1250 7650 50  0001 C CNN "Provider"
	1    1250 7650
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C?
U 1 1 5D5E916B
P 2150 7800
AR Path="/58BE27E6/5D5E916B" Ref="C?"  Part="1" 
AR Path="/58BF664D/5D5E916B" Ref="C36"  Part="1" 
F 0 "C36" H 2175 7900 50  0000 L CNN
F 1 "120uF" H 2175 7700 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 2188 7650 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDD0000/DMD0000COL92.pdf" H 2150 7800 50  0001 C CNN
F 4 "P19316CT-ND" H 2150 7800 60  0001 C CNN "Part"
F 5 "DigiKey" H 2150 7800 60  0001 C CNN "Provider"
	1    2150 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 7650 1850 7650
Wire Wire Line
	1850 7650 1850 8050
Wire Wire Line
	2150 7950 2150 8050
Wire Wire Line
	2150 8050 2450 8050
Wire Wire Line
	2450 7950 2450 8050
Wire Wire Line
	2150 7650 2150 7550
Wire Wire Line
	2150 7550 2450 7550
Wire Wire Line
	2450 7650 2450 7550
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D5E9181
P 1850 7250
AR Path="/58BE27E6/5D5E9181" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D5E9181" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 1850 7100 50  0001 C CNN
F 1 "VIN" H 1865 7423 50  0000 C CNN
F 2 "" H 1850 7250 50  0000 C CNN
F 3 "" H 1850 7250 50  0000 C CNN
	1    1850 7250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C?
U 1 1 5D5E9190
P 2450 7800
AR Path="/58BE27E6/5D5E9190" Ref="C?"  Part="1" 
AR Path="/58BF664D/5D5E9190" Ref="C37"  Part="1" 
F 0 "C37" H 2475 7900 50  0000 L CNN
F 1 "120uF" H 2475 7700 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10" H 2488 7650 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDD0000/DMD0000COL92.pdf" H 2450 7800 50  0001 C CNN
F 4 "P19316CT-ND" H 2450 7800 60  0001 C CNN "Part"
F 5 "DigiKey" H 2450 7800 60  0001 C CNN "Provider"
	1    2450 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	11600 4750 12250 4750
Wire Wire Line
	11600 4550 12250 4550
Wire Wire Line
	11600 4650 12250 4650
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5D5E91A8
P 12450 4650
AR Path="/58BE27E6/5D5E91A8" Ref="J?"  Part="1" 
AR Path="/58BF664D/5D5E91A8" Ref="J3"  Part="1" 
F 0 "J3" H 12369 4965 50  0000 C CNN
F 1 "CONN_01X03" H 12369 4874 50  0000 C CNN
F 2 "Sigmadrone:PhoenixContact_MKDS_03x7.62mm_Vertical" H 12450 4650 50  0001 C CNN
F 3 "https://media.digikey.com/PDF/Data%20Sheets/Phoenix%20Contact%20PDFs/1704936.pdf" H 12450 4650 50  0001 C CNN
F 4 "277-5954-ND" H 12450 4650 50  0001 C CNN "Part"
F 5 "DigiKey" H 12450 4650 50  0001 C CNN "Provider"
	1    12450 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D5E91BC
P 6900 7100
AR Path="/58BE27E6/5D5E91BC" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D5E91BC" Ref="#PWR080"  Part="1" 
F 0 "#PWR080" H 6900 6850 50  0001 C CNN
F 1 "GND" H 6900 6950 50  0000 C CNN
F 2 "" H 6900 7100 50  0001 C CNN
F 3 "" H 6900 7100 50  0001 C CNN
	1    6900 7100
	1    0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5D5E91C2
P 6900 7600
AR Path="/58BE27E6/5D5E91C2" Ref="#FLG?"  Part="1" 
AR Path="/58BF664D/5D5E91C2" Ref="#FLG04"  Part="1" 
F 0 "#FLG04" H 6900 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 6900 7773 50  0000 C CNN
F 2 "" H 6900 7600 50  0001 C CNN
F 3 "" H 6900 7600 50  0001 C CNN
	1    6900 7600
	-1   0    0    1   
$EndComp
Wire Wire Line
	6900 7100 6900 7400
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D5E91C9
P 6350 7100
AR Path="/58BE27E6/5D5E91C9" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D5E91C9" Ref="#PWR078"  Part="1" 
F 0 "#PWR078" H 6350 6950 50  0001 C CNN
F 1 "VIN" H 6365 7273 50  0000 C CNN
F 2 "" H 6350 7100 50  0000 C CNN
F 3 "" H 6350 7100 50  0000 C CNN
	1    6350 7100
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5D5E91CF
P 6350 7600
AR Path="/58BE27E6/5D5E91CF" Ref="#FLG?"  Part="1" 
AR Path="/58BF664D/5D5E91CF" Ref="#FLG02"  Part="1" 
F 0 "#FLG02" H 6350 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 6350 7773 50  0000 C CNN
F 2 "" H 6350 7600 50  0001 C CNN
F 3 "" H 6350 7600 50  0001 C CNN
	1    6350 7600
	-1   0    0    1   
$EndComp
Wire Wire Line
	6350 7100 6350 7600
Wire Wire Line
	1450 7550 1850 7550
Wire Wire Line
	1850 7550 1850 7250
Wire Wire Line
	1850 7550 2150 7550
Connection ~ 1850 7550
Connection ~ 2150 7550
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5D5E91E1
P 3500 7150
AR Path="/58BE27E6/5D5E91E1" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5D5E91E1" Ref="#PWR074"  Part="1" 
F 0 "#PWR074" H 3500 7000 50  0001 C CNN
F 1 "VIN" H 3515 7323 50  0000 C CNN
F 2 "" H 3500 7150 50  0000 C CNN
F 3 "" H 3500 7150 50  0000 C CNN
	1    3500 7150
	1    0    0    -1  
$EndComp
Text Label 11600 4750 0    60   ~ 0
PHASE_C
Text Label 11600 4650 0    60   ~ 0
PHASE_B
Text Label 11600 4550 0    60   ~ 0
PHASE_A
Text Label 15200 2050 2    60   ~ 0
PHASE_C
Text Label 12600 2050 2    60   ~ 0
PHASE_B
Text Label 9950 2050 2    60   ~ 0
PHASE_A
Text HLabel 9950 2050 2    60   Output ~ 0
PHASE_A
Text HLabel 12600 2050 2    60   Output ~ 0
PHASE_B
Text HLabel 15200 2050 2    60   Output ~ 0
PHASE_C
Wire Wire Line
	11300 11400 11300 12850
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5C93CF09
P 5800 7600
AR Path="/58BE27E6/5C93CF09" Ref="#FLG?"  Part="1" 
AR Path="/58BF664D/5C93CF09" Ref="#FLG0101"  Part="1" 
F 0 "#FLG0101" H 5800 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 5800 7773 50  0000 C CNN
F 2 "" H 5800 7600 50  0001 C CNN
F 3 "" H 5800 7600 50  0001 C CNN
	1    5800 7600
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C93D3B4
P 5800 7100
AR Path="/58BE27E6/5C93D3B4" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5C93D3B4" Ref="#PWR0102"  Part="1" 
F 0 "#PWR0102" H 5800 6950 50  0001 C CNN
F 1 "+5V" H 5815 7273 50  0000 C CNN
F 2 "" H 5800 7100 50  0001 C CNN
F 3 "" H 5800 7100 50  0001 C CNN
	1    5800 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 7100 5800 7600
Wire Wire Line
	13650 2050 14700 2050
Wire Wire Line
	11050 2050 12100 2050
Wire Wire Line
	8400 2050 9450 2050
$Comp
L Device:C C?
U 1 1 5C79A319
P 9900 1150
AR Path="/58BE27E6/5C79A319" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C79A319" Ref="C11"  Part="1" 
F 0 "C11" V 9950 1000 50  0000 L CNN
F 1 "1uF 100V" V 9750 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9938 1000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 9900 1150 50  0001 C CNN
F 4 "445-4468-1-ND" H 9900 1150 60  0001 C CNN "Part"
F 5 "DigiKey" H 9900 1150 60  0001 C CNN "Provider"
	1    9900 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9750 1150 9450 1150
Connection ~ 9450 1150
Wire Wire Line
	9450 1150 9450 1350
Wire Wire Line
	10050 1150 10250 1150
Wire Wire Line
	12100 1050 12100 1150
Connection ~ 12100 1150
Wire Wire Line
	12100 1150 12100 1350
$Comp
L Device:C C?
U 1 1 5C8289D7
P 12550 1150
AR Path="/58BE27E6/5C8289D7" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C8289D7" Ref="C39"  Part="1" 
F 0 "C39" V 12600 1000 50  0000 L CNN
F 1 "1uF 100V" V 12400 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 12588 1000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 12550 1150 50  0001 C CNN
F 4 "445-4468-1-ND" H 12550 1150 60  0001 C CNN "Part"
F 5 "DigiKey" H 12550 1150 60  0001 C CNN "Provider"
	1    12550 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	12400 1150 12100 1150
Wire Wire Line
	12700 1150 12900 1150
$Comp
L Device:C C?
U 1 1 5C8453D9
P 15150 1150
AR Path="/58BE27E6/5C8453D9" Ref="C?"  Part="1" 
AR Path="/58BF664D/5C8453D9" Ref="C40"  Part="1" 
F 0 "C40" V 15200 1000 50  0000 L CNN
F 1 "1uF 100V" V 15000 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 15188 1000 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 15150 1150 50  0001 C CNN
F 4 "445-4468-1-ND" H 15150 1150 60  0001 C CNN "Part"
F 5 "DigiKey" H 15150 1150 60  0001 C CNN "Provider"
	1    15150 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	15000 1150 14700 1150
Wire Wire Line
	15300 1150 15500 1150
Connection ~ 14700 1150
Wire Wire Line
	14700 1150 14700 1350
Wire Wire Line
	2150 8050 1850 8050
Connection ~ 2150 8050
Connection ~ 1850 8050
Wire Wire Line
	1850 8050 1850 8200
Wire Wire Line
	6250 5450 6250 5750
Wire Wire Line
	5550 5650 6000 5650
Connection ~ 6000 5650
Wire Wire Line
	6000 5650 6000 5750
$Comp
L sigmadrone:PGND #PWR0119
U 1 1 5C8C7B0A
P 1850 8200
F 0 "#PWR0119" H 1850 7950 50  0001 C CNN
F 1 "PGND" H 1855 8027 50  0000 C CNN
F 2 "" H 1850 8200 50  0001 C CNN
F 3 "" H 1850 8200 50  0001 C CNN
	1    1850 8200
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR0120
U 1 1 5C8C8050
P 6000 5750
F 0 "#PWR0120" H 6000 5500 50  0001 C CNN
F 1 "PGND" H 6005 5577 50  0000 C CNN
F 2 "" H 6000 5750 50  0001 C CNN
F 3 "" H 6000 5750 50  0001 C CNN
	1    6000 5750
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR0121
U 1 1 5C8C8623
P 9450 3950
F 0 "#PWR0121" H 9450 3700 50  0001 C CNN
F 1 "PGND" H 9455 3777 50  0000 C CNN
F 2 "" H 9450 3950 50  0001 C CNN
F 3 "" H 9450 3950 50  0001 C CNN
	1    9450 3950
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR0122
U 1 1 5C8C8A3F
P 12100 3950
F 0 "#PWR0122" H 12100 3700 50  0001 C CNN
F 1 "PGND" H 12105 3777 50  0000 C CNN
F 2 "" H 12100 3950 50  0001 C CNN
F 3 "" H 12100 3950 50  0001 C CNN
	1    12100 3950
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR0123
U 1 1 5C8E5E1D
P 14700 3950
F 0 "#PWR0123" H 14700 3700 50  0001 C CNN
F 1 "PGND" H 14705 3777 50  0000 C CNN
F 2 "" H 14700 3950 50  0001 C CNN
F 3 "" H 14700 3950 50  0001 C CNN
	1    14700 3950
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR0124
U 1 1 5C903323
P 15500 1150
F 0 "#PWR0124" H 15500 900 50  0001 C CNN
F 1 "PGND" V 15505 1022 50  0000 R CNN
F 2 "" H 15500 1150 50  0001 C CNN
F 3 "" H 15500 1150 50  0001 C CNN
	1    15500 1150
	0    -1   -1   0   
$EndComp
$Comp
L sigmadrone:PGND #PWR0125
U 1 1 5C920BFA
P 12900 1150
F 0 "#PWR0125" H 12900 900 50  0001 C CNN
F 1 "PGND" V 12905 1022 50  0000 R CNN
F 2 "" H 12900 1150 50  0001 C CNN
F 3 "" H 12900 1150 50  0001 C CNN
	1    12900 1150
	0    -1   -1   0   
$EndComp
$Comp
L sigmadrone:PGND #PWR0126
U 1 1 5C93E0B6
P 10250 1150
F 0 "#PWR0126" H 10250 900 50  0001 C CNN
F 1 "PGND" V 10255 1022 50  0000 R CNN
F 2 "" H 10250 1150 50  0001 C CNN
F 3 "" H 10250 1150 50  0001 C CNN
	1    10250 1150
	0    -1   -1   0   
$EndComp
$Comp
L sigmadrone:PGND #PWR0127
U 1 1 5C95C2B5
P 2350 1750
F 0 "#PWR0127" H 2350 1500 50  0001 C CNN
F 1 "PGND" H 2355 1577 50  0000 C CNN
F 2 "" H 2350 1750 50  0001 C CNN
F 3 "" H 2350 1750 50  0001 C CNN
	1    2350 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 11700 9450 11700
Wire Wire Line
	8700 10450 9450 10450
Text Label 8700 10950 0    60   ~ 0
SHR
Wire Wire Line
	8700 10950 9750 10950
Wire Wire Line
	9750 11900 9750 12850
$Comp
L Device:L L?
U 1 1 5CA944B3
P 6900 13350
AR Path="/58BE27E6/5CA944B3" Ref="L?"  Part="1" 
AR Path="/58BF664D/5CA944B3" Ref="L1"  Part="1" 
F 0 "L1" V 7090 13350 50  0000 C CNN
F 1 "22uH" V 6999 13350 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_WE-TPC-3816" H 6900 13350 50  0001 C CNN
F 3 "https://katalog.we-online.de/pbs/datasheet/744042150.pdf" H 6900 13350 50  0001 C CNN
F 4 "732-1107-1-ND" V 6900 13350 50  0001 C CNN "Part"
F 5 "DigiKey" V 6900 13350 50  0001 C CNN "Provider"
	1    6900 13350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5550 13550 6600 13550
$Comp
L Device:D_Shockley D2
U 1 1 5CA944BD
P 6600 14100
AR Path="/58BF664D/5CA944BD" Ref="D2"  Part="1" 
AR Path="/58BE27E6/5CA944BD" Ref="D?"  Part="1" 
F 0 "D2" H 6600 14200 50  0000 C CNN
F 1 "STPS0560Z" H 6600 14000 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 6600 14100 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/group1/0f/03/9b/7c/50/58/48/d7/CD00001852/files/CD00001852.pdf/jcr:content/translations/en.CD00001852.pdf" H 6600 14100 50  0001 C CNN
F 4 "497-3787-1-ND" H 6600 14100 60  0001 C CNN "Part"
F 5 "DigiKey" H 6600 14100 60  0001 C CNN "Provider"
	1    6600 14100
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 13950 6600 13550
Connection ~ 6600 13550
$Comp
L Device:R R?
U 1 1 5CA944C8
P 7650 14100
AR Path="/58BF599E/5CA944C8" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5CA944C8" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CA944C8" Ref="R16"  Part="1" 
F 0 "R16" V 7730 14100 50  0000 C CNN
F 1 "8.45k" V 7650 14100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7580 14100 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7650 14100 50  0001 C CNN
F 4 "311-8.45KHRCT-ND" V 7650 14100 60  0001 C CNN "Part"
F 5 "DigiKey" V 7650 14100 60  0001 C CNN "Provider"
	1    7650 14100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5CA944D1
P 7650 13550
AR Path="/58BF599E/5CA944D1" Ref="R?"  Part="1" 
AR Path="/58BE27E6/5CA944D1" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CA944D1" Ref="R13"  Part="1" 
F 0 "R13" V 7730 13550 50  0000 C CNN
F 1 "28k" V 7650 13550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7580 13550 50  0001 C CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RT_1-to-0.01_RoHS_L_7.pdf" H 7650 13550 50  0001 C CNN
F 4 "311-28.0KHRCT-ND" V 7650 13550 60  0001 C CNN "Part"
F 5 "DigiKey" V 7650 13550 60  0001 C CNN "Provider"
	1    7650 13550
	-1   0    0    1   
$EndComp
Wire Wire Line
	7650 14550 7650 14250
$Comp
L power:GND #PWR032
U 1 1 5CA944D9
P 7650 14550
F 0 "#PWR032" H 7650 14300 50  0001 C CNN
F 1 "GND" H 7650 14400 50  0000 C CNN
F 2 "" H 7650 14550 50  0001 C CNN
F 3 "" H 7650 14550 50  0001 C CNN
	1    7650 14550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5CA944DF
P 6600 14550
F 0 "#PWR027" H 6600 14300 50  0001 C CNN
F 1 "GND" H 6600 14400 50  0000 C CNN
F 2 "" H 6600 14550 50  0001 C CNN
F 3 "" H 6600 14550 50  0001 C CNN
	1    6600 14550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 14550 6600 14250
$Comp
L Device:CP C?
U 1 1 5CA944E8
P 7150 14100
AR Path="/58BE27E6/5CA944E8" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA944E8" Ref="C35"  Part="1" 
F 0 "C35" H 7175 14200 50  0000 L CNN
F 1 "100uF" H 7175 14000 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 7188 13950 50  0001 C CNN
F 3 "http://katalog.we-online.de/pbs/datasheet/865080545012.pdf" H 7150 14100 50  0001 C CNN
F 4 "732-8511-1-ND" H 7150 14100 60  0001 C CNN "Part"
F 5 "DigiKey" H 7150 14100 60  0001 C CNN "Provider"
	1    7150 14100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5CA944EF
P 7150 14550
F 0 "#PWR031" H 7150 14300 50  0001 C CNN
F 1 "GND" H 7150 14400 50  0000 C CNN
F 2 "" H 7150 14550 50  0001 C CNN
F 3 "" H 7150 14550 50  0001 C CNN
	1    7150 14550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 14550 7150 14250
$Comp
L Device:C C?
U 1 1 5CA944F8
P 6100 13350
AR Path="/58BE2779/5CA944F8" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA944F8" Ref="C34"  Part="1" 
F 0 "C34" V 6150 13200 50  0000 L CNN
F 1 "100nF" V 6050 13050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6138 13200 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 6100 13350 50  0001 C CNN
F 4 "490-4779-2-ND" H 6100 13350 60  0001 C CNN "Part"
F 5 "DigiKey" H 6100 13350 60  0001 C CNN "Provider"
	1    6100 13350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5550 13350 5950 13350
Wire Wire Line
	6250 13350 6600 13350
Wire Wire Line
	6600 13350 6600 13550
Wire Wire Line
	6600 13350 6750 13350
Connection ~ 6600 13350
Wire Wire Line
	7050 13350 7150 13350
Wire Wire Line
	7150 13350 7150 13950
Wire Wire Line
	7150 13350 7650 13350
Wire Wire Line
	7650 13350 7650 13400
Connection ~ 7150 13350
Wire Wire Line
	7650 13700 7650 13750
Wire Wire Line
	5550 13750 7650 13750
Connection ~ 7650 13750
Wire Wire Line
	7650 13750 7650 13950
Wire Wire Line
	7150 13350 7150 13250
$Comp
L Device:C C?
U 1 1 5CA94516
P 3250 10250
AR Path="/58BE27E6/5CA94516" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94516" Ref="C20"  Part="1" 
F 0 "C20" H 3250 10350 50  0000 L CNN
F 1 "1uF 16V" V 3100 10000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3288 10100 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3250 10250 50  0001 C CNN
F 4 "732-7971-1-ND" H 3250 10250 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 10250 60  0001 C CNN "Provider"
	1    3250 10250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CA94520
P 2350 10250
AR Path="/58BE27E6/5CA94520" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94520" Ref="C1"  Part="1" 
F 0 "C1" H 2350 10350 50  0000 L CNN
F 1 "4.7uF 50V" V 2200 10000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2388 10100 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 2350 10250 50  0001 C CNN
F 4 "490-10751-2-ND" H 2350 10250 60  0001 C CNN "Part"
F 5 "DigiKey" H 2350 10250 60  0001 C CNN "Provider"
F 6 "4.7uF 25V" H 2350 10250 50  0001 C CNN "Value1"
	1    2350 10250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CA94529
P 2800 10250
AR Path="/58BE2779/5CA94529" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94529" Ref="C9"  Part="1" 
F 0 "C9" H 2800 10350 50  0000 L CNN
F 1 "100nF" V 2650 10150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2838 10100 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2800 10250 50  0001 C CNN
F 4 "490-4779-2-ND" H 2800 10250 60  0001 C CNN "Part"
F 5 "DigiKey" H 2800 10250 60  0001 C CNN "Provider"
	1    2800 10250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 10050 3250 10050
Wire Wire Line
	3250 10050 3250 10100
Wire Wire Line
	3250 10400 3250 10450
Wire Wire Line
	3250 10450 3950 10450
Wire Wire Line
	2350 10100 2350 10050
Wire Wire Line
	2350 10050 2800 10050
Connection ~ 3250 10050
Wire Wire Line
	2800 10100 2800 10050
Connection ~ 2800 10050
Wire Wire Line
	2800 10050 3250 10050
Wire Wire Line
	2350 10400 2350 10450
Wire Wire Line
	2350 10450 2800 10450
Wire Wire Line
	2800 10450 2800 10400
$Comp
L Device:C C?
U 1 1 5CA94540
P 3350 14050
AR Path="/58BE27E6/5CA94540" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94540" Ref="C23"  Part="1" 
F 0 "C23" H 3350 14150 50  0000 L CNN
F 1 "4.7uF 50V" V 3200 13800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3388 13900 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21A475KACLRNC.jsp" H 3350 14050 50  0001 C CNN
F 4 "490-10751-2-ND" H 3350 14050 60  0001 C CNN "Part"
F 5 "DigiKey" H 3350 14050 60  0001 C CNN "Provider"
F 6 "4.7uF 25V" H 3350 14050 50  0001 C CNN "Value1"
	1    3350 14050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 13750 3800 13750
Wire Wire Line
	3350 13750 3350 13900
$Comp
L power:GND #PWR018
U 1 1 5CA94549
P 3350 14300
F 0 "#PWR018" H 3350 14050 50  0001 C CNN
F 1 "GND" H 3350 14150 50  0000 C CNN
F 2 "" H 3350 14300 50  0001 C CNN
F 3 "" H 3350 14300 50  0001 C CNN
	1    3350 14300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 14200 3350 14300
Wire Wire Line
	2350 10550 2350 10450
Connection ~ 2350 10450
$Comp
L power:GND #PWR023
U 1 1 5CA94552
P 6250 14550
F 0 "#PWR023" H 6250 14300 50  0001 C CNN
F 1 "GND" H 6250 14400 50  0000 C CNN
F 2 "" H 6250 14550 50  0001 C CNN
F 3 "" H 6250 14550 50  0001 C CNN
	1    6250 14550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 13950 6250 13950
Wire Wire Line
	6250 13950 6250 14050
Wire Wire Line
	5550 14050 6250 14050
Connection ~ 6250 14050
Wire Wire Line
	6250 14050 6250 14150
Wire Wire Line
	5550 14150 6250 14150
Connection ~ 6250 14150
Wire Wire Line
	6250 14150 6250 14250
Wire Wire Line
	5550 14250 6250 14250
Connection ~ 6250 14250
Wire Wire Line
	5550 14350 6000 14350
Wire Wire Line
	6000 14350 6000 14450
$Comp
L sigmadrone:VIN #PWR?
U 1 1 5CA94564
P 2350 9950
AR Path="/58BE27E6/5CA94564" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5CA94564" Ref="#PWR011"  Part="1" 
F 0 "#PWR011" H 2350 9800 50  0001 C CNN
F 1 "VIN" H 2365 10123 50  0000 C CNN
F 2 "" H 2350 9950 50  0000 C CNN
F 3 "" H 2350 9950 50  0000 C CNN
	1    2350 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 9950 2350 10050
Connection ~ 2350 10050
Connection ~ 3350 13750
Wire Wire Line
	3350 13650 3350 13750
$Comp
L sigmadrone:DRV8323RS U1
U 1 1 5CA94576
P 4750 12250
F 0 "U1" H 4750 14600 60  0000 C CNN
F 1 "DRV8323RS" V 4750 12250 60  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.45x5.45mm_ThermalVias" H 4750 11900 60  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/drv8323r.pdf" H 4750 11900 60  0001 C CNN
F 4 "296-47769-1-ND" H 4750 12250 50  0001 C CNN "Part"
F 5 "DigiKey" H 4750 12250 50  0001 C CNN "Provider"
	1    4750 12250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 13950 3800 13950
Wire Wire Line
	3800 13950 3800 13750
Connection ~ 3800 13750
Wire Wire Line
	3800 13750 3350 13750
$Comp
L Device:C C?
U 1 1 5CA94583
P 3250 11050
AR Path="/58BE27E6/5CA94583" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94583" Ref="C21"  Part="1" 
F 0 "C21" H 3250 11150 50  0000 L CNN
F 1 "0.047uF 100V" V 3100 10800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3288 10900 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3250 11050 50  0001 C CNN
F 4 "490-4969-1-ND" H 3250 11050 60  0001 C CNN "Part"
F 5 "DigiKey" H 3250 11050 60  0001 C CNN "Provider"
	1    3250 11050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 10850 3250 10850
Wire Wire Line
	3250 10850 3250 10900
Wire Wire Line
	3250 11200 3250 11250
Wire Wire Line
	3250 11250 3950 11250
$Comp
L Device:C C?
U 1 1 5CA94590
P 3500 11450
AR Path="/58BE27E6/5CA94590" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA94590" Ref="C33"  Part="1" 
F 0 "C33" V 3450 11500 50  0000 L CNN
F 1 "1uF 16V" V 3550 11500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3538 11300 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_soft_en.pdf" H 3500 11450 50  0001 C CNN
F 4 "732-7971-1-ND" H 3500 11450 60  0001 C CNN "Part"
F 5 "DigiKey" H 3500 11450 60  0001 C CNN "Provider"
	1    3500 11450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5CA94597
P 3100 11450
F 0 "#PWR015" H 3100 11200 50  0001 C CNN
F 1 "GND" H 3100 11300 50  0000 C CNN
F 2 "" H 3100 11450 50  0001 C CNN
F 3 "" H 3100 11450 50  0001 C CNN
	1    3100 11450
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 11450 3100 11450
Wire Wire Line
	3650 11450 3950 11450
Text HLabel 3600 11650 0    60   Input ~ 0
GATE_ENABLE
Wire Wire Line
	3600 11650 3950 11650
Text HLabel 1450 12550 0    60   Input ~ 0
DRV_SDI
Text HLabel 1450 12450 0    60   Output ~ 0
DRV_SDO
Text HLabel 1450 12650 0    60   Input ~ 0
DRV_CLK
Wire Wire Line
	3600 12150 3950 12150
Wire Wire Line
	3600 12250 3950 12250
Text HLabel 1450 12750 0    60   Input ~ 0
DRV2_CS
Text HLabel 3600 12150 0    60   Input ~ 0
PWM_RH
Text HLabel 3600 12250 0    60   Input ~ 0
PWM_RL
Wire Wire Line
	1450 12650 3950 12650
Wire Wire Line
	1450 12750 3950 12750
$Comp
L power:+3.3V #PWR013
U 1 1 5CA945C9
P 2550 11950
F 0 "#PWR013" H 2550 11800 50  0001 C CNN
F 1 "+3.3V" H 2565 12123 50  0000 C CNN
F 2 "" H 2550 11950 50  0001 C CNN
F 3 "" H 2550 11950 50  0001 C CNN
	1    2550 11950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 11950 2550 13050
$Comp
L Device:C C?
U 1 1 5CA945D2
P 2550 13700
AR Path="/58BE2779/5CA945D2" Ref="C?"  Part="1" 
AR Path="/58BF664D/5CA945D2" Ref="C2"  Part="1" 
F 0 "C2" H 2550 13800 50  0000 L CNN
F 1 "100nF" V 2400 13600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2588 13550 50  0001 C CNN
F 3 "http://psearch.en.murata.com/capacitor/product/GCM188R71H104KA57%23.pdf" H 2550 13700 50  0001 C CNN
F 4 "490-4779-2-ND" H 2550 13700 60  0001 C CNN "Part"
F 5 "DigiKey" H 2550 13700 60  0001 C CNN "Provider"
	1    2550 13700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5CA945D9
P 2550 13950
F 0 "#PWR014" H 2550 13700 50  0001 C CNN
F 1 "GND" H 2550 13800 50  0000 C CNN
F 2 "" H 2550 13950 50  0001 C CNN
F 3 "" H 2550 13950 50  0001 C CNN
	1    2550 13950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 13550 2550 13050
Wire Wire Line
	2550 13850 2550 13950
Connection ~ 2550 13050
Wire Wire Line
	2550 13050 3950 13050
Text Label 6300 10050 2    60   ~ 0
VDRAINR
Wire Wire Line
	5550 10050 6300 10050
Text Label 6300 11100 2    60   ~ 0
GHR
Wire Wire Line
	5550 11100 6300 11100
Text Label 6300 11300 2    60   ~ 0
GLR
Wire Wire Line
	5550 11300 6300 11300
Text Label 6300 11200 2    60   ~ 0
SHR
Wire Wire Line
	5550 11200 6300 11200
Wire Wire Line
	6250 14250 6250 14550
Wire Wire Line
	5550 14450 6000 14450
Connection ~ 6000 14450
Wire Wire Line
	6000 14450 6000 14550
$Comp
L sigmadrone:PGND #PWR021
U 1 1 5CA94623
P 6000 14550
F 0 "#PWR021" H 6000 14300 50  0001 C CNN
F 1 "PGND" H 6000 14400 50  0000 C CNN
F 2 "" H 6000 14550 50  0001 C CNN
F 3 "" H 6000 14550 50  0001 C CNN
	1    6000 14550
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR012
U 1 1 5CA94629
P 2350 10550
F 0 "#PWR012" H 2350 10300 50  0001 C CNN
F 1 "PGND" H 2355 10377 50  0000 C CNN
F 2 "" H 2350 10550 50  0001 C CNN
F 3 "" H 2350 10550 50  0001 C CNN
	1    2350 10550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR029
U 1 1 5CAAD5F2
P 7150 13250
F 0 "#PWR029" H 7150 13100 50  0001 C CNN
F 1 "+3.3V" H 7165 13423 50  0000 C CNN
F 2 "" H 7150 13250 50  0001 C CNN
F 3 "" H 7150 13250 50  0001 C CNN
	1    7150 13250
	1    0    0    -1  
$EndComp
NoConn ~ 3950 13150
NoConn ~ 3950 13250
NoConn ~ 3950 13350
NoConn ~ 5550 10650
NoConn ~ 5550 10750
NoConn ~ 5550 10850
NoConn ~ 5550 11750
NoConn ~ 5550 12150
NoConn ~ 5550 12250
NoConn ~ 5550 12650
NoConn ~ 5550 12750
NoConn ~ 5550 10250
NoConn ~ 5550 10350
NoConn ~ 5550 10450
Text Label 8700 10050 0    60   ~ 0
VDRAINR
Wire Wire Line
	9750 10050 8700 10050
Connection ~ 9750 10050
Wire Wire Line
	9750 10050 9750 10250
$Comp
L sigmadrone:PGND #PWR033
U 1 1 5CCF6DFB
P 9750 12850
F 0 "#PWR033" H 9750 12600 50  0001 C CNN
F 1 "PGND" H 9750 12700 50  0000 C CNN
F 2 "" H 9750 12850 50  0001 C CNN
F 3 "" H 9750 12850 50  0001 C CNN
	1    9750 12850
	1    0    0    -1  
$EndComp
$Comp
L sigmadrone:PGND #PWR034
U 1 1 5CCF6FDE
P 11300 12850
F 0 "#PWR034" H 11300 12600 50  0001 C CNN
F 1 "PGND" H 11300 12700 50  0000 C CNN
F 2 "" H 11300 12850 50  0001 C CNN
F 3 "" H 11300 12850 50  0001 C CNN
	1    11300 12850
	1    0    0    -1  
$EndComp
NoConn ~ 5550 13150
NoConn ~ 3950 11750
NoConn ~ 3950 11850
NoConn ~ 3950 11950
NoConn ~ 3950 12050
Wire Wire Line
	2200 4150 3950 4150
Wire Wire Line
	1450 4150 2200 4150
Connection ~ 2200 4150
Wire Wire Line
	2200 3550 2200 4150
Wire Wire Line
	2200 3150 2200 3250
$Comp
L power:+3.3V #PWR025
U 1 1 5CDC3DC9
P 2200 3150
F 0 "#PWR025" H 2200 3000 50  0001 C CNN
F 1 "+3.3V" H 2215 3323 50  0000 C CNN
F 2 "" H 2200 3150 50  0001 C CNN
F 3 "" H 2200 3150 50  0001 C CNN
	1    2200 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CD77F2C
P 2200 3400
AR Path="/58BE2779/5CD77F2C" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CD77F2C" Ref="R7"  Part="1" 
F 0 "R7" V 2280 3400 50  0000 C CNN
F 1 "10k" V 2200 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2130 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 2200 3400 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 2200 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 2200 3400 60  0001 C CNN "Provider"
	1    2200 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 3550 1850 3650
Wire Wire Line
	1850 3150 1850 3250
$Comp
L power:+3.3V #PWR020
U 1 1 5CF217A6
P 1850 3150
F 0 "#PWR020" H 1850 3000 50  0001 C CNN
F 1 "+3.3V" H 1865 3323 50  0000 C CNN
F 2 "" H 1850 3150 50  0001 C CNN
F 3 "" H 1850 3150 50  0001 C CNN
	1    1850 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CF2179F
P 1850 3400
AR Path="/58BE2779/5CF2179F" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CF2179F" Ref="R6"  Part="1" 
F 0 "R6" V 1930 3400 50  0000 C CNN
F 1 "10k" V 1850 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1780 3400 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 1850 3400 50  0001 C CNN
F 4 "311-10.0KHRCT-ND" V 1850 3400 60  0001 C CNN "Part"
F 5 "DigiKey" V 1850 3400 60  0001 C CNN "Provider"
	1    1850 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 12450 3500 12450
NoConn ~ 3950 12950
$Comp
L power:+5V #PWR?
U 1 1 5C9E9AA2
P 3350 13650
AR Path="/58BE27E6/5C9E9AA2" Ref="#PWR?"  Part="1" 
AR Path="/58BF664D/5C9E9AA2" Ref="#PWR0133"  Part="1" 
F 0 "#PWR0133" H 3350 13500 50  0001 C CNN
F 1 "+5V" H 3365 13823 50  0000 C CNN
F 2 "" H 3350 13650 50  0001 C CNN
F 3 "" H 3350 13650 50  0001 C CNN
	1    3350 13650
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5CA0007B
P 5250 7600
AR Path="/58BE27E6/5CA0007B" Ref="#FLG?"  Part="1" 
AR Path="/58BF664D/5CA0007B" Ref="#FLG0102"  Part="1" 
F 0 "#FLG0102" H 5250 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 5250 7773 50  0000 C CNN
F 2 "" H 5250 7600 50  0001 C CNN
F 3 "" H 5250 7600 50  0001 C CNN
	1    5250 7600
	-1   0    0    1   
$EndComp
Wire Wire Line
	5250 7100 5250 7600
$Comp
L power:+3.3V #PWR0134
U 1 1 5CA14148
P 5250 7100
F 0 "#PWR0134" H 5250 6950 50  0001 C CNN
F 1 "+3.3V" H 5265 7273 50  0000 C CNN
F 2 "" H 5250 7100 50  0001 C CNN
F 3 "" H 5250 7100 50  0001 C CNN
	1    5250 7100
	1    0    0    -1  
$EndComp
NoConn ~ 3950 12850
$Comp
L sigmadrone:PGND #PWR0135
U 1 1 5CA2A7BF
P 7450 7100
F 0 "#PWR0135" H 7450 6850 50  0001 C CNN
F 1 "PGND" H 7450 6950 50  0000 C CNN
F 2 "" H 7450 7100 50  0001 C CNN
F 3 "" H 7450 7100 50  0001 C CNN
	1    7450 7100
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5CA3E7EA
P 7450 7600
AR Path="/58BE27E6/5CA3E7EA" Ref="#FLG?"  Part="1" 
AR Path="/58BF664D/5CA3E7EA" Ref="#FLG0103"  Part="1" 
F 0 "#FLG0103" H 7450 7675 50  0001 C CNN
F 1 "PWR_FLAG" H 7450 7773 50  0000 C CNN
F 2 "" H 7450 7600 50  0001 C CNN
F 3 "" H 7450 7600 50  0001 C CNN
	1    7450 7600
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 7100 7450 7400
$Comp
L Device:R R?
U 1 1 5CD745A7
P 3650 3650
AR Path="/58BE2779/5CD745A7" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CD745A7" Ref="R11"  Part="1" 
F 0 "R11" V 3700 3800 50  0000 C CNN
F 1 "21" V 3650 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3580 3650 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 3650 3650 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 3650 3650 60  0001 C CNN "Part"
F 5 "DigiKey" V 3650 3650 60  0001 C CNN "Provider"
	1    3650 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3950 3650 3800 3650
Wire Wire Line
	1450 3650 1850 3650
Connection ~ 1850 3650
Wire Wire Line
	1850 3650 3500 3650
$Comp
L Device:R R?
U 1 1 5CDB071A
P 3650 12450
AR Path="/58BE2779/5CDB071A" Ref="R?"  Part="1" 
AR Path="/58BF664D/5CDB071A" Ref="R12"  Part="1" 
F 0 "R12" V 3700 12600 50  0000 C CNN
F 1 "21" V 3650 12450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3580 12450 50  0001 C CNN
F 3 "http://www.yageo.com.tw/exep/pages/download/literatures/PYu-R_INT-thick_7.pdf" H 3650 12450 50  0001 C CNN
F 4 "311-21.0HRCT-ND" V 3650 12450 60  0001 C CNN "Part"
F 5 "DigiKey" V 3650 12450 60  0001 C CNN "Provider"
	1    3650 12450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 12450 3950 12450
$Comp
L Device:D_Shockley D?
U 1 1 5D088870
P 8150 4550
AR Path="/5D07002B/5D088870" Ref="D?"  Part="1" 
AR Path="/58BF664D/5D088870" Ref="D5"  Part="1" 
F 0 "D5" H 8150 4650 50  0000 C CNN
F 1 "BAT30KFILM" H 8150 4450 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-523" H 8150 4550 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b5/44/50/dc/a4/84/48/70/CD00126118.pdf/files/CD00126118.pdf/jcr:content/translations/en.CD00126118.pdf" H 8150 4550 50  0001 C CNN
F 4 "497-5552-1-ND" H 8150 4550 60  0001 C CNN "Part"
F 5 "DigiKey" H 8150 4550 60  0001 C CNN "Provider"
	1    8150 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 4550 8000 4550
Connection ~ 7650 4550
Text HLabel 8500 4550 2    60   Output ~ 0
EXT_5V
Wire Wire Line
	8300 4550 8500 4550
$Comp
L Device:Net-Tie_2 NT1
U 1 1 5D4D5931
P 7200 7400
F 0 "NT1" H 7200 7578 50  0000 C CNN
F 1 "Net-Tie_2" H 7200 7487 50  0000 C CNN
F 2 "NetTie:NetTie-2_SMD_Pad0.5mm" H 7200 7400 50  0001 C CNN
F 3 "~" H 7200 7400 50  0001 C CNN
	1    7200 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 7400 6900 7400
Connection ~ 6900 7400
Wire Wire Line
	6900 7400 6900 7600
Wire Wire Line
	7300 7400 7450 7400
Connection ~ 7450 7400
Wire Wire Line
	7450 7400 7450 7600
Wire Wire Line
	1450 12550 3950 12550
$EndSCHEMATC