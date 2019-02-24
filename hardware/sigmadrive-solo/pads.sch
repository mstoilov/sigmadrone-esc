EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:contrib
LIBS:valves
LIBS:sigmadrone
LIBS:esc-sigma-4s-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L PAD P?
U 1 1 58BE2E20
P 3300 2200
F 0 "P?" H 3350 2300 60  0000 C CNN
F 1 "PAD" H 3300 2200 60  0000 C CNN
F 2 "" H 3300 2200 60  0001 C CNN
F 3 "" H 3300 2200 60  0001 C CNN
	1    3300 2200
	1    0    0    -1  
$EndComp
$Comp
L PAD P?
U 1 1 58BE2F0E
P 3300 2500
F 0 "P?" H 3350 2600 60  0000 C CNN
F 1 "PAD" H 3300 2500 60  0000 C CNN
F 2 "" H 3300 2500 60  0001 C CNN
F 3 "" H 3300 2500 60  0001 C CNN
	1    3300 2500
	1    0    0    -1  
$EndComp
Text HLabel 2200 2200 0    60   UnSpc ~ 0
VBAT
Text HLabel 2200 2500 0    60   UnSpc ~ 0
GND
Wire Wire Line
	2200 2200 3000 2200
Wire Wire Line
	2200 2500 3000 2500
$Comp
L PAD P?
U 1 1 58BE2F6D
P 3300 3400
F 0 "P?" H 3350 3500 60  0000 C CNN
F 1 "PAD" H 3300 3400 60  0000 C CNN
F 2 "" H 3300 3400 60  0001 C CNN
F 3 "" H 3300 3400 60  0001 C CNN
	1    3300 3400
	1    0    0    -1  
$EndComp
$Comp
L PAD P?
U 1 1 58BE2FC0
P 3300 3700
F 0 "P?" H 3350 3800 60  0000 C CNN
F 1 "PAD" H 3300 3700 60  0000 C CNN
F 2 "" H 3300 3700 60  0001 C CNN
F 3 "" H 3300 3700 60  0001 C CNN
	1    3300 3700
	1    0    0    -1  
$EndComp
$Comp
L PAD P?
U 1 1 58BE2FE8
P 3300 4000
F 0 "P?" H 3350 4100 60  0000 C CNN
F 1 "PAD" H 3300 4000 60  0000 C CNN
F 2 "" H 3300 4000 60  0001 C CNN
F 3 "" H 3300 4000 60  0001 C CNN
	1    3300 4000
	1    0    0    -1  
$EndComp
Text HLabel 2200 3400 0    60   UnSpc ~ 0
PHASE_A
Text HLabel 2200 3700 0    60   UnSpc ~ 0
PHASE_B
Text HLabel 2200 4000 0    60   UnSpc ~ 0
PHASE_C
Wire Wire Line
	2200 3400 3000 3400
Wire Wire Line
	2200 3700 3000 3700
Wire Wire Line
	2200 4000 3000 4000
$EndSCHEMATC
