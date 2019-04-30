EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5CB66320
P 1550 1850
F 0 "A1" H 1550 761 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 1550 670 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 1700 900 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 1550 850 50  0001 C CNN
	1    1550 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2250 2500 2250
Wire Wire Line
	2050 2350 2300 2350
Wire Wire Line
	2300 2350 2300 3300
Wire Wire Line
	2300 3300 4750 3300
Wire Wire Line
	2500 2250 2500 3150
Wire Wire Line
	2500 3150 4950 3150
Wire Wire Line
	4950 2550 4950 3150
Connection ~ 4950 3150
Wire Wire Line
	4950 3150 6850 3150
Wire Wire Line
	4750 2650 4750 3300
Connection ~ 4750 3300
Wire Wire Line
	4750 3300 6600 3300
Wire Wire Line
	6850 2550 6850 3150
Connection ~ 6850 3150
Wire Wire Line
	6600 2650 6600 3300
Connection ~ 6600 3300
Wire Wire Line
	6600 3300 8350 3300
Wire Wire Line
	8500 2550 8500 3150
Wire Wire Line
	6850 3150 8500 3150
Wire Wire Line
	8350 2650 8350 3300
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5CB80ED2
P 3800 2550
F 0 "J1" H 3908 2731 50  0000 C CNN
F 1 "Conn_01x02_Male" H 3908 2640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3800 2550 50  0001 C CNN
F 3 "~" H 3800 2550 50  0001 C CNN
	1    3800 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 5CB8152E
P 6050 2550
F 0 "J2" H 6158 2731 50  0000 C CNN
F 1 "Conn_01x02_Male" H 6158 2640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6050 2550 50  0001 C CNN
F 3 "~" H 6050 2550 50  0001 C CNN
	1    6050 2550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 5CB81E08
P 7550 2550
F 0 "J3" H 7658 2731 50  0000 C CNN
F 1 "Conn_01x02_Male" H 7658 2640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7550 2550 50  0001 C CNN
F 3 "~" H 7550 2550 50  0001 C CNN
	1    7550 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2650 4750 2650
Wire Wire Line
	4000 2550 4950 2550
Wire Wire Line
	6600 2650 6250 2650
Wire Wire Line
	6250 2550 6850 2550
Wire Wire Line
	7750 2550 8500 2550
Wire Wire Line
	7750 2650 8350 2650
$EndSCHEMATC
