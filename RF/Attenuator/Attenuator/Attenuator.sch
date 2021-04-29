EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:R R1
U 1 1 60087714
P 2850 2300
F 0 "R1" H 2780 2254 50  0000 R CNN
F 1 "100 50W" H 2780 2345 50  0000 R CNN
F 2 "MyPCBLib:TO-220-2_External_Thermal_Mount_Short" V 2780 2300 50  0001 C CNN
F 3 "~" H 2850 2300 50  0001 C CNN
	1    2850 2300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 60087BA3
P 3200 2000
F 0 "R2" V 3407 2000 50  0000 C CNN
F 1 "75 30W" V 3316 2000 50  0000 C CNN
F 2 "MyPCBLib:TO-220-2_External_Thermal_Mount_Short" V 3130 2000 50  0001 C CNN
F 3 "~" H 3200 2000 50  0001 C CNN
	1    3200 2000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 60088407
P 3550 2300
F 0 "R3" H 3620 2346 50  0000 L CNN
F 1 "100 30W" H 3620 2255 50  0000 L CNN
F 2 "MyPCBLib:TO-220-2_External_Thermal_Mount_Short" V 3480 2300 50  0001 C CNN
F 3 "~" H 3550 2300 50  0001 C CNN
	1    3550 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2150 2850 2000
Wire Wire Line
	2850 2000 3050 2000
Wire Wire Line
	3350 2000 3550 2000
Wire Wire Line
	3550 2000 3550 2150
Wire Wire Line
	2850 2450 2850 2700
Wire Wire Line
	3550 2450 3550 2700
Wire Wire Line
	2850 2700 3200 2700
Wire Wire Line
	3200 2700 3200 2850
Connection ~ 3200 2700
Wire Wire Line
	3200 2700 3550 2700
$Comp
L power:GND #PWR03
U 1 1 60088FF3
P 3200 2850
F 0 "#PWR03" H 3200 2600 50  0001 C CNN
F 1 "GND" H 3205 2677 50  0000 C CNN
F 2 "" H 3200 2850 50  0001 C CNN
F 3 "" H 3200 2850 50  0001 C CNN
	1    3200 2850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J3
U 1 1 60089677
P 4400 2000
F 0 "J3" H 4500 1975 50  0000 L CNN
F 1 "RF out" H 4500 1884 50  0000 L CNN
F 2 "MyPCBLib:N-KY_PCB_EDGE_MOUNT_SMA-KWE" H 4400 2000 50  0001 C CNN
F 3 " ~" H 4400 2000 50  0001 C CNN
	1    4400 2000
	1    0    0    -1  
$EndComp
Connection ~ 2850 2000
$Comp
L power:GND #PWR02
U 1 1 6008A32A
P 1900 2450
F 0 "#PWR02" H 1900 2200 50  0001 C CNN
F 1 "GND" H 1905 2277 50  0000 C CNN
F 2 "" H 1900 2450 50  0001 C CNN
F 3 "" H 1900 2450 50  0001 C CNN
	1    1900 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2200 1900 2450
$Comp
L Connector:Conn_Coaxial J4
U 1 1 6008A8E1
P 1900 2000
F 0 "J4" H 1828 2238 50  0000 C CNN
F 1 "RF in" H 1828 2147 50  0000 C CNN
F 2 "MyPCBLib:N-KY_PCB_EDGE_MOUNT_SMA-KWE" H 1900 2000 50  0001 C CNN
F 3 " ~" H 1900 2000 50  0001 C CNN
	1    1900 2000
	-1   0    0    -1  
$EndComp
Connection ~ 3550 2000
$Comp
L power:GND #PWR04
U 1 1 6008B4D1
P 4400 2400
F 0 "#PWR04" H 4400 2150 50  0001 C CNN
F 1 "GND" H 4405 2227 50  0000 C CNN
F 2 "" H 4400 2400 50  0001 C CNN
F 3 "" H 4400 2400 50  0001 C CNN
	1    4400 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2200 4400 2400
Wire Wire Line
	2100 2000 2850 2000
Wire Wire Line
	3550 2000 4200 2000
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 6009762B
P 1600 3150
F 0 "J2" V 1564 3062 50  0000 R CNN
F 1 "Conn_01x01" V 1473 3062 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 1600 3150 50  0001 C CNN
F 3 "~" H 1600 3150 50  0001 C CNN
	1    1600 3150
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 60097AC9
P 1100 3150
F 0 "J1" V 1064 3062 50  0000 R CNN
F 1 "Conn_01x01" V 973 3062 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 1100 3150 50  0001 C CNN
F 3 "~" H 1100 3150 50  0001 C CNN
	1    1100 3150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 60098081
P 1300 3450
F 0 "#PWR01" H 1300 3200 50  0001 C CNN
F 1 "GND" H 1305 3277 50  0000 C CNN
F 2 "" H 1300 3450 50  0001 C CNN
F 3 "" H 1300 3450 50  0001 C CNN
	1    1300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 3350 1100 3450
Wire Wire Line
	1100 3450 1300 3450
Wire Wire Line
	1600 3450 1600 3350
Connection ~ 1300 3450
Wire Wire Line
	1300 3450 1600 3450
$EndSCHEMATC