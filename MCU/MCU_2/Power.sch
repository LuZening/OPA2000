EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
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
L power:GND #PWR0109
U 1 1 5F5D1ADE
P 3950 2300
F 0 "#PWR0109" H 3950 2050 50  0001 C CNN
F 1 "GND" H 3955 2127 50  0000 C CNN
F 2 "" H 3950 2300 50  0001 C CNN
F 3 "" H 3950 2300 50  0001 C CNN
	1    3950 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2000 3950 1950
$Comp
L Device:R_Small R68
U 1 1 5F5D1755
P 3950 2100
F 0 "R68" H 4009 2146 50  0000 L CNN
F 1 "10K" H 4009 2055 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3950 2100 50  0001 C CNN
F 3 "~" H 3950 2100 50  0001 C CNN
	1    3950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 1650 3850 1650
$Comp
L Device:R_Small R67
U 1 1 5F5D10AD
P 3950 1750
F 0 "R67" H 4009 1796 50  0000 L CNN
F 1 "68K" H 4009 1705 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3950 1750 50  0001 C CNN
F 3 "~" H 3950 1750 50  0001 C CNN
	1    3950 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:L_Core_Ferrite L5
U 1 1 5F5CF7DB
P 3700 1650
F 0 "L5" V 3925 1650 50  0000 C CNN
F 1 "10uH" V 3834 1650 50  0000 C CNN
F 2 "MyPCBLib:L_SMD4030_hand" H 3700 1650 50  0001 C CNN
F 3 "~" H 3700 1650 50  0001 C CNN
	1    3700 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 1650 3450 1650
Wire Wire Line
	3450 1650 3300 1650
Connection ~ 3450 1650
Wire Wire Line
	3450 1300 3450 1650
Wire Wire Line
	3350 1300 3450 1300
Wire Wire Line
	2850 1300 2850 1350
Wire Wire Line
	3150 1300 2850 1300
$Comp
L Device:C_Small C43
U 1 1 5F5CD6B3
P 3250 1300
F 0 "C43" V 3100 1250 50  0000 L CNN
F 1 "100nF" V 3400 1100 50  0000 L CNN
F 2 "lc_lib:0805_C" H 3250 1300 50  0001 C CNN
F 3 "~" H 3250 1300 50  0001 C CNN
	1    3250 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 2050 1900 2000
Wire Wire Line
	2100 2050 1900 2050
Wire Wire Line
	2100 2000 2100 2050
Wire Wire Line
	2100 1650 2350 1650
Wire Wire Line
	2100 1800 2100 1650
$Comp
L Device:CP_Small C42
U 1 1 5F5CC649
P 2100 1900
F 0 "C42" H 2188 1946 50  0000 L CNN
F 1 "10u" H 2188 1855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2100 1900 50  0001 C CNN
F 3 "~" H 2100 1900 50  0001 C CNN
	1    2100 1900
	1    0    0    -1  
$EndComp
Connection ~ 1900 2050
Wire Wire Line
	1900 2100 1900 2050
$Comp
L power:GND #PWR0104
U 1 1 5F5CC212
P 1900 2100
F 0 "#PWR0104" H 1900 1850 50  0001 C CNN
F 1 "GND" H 1905 1927 50  0000 C CNN
F 2 "" H 1900 2100 50  0001 C CNN
F 3 "" H 1900 2100 50  0001 C CNN
	1    1900 2100
	1    0    0    -1  
$EndComp
Connection ~ 2100 1650
Wire Wire Line
	1900 1650 2100 1650
Wire Wire Line
	1900 1800 1900 1650
$Comp
L Device:C_Small C41
U 1 1 5F5CB959
P 1900 1900
F 0 "C41" H 1700 1900 50  0000 L CNN
F 1 "104" H 1650 1800 50  0000 L CNN
F 2 "lc_lib:0805_C" H 1900 1900 50  0001 C CNN
F 3 "~" H 1900 1900 50  0001 C CNN
	1    1900 1900
	1    0    0    -1  
$EndComp
Connection ~ 1900 1650
Wire Wire Line
	3700 1950 3950 1950
Connection ~ 3950 1950
Wire Wire Line
	3950 1950 3950 1850
Wire Wire Line
	3950 2200 3950 2300
Wire Wire Line
	2350 1700 2350 1650
Connection ~ 2350 1650
Wire Wire Line
	2350 1650 2400 1650
$Comp
L power:GND #PWR0106
U 1 1 5F5D722E
P 2850 2400
F 0 "#PWR0106" H 2850 2150 50  0001 C CNN
F 1 "GND" H 2855 2227 50  0000 C CNN
F 2 "" H 2850 2400 50  0001 C CNN
F 3 "" H 2850 2400 50  0001 C CNN
	1    2850 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C44
U 1 1 5F5D7A88
P 4400 1800
F 0 "C44" H 4300 1600 50  0000 L CNN
F 1 "47pF" H 4200 1700 50  0000 L CNN
F 2 "lc_lib:0805_C" H 4400 1800 50  0001 C CNN
F 3 "~" H 4400 1800 50  0001 C CNN
	1    4400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1650 3950 1650
Connection ~ 3950 1650
Wire Wire Line
	4400 1950 3950 1950
Wire Wire Line
	4400 1900 4400 1950
Wire Wire Line
	4400 1650 4400 1700
Wire Wire Line
	4700 1900 4700 1650
Wire Wire Line
	4700 1650 4550 1650
Connection ~ 4400 1650
$Comp
L power:GND #PWR0113
U 1 1 5F5DB7B4
P 4700 2250
F 0 "#PWR0113" H 4700 2000 50  0001 C CNN
F 1 "GND" H 4705 2077 50  0000 C CNN
F 2 "" H 4700 2250 50  0001 C CNN
F 3 "" H 4700 2250 50  0001 C CNN
	1    4700 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2250 4700 2200
$Comp
L Device:C_Small C47
U 1 1 5F5DD8A8
P 5000 2000
F 0 "C47" H 5100 2050 50  0000 L CNN
F 1 "104" H 5100 1900 50  0000 L CNN
F 2 "lc_lib:0805_C" H 5000 2000 50  0001 C CNN
F 3 "~" H 5000 2000 50  0001 C CNN
	1    5000 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1900 5000 1650
Wire Wire Line
	5000 1650 4700 1650
Connection ~ 4700 1650
Wire Wire Line
	5000 2100 5000 2200
Wire Wire Line
	5000 2200 4700 2200
Connection ~ 4700 2200
Wire Wire Line
	4700 2200 4700 2100
$Comp
L Device:R_Small R71
U 1 1 5F5E1BB4
P 5150 1650
F 0 "R71" V 4954 1650 50  0000 C CNN
F 1 "0" V 5045 1650 50  0000 C CNN
F 2 "lc_lib:1206_R" H 5150 1650 50  0001 C CNN
F 3 "~" H 5150 1650 50  0001 C CNN
	1    5150 1650
	0    1    1    0   
$EndComp
Connection ~ 5000 1650
$Comp
L Regulator_Linear:AZ1117-3.3 U13
U 1 1 5F5E7C5B
P 5750 1650
F 0 "U13" H 5750 1892 50  0000 C CNN
F 1 "AZ1117-3.3" H 5750 1801 50  0000 C CNN
F 2 "lc_lib:SOT-223" H 5750 1900 50  0001 C CIN
F 3 "https://www.diodes.com/assets/Datasheets/AZ1117.pdf" H 5750 1650 50  0001 C CNN
	1    5750 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5F5E8CBE
P 5750 2150
F 0 "#PWR0116" H 5750 1900 50  0001 C CNN
F 1 "GND" H 5755 1977 50  0000 C CNN
F 2 "" H 5750 2150 50  0001 C CNN
F 3 "" H 5750 2150 50  0001 C CNN
	1    5750 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1950 5750 2000
$Comp
L Device:C_Small C49
U 1 1 5F5E9A99
P 6250 1900
F 0 "C49" H 6350 1950 50  0000 L CNN
F 1 "10uF" H 6350 1800 50  0000 L CNN
F 2 "lc_lib:0805_C" H 6250 1900 50  0001 C CNN
F 3 "~" H 6250 1900 50  0001 C CNN
	1    6250 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1800 6250 1650
Wire Wire Line
	6250 1650 6050 1650
Wire Wire Line
	6250 2000 5750 2000
Connection ~ 5750 2000
Wire Wire Line
	5750 2000 5750 2150
Connection ~ 6250 1650
$Comp
L Regulator_Linear:AZ1117-3.3 U14
U 1 1 5F5F2364
P 5750 2800
F 0 "U14" H 5750 3042 50  0000 C CNN
F 1 "AZ1117-3.3" H 5750 2951 50  0000 C CNN
F 2 "lc_lib:SOT-223" H 5750 3050 50  0001 C CIN
F 3 "https://www.diodes.com/assets/Datasheets/AZ1117.pdf" H 5750 2800 50  0001 C CNN
	1    5750 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2800 5300 2800
$Comp
L power:GND #PWR0117
U 1 1 5F5F236B
P 5750 3300
F 0 "#PWR0117" H 5750 3050 50  0001 C CNN
F 1 "GND" H 5755 3127 50  0000 C CNN
F 2 "" H 5750 3300 50  0001 C CNN
F 3 "" H 5750 3300 50  0001 C CNN
	1    5750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3100 5750 3150
$Comp
L Device:C_Small C50
U 1 1 5F5F2372
P 6250 3050
F 0 "C50" H 6350 3100 50  0000 L CNN
F 1 "10uF" H 6350 2950 50  0000 L CNN
F 2 "lc_lib:0805_C" H 6250 3050 50  0001 C CNN
F 3 "~" H 6250 3050 50  0001 C CNN
	1    6250 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2950 6250 2800
Wire Wire Line
	6250 2800 6050 2800
Wire Wire Line
	6250 3150 5750 3150
Connection ~ 5750 3150
Wire Wire Line
	5750 3150 5750 3300
Connection ~ 6250 2800
Wire Wire Line
	1200 1650 1400 1650
$Comp
L MySchLib:D_Zener D8
U 1 1 5E220FED
P 1600 2100
F 0 "D8" V 1554 2179 50  0000 L CNN
F 1 "16V" V 1645 2179 50  0000 L CNN
F 2 "lc_lib:LL-34" H 1600 2100 50  0001 C CNN
F 3 "~" H 1600 2100 50  0001 C CNN
	1    1600 2100
	0    1    1    0   
$EndComp
$Comp
L Device:Fuse F1
U 1 1 5E222A3A
P 1200 1950
F 0 "F1" V 1300 1950 50  0000 C CNN
F 1 "500mA" V 1094 1950 50  0000 C CNN
F 2 "lc_lib:1206_R" V 1130 1950 50  0001 C CNN
F 3 "~" H 1200 1950 50  0001 C CNN
	1    1200 1950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5E2248FE
P 1600 2300
F 0 "#PWR0103" H 1600 2050 50  0001 C CNN
F 1 "GND" H 1605 2127 50  0000 C CNN
F 2 "" H 1600 2300 50  0001 C CNN
F 3 "" H 1600 2300 50  0001 C CNN
	1    1600 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2250 1600 2300
$Comp
L Device:C_Small C45
U 1 1 5E2A0E61
P 4700 2000
F 0 "C45" H 4800 2050 50  0000 L CNN
F 1 "10uF" H 4800 1900 50  0000 L CNN
F 2 "lc_lib:0805_C" H 4700 2000 50  0001 C CNN
F 3 "~" H 4700 2000 50  0001 C CNN
	1    4700 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2150 1200 2100
Wire Wire Line
	1200 1800 1200 1750
$Comp
L Device:L_Core_Ferrite L4
U 1 1 5E2A38D9
P 1550 1650
F 0 "L4" V 1775 1650 50  0000 C CNN
F 1 "BEAD" V 1684 1650 50  0000 C CNN
F 2 "lc_lib:1206_L" H 1550 1650 50  0001 C CNN
F 3 "~" H 1550 1650 50  0001 C CNN
	1    1550 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 1650 5050 1650
Connection ~ 5300 1650
Wire Wire Line
	5300 1650 5450 1650
$Comp
L Connector_Generic:Conn_01x01 SC1
U 1 1 5E28AF84
P 1300 5100
F 0 "SC1" V 1264 5012 50  0000 R CNN
F 1 "GND" V 1173 5012 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 1300 5100 50  0001 C CNN
F 3 "~" H 1300 5100 50  0001 C CNN
	1    1300 5100
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 SC2
U 1 1 5E28B48A
P 1750 5100
F 0 "SC2" V 1714 5012 50  0000 R CNN
F 1 "GND" V 1623 5012 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 1750 5100 50  0001 C CNN
F 3 "~" H 1750 5100 50  0001 C CNN
	1    1750 5100
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 SC3
U 1 1 5E28B555
P 2150 5100
F 0 "SC3" V 2114 5012 50  0000 R CNN
F 1 "GND" V 2023 5012 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 2150 5100 50  0001 C CNN
F 3 "~" H 2150 5100 50  0001 C CNN
	1    2150 5100
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 SC4
U 1 1 5E28B73A
P 2500 5100
F 0 "SC4" V 2464 5012 50  0000 R CNN
F 1 "GND" V 2373 5012 50  0000 R CNN
F 2 "MyPCBLib:M3_Screwhole" H 2500 5100 50  0001 C CNN
F 3 "~" H 2500 5100 50  0001 C CNN
	1    2500 5100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 5300 1750 5300
Connection ~ 1750 5300
Wire Wire Line
	1750 5300 1950 5300
Connection ~ 2150 5300
Wire Wire Line
	2150 5300 2500 5300
$Comp
L power:GND #PWR0105
U 1 1 5E28F1FE
P 1950 5450
F 0 "#PWR0105" H 1950 5200 50  0001 C CNN
F 1 "GND" H 1955 5277 50  0000 C CNN
F 2 "" H 1950 5450 50  0001 C CNN
F 3 "" H 1950 5450 50  0001 C CNN
	1    1950 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5450 1950 5300
Connection ~ 1950 5300
Wire Wire Line
	1950 5300 2150 5300
$Comp
L Motor:Fan M1
U 1 1 5E2EA8D8
P 4450 4150
F 0 "M1" H 4608 4246 50  0000 L CNN
F 1 "Fan" H 4608 4155 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B2B-PH-SM4-TB_1x02-1MP_P2.00mm_Vertical" H 4450 4160 50  0001 C CNN
F 3 "~" H 4450 4160 50  0001 C CNN
	1    4450 4150
	1    0    0    -1  
$EndComp
$Comp
L Motor:Fan M2
U 1 1 5E2EAEA8
P 6300 4150
F 0 "M2" H 6458 4246 50  0000 L CNN
F 1 "Fan" H 6458 4155 50  0000 L CNN
F 2 "Connector_JST:JST_PH_B2B-PH-SM4-TB_1x02-1MP_P2.00mm_Vertical" H 6300 4160 50  0001 C CNN
F 3 "~" H 6300 4160 50  0001 C CNN
	1    6300 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q10
U 1 1 5E2EC8A5
P 4350 4700
F 0 "Q10" H 4556 4746 50  0000 L CNN
F 1 "AO3402" H 4556 4655 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 4550 4800 50  0001 C CNN
F 3 "~" H 4350 4700 50  0001 C CNN
	1    4350 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q11
U 1 1 5E2ED19E
P 6200 4700
F 0 "Q11" H 6406 4746 50  0000 L CNN
F 1 "AO3402" H 6406 4655 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 6400 4800 50  0001 C CNN
F 3 "~" H 6200 4700 50  0001 C CNN
	1    6200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4500 4450 4450
Wire Wire Line
	6300 4350 6300 4450
$Comp
L power:GND #PWR0111
U 1 1 5E2F8803
P 4450 5200
F 0 "#PWR0111" H 4450 4950 50  0001 C CNN
F 1 "GND" H 4455 5027 50  0000 C CNN
F 2 "" H 4450 5200 50  0001 C CNN
F 3 "" H 4450 5200 50  0001 C CNN
	1    4450 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 5200 4450 4900
$Comp
L power:GND #PWR0120
U 1 1 5E2FA946
P 6300 5200
F 0 "#PWR0120" H 6300 4950 50  0001 C CNN
F 1 "GND" H 6305 5027 50  0000 C CNN
F 2 "" H 6300 5200 50  0001 C CNN
F 3 "" H 6300 5200 50  0001 C CNN
	1    6300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4900 6300 5200
$Comp
L Device:R_Small R66
U 1 1 5E2FC9F5
P 3900 4550
F 0 "R66" H 3959 4596 50  0000 L CNN
F 1 "10K" H 3959 4505 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3900 4550 50  0001 C CNN
F 3 "~" H 3900 4550 50  0001 C CNN
	1    3900 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4650 3900 4700
Wire Wire Line
	3900 4700 4150 4700
$Comp
L Device:R_Small R69
U 1 1 5E2FEF5D
P 5800 4550
F 0 "R69" H 5859 4596 50  0000 L CNN
F 1 "10K" H 5859 4505 50  0000 L CNN
F 2 "lc_lib:0805_R" H 5800 4550 50  0001 C CNN
F 3 "~" H 5800 4550 50  0001 C CNN
	1    5800 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4650 5800 4700
Wire Wire Line
	5800 4700 6000 4700
Wire Wire Line
	3900 4700 3900 4900
Connection ~ 3900 4700
Text HLabel 3650 4900 0    50   Input ~ 0
FAN1
Text HLabel 5800 5000 0    50   Input ~ 0
FAN2
Wire Wire Line
	5800 4700 5800 4750
Connection ~ 5800 4700
$Comp
L Device:CP_Small C46
U 1 1 5E31B393
P 4850 3950
F 0 "C46" H 4938 3996 50  0000 L CNN
F 1 "25V 47uF" H 4938 3905 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4850 3950 50  0001 C CNN
F 3 "~" H 4850 3950 50  0001 C CNN
	1    4850 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3850 4850 3750
Wire Wire Line
	4850 3750 4450 3750
Connection ~ 4450 3750
Wire Wire Line
	4450 3750 4450 3850
$Comp
L power:GND #PWR0114
U 1 1 5E320503
P 4850 4250
F 0 "#PWR0114" H 4850 4000 50  0001 C CNN
F 1 "GND" H 4855 4077 50  0000 C CNN
F 2 "" H 4850 4250 50  0001 C CNN
F 3 "" H 4850 4250 50  0001 C CNN
	1    4850 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4050 4850 4250
$Comp
L Device:CP_Small C51
U 1 1 5E322D9C
P 6750 4000
F 0 "C51" H 6838 4046 50  0000 L CNN
F 1 "25V 47uF" H 6838 3955 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 6750 4000 50  0001 C CNN
F 3 "~" H 6750 4000 50  0001 C CNN
	1    6750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3900 6750 3800
Wire Wire Line
	6750 3800 6300 3800
Connection ~ 6300 3800
Wire Wire Line
	6300 3800 6300 3850
$Comp
L power:GND #PWR0122
U 1 1 5E3257D0
P 6750 4200
F 0 "#PWR0122" H 6750 3950 50  0001 C CNN
F 1 "GND" H 6755 4027 50  0000 C CNN
F 2 "" H 6750 4200 50  0001 C CNN
F 3 "" H 6750 4200 50  0001 C CNN
	1    6750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 4100 6750 4200
$Comp
L MySchLib:D D9
U 1 1 5E329592
P 4150 4200
F 0 "D9" V 4104 4279 50  0000 L CNN
F 1 "SS14" V 4195 4279 50  0000 L CNN
F 2 "lc_lib:SMA(DO-214AC)_S1" H 4150 4200 50  0001 C CNN
F 3 "~" H 4150 4200 50  0001 C CNN
	1    4150 4200
	0    1    1    0   
$EndComp
$Comp
L MySchLib:D D10
U 1 1 5E32A401
P 6000 4150
F 0 "D10" V 5954 4229 50  0000 L CNN
F 1 "SS14" V 6045 4229 50  0000 L CNN
F 2 "lc_lib:SMA(DO-214AC)_S1" H 6000 4150 50  0001 C CNN
F 3 "~" H 6000 4150 50  0001 C CNN
	1    6000 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 4300 6000 4450
Wire Wire Line
	6000 4450 6300 4450
Connection ~ 6300 4450
Wire Wire Line
	6300 4450 6300 4500
Wire Wire Line
	4150 4350 4150 4450
Wire Wire Line
	4150 4450 4450 4450
Connection ~ 4450 4450
Wire Wire Line
	4450 4450 4450 4350
Wire Wire Line
	4150 4050 4150 3750
Wire Wire Line
	4150 3750 4450 3750
Wire Wire Line
	6000 4000 6000 3800
Wire Wire Line
	6000 3800 6300 3800
$Comp
L Device:R_Small R65
U 1 1 5E349F7B
P 3800 4900
F 0 "R65" H 3859 4946 50  0000 L CNN
F 1 "100" H 3859 4855 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3800 4900 50  0001 C CNN
F 3 "~" H 3800 4900 50  0001 C CNN
	1    3800 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 4900 3650 4900
$Comp
L Device:R_Small R70
U 1 1 5E34A7A2
P 5800 4850
F 0 "R70" H 5859 4896 50  0000 L CNN
F 1 "100" H 5859 4805 50  0000 L CNN
F 2 "lc_lib:0805_R" H 5800 4850 50  0001 C CNN
F 3 "~" H 5800 4850 50  0001 C CNN
	1    5800 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5800 4950 5800 5000
Wire Wire Line
	1700 1650 1900 1650
Wire Wire Line
	1600 1750 1350 1750
Connection ~ 1200 1750
Wire Wire Line
	1200 1750 1200 1650
$Comp
L power:+5V #PWR0112
U 1 1 5E450B43
P 4700 1600
F 0 "#PWR0112" H 4700 1450 50  0001 C CNN
F 1 "+5V" H 4715 1773 50  0000 C CNN
F 2 "" H 4700 1600 50  0001 C CNN
F 3 "" H 4700 1600 50  0001 C CNN
	1    4700 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 1600 4700 1650
$Comp
L Device:C_Small C48
U 1 1 5E458526
P 5300 3050
F 0 "C48" H 5400 3100 50  0000 L CNN
F 1 "104" H 5400 2950 50  0000 L CNN
F 2 "lc_lib:0805_C" H 5300 3050 50  0001 C CNN
F 3 "~" H 5300 3050 50  0001 C CNN
	1    5300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2950 5300 2800
Connection ~ 5300 2800
$Comp
L power:GND #PWR0115
U 1 1 5E45BE1A
P 5300 3250
F 0 "#PWR0115" H 5300 3000 50  0001 C CNN
F 1 "GND" H 5305 3077 50  0000 C CNN
F 2 "" H 5300 3250 50  0001 C CNN
F 3 "" H 5300 3250 50  0001 C CNN
	1    5300 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3150 5300 3250
Wire Wire Line
	5250 1650 5300 1650
Wire Wire Line
	6250 1650 6600 1650
$Comp
L Device:R_Small R80
U 1 1 5E6BDFC2
P 4550 1300
F 0 "R80" H 4609 1346 50  0000 L CNN
F 1 "2.2K" H 4609 1255 50  0000 L CNN
F 2 "lc_lib:0805_R" H 4550 1300 50  0001 C CNN
F 3 "~" H 4550 1300 50  0001 C CNN
	1    4550 1300
	1    0    0    -1  
$EndComp
$Comp
L MySchLib:LED D22
U 1 1 5E6BF0FB
P 4400 1100
F 0 "D22" H 4393 1316 50  0000 C CNN
F 1 "Power" H 4393 1225 50  0000 C CNN
F 2 "lc_lib:0805_LED_S1" H 4400 1100 50  0001 C CNN
F 3 "~" H 4400 1100 50  0001 C CNN
	1    4400 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1100 4550 1200
Wire Wire Line
	4550 1400 4550 1650
Connection ~ 4550 1650
Wire Wire Line
	4550 1650 4400 1650
$Comp
L power:GND #PWR0139
U 1 1 5E6C79FA
P 4100 1150
F 0 "#PWR0139" H 4100 900 50  0001 C CNN
F 1 "GND" H 4105 977 50  0000 C CNN
F 2 "" H 4100 1150 50  0001 C CNN
F 3 "" H 4100 1150 50  0001 C CNN
	1    4100 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1150 4100 1100
Wire Wire Line
	4100 1100 4250 1100
$Comp
L Device:Ferrite_Bead_Small FB7
U 1 1 5E4713FC
P 3750 5950
F 0 "FB7" V 3513 5950 50  0000 C CNN
F 1 "1K" V 3604 5950 50  0000 C CNN
F 2 "lc_lib:1206_R" V 3680 5950 50  0001 C CNN
F 3 "~" H 3750 5950 50  0001 C CNN
	1    3750 5950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0150
U 1 1 5E472F1D
P 4050 6050
F 0 "#PWR0150" H 4050 5800 50  0001 C CNN
F 1 "GND" H 4055 5877 50  0000 C CNN
F 2 "" H 4050 6050 50  0001 C CNN
F 3 "" H 4050 6050 50  0001 C CNN
	1    4050 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0151
U 1 1 5E473989
P 4300 6050
F 0 "#PWR0151" H 4300 5800 50  0001 C CNN
F 1 "GNDA" H 4305 5877 50  0000 C CNN
F 2 "" H 4300 6050 50  0001 C CNN
F 3 "" H 4300 6050 50  0001 C CNN
	1    4300 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 6050 4300 5950
Wire Wire Line
	4300 5950 4050 5950
Wire Wire Line
	4050 6050 4050 5950
Connection ~ 4050 5950
Wire Wire Line
	4050 5950 3850 5950
$Comp
L power:GNDD #PWR0152
U 1 1 5E47C8AA
P 3500 6150
F 0 "#PWR0152" H 3500 5900 50  0001 C CNN
F 1 "GNDD" H 3504 5995 50  0000 C CNN
F 2 "" H 3500 6150 50  0001 C CNN
F 3 "" H 3500 6150 50  0001 C CNN
	1    3500 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6150 3500 5950
Wire Wire Line
	3500 5950 3650 5950
$Comp
L Device:R_Small JP1
U 1 1 60390E7A
P 6050 2550
F 0 "JP1" H 6109 2596 50  0000 L CNN
F 1 "0" H 6109 2505 50  0000 L CNN
F 2 "lc_lib:0805_R" H 6050 2550 50  0001 C CNN
F 3 "~" H 6050 2550 50  0001 C CNN
	1    6050 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5950 2550 5300 2550
Wire Wire Line
	5300 1650 5300 2550
Connection ~ 5300 2550
Wire Wire Line
	5300 2550 5300 2800
Wire Wire Line
	6150 2550 6250 2550
Wire Wire Line
	6250 2550 6250 2800
Text Notes 6250 2500 0    50   ~ 0
Use 5V Backlight Volt
Wire Wire Line
	2850 2400 2850 2300
Wire Wire Line
	2350 2000 2400 2000
Wire Wire Line
	2350 1900 2350 2000
$Comp
L Device:R_Small R63
U 1 1 5F5D6515
P 2350 1800
F 0 "R63" H 2409 1846 50  0000 L CNN
F 1 "100K" H 2409 1755 50  0000 L CNN
F 2 "lc_lib:0805_R" H 2350 1800 50  0001 C CNN
F 3 "~" H 2350 1800 50  0001 C CNN
	1    2350 1800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J22
U 1 1 603B7427
P 650 2150
F 0 "J22" H 568 2367 50  0000 C CNN
F 1 "12Vin" H 568 2276 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B2B-PH-SM4-TB_1x02-1MP_P2.00mm_Vertical" H 650 2150 50  0001 C CNN
F 3 "~" H 650 2150 50  0001 C CNN
	1    650  2150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	850  2250 900  2250
Wire Wire Line
	900  2250 900  2400
$Comp
L power:GND #PWR0170
U 1 1 603C2990
P 900 2400
F 0 "#PWR0170" H 900 2150 50  0001 C CNN
F 1 "GND" H 905 2227 50  0000 C CNN
F 2 "" H 900 2400 50  0001 C CNN
F 3 "" H 900 2400 50  0001 C CNN
	1    900  2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  2150 1200 2150
$Comp
L Device:R_Small R57
U 1 1 603DAB91
P 3250 2750
F 0 "R57" H 3309 2796 50  0000 L CNN
F 1 "470" H 3309 2705 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3250 2750 50  0001 C CNN
F 3 "~" H 3250 2750 50  0001 C CNN
	1    3250 2750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0172
U 1 1 603E4F24
P 3550 3350
F 0 "#PWR0172" H 3550 3100 50  0001 C CNN
F 1 "GND" H 3555 3177 50  0000 C CNN
F 2 "" H 3550 3350 50  0001 C CNN
F 3 "" H 3550 3350 50  0001 C CNN
	1    3550 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3350 3550 3300
Wire Wire Line
	3650 3000 3700 3000
Wire Wire Line
	3350 2750 3550 2750
Wire Wire Line
	3550 2750 3550 2900
Wire Wire Line
	3100 2750 3150 2750
$Comp
L Device:R_Small R58
U 1 1 6040D2A4
P 3700 2850
F 0 "R58" H 3550 2900 50  0000 L CNN
F 1 "1.5K" H 3450 2800 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3700 2850 50  0001 C CNN
F 3 "~" H 3700 2850 50  0001 C CNN
	1    3700 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 2950 3700 3000
Wire Wire Line
	3550 2750 3700 2750
Connection ~ 3550 2750
$Comp
L Device:R_Small R59
U 1 1 60418B10
P 3700 3150
F 0 "R59" H 3500 3200 50  0000 L CNN
F 1 "4.7K" H 3450 3100 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3700 3150 50  0001 C CNN
F 3 "~" H 3700 3150 50  0001 C CNN
	1    3700 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 3000 3700 3050
Connection ~ 3700 3000
Wire Wire Line
	3700 3250 3700 3300
Wire Wire Line
	3700 3300 3550 3300
Connection ~ 3550 3300
Wire Wire Line
	3550 3300 3550 3100
$Comp
L Device:C_Small C62
U 1 1 6043C826
P 4050 3000
F 0 "C62" H 4150 3050 50  0000 L CNN
F 1 "104" H 4150 2900 50  0000 L CNN
F 2 "lc_lib:0805_C" H 4050 3000 50  0001 C CNN
F 3 "~" H 4050 3000 50  0001 C CNN
	1    4050 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2900 4050 2750
Wire Wire Line
	4050 2750 3700 2750
Connection ~ 3700 2750
$Comp
L power:GND #PWR0173
U 1 1 60443B22
P 4050 3200
F 0 "#PWR0173" H 4050 2950 50  0001 C CNN
F 1 "GND" H 4250 3150 50  0000 C CNN
F 2 "" H 4050 3200 50  0001 C CNN
F 3 "" H 4050 3200 50  0001 C CNN
	1    4050 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3100 4050 3150
Wire Wire Line
	4050 2750 4400 2750
Connection ~ 4050 2750
Text HLabel 4500 2750 2    50   Output ~ 0
VREF
$Comp
L MySchLib:SY8120 U10
U 1 1 603DDD03
P 2750 1550
F 0 "U10" H 2850 1931 50  0000 C CNN
F 1 "MT2492/SY8120" H 2850 1840 50  0000 C CNN
F 2 "lc_lib:SOT-23-6" H 2750 1550 50  0001 C CNN
F 3 "" H 2750 1550 50  0001 C CNN
	1    2750 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2000 3700 1950
Wire Wire Line
	3300 2000 3700 2000
$Comp
L Device:CP_Small C63
U 1 1 60407092
P 4400 3000
F 0 "C63" H 4488 3046 50  0000 L CNN
F 1 "10V 100uF" H 4488 2955 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4400 3000 50  0001 C CNN
F 3 "~" H 4400 3000 50  0001 C CNN
	1    4400 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2900 4400 2750
Connection ~ 4400 2750
Wire Wire Line
	4400 2750 4500 2750
Wire Wire Line
	4400 3100 4400 3150
Wire Wire Line
	4400 3150 4050 3150
Connection ~ 4050 3150
Wire Wire Line
	4050 3150 4050 3200
$Comp
L Connector_Generic:Conn_01x02 J23
U 1 1 60422006
P 650 3400
F 0 "J23" H 568 3617 50  0000 C CNN
F 1 "BIAS" H 568 3526 50  0000 C CNN
F 2 "Connector_JST:JST_PH_B2B-PH-SM4-TB_1x02-1MP_P2.00mm_Vertical" H 650 3400 50  0001 C CNN
F 3 "~" H 650 3400 50  0001 C CNN
	1    650  3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1200 3400 850  3400
$Comp
L Device:C_Small C37
U 1 1 6042B935
P 1350 3450
F 0 "C37" H 1150 3450 50  0000 L CNN
F 1 "103" H 1100 3350 50  0000 L CNN
F 2 "lc_lib:0805_C" H 1350 3450 50  0001 C CNN
F 3 "~" H 1350 3450 50  0001 C CNN
	1    1350 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3250 1350 3250
Wire Wire Line
	1600 3250 1600 3350
Wire Wire Line
	1200 3250 1200 3400
Wire Wire Line
	1350 3350 1350 3250
Connection ~ 1350 3250
Wire Wire Line
	1350 3250 1600 3250
$Comp
L Device:C_Small C61
U 1 1 6043AC14
P 2000 3450
F 0 "C61" H 1800 3450 50  0000 L CNN
F 1 "104" H 1750 3350 50  0000 L CNN
F 2 "lc_lib:0805_C" H 2000 3450 50  0001 C CNN
F 3 "~" H 2000 3450 50  0001 C CNN
	1    2000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3350 2000 3250
Wire Wire Line
	2000 3250 1600 3250
Connection ~ 1600 3250
Wire Wire Line
	1350 3550 1350 3600
Wire Wire Line
	1350 3600 1600 3600
Wire Wire Line
	2000 3600 2000 3550
Wire Wire Line
	1600 3550 1600 3600
Connection ~ 1600 3600
Wire Wire Line
	1600 3600 2000 3600
$Comp
L power:GND #PWR0174
U 1 1 60453D22
P 1600 3900
F 0 "#PWR0174" H 1600 3650 50  0001 C CNN
F 1 "GND" H 1605 3727 50  0000 C CNN
F 2 "" H 1600 3900 50  0001 C CNN
F 3 "" H 1600 3900 50  0001 C CNN
	1    1600 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3900 1600 3600
Wire Wire Line
	850  3500 1000 3500
$Comp
L power:GND #PWR0175
U 1 1 604655C1
P 1000 3900
F 0 "#PWR0175" H 1000 3650 50  0001 C CNN
F 1 "GND" H 1005 3727 50  0000 C CNN
F 2 "" H 1000 3900 50  0001 C CNN
F 3 "" H 1000 3900 50  0001 C CNN
	1    1000 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 3500 1000 3900
$Comp
L Device:R_Small R54
U 1 1 6046FF1A
P 1350 3150
F 0 "R54" H 1409 3196 50  0000 L CNN
F 1 "100" H 1409 3105 50  0000 L CNN
F 2 "lc_lib:0805_R" H 1350 3150 50  0001 C CNN
F 3 "~" H 1350 3150 50  0001 C CNN
	1    1350 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C39
U 1 1 6042BC81
P 1600 3450
F 0 "C39" H 1688 3496 50  0000 L CNN
F 1 "10u" H 1688 3405 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 1600 3450 50  0001 C CNN
F 3 "~" H 1600 3450 50  0001 C CNN
	1    1600 3450
	1    0    0    -1  
$EndComp
$Comp
L MySchLib:8550 Q16
U 1 1 6047EB8B
P 1450 2800
F 0 "Q16" H 1641 2754 50  0000 L CNN
F 1 "2N5401" H 1641 2845 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 1650 2725 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8550.pdf" H 1450 2800 50  0001 L CNN
	1    1450 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	1600 1950 1600 1750
Wire Wire Line
	1350 3000 1350 3050
$Comp
L Device:R_Small R56
U 1 1 604BA41B
P 1850 2800
F 0 "R56" H 1909 2846 50  0000 L CNN
F 1 "47K" H 1909 2755 50  0000 L CNN
F 2 "lc_lib:0805_R" H 1850 2800 50  0001 C CNN
F 3 "~" H 1850 2800 50  0001 C CNN
	1    1850 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	1750 2800 1700 2800
Wire Wire Line
	1350 2600 1350 2550
Connection ~ 1350 1750
Wire Wire Line
	1350 1750 1200 1750
Wire Wire Line
	1950 2800 2100 2800
$Comp
L Device:R_Small R55
U 1 1 604DBC84
P 1550 2550
F 0 "R55" H 1609 2596 50  0000 L CNN
F 1 "100K" H 1600 2650 50  0000 L CNN
F 2 "lc_lib:0805_R" H 1550 2550 50  0001 C CNN
F 3 "~" H 1550 2550 50  0001 C CNN
	1    1550 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 2550 1450 2550
Connection ~ 1350 2550
Wire Wire Line
	1350 2550 1350 1750
Wire Wire Line
	1650 2550 1700 2550
Wire Wire Line
	1700 2550 1700 2800
Connection ~ 1700 2800
Wire Wire Line
	1700 2800 1650 2800
Wire Wire Line
	2500 2800 2650 2800
$Comp
L power:GND #PWR0176
U 1 1 60510F07
P 2650 2800
F 0 "#PWR0176" H 2650 2550 50  0001 C CNN
F 1 "GND" H 2655 2627 50  0000 C CNN
F 2 "" H 2650 2800 50  0001 C CNN
F 3 "" H 2650 2800 50  0001 C CNN
	1    2650 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2300 3150 2300 3250
Text HLabel 2300 3250 3    50   Input ~ 0
BIAS_ON
$Comp
L Transistor_BJT:DTC144E Q17
U 1 1 6206161D
P 2300 2900
F 0 "Q17" V 2625 2900 50  0000 C CNN
F 1 "DTC144E" V 2534 2900 50  0000 C CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 2300 2900 50  0001 L CNN
F 3 "" H 2300 2900 50  0001 L CNN
	1    2300 2900
	0    -1   -1   0   
$EndComp
Text HLabel 1200 1450 1    50   Output ~ 0
+12V
Wire Wire Line
	1200 1450 1200 1650
Connection ~ 1200 1650
Text HLabel 5300 1300 1    50   Output ~ 0
+5VA
Wire Wire Line
	5300 1300 5300 1650
Wire Wire Line
	3100 2550 5300 2550
Wire Wire Line
	3100 2550 3100 2750
Wire Wire Line
	4450 3600 4450 3750
Wire Wire Line
	6300 3600 6300 3800
Text HLabel 6650 2800 2    50   Output ~ 0
VBL
Text HLabel 6600 1500 1    50   Output ~ 0
+3.3V
Wire Wire Line
	6600 1500 6600 1650
Wire Wire Line
	3900 4250 3900 4450
Wire Wire Line
	5800 4250 5800 4450
Text Label 1200 1650 0    50   ~ 0
+12V
Text Label 4450 3600 0    50   ~ 0
+12V
Text Label 6300 3600 0    50   ~ 0
+12V
Text Label 6600 1650 0    50   ~ 0
+3.3V
Text Label 5800 4250 1    50   ~ 0
+3.3V
Text Label 3900 4250 1    50   ~ 0
+3.3V
Wire Wire Line
	6250 2800 6650 2800
$Comp
L MySchLib:TL431 U12
U 1 1 617CC703
P 3550 3000
F 0 "U12" V 3596 2931 50  0000 R CNN
F 1 "TL431" V 3505 2931 50  0000 R CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 3550 2750 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tl431.pdf" H 3550 3000 50  0001 C CIN
	1    3550 3000
	0    1    -1   0   
$EndComp
$EndSCHEMATC