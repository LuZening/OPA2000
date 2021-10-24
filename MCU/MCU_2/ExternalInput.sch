EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
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
L Connector_Generic:Conn_01x02 J?
U 1 1 5EA99E3C
P 950 2000
AR Path="/5E7D94C5/5EA99E3C" Ref="J?"  Part="1" 
AR Path="/5EA95119/5EA99E3C" Ref="J14"  Part="1" 
F 0 "J14" V 914 1812 50  0000 R CNN
F 1 "TX_GND" V 823 1812 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 950 2000 50  0001 C CNN
F 3 "~" H 950 2000 50  0001 C CNN
	1    950  2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1150 2350 1150 2300
Wire Wire Line
	1150 2300 1050 2300
Wire Wire Line
	1050 2300 1050 2200
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5EA9F740
P 1700 5050
AR Path="/5E814668/5EA9F740" Ref="J?"  Part="1" 
AR Path="/5EA95119/5EA9F740" Ref="J15"  Part="1" 
F 0 "J15" H 1618 4725 50  0000 C CNN
F 1 "SW" H 1618 4816 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1700 5050 50  0001 C CNN
F 3 "~" H 1700 5050 50  0001 C CNN
	1    1700 5050
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EA9F747
P 2000 5250
AR Path="/5E814668/5EA9F747" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/5EA9F747" Ref="#PWR096"  Part="1" 
F 0 "#PWR096" H 2000 5000 50  0001 C CNN
F 1 "GND" H 2005 5077 50  0000 C CNN
F 2 "" H 2000 5250 50  0001 C CNN
F 3 "" H 2000 5250 50  0001 C CNN
	1    2000 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 5150 2000 5150
Wire Wire Line
	2000 5150 2000 5250
Text Notes 1500 5350 0    50   ~ 0
Power On
Wire Wire Line
	950  2200 950  2400
Text Notes 900  1900 0    50   ~ 0
TX GND
Wire Wire Line
	1750 2200 1750 2400
Text Notes 1700 1900 0    50   ~ 0
Computer
$Comp
L Connector_Generic:Conn_01x04 J17
U 1 1 5F1DAB3E
P 1850 2000
F 0 "J17" V 1814 1712 50  0000 R CNN
F 1 "COM" V 1723 1712 50  0000 R CNN
F 2 "Connector_JST:JST_PH_B4B-PH-SM4-TB_1x04-1MP_P2.00mm_Vertical" H 1850 2000 50  0001 C CNN
F 3 "~" H 1850 2000 50  0001 C CNN
	1    1850 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 2200 2050 2200
Wire Wire Line
	2200 2200 2200 2350
Text HLabel 3650 6350 0    50   Input ~ 0
TX
$Comp
L Device:R_Small R60
U 1 1 5F3871D3
P 3850 5950
F 0 "R60" H 3700 5800 50  0000 L CNN
F 1 "1K" H 3700 5900 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3850 5950 50  0001 C CNN
F 3 "~" H 3850 5950 50  0001 C CNN
	1    3850 5950
	0    1    1    0   
$EndComp
$Comp
L Interface_UART:MAX3232 U16
U 1 1 5E403A54
P 5050 6250
F 0 "U16" H 5050 7631 50  0000 C CNN
F 1 "MAX3232" H 5050 7540 50  0000 C CNN
F 2 "lc_lib:SOIC-16_150MIL" H 5100 5200 50  0001 L CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX3222-MAX3241.pdf" H 5050 6350 50  0001 C CNN
	1    5050 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C52
U 1 1 5E40D43D
P 4100 5500
F 0 "C52" V 3871 5500 50  0000 C CNN
F 1 "104" V 3962 5500 50  0000 C CNN
F 2 "lc_lib:0805_C" H 4100 5500 50  0001 C CNN
F 3 "~" H 4100 5500 50  0001 C CNN
	1    4100 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 5400 4100 5350
Wire Wire Line
	4100 5350 4250 5350
Wire Wire Line
	4100 5600 4100 5650
Wire Wire Line
	4100 5650 4250 5650
$Comp
L Device:C_Small C54
U 1 1 5E40EDA9
P 6000 5500
F 0 "C54" V 5771 5500 50  0000 C CNN
F 1 "104" V 5862 5500 50  0000 C CNN
F 2 "lc_lib:0805_C" H 6000 5500 50  0001 C CNN
F 3 "~" H 6000 5500 50  0001 C CNN
	1    6000 5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 5400 6000 5350
Wire Wire Line
	6000 5350 5850 5350
Wire Wire Line
	5850 5650 6000 5650
Wire Wire Line
	6000 5650 6000 5600
$Comp
L Device:C_Small C56
U 1 1 5E4100BC
P 6400 5750
F 0 "C56" V 6171 5750 50  0000 C CNN
F 1 "104" V 6262 5750 50  0000 C CNN
F 2 "lc_lib:0805_C" H 6400 5750 50  0001 C CNN
F 3 "~" H 6400 5750 50  0001 C CNN
	1    6400 5750
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 5850 5850 5850
Wire Wire Line
	5050 5050 6400 5050
Wire Wire Line
	6400 5050 6400 5650
$Comp
L Device:C_Small C55
U 1 1 5E412E60
P 6100 6150
F 0 "C55" V 5871 6150 50  0000 C CNN
F 1 "104" V 5962 6150 50  0000 C CNN
F 2 "lc_lib:0805_C" H 6100 6150 50  0001 C CNN
F 3 "~" H 6100 6150 50  0001 C CNN
	1    6100 6150
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 6150 5850 6150
$Comp
L power:GNDD #PWR0123
U 1 1 5E417E0C
P 6650 6150
F 0 "#PWR0123" H 6650 5900 50  0001 C CNN
F 1 "GNDD" H 6654 5995 50  0000 C CNN
F 2 "" H 6650 6150 50  0001 C CNN
F 3 "" H 6650 6150 50  0001 C CNN
	1    6650 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 6150 6650 6150
Wire Wire Line
	5050 5050 5050 4900
Connection ~ 5050 5050
$Comp
L power:+3.3V #PWR0124
U 1 1 5E4194DA
P 5050 4800
F 0 "#PWR0124" H 5050 4650 50  0001 C CNN
F 1 "+3.3V" H 5065 4973 50  0000 C CNN
F 2 "" H 5050 4800 50  0001 C CNN
F 3 "" H 5050 4800 50  0001 C CNN
	1    5050 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C53
U 1 1 5E419ABF
P 4850 4900
F 0 "C53" V 4621 4900 50  0000 C CNN
F 1 "104" V 4712 4900 50  0000 C CNN
F 2 "lc_lib:0805_C" H 4850 4900 50  0001 C CNN
F 3 "~" H 4850 4900 50  0001 C CNN
	1    4850 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 4900 5050 4900
Connection ~ 5050 4900
Wire Wire Line
	5050 4900 5050 4800
Wire Wire Line
	4750 5000 4750 4900
Text HLabel 3650 6750 0    50   Output ~ 0
RX
Wire Wire Line
	3650 6350 4100 6350
$Comp
L MySchLib:LED D13
U 1 1 5E42B5DA
P 4100 6100
F 0 "D13" V 4139 5983 50  0000 R CNN
F 1 "LED" V 4048 5983 50  0000 R CNN
F 2 "lc_lib:0805_LED_S1" H 4100 6100 50  0001 C CNN
F 3 "~" H 4100 6100 50  0001 C CNN
	1    4100 6100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3950 5950 4100 5950
Wire Wire Line
	4100 6250 4100 6350
Connection ~ 4100 6350
Wire Wire Line
	4100 6350 4250 6350
Wire Wire Line
	3750 5950 3550 5950
Wire Wire Line
	3550 5950 3550 5850
$Comp
L power:+3.3V #PWR0126
U 1 1 5E42DFD1
P 3550 5850
F 0 "#PWR0126" H 3550 5700 50  0001 C CNN
F 1 "+3.3V" H 3565 6023 50  0000 C CNN
F 2 "" H 3550 5850 50  0001 C CNN
F 3 "" H 3550 5850 50  0001 C CNN
	1    3550 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0128
U 1 1 5E435017
P 5050 7550
F 0 "#PWR0128" H 5050 7300 50  0001 C CNN
F 1 "GNDD" H 5054 7395 50  0000 C CNN
F 2 "" H 5050 7550 50  0001 C CNN
F 3 "" H 5050 7550 50  0001 C CNN
	1    5050 7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 7450 5050 7550
Wire Wire Line
	5850 6350 5900 6350
Text Label 6150 6350 0    50   ~ 0
RX232
Text Label 6150 6750 0    50   ~ 0
TX232
Text Label 1850 3200 3    50   ~ 0
TX232
Text Label 1950 3200 3    50   ~ 0
RX232
Wire Wire Line
	5850 6750 5900 6750
$Comp
L Device:Ferrite_Bead_Small FB5
U 1 1 5E44194A
P 6000 6350
F 0 "FB5" V 6237 6350 50  0000 C CNN
F 1 "1K" V 6146 6350 50  0000 C CNN
F 2 "lc_lib:0805_R" V 5930 6350 50  0001 C CNN
F 3 "~" H 6000 6350 50  0001 C CNN
	1    6000 6350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 6350 6150 6350
$Comp
L Device:Ferrite_Bead_Small FB6
U 1 1 5E444ADA
P 6000 6750
F 0 "FB6" V 6237 6750 50  0000 C CNN
F 1 "1K" V 6146 6750 50  0000 C CNN
F 2 "lc_lib:0805_R" V 5930 6750 50  0001 C CNN
F 3 "~" H 6000 6750 50  0001 C CNN
	1    6000 6750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 6750 6150 6750
Wire Wire Line
	1550 2500 1550 2400
Wire Wire Line
	1550 2400 1750 2400
Wire Wire Line
	800  2450 800  2400
Wire Wire Line
	800  2400 950  2400
Wire Wire Line
	800  2650 800  2700
Wire Wire Line
	1550 2700 1550 2800
$Comp
L power:GNDD #PWR0146
U 1 1 5E65BD42
P 2200 2350
F 0 "#PWR0146" H 2200 2100 50  0001 C CNN
F 1 "GNDD" H 2204 2195 50  0000 C CNN
F 2 "" H 2200 2350 50  0001 C CNN
F 3 "" H 2200 2350 50  0001 C CNN
	1    2200 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E65D41A
P 1150 2350
AR Path="/5E7D94C5/5E65D41A" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/5E65D41A" Ref="#PWR0148"  Part="1" 
F 0 "#PWR0148" H 1150 2100 50  0001 C CNN
F 1 "GND" H 1155 2177 50  0000 C CNN
F 2 "" H 1150 2350 50  0001 C CNN
F 3 "" H 1150 2350 50  0001 C CNN
	1    1150 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0149
U 1 1 5E65E6DB
P 4750 5000
F 0 "#PWR0149" H 4750 4750 50  0001 C CNN
F 1 "GNDD" H 4754 4845 50  0000 C CNN
F 2 "" H 4750 5000 50  0001 C CNN
F 3 "" H 4750 5000 50  0001 C CNN
	1    4750 5000
	1    0    0    -1  
$EndComp
$Comp
L MySchLib:D_Schottky_Small D?
U 1 1 5E520B7E
P 1550 2600
AR Path="/5E7D94C5/5E520B7E" Ref="D?"  Part="1" 
AR Path="/5EA95119/5E520B7E" Ref="D25"  Part="1" 
F 0 "D25" V 1596 2532 50  0000 R CNN
F 1 "SD103" V 1505 2532 50  0000 R CNN
F 2 "lc_lib:SOD-123" V 1550 2600 50  0001 C CNN
F 3 "~" V 1550 2600 50  0001 C CNN
	1    1550 2600
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J20
U 1 1 5E54ECE3
P 7200 2150
F 0 "J20" V 7164 2230 50  0000 L CNN
F 1 "CAN" V 7073 2230 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7200 2150 50  0001 C CNN
F 3 "~" H 7200 2150 50  0001 C CNN
	1    7200 2150
	0    -1   -1   0   
$EndComp
$Comp
L Interface_CAN_LIN:TJA1042T U17
U 1 1 5E54FC2A
P 7250 3350
F 0 "U17" V 7296 2906 50  0000 R CNN
F 1 "TJA1042T" V 7205 2906 50  0000 R CNN
F 2 "lc_lib:SOIC-8_150MIL" H 7250 2850 50  0001 C CIN
F 3 "http://www.nxp.com/documents/data_sheet/TJA1042.pdf" H 7250 3350 50  0001 C CNN
	1    7250 3350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R87
U 1 1 5E551309
P 7250 2650
F 0 "R87" H 7309 2696 50  0000 L CNN
F 1 "120" H 7309 2605 50  0000 L CNN
F 2 "lc_lib:0805_R" H 7250 2650 50  0001 C CNN
F 3 "~" H 7250 2650 50  0001 C CNN
	1    7250 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7150 2850 7150 2650
Wire Wire Line
	7150 2650 7150 2450
Wire Wire Line
	7150 2450 7200 2450
Wire Wire Line
	7200 2450 7200 2350
Connection ~ 7150 2650
Wire Wire Line
	7300 2350 7300 2450
Wire Wire Line
	7300 2450 7350 2450
Wire Wire Line
	7350 2450 7350 2650
Wire Wire Line
	7350 2650 7350 2850
Connection ~ 7350 2650
Wire Wire Line
	6650 3300 6650 3350
Wire Wire Line
	6650 3350 6850 3350
$Comp
L Device:C_Small C60
U 1 1 5E55CC73
P 6650 3500
F 0 "C60" V 6750 3500 50  0000 C CNN
F 1 "104" V 6800 3500 50  0000 C CNN
F 2 "lc_lib:0805_C" H 6650 3500 50  0001 C CNN
F 3 "~" H 6650 3500 50  0001 C CNN
	1    6650 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	6650 3400 6650 3350
Connection ~ 6650 3350
$Comp
L power:GND #PWR?
U 1 1 5E55F53A
P 6650 3650
AR Path="/5E7D94C5/5E55F53A" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/5E55F53A" Ref="#PWR0155"  Part="1" 
F 0 "#PWR0155" H 6650 3400 50  0001 C CNN
F 1 "GND" H 6655 3477 50  0000 C CNN
F 2 "" H 6650 3650 50  0001 C CNN
F 3 "" H 6650 3650 50  0001 C CNN
	1    6650 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3650 6650 3600
$Comp
L power:GND #PWR?
U 1 1 5E561C2E
P 7750 3400
AR Path="/5E7D94C5/5E561C2E" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/5E561C2E" Ref="#PWR0156"  Part="1" 
F 0 "#PWR0156" H 7750 3150 50  0001 C CNN
F 1 "GND" H 7755 3227 50  0000 C CNN
F 2 "" H 7750 3400 50  0001 C CNN
F 3 "" H 7750 3400 50  0001 C CNN
	1    7750 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3400 7750 3350
Wire Wire Line
	7750 3350 7650 3350
Text HLabel 7050 4100 3    50   Input ~ 0
CAN_TX
Text HLabel 7200 4250 3    50   Output ~ 0
CAN_RX
Wire Wire Line
	7200 4250 7200 4200
Wire Wire Line
	7200 4200 7150 4200
$Comp
L power:GND #PWR?
U 1 1 5E56A7E1
P 7450 3950
AR Path="/5E7D94C5/5E56A7E1" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/5E56A7E1" Ref="#PWR0157"  Part="1" 
F 0 "#PWR0157" H 7450 3700 50  0001 C CNN
F 1 "GND" H 7455 3777 50  0000 C CNN
F 2 "" H 7450 3950 50  0001 C CNN
F 3 "" H 7450 3950 50  0001 C CNN
	1    7450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3850 7450 3950
$Comp
L Device:R_Small R85
U 1 1 5E56E527
P 7050 3950
F 0 "R85" H 7109 3996 50  0000 L CNN
F 1 "100" H 7109 3905 50  0000 L CNN
F 2 "lc_lib:0805_R" H 7050 3950 50  0001 C CNN
F 3 "~" H 7050 3950 50  0001 C CNN
	1    7050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4050 7050 4100
$Comp
L Device:R_Small R86
U 1 1 5E56E9EE
P 7150 4100
F 0 "R86" H 7209 4146 50  0000 L CNN
F 1 "100" H 7209 4055 50  0000 L CNN
F 2 "lc_lib:0805_R" H 7150 4100 50  0001 C CNN
F 3 "~" H 7150 4100 50  0001 C CNN
	1    7150 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3850 7150 4000
Text GLabel 7100 2450 0    50   Input ~ 0
CAN+
Wire Wire Line
	7100 2450 7150 2450
Connection ~ 7150 2450
Text GLabel 7400 2450 2    50   Input ~ 0
CAN-
Wire Wire Line
	7400 2450 7350 2450
Connection ~ 7350 2450
$Comp
L power:+5VA #PWR0154
U 1 1 5E65814C
P 6650 3300
F 0 "#PWR0154" H 6650 3150 50  0001 C CNN
F 1 "+5VA" H 6665 3473 50  0000 C CNN
F 2 "" H 6650 3300 50  0001 C CNN
F 3 "" H 6650 3300 50  0001 C CNN
	1    6650 3300
	1    0    0    -1  
$EndComp
Text Notes 1550 1800 0    50   ~ 0
RTS IS HIGH EFFECTIVE
Wire Wire Line
	3150 2200 3150 2400
Text Notes 3100 1900 0    50   ~ 0
Radio
$Comp
L Connector_Generic:Conn_01x04 J21
U 1 1 620F832A
P 3250 2000
F 0 "J21" V 3214 1712 50  0000 R CNN
F 1 "COM_R" V 3123 1712 50  0000 R CNN
F 2 "Connector_JST:JST_PH_B4B-PH-SM4-TB_1x04-1MP_P2.00mm_Vertical" H 3250 2000 50  0001 C CNN
F 3 "~" H 3250 2000 50  0001 C CNN
	1    3250 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3600 2200 3450 2200
Wire Wire Line
	3600 2200 3600 2350
Text Label 3250 3000 3    50   ~ 0
TX232_R
Text Label 3350 3000 3    50   ~ 0
RX232_R
Wire Wire Line
	2950 2500 2950 2400
Wire Wire Line
	2950 2400 3150 2400
Wire Wire Line
	2950 2700 2950 2800
$Comp
L power:GNDD #PWR0162
U 1 1 620F8339
P 3600 2350
F 0 "#PWR0162" H 3600 2100 50  0001 C CNN
F 1 "GNDD" H 3604 2195 50  0000 C CNN
F 2 "" H 3600 2350 50  0001 C CNN
F 3 "" H 3600 2350 50  0001 C CNN
	1    3600 2350
	1    0    0    -1  
$EndComp
$Comp
L MySchLib:D_Schottky_Small D?
U 1 1 620F833F
P 2950 2600
AR Path="/5E7D94C5/620F833F" Ref="D?"  Part="1" 
AR Path="/5EA95119/620F833F" Ref="D3"  Part="1" 
F 0 "D3" V 2996 2532 50  0000 R CNN
F 1 "SD103" V 2905 2532 50  0000 R CNN
F 2 "lc_lib:SOD-123" V 2950 2600 50  0001 C CNN
F 3 "~" V 2950 2600 50  0001 C CNN
	1    2950 2600
	0    -1   -1   0   
$EndComp
Text Notes 3000 1800 0    50   ~ 0
RTS IS HIGH EFFECTIVE
$Comp
L Device:Ferrite_Bead_Small FB4
U 1 1 620FF413
P 6300 6550
F 0 "FB4" V 6537 6550 50  0000 C CNN
F 1 "1K" V 6446 6550 50  0000 C CNN
F 2 "lc_lib:0805_R" V 6230 6550 50  0001 C CNN
F 3 "~" H 6300 6550 50  0001 C CNN
	1    6300 6550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6200 6550 5850 6550
Text Label 6500 6550 0    50   ~ 0
RX232_R
Wire Wire Line
	6500 6550 6400 6550
$Comp
L Device:Ferrite_Bead_Small FB1
U 1 1 62106EB2
P 6250 6950
F 0 "FB1" V 6100 6900 50  0000 C CNN
F 1 "1K" V 6100 7050 50  0000 C CNN
F 2 "lc_lib:0805_R" V 6180 6950 50  0001 C CNN
F 3 "~" H 6250 6950 50  0001 C CNN
	1    6250 6950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6150 6950 5850 6950
Text Label 6500 6950 0    50   ~ 0
TX232_R
Wire Wire Line
	6500 6950 6350 6950
Text HLabel 3650 6550 0    50   Input ~ 0
TX_R
Wire Wire Line
	3650 6550 4250 6550
Text HLabel 3650 6950 0    50   Output ~ 0
RX_R
Wire Wire Line
	3650 6750 3800 6750
Wire Wire Line
	3650 6950 4000 6950
Wire Wire Line
	3800 6850 3800 6750
Connection ~ 3800 6750
Wire Wire Line
	3800 6750 4250 6750
$Comp
L Device:R_Small R52
U 1 1 62126E97
P 3750 7350
F 0 "R52" H 3850 7200 50  0000 L CNN
F 1 "1K" H 3850 7300 50  0000 L CNN
F 2 "lc_lib:0805_R" H 3750 7350 50  0001 C CNN
F 3 "~" H 3750 7350 50  0001 C CNN
	1    3750 7350
	0    1    1    0   
$EndComp
$Comp
L MySchLib:LED D4
U 1 1 62126E9D
P 4000 7200
F 0 "D4" V 3947 7278 50  0000 L CNN
F 1 "LED" V 4038 7278 50  0000 L CNN
F 2 "lc_lib:0805_LED_S1" H 4000 7200 50  0001 C CNN
F 3 "~" H 4000 7200 50  0001 C CNN
	1    4000 7200
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 7350 4000 7350
$Comp
L power:+3.3V #PWR0163
U 1 1 62126EA4
P 3550 7300
F 0 "#PWR0163" H 3550 7150 50  0001 C CNN
F 1 "+3.3V" H 3565 7473 50  0000 C CNN
F 2 "" H 3550 7300 50  0001 C CNN
F 3 "" H 3550 7300 50  0001 C CNN
	1    3550 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 7300 3550 7350
Wire Wire Line
	3550 7350 3650 7350
Wire Wire Line
	4000 7050 4000 6950
Connection ~ 4000 6950
Wire Wire Line
	4000 6950 4250 6950
$Comp
L Device:R_Small R48
U 1 1 625531A7
P 800 2550
F 0 "R48" H 859 2596 50  0000 L CNN
F 1 "1K" H 859 2505 50  0000 L CNN
F 2 "lc_lib:0805_R" H 800 2550 50  0001 C CNN
F 3 "~" H 800 2550 50  0001 C CNN
	1    800  2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C33
U 1 1 62556306
P 1000 2700
F 0 "C33" V 1100 2700 50  0000 C CNN
F 1 "103" V 1150 2700 50  0000 C CNN
F 2 "lc_lib:0805_C" H 1000 2700 50  0001 C CNN
F 3 "~" H 1000 2700 50  0001 C CNN
	1    1000 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	900  2700 800  2700
Connection ~ 800  2700
Wire Wire Line
	800  2700 800  3250
$Comp
L power:GND #PWR?
U 1 1 6255EBCE
P 1200 2800
AR Path="/5E7D94C5/6255EBCE" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/6255EBCE" Ref="#PWR0164"  Part="1" 
F 0 "#PWR0164" H 1200 2550 50  0001 C CNN
F 1 "GND" H 1205 2627 50  0000 C CNN
F 2 "" H 1200 2800 50  0001 C CNN
F 3 "" H 1200 2800 50  0001 C CNN
	1    1200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2800 1200 2700
Wire Wire Line
	1200 2700 1100 2700
$Comp
L Device:D_TVS D2
U 1 1 629824A0
P 1050 3250
F 0 "D2" H 1050 3467 50  0000 C CNN
F 1 "16V" H 1050 3376 50  0000 C CNN
F 2 "lc_lib:SMA(DO-214AC)_S4" H 1050 3250 50  0001 C CNN
F 3 "~" H 1050 3250 50  0001 C CNN
	1    1050 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  3250 800  3250
Connection ~ 800  3250
Wire Wire Line
	800  3250 800  3450
$Comp
L power:GND #PWR?
U 1 1 62987069
P 1250 3400
AR Path="/5E7D94C5/62987069" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/62987069" Ref="#PWR0165"  Part="1" 
F 0 "#PWR0165" H 1250 3150 50  0001 C CNN
F 1 "GND" H 1255 3227 50  0000 C CNN
F 2 "" H 1250 3400 50  0001 C CNN
F 3 "" H 1250 3400 50  0001 C CNN
	1    1250 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3400 1250 3250
Wire Wire Line
	1250 3250 1200 3250
$Comp
L Device:C_Small C?
U 1 1 62A4E345
P 3350 4400
AR Path="/62A4E345" Ref="C?"  Part="1" 
AR Path="/5EA95119/62A4E345" Ref="C34"  Part="1" 
F 0 "C34" H 3400 4300 50  0000 L CNN
F 1 "104" H 3400 4200 50  0000 L CNN
F 2 "lc_lib:0805_C" H 3350 4400 50  0001 C CNN
F 3 "~" H 3350 4400 50  0001 C CNN
	1    3350 4400
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:DTC124E Q?
U 1 1 62A4E34B
P 3050 4200
AR Path="/62A4E34B" Ref="Q?"  Part="1" 
AR Path="/5EA95119/62A4E34B" Ref="Q15"  Part="1" 
F 0 "Q15" H 2750 4400 50  0000 L CNN
F 1 "DTC124E" H 2550 4300 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 3050 4200 50  0001 L CNN
F 3 "" H 3050 4200 50  0001 L CNN
	1    3050 4200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3300 4200 3350 4200
Wire Wire Line
	3350 4300 3350 4200
Connection ~ 3350 4200
Wire Wire Line
	3350 4200 3700 4200
$Comp
L power:GNDD #PWR?
U 1 1 62A4E355
P 3350 4500
AR Path="/62A4E355" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/62A4E355" Ref="#PWR0167"  Part="1" 
F 0 "#PWR0167" H 3350 4250 50  0001 C CNN
F 1 "GNDD" H 3354 4345 50  0000 C CNN
F 2 "" H 3350 4500 50  0001 C CNN
F 3 "" H 3350 4500 50  0001 C CNN
	1    3350 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4400 2950 4500
Wire Wire Line
	2950 4500 3350 4500
Connection ~ 3350 4500
$Comp
L power:+3.3V #PWR0168
U 1 1 62A58D39
P 2950 3550
F 0 "#PWR0168" H 2950 3400 50  0001 C CNN
F 1 "+3.3V" H 2965 3723 50  0000 C CNN
F 2 "" H 2950 3550 50  0001 C CNN
F 3 "" H 2950 3550 50  0001 C CNN
	1    2950 3550
	1    0    0    -1  
$EndComp
Text Label 1550 2800 3    50   ~ 0
PTT_RTS
Wire Wire Line
	2950 3700 2950 3550
$Comp
L Device:R_Small R?
U 1 1 62A4E35B
P 2950 3800
AR Path="/62A4E35B" Ref="R?"  Part="1" 
AR Path="/5EA95119/62A4E35B" Ref="R51"  Part="1" 
F 0 "R51" V 3050 3550 50  0000 L CNN
F 1 "47K" V 3050 3700 50  0000 L CNN
F 2 "lc_lib:0805_R" H 2950 3800 50  0001 C CNN
F 3 "~" H 2950 3800 50  0001 C CNN
	1    2950 3800
	-1   0    0    1   
$EndComp
Text Label 800  3450 3    50   ~ 0
TX_GND
Wire Wire Line
	2950 3950 2950 4000
Wire Wire Line
	2950 3900 2950 3950
Connection ~ 2950 3950
Wire Wire Line
	2200 4000 2200 3950
Wire Wire Line
	2200 4450 2200 4400
$Comp
L power:GNDD #PWR?
U 1 1 62A85E3C
P 2200 4450
AR Path="/62A85E3C" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/62A85E3C" Ref="#PWR0169"  Part="1" 
F 0 "#PWR0169" H 2200 4200 50  0001 C CNN
F 1 "GNDD" H 2204 4295 50  0000 C CNN
F 2 "" H 2200 4450 50  0001 C CNN
F 3 "" H 2200 4450 50  0001 C CNN
	1    2200 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 4200 1850 4200
$Comp
L Transistor_BJT:DTA144E Q13
U 1 1 62A76313
P 2100 4200
F 0 "Q13" H 1750 4300 50  0000 L CNN
F 1 "DTA144E" H 1600 4400 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 2100 4200 50  0001 L CNN
F 3 "" H 2100 4200 50  0001 L CNN
	1    2100 4200
	1    0    0    1   
$EndComp
Text Label 1500 4200 0    50   ~ 0
TX_GND
Wire Wire Line
	2700 3550 2950 3550
Connection ~ 2950 3550
Wire Wire Line
	2350 3750 2250 3750
$Comp
L Transistor_BJT:DTA124E Q14
U 1 1 62AD411F
P 2600 3750
F 0 "Q14" H 2200 3600 50  0000 L CNN
F 1 "DTA124E" H 2050 3700 50  0000 L CNN
F 2 "lc_lib:SOT-23(SOT-23-3)" H 2600 3750 50  0001 L CNN
F 3 "" H 2600 3750 50  0001 L CNN
	1    2600 3750
	1    0    0    1   
$EndComp
Wire Wire Line
	2700 3950 2950 3950
Text HLabel 2100 3750 0    50   Input ~ 0
\PTT_SUPPRESS
$Comp
L Device:R_Small R49
U 1 1 62AE808D
P 2500 3950
F 0 "R49" H 2559 3996 50  0000 L CNN
F 1 "4.7K" H 2559 3905 50  0000 L CNN
F 2 "lc_lib:0805_R" H 2500 3950 50  0001 C CNN
F 3 "~" H 2500 3950 50  0001 C CNN
	1    2500 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 3950 2700 3950
Connection ~ 2700 3950
Wire Wire Line
	2400 3950 2200 3950
Wire Wire Line
	2950 3950 3250 3950
Wire Wire Line
	3250 3950 3250 3700
Wire Wire Line
	3250 3700 3500 3700
Text HLabel 3500 3700 2    50   Output ~ 0
\PTT_OUT
Text Label 2950 2800 3    50   ~ 0
PTT_RTS
Text Label 3700 4200 0    50   ~ 0
PTT_RTS
Text HLabel 2100 3900 0    50   Input ~ 0
\Fatal
Wire Wire Line
	2100 3900 2250 3900
Wire Wire Line
	2250 3900 2250 3750
Connection ~ 2250 3750
Wire Wire Line
	2250 3750 2100 3750
$Comp
L Device:D_TVS D11
U 1 1 617EA0AB
P 2250 2600
F 0 "D11" H 2250 2817 50  0000 C CNN
F 1 "16V" H 2250 2726 50  0000 C CNN
F 2 "lc_lib:SMA(DO-214AC)_S4" H 2250 2600 50  0001 C CNN
F 3 "~" H 2250 2600 50  0001 C CNN
	1    2250 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 2600 1750 2600
Wire Wire Line
	1750 2600 1750 2400
Connection ~ 1750 2400
Wire Wire Line
	2500 2600 2400 2600
$Comp
L Device:D_TVS D16
U 1 1 6182E4B0
P 3700 2600
F 0 "D16" H 3700 2817 50  0000 C CNN
F 1 "16V" H 3700 2726 50  0000 C CNN
F 2 "lc_lib:SMA(DO-214AC)_S4" H 3700 2600 50  0001 C CNN
F 3 "~" H 3700 2600 50  0001 C CNN
	1    3700 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 2600 3150 2600
Wire Wire Line
	3150 2600 3150 2400
Connection ~ 3150 2400
Wire Wire Line
	2750 5050 2850 5050
$Comp
L Device:Ferrite_Bead_Small FB2
U 1 1 5E4A692C
P 2650 5050
F 0 "FB2" V 2413 5050 50  0000 C CNN
F 1 "1K" V 2504 5050 50  0000 C CNN
F 2 "lc_lib:0805_R" V 2580 5050 50  0001 C CNN
F 3 "~" H 2650 5050 50  0001 C CNN
	1    2650 5050
	0    1    1    0   
$EndComp
Text GLabel 2850 5050 2    50   Input ~ 0
SW_PWR
$Comp
L Device:D_TVS D12
U 1 1 61881392
P 2350 5200
F 0 "D12" H 2350 5417 50  0000 C CNN
F 1 "16V" H 2350 5326 50  0000 C CNN
F 2 "lc_lib:SMA(DO-214AC)_S4" H 2350 5200 50  0001 C CNN
F 3 "~" H 2350 5200 50  0001 C CNN
	1    2350 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1900 5050 2350 5050
Connection ~ 2350 5050
Wire Wire Line
	2350 5050 2550 5050
$Comp
L power:GND #PWR?
U 1 1 618A5EAE
P 2350 5350
AR Path="/5E814668/618A5EAE" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/618A5EAE" Ref="#PWR0110"  Part="1" 
F 0 "#PWR0110" H 2350 5100 50  0001 C CNN
F 1 "GND" H 2355 5177 50  0000 C CNN
F 2 "" H 2350 5350 50  0001 C CNN
F 3 "" H 2350 5350 50  0001 C CNN
	1    2350 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2200 3350 3000
Wire Wire Line
	3250 2200 3250 3000
Wire Wire Line
	1950 2200 1950 3200
Wire Wire Line
	1850 2200 1850 3200
$Comp
L power:GND #PWR?
U 1 1 618ED0D1
P 2500 2700
AR Path="/5E7D94C5/618ED0D1" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/618ED0D1" Ref="#PWR0119"  Part="1" 
F 0 "#PWR0119" H 2500 2450 50  0001 C CNN
F 1 "GND" H 2505 2527 50  0000 C CNN
F 2 "" H 2500 2700 50  0001 C CNN
F 3 "" H 2500 2700 50  0001 C CNN
	1    2500 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2700 2500 2600
Wire Wire Line
	3100 6850 3800 6850
Wire Wire Line
	3100 6900 3100 6850
Wire Wire Line
	2650 7200 2750 7200
Wire Wire Line
	2650 7150 2650 7200
$Comp
L power:+3.3V #PWR0127
U 1 1 5E431F98
P 2650 7150
F 0 "#PWR0127" H 2650 7000 50  0001 C CNN
F 1 "+3.3V" H 2665 7323 50  0000 C CNN
F 2 "" H 2650 7150 50  0001 C CNN
F 3 "" H 2650 7150 50  0001 C CNN
	1    2650 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 7200 3100 7200
$Comp
L MySchLib:LED D14
U 1 1 5E42F239
P 3100 7050
F 0 "D14" V 3047 7128 50  0000 L CNN
F 1 "LED" V 3138 7128 50  0000 L CNN
F 2 "lc_lib:0805_LED_S1" H 3100 7050 50  0001 C CNN
F 3 "~" H 3100 7050 50  0001 C CNN
	1    3100 7050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R61
U 1 1 5F387D6B
P 2850 7200
F 0 "R61" H 2950 7050 50  0000 L CNN
F 1 "1K" H 2950 7150 50  0000 L CNN
F 2 "lc_lib:0805_R" H 2850 7200 50  0001 C CNN
F 3 "~" H 2850 7200 50  0001 C CNN
	1    2850 7200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6230A516
P 4000 2700
AR Path="/5E7D94C5/6230A516" Ref="#PWR?"  Part="1" 
AR Path="/5EA95119/6230A516" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4000 2450 50  0001 C CNN
F 1 "GND" H 4005 2527 50  0000 C CNN
F 2 "" H 4000 2700 50  0001 C CNN
F 3 "" H 4000 2700 50  0001 C CNN
	1    4000 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2700 4000 2600
Wire Wire Line
	3850 2600 4000 2600
$EndSCHEMATC
