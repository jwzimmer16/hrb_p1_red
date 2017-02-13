
# 
# WARNING: this file was automatically generated using pictopy.py
#
# Python interface generate from P18F2680.INC
#



#=========================================================================
#  MPASM PIC18F2680 processor includ
#
#  (c) Copyright 1999-2007 Microchip Technology, All rights reserve
#=========================================================================


#=========================================================================
#  This header file defines configurations, registers, and other usefu
#  bits of information for the PIC18F2680 microcontroller.  These name
#  are taken to match the data sheets as closely as possible
#
#  Note that the processor must be selected before this file is included
#  The processor may be selected the following ways
#
#       1. Command line switch
#               C:\MPASM MYFILE.ASM /PIC18F268
#       2. LIST directive in the source fil
#               LIST   P=PIC18F268
#       3. Processor Type entry in the MPASM full-screen interfac
#       4. Setting the processor in the MPLAB Project Dialo
#=========================================================================

#=========================================================================
#
#       Verify Processo
#
#=========================================================================

#=========================================================================
#       18xxxx Family        EQUate
#=========================================================================


#=========================================================================

#=========================================================================
#       16Cxxx/17Cxxx Substitution
#=========================================================================

#=========================================================================
#
#       Register Definition
#
#=========================================================================

#----- Register Files ----------------------------------------------------
RXF6SIDH = 0x0D60
RXF6SIDL = 0x0D61
RXF6EIDH = 0x0D62
RXF6EIDL = 0x0D63
RXF7SIDH = 0x0D64
RXF7SIDL = 0x0D65
RXF7EIDH = 0x0D66
RXF7EIDL = 0x0D67
RXF8SIDH = 0x0D68
RXF8SIDL = 0x0D69
RXF8EIDH = 0x0D6A
RXF8EIDL = 0x0D6B
RXF9SIDH = 0x0D70
RXF9SIDL = 0x0D71
RXF9EIDH = 0x0D72
RXF9EIDL = 0x0D73
RXF10SIDH = 0x0D74
RXF10SIDL = 0x0D75
RXF10EIDH = 0x0D76
RXF10EIDL = 0x0D77
RXF11SIDH = 0x0D78
RXF11SIDL = 0x0D79
RXF11EIDH = 0x0D7A
RXF11EIDL = 0x0D7B
RXF12SIDH = 0x0D80
RXF12SIDL = 0x0D81
RXF12EIDH = 0x0D82
RXF12EIDL = 0x0D83
RXF13SIDH = 0x0D84
RXF13SIDL = 0x0D85
RXF13EIDH = 0x0D86
RXF13EIDL = 0x0D87
RXF14SIDH = 0x0D88
RXF14SIDL = 0x0D89
RXF14EIDH = 0x0D8A
RXF14EIDL = 0x0D8B
RXF15SIDH = 0x0D90
RXF15SIDL = 0x0D91
RXF15EIDH = 0x0D92
RXF15EIDL = 0x0D93
RXFCON0 = 0x0DD4
RXFCON1 = 0x0DD5
SDFLC = 0x0DD8
RXFBCON0 = 0x0DE0
RXFBCON1 = 0x0DE1
RXFBCON2 = 0x0DE2
RXFBCON3 = 0x0DE3
RXFBCON4 = 0x0DE4
RXFBCON5 = 0x0DE5
RXFBCON6 = 0x0DE6
RXFBCON7 = 0x0DE7
MSEL0 = 0x0DF0
MSEL1 = 0x0DF1
MSEL2 = 0x0DF2
MSEL3 = 0x0DF3
BSEL0 = 0x0DF8
BIE0 = 0x0DFA
TXBIE = 0x0DFC
B0CON = 0x0E20
B0SIDH = 0x0E21
B0SIDL = 0x0E22
B0EIDH = 0x0E23
B0EIDL = 0x0E24
B0DLC = 0x0E25
B0D0 = 0x0E26
B0D1 = 0x0E27
B0D2 = 0x0E28
B0D3 = 0x0E29
B0D4 = 0x0E2A
B0D5 = 0x0E2B
B0D6 = 0x0E2C
B0D7 = 0x0E2D
CANSTAT_RO9 = 0x0E2E
CANCON_RO9 = 0x0E2F
B1CON = 0x0E30
B1SIDH = 0x0E31
B1SIDL = 0x0E32
B1EIDH = 0x0E33
B1EIDL = 0x0E34
B1DLC = 0x0E35
B1D0 = 0x0E36
B1D1 = 0x0E37
B1D2 = 0x0E38
B1D3 = 0x0E39
B1D4 = 0x0E3A
B1D5 = 0x0E3B
B1D6 = 0x0E3C
B1D7 = 0x0E3D
CANSTAT_RO8 = 0x0E3E
CANCON_RO8 = 0x0E3F
B2CON = 0x0E40
B2SIDH = 0x0E41
B2SIDL = 0x0E42
B2EIDH = 0x0E43
B2EIDL = 0x0E44
B2DLC = 0x0E45
B2D0 = 0x0E46
B2D1 = 0x0E47
B2D2 = 0x0E48
B2D3 = 0x0E49
B2D4 = 0x0E4A
B2D5 = 0x0E4B
B2D6 = 0x0E4C
B2D7 = 0x0E4D
CANSTAT_RO7 = 0x0E4E
CANCON_RO7 = 0x0E4F
B3CON = 0x0E50
B3SIDH = 0x0E51
B3SIDL = 0x0E52
B3EIDH = 0x0E53
B3EIDL = 0x0E54
B3DLC = 0x0E55
B3D0 = 0x0E56
B3D1 = 0x0E57
B3D2 = 0x0E58
B3D3 = 0x0E59
B3D4 = 0x0E5A
B3D5 = 0x0E5B
B3D6 = 0x0E5C
B3D7 = 0x0E5D
CANSTAT_RO6 = 0x0E5E
CANCON_RO6 = 0x0E5F
B4CON = 0x0E60
B4SIDH = 0x0E61
B4SIDL = 0x0E62
B4EIDH = 0x0E63
B4EIDL = 0x0E64
B4DLC = 0x0E65
B4D0 = 0x0E66
B4D1 = 0x0E67
B4D2 = 0x0E68
B4D3 = 0x0E69
B4D4 = 0x0E6A
B4D5 = 0x0E6B
B4D6 = 0x0E6C
B4D7 = 0x0E6D
CANSTAT_RO5 = 0x0E6E
CANCON_RO5 = 0x0E6F
B5CON = 0x0E70
B5SIDH = 0x0E71
B5SIDL = 0x0E72
B5EIDH = 0x0E73
B5EIDL = 0x0E74
B5DLC = 0x0E75
B5D0 = 0x0E76
B5D1 = 0x0E77
B5D2 = 0x0E78
B5D3 = 0x0E79
B5D4 = 0x0E7A
B5D5 = 0x0E7B
B5D6 = 0x0E7C
B5D7 = 0x0E7D
CANSTAT_RO4 = 0x0E7E
CANCON_RO4 = 0x0E7F
RXF0SIDH = 0x0F00
RXF0SIDL = 0x0F01
RXF0EIDH = 0x0F02
RXF0EIDL = 0x0F03
RXF1SIDH = 0x0F04
RXF1SIDL = 0x0F05
RXF1EIDH = 0x0F06
RXF1EIDL = 0x0F07
RXF2SIDH = 0x0F08
RXF2SIDL = 0x0F09
RXF2EIDH = 0x0F0A
RXF2EIDL = 0x0F0B
RXF3SIDH = 0x0F0C
RXF3SIDL = 0x0F0D
RXF3EIDH = 0x0F0E
RXF3EIDL = 0x0F0F
RXF4SIDH = 0x0F10
RXF4SIDL = 0x0F11
RXF4EIDH = 0x0F12
RXF4EIDL = 0x0F13
RXF5SIDH = 0x0F14
RXF5SIDL = 0x0F15
RXF5EIDH = 0x0F16
RXF5EIDL = 0x0F17
RXM0SIDH = 0x0F18
RXM0SIDL = 0x0F19
RXM0EIDH = 0x0F1A
RXM0EIDL = 0x0F1B
RXM1SIDH = 0x0F1C
RXM1SIDL = 0x0F1D
RXM1EIDH = 0x0F1E
RXM1EIDL = 0x0F1F
TXB2CON = 0x0F20
TXB2SIDH = 0x0F21
TXB2SIDL = 0x0F22
TXB2EIDH = 0x0F23
TXB2EIDL = 0x0F24
TXB2DLC = 0x0F25
TXB2D0 = 0x0F26
TXB2D1 = 0x0F27
TXB2D2 = 0x0F28
TXB2D3 = 0x0F29
TXB2D4 = 0x0F2A
TXB2D5 = 0x0F2B
TXB2D6 = 0x0F2C
TXB2D7 = 0x0F2D
CANSTAT_RO3 = 0x0F2E
CANCON_RO3 = 0x0F2F
TXB1CON = 0x0F30
TXB1SIDH = 0x0F31
TXB1SIDL = 0x0F32
TXB1EIDH = 0x0F33
TXB1EIDL = 0x0F34
TXB1DLC = 0x0F35
TXB1D0 = 0x0F36
TXB1D1 = 0x0F37
TXB1D2 = 0x0F38
TXB1D3 = 0x0F39
TXB1D4 = 0x0F3A
TXB1D5 = 0x0F3B
TXB1D6 = 0x0F3C
TXB1D7 = 0x0F3D
CANSTAT_RO2 = 0x0F3E
CANCON_RO2 = 0x0F3F
TXB0CON = 0x0F40
TXB0SIDH = 0x0F41
TXB0SIDL = 0x0F42
TXB0EIDH = 0x0F43
TXB0EIDL = 0x0F44
TXB0DLC = 0x0F45
TXB0D0 = 0x0F46
TXB0D1 = 0x0F47
TXB0D2 = 0x0F48
TXB0D3 = 0x0F49
TXB0D4 = 0x0F4A
TXB0D5 = 0x0F4B
TXB0D6 = 0x0F4C
TXB0D7 = 0x0F4D
CANSTAT_RO1 = 0x0F4E
CANCON_RO1 = 0x0F4F
RXB1CON = 0x0F50
RXB1SIDH = 0x0F51
RXB1SIDL = 0x0F52
RXB1EIDH = 0x0F53
RXB1EIDL = 0x0F54
RXB1DLC = 0x0F55
RXB1D0 = 0x0F56
RXB1D1 = 0x0F57
RXB1D2 = 0x0F58
RXB1D3 = 0x0F59
RXB1D4 = 0x0F5A
RXB1D5 = 0x0F5B
RXB1D6 = 0x0F5C
RXB1D7 = 0x0F5D
CANSTAT_RO0 = 0x0F5E
CANCON_RO0 = 0x0F5F
RXB0CON = 0x0F60
RXB0SIDH = 0x0F61
RXB0SIDL = 0x0F62
RXB0EIDH = 0x0F63
RXB0EIDL = 0x0F64
RXB0DLC = 0x0F65
RXB0D0 = 0x0F66
RXB0D1 = 0x0F67
RXB0D2 = 0x0F68
RXB0D3 = 0x0F69
RXB0D4 = 0x0F6A
RXB0D5 = 0x0F6B
RXB0D6 = 0x0F6C
RXB0D7 = 0x0F6D
CANSTAT = 0x0F6E
CANCON = 0x0F6F
BRGCON1 = 0x0F70
BRGCON2 = 0x0F71
BRGCON3 = 0x0F72
CIOCON = 0x0F73
COMSTAT = 0x0F74
RXERRCNT = 0x0F75
TXERRCNT = 0x0F76
ECANCON = 0x0F77
PORTA = 0x0F80
PORTB = 0x0F81
PORTC = 0x0F82
PORTE = 0x0F84
LATA = 0x0F89
LATB = 0x0F8A
LATC = 0x0F8B
DDRA = 0x0F92
TRISA = 0x0F92
DDRB = 0x0F93
TRISB = 0x0F93
DDRC = 0x0F94
TRISC = 0x0F94
OSCTUNE = 0x0F9B
PIE1 = 0x0F9D
PIR1 = 0x0F9E
IPR1 = 0x0F9F
PIE2 = 0x0FA0
PIR2 = 0x0FA1
IPR2 = 0x0FA2
PIE3 = 0x0FA3
PIR3 = 0x0FA4
IPR3 = 0x0FA5
EECON1 = 0x0FA6
EECON2 = 0x0FA7
EEDATA = 0x0FA8
EEADR = 0x0FA9
EEADRH = 0x0FAA
RCSTA = 0x0FAB
TXSTA = 0x0FAC
TXREG = 0x0FAD
RCREG = 0x0FAE
SPBRG = 0x0FAF
SPBRGH = 0x0FB0
T3CON = 0x0FB1
TMR3L = 0x0FB2
TMR3H = 0x0FB3
BAUDCON = 0x0FB8
CCP1CON = 0x0FBD
CCPR1 = 0x0FBE
CCPR1L = 0x0FBE
CCPR1H = 0x0FBF
ADCON2 = 0x0FC0
ADCON1 = 0x0FC1
ADCON0 = 0x0FC2
ADRES = 0x0FC3
ADRESL = 0x0FC3
ADRESH = 0x0FC4
SSPCON2 = 0x0FC5
SSPCON1 = 0x0FC6
SSPSTAT = 0x0FC7
SSPADD = 0x0FC8
SSPBUF = 0x0FC9
T2CON = 0x0FCA
PR2 = 0x0FCB
TMR2 = 0x0FCC
T1CON = 0x0FCD
TMR1L = 0x0FCE
TMR1H = 0x0FCF
RCON = 0x0FD0
WDTCON = 0x0FD1
HLVDCON = 0x0FD2
LVDCON = 0x0FD2
OSCCON = 0x0FD3
T0CON = 0x0FD5
TMR0L = 0x0FD6
TMR0H = 0x0FD7
STATUS = 0x0FD8
FSR2L = 0x0FD9
FSR2H = 0x0FDA
PLUSW2 = 0x0FDB
PREINC2 = 0x0FDC
POSTDEC2 = 0x0FDD
POSTINC2 = 0x0FDE
INDF2 = 0x0FDF
BSR = 0x0FE0
FSR1L = 0x0FE1
FSR1H = 0x0FE2
PLUSW1 = 0x0FE3
PREINC1 = 0x0FE4
POSTDEC1 = 0x0FE5
POSTINC1 = 0x0FE6
INDF1 = 0x0FE7
WREG = 0x0FE8
FSR0L = 0x0FE9
FSR0H = 0x0FEA
PLUSW0 = 0x0FEB
PREINC0 = 0x0FEC
POSTDEC0 = 0x0FED
POSTINC0 = 0x0FEE
INDF0 = 0x0FEF
INTCON3 = 0x0FF0
INTCON2 = 0x0FF1
INTCON = 0x0FF2
PROD = 0x0FF3
PRODL = 0x0FF3
PRODH = 0x0FF4
TABLAT = 0x0FF5
TBLPTR = 0x0FF6
TBLPTRL = 0x0FF6
TBLPTRH = 0x0FF7
TBLPTRU = 0x0FF8
PC = 0x0FF9
PCL = 0x0FF9
PCLATH = 0x0FFA
PCLATU = 0x0FFB
STKPTR = 0x0FFC
TOS = 0x0FFD
TOSL = 0x0FFD
TOSH = 0x0FFE
TOSU = 0x0FFF

#;----- RXF6SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF6SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF6EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF6EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF7SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF7SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF7EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF7EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF8SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF8SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF8EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF8EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF9SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF9SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF9EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF9EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF10SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF10SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF10EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF10EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF11SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF11SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF11EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF11EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF12SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF12SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF12EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF12EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF13SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF13SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF13EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF13EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF14SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF14SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF14EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF14EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF15SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF15SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF15EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF15EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXFCON0 Bits -----------------------------------------------------
RXF0EN_BIT = 0x0000
RXF1EN_BIT = 0x0001
RXF2EN_BIT = 0x0002
RXF3EN_BIT = 0x0003
RXF4EN_BIT = 0x0004
RXF5EN_BIT = 0x0005
RXF6EN_BIT = 0x0006
RXF7EN_BIT = 0x0007


#;----- RXFCON1 Bits -----------------------------------------------------
RXF8EN_BIT = 0x0000
RXF9EN_BIT = 0x0001
RXF10EN_BIT = 0x0002
RXF11EN_BIT = 0x0003
RXF12EN_BIT = 0x0004
RXF13EN_BIT = 0x0005
RXF14EN_BIT = 0x0006
RXF15EN_BIT = 0x0007


#;----- SDFLC Bits -----------------------------------------------------
DFLC0_BIT = 0x0000
DFLC1_BIT = 0x0001
DFLC2_BIT = 0x0002
DFLC3_BIT = 0x0003
DFLC4_BIT = 0x0004

FLC0_BIT = 0x0000
FLC1_BIT = 0x0001
FLC2_BIT = 0x0002
FLC3_BIT = 0x0003
FLC4_BIT = 0x0004


#;----- RXFBCON0 Bits -----------------------------------------------------
F0BP_0_BIT = 0x0000
F0BP_1_BIT = 0x0001
F0BP_2_BIT = 0x0002
F0BP_3_BIT = 0x0003
F1BP_0_BIT = 0x0004
F1BP_1_BIT = 0x0005
F1BP_2_BIT = 0x0006
F1BP_3_BIT = 0x0007


#;----- RXFBCON1 Bits -----------------------------------------------------
F2BP_0_BIT = 0x0000
F2BP_1_BIT = 0x0001
F2BP_2_BIT = 0x0002
F2BP_3_BIT = 0x0003
F3BP_0_BIT = 0x0004
F3BP_1_BIT = 0x0005
F3BP_2_BIT = 0x0006
F3BP_3_BIT = 0x0007


#;----- RXFBCON2 Bits -----------------------------------------------------
F4BP_0_BIT = 0x0000
F4BP_1_BIT = 0x0001
F4BP_2_BIT = 0x0002
F4BP_3_BIT = 0x0003
F5BP_0_BIT = 0x0004
F5BP_1_BIT = 0x0005
F5BP_2_BIT = 0x0006
F5BP_3_BIT = 0x0007


#;----- RXFBCON3 Bits -----------------------------------------------------
F6BP_0_BIT = 0x0000
F6BP_1_BIT = 0x0001
F6BP_2_BIT = 0x0002
F6BP_3_BIT = 0x0003
F7BP_0_BIT = 0x0004
F7BP_1_BIT = 0x0005
F7BP_2_BIT = 0x0006
F7BP_3_BIT = 0x0007


#;----- RXFBCON4 Bits -----------------------------------------------------
F8BP_0_BIT = 0x0000
F8BP_1_BIT = 0x0001
F8BP_2_BIT = 0x0002
F8BP_3_BIT = 0x0003
F9BP_0_BIT = 0x0004
F9BP_1_BIT = 0x0005
F9BP_2_BIT = 0x0006
F9BP_3_BIT = 0x0007


#;----- RXFBCON5 Bits -----------------------------------------------------
F10BP_0_BIT = 0x0000
F10BP_1_BIT = 0x0001
F10BP_2_BIT = 0x0002
F10BP_3_BIT = 0x0003
F11BP_0_BIT = 0x0004
F11BP_1_BIT = 0x0005
F11BP_2_BIT = 0x0006
F11BP_3_BIT = 0x0007


#;----- RXFBCON6 Bits -----------------------------------------------------
F12BP_0_BIT = 0x0000
F12BP_1_BIT = 0x0001
F12BP_2_BIT = 0x0002
F12BP_3_BIT = 0x0003
F13BP_0_BIT = 0x0004
F13BP_1_BIT = 0x0005
F13BP_2_BIT = 0x0006
F13BP_3_BIT = 0x0007


#;----- RXFBCON7 Bits -----------------------------------------------------
F14BP_0_BIT = 0x0000
F14BP_1_BIT = 0x0001
F14BP_2_BIT = 0x0002
F14BP_3_BIT = 0x0003
F15BP_0_BIT = 0x0004
F15BP_1_BIT = 0x0005
F15BP_2_BIT = 0x0006
F15BP_3_BIT = 0x0007


#;----- MSEL0 Bits -----------------------------------------------------
FIL0_0_BIT = 0x0000
FIL0_1_BIT = 0x0001
FIL1_0_BIT = 0x0002
FIL1_1_BIT = 0x0003
FIL2_0_BIT = 0x0004
FIL2_1_BIT = 0x0005
FIL3_0_BIT = 0x0006
FIL3_1_BIT = 0x0007


#;----- MSEL1 Bits -----------------------------------------------------
FIL4_0_BIT = 0x0000
FIL4_1_BIT = 0x0001
FIL5_0_BIT = 0x0002
FIL5_1_BIT = 0x0003
FIL6_0_BIT = 0x0004
FIL6_1_BIT = 0x0005
FIL7_0_BIT = 0x0006
FIL7_1_BIT = 0x0007


#;----- MSEL2 Bits -----------------------------------------------------
FIL8_0_BIT = 0x0000
FIL8_1_BIT = 0x0001
FIL9_0_BIT = 0x0002
FIL9_1_BIT = 0x0003
FIL10_0_BIT = 0x0004
FIL10_1_BIT = 0x0005
FIL11_0_BIT = 0x0006
FIL11_1_BIT = 0x0007


#;----- MSEL3 Bits -----------------------------------------------------
FIL12_0_BIT = 0x0000
FIL12_1_BIT = 0x0001
FIL13_0_BIT = 0x0002
FIL13_1_BIT = 0x0003
FIL14_0_BIT = 0x0004
FIL14_1_BIT = 0x0005
FIL15_0_BIT = 0x0006
FIL15_1_BIT = 0x0007


#;----- BSEL0 Bits -----------------------------------------------------
B0TXEN_BIT = 0x0002
B1TXEN_BIT = 0x0003
B2TXEN_BIT = 0x0004
B3TXEN_BIT = 0x0005
B4TXEN_BIT = 0x0006
B5TXEN_BIT = 0x0007


#;----- BIE0 Bits -----------------------------------------------------
RXB0IE_BIT = 0x0000
RXB1IE_BIT = 0x0001
B0IE_BIT = 0x0002
B1IE_BIT = 0x0003
B2IE_BIT = 0x0004
B3IE_BIT = 0x0005
B4IE_BIT = 0x0006
B5IE_BIT = 0x0007


#;----- TXBIE Bits -----------------------------------------------------
TXB0IE_BIT = 0x0002
TXB1IE_BIT = 0x0003
TXB2IE_BIT = 0x0004


#;----- B0CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B0CON_BIT = 0x0005


#;----- B0SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B0SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDE_BIT = 0x0003


#;----- B0EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B0EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B0DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

TXRTR_BIT = 0x0006

RB0_B0DLC_BIT = 0x0004
RB1_B0DLC_BIT = 0x0005


#;----- B0D0 Bits -----------------------------------------------------
B0D00_BIT = 0x0000
B0D01_BIT = 0x0001
B0D02_BIT = 0x0002
B0D03_BIT = 0x0003
B0D04_BIT = 0x0004
B0D05_BIT = 0x0005
B0D06_BIT = 0x0006
B0D07_BIT = 0x0007


#;----- B0D1 Bits -----------------------------------------------------
B0D10_BIT = 0x0000
B0D11_BIT = 0x0001
B0D12_BIT = 0x0002
B0D13_BIT = 0x0003
B0D14_BIT = 0x0004
B0D15_BIT = 0x0005
B0D16_BIT = 0x0006
B0D17_BIT = 0x0007


#;----- B0D2 Bits -----------------------------------------------------
B0D20_BIT = 0x0000
B0D21_BIT = 0x0001
B0D22_BIT = 0x0002
B0D23_BIT = 0x0003
B0D24_BIT = 0x0004
B0D25_BIT = 0x0005
B0D26_BIT = 0x0006
B0D27_BIT = 0x0007


#;----- B0D3 Bits -----------------------------------------------------
B0D30_BIT = 0x0000
B0D31_BIT = 0x0001
B0D32_BIT = 0x0002
B0D33_BIT = 0x0003
B0D34_BIT = 0x0004
B0D35_BIT = 0x0005
B0D36_BIT = 0x0006
B0D37_BIT = 0x0007


#;----- B0D4 Bits -----------------------------------------------------
B0D40_BIT = 0x0000
B0D41_BIT = 0x0001
B0D42_BIT = 0x0002
B0D43_BIT = 0x0003
B0D44_BIT = 0x0004
B0D45_BIT = 0x0005
B0D46_BIT = 0x0006
B0D47_BIT = 0x0007


#;----- B0D5 Bits -----------------------------------------------------
B0D50_BIT = 0x0000
B0D51_BIT = 0x0001
B0D52_BIT = 0x0002
B0D53_BIT = 0x0003
B0D54_BIT = 0x0004
B0D55_BIT = 0x0005
B0D56_BIT = 0x0006
B0D57_BIT = 0x0007


#;----- B0D6 Bits -----------------------------------------------------
B0D60_BIT = 0x0000
B0D61_BIT = 0x0001
B0D62_BIT = 0x0002
B0D63_BIT = 0x0003
B0D64_BIT = 0x0004
B0D65_BIT = 0x0005
B0D66_BIT = 0x0006
B0D67_BIT = 0x0007


#;----- B0D7 Bits -----------------------------------------------------
B0D70_BIT = 0x0000
B0D71_BIT = 0x0001
B0D72_BIT = 0x0002
B0D73_BIT = 0x0003
B0D74_BIT = 0x0004
B0D75_BIT = 0x0005
B0D76_BIT = 0x0006
B0D77_BIT = 0x0007


#;----- CANSTAT_RO9 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO9 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- B1CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B1CON_BIT = 0x0005


#;----- B1SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B1SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDE_BIT = 0x0003


#;----- B1EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B1EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B1DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

RB0_B1DLC_BIT = 0x0004
RB1_B1DLC_BIT = 0x0005

TXRTR_BIT = 0x0006


#;----- B1D0 Bits -----------------------------------------------------
B1D00_BIT = 0x0000
B1D01_BIT = 0x0001
B1D02_BIT = 0x0002
B1D03_BIT = 0x0003
B1D04_BIT = 0x0004
B1D05_BIT = 0x0005
B1D06_BIT = 0x0006
B1D07_BIT = 0x0007


#;----- B1D1 Bits -----------------------------------------------------
B1D10_BIT = 0x0000
B1D11_BIT = 0x0001
B1D12_BIT = 0x0002
B1D13_BIT = 0x0003
B1D14_BIT = 0x0004
B1D15_BIT = 0x0005
B1D16_BIT = 0x0006
B1D17_BIT = 0x0007


#;----- B1D2 Bits -----------------------------------------------------
B1D20_BIT = 0x0000
B1D21_BIT = 0x0001
B1D22_BIT = 0x0002
B1D23_BIT = 0x0003
B1D24_BIT = 0x0004
B1D25_BIT = 0x0005
B1D26_BIT = 0x0006
B1D27_BIT = 0x0007


#;----- B1D3 Bits -----------------------------------------------------
B1D30_BIT = 0x0000
B1D31_BIT = 0x0001
B1D32_BIT = 0x0002
B1D33_BIT = 0x0003
B1D34_BIT = 0x0004
B1D35_BIT = 0x0005
B1D36_BIT = 0x0006
B1D37_BIT = 0x0007


#;----- B1D4 Bits -----------------------------------------------------
B1D40_BIT = 0x0000
B1D41_BIT = 0x0001
B1D42_BIT = 0x0002
B1D43_BIT = 0x0003
B1D44_BIT = 0x0004
B1D45_BIT = 0x0005
B1D46_BIT = 0x0006
B1D47_BIT = 0x0007


#;----- B1D5 Bits -----------------------------------------------------
B1D50_BIT = 0x0000
B1D51_BIT = 0x0001
B1D52_BIT = 0x0002
B1D53_BIT = 0x0003
B1D54_BIT = 0x0004
B1D55_BIT = 0x0005
B1D56_BIT = 0x0006
B1D57_BIT = 0x0007


#;----- B1D6 Bits -----------------------------------------------------
B1D60_BIT = 0x0000
B1D61_BIT = 0x0001
B1D62_BIT = 0x0002
B1D63_BIT = 0x0003
B1D64_BIT = 0x0004
B1D65_BIT = 0x0005
B1D66_BIT = 0x0006
B1D67_BIT = 0x0007


#;----- B1D7 Bits -----------------------------------------------------
B1D70_BIT = 0x0000
B1D71_BIT = 0x0001
B1D72_BIT = 0x0002
B1D73_BIT = 0x0003
B1D74_BIT = 0x0004
B1D75_BIT = 0x0005
B1D76_BIT = 0x0006
B1D77_BIT = 0x0007


#;----- CANSTAT_RO8 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO8 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- B2CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B2CON_BIT = 0x0005


#;----- B2SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B2SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDE_BIT = 0x0003


#;----- B2EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B2EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B2DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

TXRTR_BIT = 0x0006

RB0_B2DLC_BIT = 0x0004
RB1_B2DLC_BIT = 0x0005


#;----- B2D0 Bits -----------------------------------------------------
B2D00_BIT = 0x0000
B2D01_BIT = 0x0001
B2D02_BIT = 0x0002
B2D03_BIT = 0x0003
B2D04_BIT = 0x0004
B2D05_BIT = 0x0005
B2D06_BIT = 0x0006
B2D07_BIT = 0x0007


#;----- B2D1 Bits -----------------------------------------------------
B2D10_BIT = 0x0000
B2D11_BIT = 0x0001
B2D12_BIT = 0x0002
B2D13_BIT = 0x0003
B2D14_BIT = 0x0004
B2D15_BIT = 0x0005
B2D16_BIT = 0x0006
B2D17_BIT = 0x0007


#;----- B2D2 Bits -----------------------------------------------------
B2D20_BIT = 0x0000
B2D21_BIT = 0x0001
B2D22_BIT = 0x0002
B2D23_BIT = 0x0003
B2D24_BIT = 0x0004
B2D25_BIT = 0x0005
B2D26_BIT = 0x0006
B2D27_BIT = 0x0007


#;----- B2D3 Bits -----------------------------------------------------
B2D30_BIT = 0x0000
B2D31_BIT = 0x0001
B2D32_BIT = 0x0002
B2D33_BIT = 0x0003
B2D34_BIT = 0x0004
B2D35_BIT = 0x0005
B2D36_BIT = 0x0006
B2D37_BIT = 0x0007


#;----- B2D4 Bits -----------------------------------------------------
B2D40_BIT = 0x0000
B2D41_BIT = 0x0001
B2D42_BIT = 0x0002
B2D43_BIT = 0x0003
B2D44_BIT = 0x0004
B2D45_BIT = 0x0005
B2D46_BIT = 0x0006
B2D47_BIT = 0x0007


#;----- B2D5 Bits -----------------------------------------------------
B2D50_BIT = 0x0000
B2D51_BIT = 0x0001
B2D52_BIT = 0x0002
B2D53_BIT = 0x0003
B2D54_BIT = 0x0004
B2D55_BIT = 0x0005
B2D56_BIT = 0x0006
B2D57_BIT = 0x0007


#;----- B2D6 Bits -----------------------------------------------------
B2D60_BIT = 0x0000
B2D61_BIT = 0x0001
B2D62_BIT = 0x0002
B2D63_BIT = 0x0003
B2D64_BIT = 0x0004
B2D65_BIT = 0x0005
B2D66_BIT = 0x0006
B2D67_BIT = 0x0007


#;----- B2D7 Bits -----------------------------------------------------
B2D70_BIT = 0x0000
B2D71_BIT = 0x0001
B2D72_BIT = 0x0002
B2D73_BIT = 0x0003
B2D74_BIT = 0x0004
B2D75_BIT = 0x0005
B2D76_BIT = 0x0006
B2D77_BIT = 0x0007


#;----- CANSTAT_RO7 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO7 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- B3CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B3CON_BIT = 0x0005


#;----- B3SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B3SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDE_BIT = 0x0003


#;----- B3EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B3EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B3DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

TXRTR_BIT = 0x0006

RB0_B3DLC_BIT = 0x0004
RB1_B3DLC_BIT = 0x0005


#;----- B3D0 Bits -----------------------------------------------------
B3D00_BIT = 0x0000
B3D01_BIT = 0x0001
B3D02_BIT = 0x0002
B3D03_BIT = 0x0003
B3D04_BIT = 0x0004
B3D05_BIT = 0x0005
B3D06_BIT = 0x0006
B3D07_BIT = 0x0007


#;----- B3D1 Bits -----------------------------------------------------
B3D10_BIT = 0x0000
B3D11_BIT = 0x0001
B3D12_BIT = 0x0002
B3D13_BIT = 0x0003
B3D14_BIT = 0x0004
B3D15_BIT = 0x0005
B3D16_BIT = 0x0006
B3D17_BIT = 0x0007


#;----- B3D2 Bits -----------------------------------------------------
B3D20_BIT = 0x0000
B3D21_BIT = 0x0001
B3D22_BIT = 0x0002
B3D23_BIT = 0x0003
B3D24_BIT = 0x0004
B3D25_BIT = 0x0005
B3D26_BIT = 0x0006
B3D27_BIT = 0x0007


#;----- B3D3 Bits -----------------------------------------------------
B3D30_BIT = 0x0000
B3D31_BIT = 0x0001
B3D32_BIT = 0x0002
B3D33_BIT = 0x0003
B3D34_BIT = 0x0004
B3D35_BIT = 0x0005
B3D36_BIT = 0x0006
B3D37_BIT = 0x0007


#;----- B3D4 Bits -----------------------------------------------------
B3D40_BIT = 0x0000
B3D41_BIT = 0x0001
B3D42_BIT = 0x0002
B3D43_BIT = 0x0003
B3D44_BIT = 0x0004
B3D45_BIT = 0x0005
B3D46_BIT = 0x0006
B3D47_BIT = 0x0007


#;----- B3D5 Bits -----------------------------------------------------
B3D50_BIT = 0x0000
B3D51_BIT = 0x0001
B3D52_BIT = 0x0002
B3D53_BIT = 0x0003
B3D54_BIT = 0x0004
B3D55_BIT = 0x0005
B3D56_BIT = 0x0006
B3D57_BIT = 0x0007


#;----- B3D6 Bits -----------------------------------------------------
B3D60_BIT = 0x0000
B3D61_BIT = 0x0001
B3D62_BIT = 0x0002
B3D63_BIT = 0x0003
B3D64_BIT = 0x0004
B3D65_BIT = 0x0005
B3D66_BIT = 0x0006
B3D67_BIT = 0x0007


#;----- B3D7 Bits -----------------------------------------------------
B3D70_BIT = 0x0000
B3D71_BIT = 0x0001
B3D72_BIT = 0x0002
B3D73_BIT = 0x0003
B3D74_BIT = 0x0004
B3D75_BIT = 0x0005
B3D76_BIT = 0x0006
B3D77_BIT = 0x0007


#;----- CANSTAT_RO6 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO6 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- B4CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B4CON_BIT = 0x0005


#;----- B4SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B4SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDE_BIT = 0x0003


#;----- B4EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B4EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B4DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

TXRTR_BIT = 0x0006

RB0_B4DLC_BIT = 0x0004
RB1_B4DLC_BIT = 0x0005


#;----- B4D0 Bits -----------------------------------------------------
B4D00_BIT = 0x0000
B4D01_BIT = 0x0001
B4D02_BIT = 0x0002
B4D03_BIT = 0x0003
B4D04_BIT = 0x0004
B4D05_BIT = 0x0005
B4D06_BIT = 0x0006
B4D07_BIT = 0x0007


#;----- B4D1 Bits -----------------------------------------------------
B4D10_BIT = 0x0000
B4D11_BIT = 0x0001
B4D12_BIT = 0x0002
B4D13_BIT = 0x0003
B4D14_BIT = 0x0004
B4D15_BIT = 0x0005
B4D16_BIT = 0x0006
B4D17_BIT = 0x0007


#;----- B4D2 Bits -----------------------------------------------------
B4D20_BIT = 0x0000
B4D21_BIT = 0x0001
B4D22_BIT = 0x0002
B4D23_BIT = 0x0003
B4D24_BIT = 0x0004
B4D25_BIT = 0x0005
B4D26_BIT = 0x0006
B4D27_BIT = 0x0007


#;----- B4D3 Bits -----------------------------------------------------
B4D30_BIT = 0x0000
B4D31_BIT = 0x0001
B4D32_BIT = 0x0002
B4D33_BIT = 0x0003
B4D34_BIT = 0x0004
B4D35_BIT = 0x0005
B4D36_BIT = 0x0006
B4D37_BIT = 0x0007


#;----- B4D4 Bits -----------------------------------------------------
B4D40_BIT = 0x0000
B4D41_BIT = 0x0001
B4D42_BIT = 0x0002
B4D43_BIT = 0x0003
B4D44_BIT = 0x0004
B4D45_BIT = 0x0005
B4D46_BIT = 0x0006
B4D47_BIT = 0x0007


#;----- B4D5 Bits -----------------------------------------------------
B4D50_BIT = 0x0000
B4D51_BIT = 0x0001
B4D52_BIT = 0x0002
B4D53_BIT = 0x0003
B4D54_BIT = 0x0004
B4D55_BIT = 0x0005
B4D56_BIT = 0x0006
B4D57_BIT = 0x0007


#;----- B4D6 Bits -----------------------------------------------------
B4D60_BIT = 0x0000
B4D61_BIT = 0x0001
B4D62_BIT = 0x0002
B4D63_BIT = 0x0003
B4D64_BIT = 0x0004
B4D65_BIT = 0x0005
B4D66_BIT = 0x0006
B4D67_BIT = 0x0007


#;----- B4D7 Bits -----------------------------------------------------
B4D70_BIT = 0x0000
B4D71_BIT = 0x0001
B4D72_BIT = 0x0002
B4D73_BIT = 0x0003
B4D74_BIT = 0x0004
B4D75_BIT = 0x0005
B4D76_BIT = 0x0006
B46D77_BIT = 0x0007

B4D77_BIT = 0x0007


#;----- CANSTAT_RO5 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO5 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- B5CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
RTREN_BIT = 0x0002
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIF_BIT = 0x0007

RXRTRRO_B5CON_BIT = 0x0005


#;----- B5SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- B5SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- B5EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- B5EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- B5DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

RB0_B5DLC_BIT = 0x0004
RB1_B5DLC_BIT = 0x0005


#;----- B5D0 Bits -----------------------------------------------------
B5D00_BIT = 0x0000
B5D01_BIT = 0x0001
B5D02_BIT = 0x0002
B5D03_BIT = 0x0003
B5D04_BIT = 0x0004
B5D05_BIT = 0x0005
B5D06_BIT = 0x0006
B57D07_BIT = 0x0007

B5D07_BIT = 0x0007


#;----- B5D1 Bits -----------------------------------------------------
B5D10_BIT = 0x0000
B5D11_BIT = 0x0001
B5D12_BIT = 0x0002
B5D13_BIT = 0x0003
B5D14_BIT = 0x0004
B5D15_BIT = 0x0005
B5D16_BIT = 0x0006
B5D17_BIT = 0x0007


#;----- B5D2 Bits -----------------------------------------------------
B5D20_BIT = 0x0000
B5D21_BIT = 0x0001
B5D22_BIT = 0x0002
B57D23_BIT = 0x0003
B5D24_BIT = 0x0004
B5D25_BIT = 0x0005
B5D26_BIT = 0x0006
B5D27_BIT = 0x0007

B5D23_BIT = 0x0003


#;----- B5D3 Bits -----------------------------------------------------
B5D30_BIT = 0x0000
B5D31_BIT = 0x0001
B5D32_BIT = 0x0002
B5D33_BIT = 0x0003
B5D34_BIT = 0x0004
B5D35_BIT = 0x0005
B5D36_BIT = 0x0006
B5D37_BIT = 0x0007


#;----- B5D4 Bits -----------------------------------------------------
B5D40_BIT = 0x0000
B5D41_BIT = 0x0001
B5D42_BIT = 0x0002
B5D43_BIT = 0x0003
B5D44_BIT = 0x0004
B5D45_BIT = 0x0005
B5D46_BIT = 0x0006
B5D47_BIT = 0x0007


#;----- B5D5 Bits -----------------------------------------------------
B5D50_BIT = 0x0000
B5D51_BIT = 0x0001
B5D52_BIT = 0x0002
B5D53_BIT = 0x0003
B5D54_BIT = 0x0004
B5D55_BIT = 0x0005
B5D56_BIT = 0x0006
B5D57_BIT = 0x0007


#;----- B5D6 Bits -----------------------------------------------------
B5D60_BIT = 0x0000
B5D61_BIT = 0x0001
B5D62_BIT = 0x0002
B5D63_BIT = 0x0003
B5D64_BIT = 0x0004
B5D65_BIT = 0x0005
B5D66_BIT = 0x0006
B5D67_BIT = 0x0007


#;----- B5D7 Bits -----------------------------------------------------
B5D70_BIT = 0x0000
B5D71_BIT = 0x0001
B5D72_BIT = 0x0002
B5D73_BIT = 0x0003
B5D74_BIT = 0x0004
B5D75_BIT = 0x0005
B5D76_BIT = 0x0006
B5D77_BIT = 0x0007


#;----- CANSTAT_RO4 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO4 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- RXF0SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF0SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF0EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF0EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF1SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF1SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF1EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF1EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF2SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF2SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF2EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF2EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF3SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF3SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF3EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF3EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF4SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF4SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF4EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF4EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXF5SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXF5SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007

EXIDEN_BIT = 0x0003


#;----- RXF5EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXF5EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXM0SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXM0SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDEN_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- RXM0EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXM0EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXM1SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXM1SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDEN_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- RXM1EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXM1EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- TXB2CON Bits -----------------------------------------------------
TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIFBXB2CON_BIT = 0x0007

TXBIF_BIT = 0x0007


#;----- TXB2SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- TXB2SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- TXB2EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- TXB2EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- TXB2DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
TXRTR_BIT = 0x0006


#;----- TXB2D0 Bits -----------------------------------------------------
TXB2D00_BIT = 0x0000
TXB2D01_BIT = 0x0001
TXB2D02_BIT = 0x0002
TXB2D03_BIT = 0x0003
TXB2D04_BIT = 0x0004
TXB2D05_BIT = 0x0005
TXB2D06_BIT = 0x0006
TXB2D07_BIT = 0x0007


#;----- TXB2D1 Bits -----------------------------------------------------
TXB2D10_BIT = 0x0000
TXB2D11_BIT = 0x0001
TXB2D12_BIT = 0x0002
TXB2D13_BIT = 0x0003
TXB2D14_BIT = 0x0004
TXB2D15_BIT = 0x0005
TXB2D16_BIT = 0x0006
TXB2D17_BIT = 0x0007


#;----- TXB2D2 Bits -----------------------------------------------------
TXB2D20_BIT = 0x0000
TXB2D21_BIT = 0x0001
TXB2D22_BIT = 0x0002
TXB2D23_BIT = 0x0003
TXB2D24_BIT = 0x0004
TXB2D25_BIT = 0x0005
TXB2D26_BIT = 0x0006
TXB2D27_BIT = 0x0007


#;----- TXB2D3 Bits -----------------------------------------------------
TXB2D30_BIT = 0x0000
TXB2D31_BIT = 0x0001
TXB2D32_BIT = 0x0002
TXB2D33_BIT = 0x0003
TXB2D34_BIT = 0x0004
TXB2D35_BIT = 0x0005
TXB2D36_BIT = 0x0006
TXB2D37_BIT = 0x0007


#;----- TXB2D4 Bits -----------------------------------------------------
TXB2D40_BIT = 0x0000
TXB2D41_BIT = 0x0001
TXB2D42_BIT = 0x0002
TXB2D43_BIT = 0x0003
TXB2D44_BIT = 0x0004
TXB2D45_BIT = 0x0005
TXB2D46_BIT = 0x0006
TXB2D47_BIT = 0x0007


#;----- TXB2D5 Bits -----------------------------------------------------
TXB2D50_BIT = 0x0000
TXB2D51_BIT = 0x0001
TXB2D52_BIT = 0x0002
TXB2D53_BIT = 0x0003
TXB2D54_BIT = 0x0004
TXB2D55_BIT = 0x0005
TXB2D56_BIT = 0x0006
TXB2D57_BIT = 0x0007


#;----- TXB2D6 Bits -----------------------------------------------------
TXB2D60_BIT = 0x0000
TXB2D61_BIT = 0x0001
TXB2D62_BIT = 0x0002
TXB2D63_BIT = 0x0003
TXB2D64_BIT = 0x0004
TXB2D65_BIT = 0x0005
TXB2D66_BIT = 0x0006
TXB2D67_BIT = 0x0007


#;----- TXB2D7 Bits -----------------------------------------------------
TXB2D70_BIT = 0x0000
TXB2D71_BIT = 0x0001
TXB2D72_BIT = 0x0002
TXB2D73_BIT = 0x0003
TXB2D74_BIT = 0x0004
TXB2D75_BIT = 0x0005
TXB2D76_BIT = 0x0006
TXB2D77_BIT = 0x0007


#;----- CANSTAT_RO3 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO3 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- TXB1CON Bits -----------------------------------------------------
TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006
TXBIFTXB1CON_BIT = 0x0007

TXBIF_BIT = 0x0007


#;----- TXB1SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- TXB1SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- TXB1EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- TXB1EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- TXB1DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
TXRTR_BIT = 0x0006


#;----- TXB1D0 Bits -----------------------------------------------------
TXB1D00_BIT = 0x0000
TXB1D01_BIT = 0x0001
TXB1D02_BIT = 0x0002
TXB1D03_BIT = 0x0003
TXB1D04_BIT = 0x0004
TXB1D05_BIT = 0x0005
TXB1D06_BIT = 0x0006
TXB1D07_BIT = 0x0007


#;----- TXB1D1 Bits -----------------------------------------------------
TXB1D10_BIT = 0x0000
TXB1D11_BIT = 0x0001
TXB1D12_BIT = 0x0002
TXB1D13_BIT = 0x0003
TXB1D14_BIT = 0x0004
TXB1D15_BIT = 0x0005
TXB1D16_BIT = 0x0006
TXB1D17_BIT = 0x0007


#;----- TXB1D2 Bits -----------------------------------------------------
TXB1D20_BIT = 0x0000
TXB1D21_BIT = 0x0001
TXB1D22_BIT = 0x0002
TXB1D23_BIT = 0x0003
TXB1D24_BIT = 0x0004
TXB1D25_BIT = 0x0005
TXB1D26_BIT = 0x0006
TXB1D27_BIT = 0x0007


#;----- TXB1D3 Bits -----------------------------------------------------
TXB1D30_BIT = 0x0000
TXB1D31_BIT = 0x0001
TXB1D32_BIT = 0x0002
TXB1D33_BIT = 0x0003
TXB1D34_BIT = 0x0004
TXB1D35_BIT = 0x0005
TXB1D36_BIT = 0x0006
TXB1D37_BIT = 0x0007


#;----- TXB1D4 Bits -----------------------------------------------------
TXB1D40_BIT = 0x0000
TXB1D41_BIT = 0x0001
TXB1D42_BIT = 0x0002
TXB1D43_BIT = 0x0003
TXB1D44_BIT = 0x0004
TXB1D45_BIT = 0x0005
TXB1D46_BIT = 0x0006
TXB1D47_BIT = 0x0007


#;----- TXB1D5 Bits -----------------------------------------------------
TXB1D50_BIT = 0x0000
TXB1D51_BIT = 0x0001
TXB1D52_BIT = 0x0002
TXB1D53_BIT = 0x0003
TXB1D54_BIT = 0x0004
TXB1D55_BIT = 0x0005
TXB1D56_BIT = 0x0006
TXB1D57_BIT = 0x0007


#;----- TXB1D6 Bits -----------------------------------------------------
TXB1D60_BIT = 0x0000
TXB1D61_BIT = 0x0001
TXB1D62_BIT = 0x0002
TXB1D63_BIT = 0x0003
TXB1D64_BIT = 0x0004
TXB1D65_BIT = 0x0005
TXB1D66_BIT = 0x0006
TXB1D67_BIT = 0x0007


#;----- TXB1D7 Bits -----------------------------------------------------
TXB1D70_BIT = 0x0000
TXB1D71_BIT = 0x0001
TXB1D72_BIT = 0x0002
TXB1D73_BIT = 0x0003
TXB1D74_BIT = 0x0004
TXB1D75_BIT = 0x0005
TXB1D76_BIT = 0x0006
TXB1D77_BIT = 0x0007


#;----- CANSTAT_RO2 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO2 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- TXB0CON Bits -----------------------------------------------------
TXPRI0_BIT = 0x0000
TXPRI1_BIT = 0x0001
TXREQ_BIT = 0x0003
TXERR_BIT = 0x0004
TXLARB_BIT = 0x0005
TXABT_BIT = 0x0006

TXBIF_BIT = 0x0007


#;----- TXB0SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- TXB0SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXIDE_BIT = 0x0003
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- TXB0EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- TXB0EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- TXB0DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
TXRTR_BIT = 0x0006


#;----- TXB0D0 Bits -----------------------------------------------------
TXB0D00_BIT = 0x0000
TXB0D01_BIT = 0x0001
TXB0D02_BIT = 0x0002
TXB0D03_BIT = 0x0003
TXB0D04_BIT = 0x0004
TXB0D05_BIT = 0x0005
TXB0D06_BIT = 0x0006
TXB0D07_BIT = 0x0007


#;----- TXB0D1 Bits -----------------------------------------------------
TXB0D10_BIT = 0x0000
TXB0D11_BIT = 0x0001
TXB0D12_BIT = 0x0002
TXB0D13_BIT = 0x0003
TXB0D14_BIT = 0x0004
TXB0D15_BIT = 0x0005
TXB0D16_BIT = 0x0006
TXB0D17_BIT = 0x0007


#;----- TXB0D2 Bits -----------------------------------------------------
TXB0D20_BIT = 0x0000
TXB0D21_BIT = 0x0001
TXB0D22_BIT = 0x0002
TXB0D23_BIT = 0x0003
TXB0D24_BIT = 0x0004
TXB0D25_BIT = 0x0005
TXB0D26_BIT = 0x0006
TXB0D27_BIT = 0x0007


#;----- TXB0D3 Bits -----------------------------------------------------
TXB0D30_BIT = 0x0000
TXB0D31_BIT = 0x0001
TXB0D32_BIT = 0x0002
TXB0D33_BIT = 0x0003
TXB0D34_BIT = 0x0004
TXB0D35_BIT = 0x0005
TXB0D36_BIT = 0x0006
TXB0D37_BIT = 0x0007


#;----- TXB0D4 Bits -----------------------------------------------------
TXB0D40_BIT = 0x0000
TXB0D41_BIT = 0x0001
TXB0D42_BIT = 0x0002
TXB0D43_BIT = 0x0003
TXB0D44_BIT = 0x0004
TXB0D45_BIT = 0x0005
TXB0D46_BIT = 0x0006
TXB0D47_BIT = 0x0007


#;----- TXB0D5 Bits -----------------------------------------------------
TXB0D50_BIT = 0x0000
TXB0D51_BIT = 0x0001
TXB0D52_BIT = 0x0002
TXB0D53_BIT = 0x0003
TXB0D54_BIT = 0x0004
TXB0D55_BIT = 0x0005
TXB0D56_BIT = 0x0006
TXB0D57_BIT = 0x0007


#;----- TXB0D6 Bits -----------------------------------------------------
TXB0D60_BIT = 0x0000
TXB0D61_BIT = 0x0001
TXB0D62_BIT = 0x0002
TXB0D63_BIT = 0x0003
TXB0D64_BIT = 0x0004
TXB0D65_BIT = 0x0005
TXB0D66_BIT = 0x0006
TXB0D67_BIT = 0x0007


#;----- TXB0D7 Bits -----------------------------------------------------
TXB0D70_BIT = 0x0000
TXB0D71_BIT = 0x0001
TXB0D72_BIT = 0x0002
TXB0D73_BIT = 0x0003
TXB0D74_BIT = 0x0004
TXB0D75_BIT = 0x0005
TXB0D76_BIT = 0x0006
TXB0D77_BIT = 0x0007


#;----- CANSTAT_RO1 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO1 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- RXB1CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
RXRTRRO_RXB1CON_BIT = 0x0003
RXM0_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005


#;----- RXB1SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXB1SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- RXB1EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXB1EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXB1DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

RB0_RXB1DLC_BIT = 0x0004
RB1_RXB1DLC_BIT = 0x0005


#;----- RXB1D0 Bits -----------------------------------------------------
RXB1D00_BIT = 0x0000
RXB1D01_BIT = 0x0001
RXB1D02_BIT = 0x0002
RXB1D03_BIT = 0x0003
RXB1D04_BIT = 0x0004
RXB1D05_BIT = 0x0005
RXB1D06_BIT = 0x0006
RXB1D07_BIT = 0x0007


#;----- RXB1D1 Bits -----------------------------------------------------
RXB1D10_BIT = 0x0000
RXB1D11_BIT = 0x0001
RXB1D12_BIT = 0x0002
RXB1D13_BIT = 0x0003
RXB1D14_BIT = 0x0004
RXB1D15_BIT = 0x0005
RXB1D16_BIT = 0x0006
RXB1D17_BIT = 0x0007


#;----- RXB1D2 Bits -----------------------------------------------------
RXB1D20_BIT = 0x0000
RXB1D21_BIT = 0x0001
RXB1D22_BIT = 0x0002
RXB1D23_BIT = 0x0003
RXB1D24_BIT = 0x0004
RXB1D25_BIT = 0x0005
RXB1D26_BIT = 0x0006
RXB1D27_BIT = 0x0007


#;----- RXB1D3 Bits -----------------------------------------------------
RXB1D30_BIT = 0x0000
RXB1D31_BIT = 0x0001
RXB1D32_BIT = 0x0002
RXB1D33_BIT = 0x0003
RXB1D34_BIT = 0x0004
RXB1D35_BIT = 0x0005
RXB1D36_BIT = 0x0006
RXB1D37_BIT = 0x0007


#;----- RXB1D4 Bits -----------------------------------------------------
RXB1D40_BIT = 0x0000
RXB1D41_BIT = 0x0001
RXB1D42_BIT = 0x0002
RXB1D43_BIT = 0x0003
RXB1D44_BIT = 0x0004
RXB1D45_BIT = 0x0005
RXB1D46_BIT = 0x0006
RXB1D47_BIT = 0x0007


#;----- RXB1D5 Bits -----------------------------------------------------
RXB1D50_BIT = 0x0000
RXB1D51_BIT = 0x0001
RXB1D52_BIT = 0x0002
RXB1D53_BIT = 0x0003
RXB1D54_BIT = 0x0004
RXB1D55_BIT = 0x0005
RXB1D56_BIT = 0x0006
RXB1D57_BIT = 0x0007


#;----- RXB1D6 Bits -----------------------------------------------------
RXB1D60_BIT = 0x0000
RXB1D61_BIT = 0x0001
RXB1D62_BIT = 0x0002
RXB1D63_BIT = 0x0003
RXB1D64_BIT = 0x0004
RXB1D65_BIT = 0x0005
RXB1D66_BIT = 0x0006
RXB1D67_BIT = 0x0007


#;----- RXB1D7 Bits -----------------------------------------------------
RXB1D70_BIT = 0x0000
RXB1D71_BIT = 0x0001
RXB1D72_BIT = 0x0002
RXB1D73_BIT = 0x0003
RXB1D74_BIT = 0x0004
RXB1D75_BIT = 0x0005
RXB1D76_BIT = 0x0006
RXB1D77_BIT = 0x0007


#;----- CANSTAT_RO0 Bits -----------------------------------------------------
ICODE0_BIT = 0x0000
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
ICODE4_BIT = 0x0004
OPMODE_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- CANCON_RO0 Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007


#;----- RXB0CON Bits -----------------------------------------------------
FILHIT0_BIT = 0x0000
JTOFF_BIT = 0x0001
RXBODBEN_BIT = 0x0002
RXRTRRO_RXB0CON_BIT = 0x0003
RXM0_BIT = 0x0005
RXM1_BIT = 0x0006
RXFUL_BIT = 0x0007

FILHIT1_BIT = 0x0001
FILHIT2_BIT = 0x0002
FILHIT3_BIT = 0x0003
FILHIT4_BIT = 0x0004
RTRRO_BIT = 0x0005

RXB0DBEN_BIT = 0x0002


#;----- RXB0SIDH Bits -----------------------------------------------------
SID3_BIT = 0x0000
SID4_BIT = 0x0001
SID5_BIT = 0x0002
SID6_BIT = 0x0003
SID7_BIT = 0x0004
SID8_BIT = 0x0005
SID9_BIT = 0x0006
SID10_BIT = 0x0007


#;----- RXB0SIDL Bits -----------------------------------------------------
EID16_BIT = 0x0000
EID17_BIT = 0x0001
EXID_BIT = 0x0003
SRR_BIT = 0x0004
SID0_BIT = 0x0005
SID1_BIT = 0x0006
SID2_BIT = 0x0007


#;----- RXB0EIDH Bits -----------------------------------------------------
EID8_BIT = 0x0000
EID9_BIT = 0x0001
EID10_BIT = 0x0002
EID11_BIT = 0x0003
EID12_BIT = 0x0004
EID13_BIT = 0x0005
EID14_BIT = 0x0006
EID15_BIT = 0x0007


#;----- RXB0EIDL Bits -----------------------------------------------------
EID0_BIT = 0x0000
EID1_BIT = 0x0001
EID2_BIT = 0x0002
EID3_BIT = 0x0003
EID4_BIT = 0x0004
EID5_BIT = 0x0005
EID6_BIT = 0x0006
EID7_BIT = 0x0007


#;----- RXB0DLC Bits -----------------------------------------------------
DLC0_BIT = 0x0000
DLC1_BIT = 0x0001
DLC2_BIT = 0x0002
DLC3_BIT = 0x0003
RESRB0_BIT = 0x0004
RESRB1_BIT = 0x0005
RXRTR_BIT = 0x0006

RB0_RXB0DLC_BIT = 0x0004
RB1_RXB0DLC_BIT = 0x0005


#;----- RXB0D0 Bits -----------------------------------------------------
RXB0D00_BIT = 0x0000
RXB0D01_BIT = 0x0001
RXB0D02_BIT = 0x0002
RXB0D03_BIT = 0x0003
RXB0D04_BIT = 0x0004
RXB0D05_BIT = 0x0005
RXB0D06_BIT = 0x0006
RXB0D07_BIT = 0x0007


#;----- RXB0D1 Bits -----------------------------------------------------
RXB0D10_BIT = 0x0000
RXB0D11_BIT = 0x0001
RXB0D12_BIT = 0x0002
RXB0D13_BIT = 0x0003
RXB0D14_BIT = 0x0004
RXB0D15_BIT = 0x0005
RXB0D16_BIT = 0x0006
RXB0D17_BIT = 0x0007


#;----- RXB0D2 Bits -----------------------------------------------------
RXB0D20_BIT = 0x0000
RXB0D21_BIT = 0x0001
RXB0D22_BIT = 0x0002
RXB0D23_BIT = 0x0003
RXB0D24_BIT = 0x0004
RXB0D25_BIT = 0x0005
RXB0D26_BIT = 0x0006
RXB0D27_BIT = 0x0007


#;----- RXB0D3 Bits -----------------------------------------------------
RXB0D30_BIT = 0x0000
RXB0D31_BIT = 0x0001
RXB0D32_BIT = 0x0002
RXB0D33_BIT = 0x0003
RXB0D34_BIT = 0x0004
RXB0D35_BIT = 0x0005
RXB0D36_BIT = 0x0006
RXB0D37_BIT = 0x0007


#;----- RXB0D4 Bits -----------------------------------------------------
RXB0D40_BIT = 0x0000
RXB0D41_BIT = 0x0001
RXB0D42_BIT = 0x0002
RXB0D43_BIT = 0x0003
RXB0D44_BIT = 0x0004
RXB0D45_BIT = 0x0005
RXB0D46_BIT = 0x0006
RXB0D47_BIT = 0x0007


#;----- RXB0D5 Bits -----------------------------------------------------
RXB0D50_BIT = 0x0000
RXB0D51_BIT = 0x0001
RXB0D52_BIT = 0x0002
RXB0D53_BIT = 0x0003
RXB0D54_BIT = 0x0004
RXB0D55_BIT = 0x0005
RXB0D56_BIT = 0x0006
RXB0D57_BIT = 0x0007


#;----- RXB0D6 Bits -----------------------------------------------------
RXB0D60_BIT = 0x0000
RXB0D61_BIT = 0x0001
RXB0D62_BIT = 0x0002
RXB0D63_BIT = 0x0003
RXB0D64_BIT = 0x0004
RXB0D65_BIT = 0x0005
RXB0D66_BIT = 0x0006
RXB0D67_BIT = 0x0007


#;----- RXB0D7 Bits -----------------------------------------------------
RXB0D70_BIT = 0x0000
RXB0D71_BIT = 0x0001
RXB0D72_BIT = 0x0002
RXB0D73_BIT = 0x0003
RXB0D74_BIT = 0x0004
RXB0D75_BIT = 0x0005
RXB0D76_BIT = 0x0006
RXB0D77_BIT = 0x0007


#;----- CANSTAT Bits -----------------------------------------------------
ICODE1_BIT = 0x0001
ICODE2_BIT = 0x0002
ICODE3_BIT = 0x0003
OPMODE0_BIT = 0x0005
OPMODE1_BIT = 0x0006
OPMODE2_BIT = 0x0007

EICODE0_BIT = 0x0000
EICODE1_BIT = 0x0001
EICODE2_BIT = 0x0002
EICODE3_BIT = 0x0003
EICODE4_BIT = 0x0004


#;----- CANCON Bits -----------------------------------------------------
WIN0_BIT = 0x0001
WIN1_BIT = 0x0002
WIN2_BIT = 0x0003
ABAT_BIT = 0x0004
REQOP0_BIT = 0x0005
REQOP1_BIT = 0x0006
REQOP2_BIT = 0x0007

FP0_BIT = 0x0000
FP1_BIT = 0x0001
FP2_BIT = 0x0002
FP3_BIT = 0x0003


#;----- BRGCON1 Bits -----------------------------------------------------
BRP0_BIT = 0x0000
BRP1_BIT = 0x0001
BRP2_BIT = 0x0002
BRP3_BIT = 0x0003
BRP4_BIT = 0x0004
BRP5_BIT = 0x0005
SJW0_BIT = 0x0006
SJW1_BIT = 0x0007


#;----- BRGCON2 Bits -----------------------------------------------------
PRSEG0_BIT = 0x0000
PRSEG1_BIT = 0x0001
PRSEG2_BIT = 0x0002
SEG1PH0_BIT = 0x0003
SEG1PH1_BIT = 0x0004
SEG1PH2_BIT = 0x0005
SAM_BIT = 0x0006
SEG2PHTS_BIT = 0x0007

SEG2PHT_BIT = 0x0007


#;----- BRGCON3 Bits -----------------------------------------------------
SEG2PH0_BIT = 0x0000
SEG2PH1_BIT = 0x0001
SEG2PH2_BIT = 0x0002
WAKFIL_BIT = 0x0006
WAKDIS_BIT = 0x0007


#;----- CIOCON Bits -----------------------------------------------------
CANCAP_BIT = 0x0004
ENDRHI_BIT = 0x0005


#;----- COMSTAT Bits -----------------------------------------------------
EWARN_BIT = 0x0000
RXWARN_BIT = 0x0001
TXWARN_BIT = 0x0002
RXBP_BIT = 0x0003
TXBP_BIT = 0x0004
TXBO_BIT = 0x0005
RXB1OVFL_BIT = 0x0006
RXB0OVFL_BIT = 0x0007

RXBnOVFL_BIT = 0x0006

FIFOEMPTY_BIT = 0x0007


#;----- RXERRCNT Bits -----------------------------------------------------
REC0_BIT = 0x0000
REC1_BIT = 0x0001
REC2_BIT = 0x0002
REC3_BIT = 0x0003
REC4_BIT = 0x0004
REC5_BIT = 0x0005
REC6_BIT = 0x0006
REC7_BIT = 0x0007


#;----- TXERRCNT Bits -----------------------------------------------------
TEC0_BIT = 0x0000
TEC1_BIT = 0x0001
TEC2_BIT = 0x0002
TEC3_BIT = 0x0003
TEC4_BIT = 0x0004
TEC5_BIT = 0x0005
TEC6_BIT = 0x0006
TEC7_BIT = 0x0007


#;----- ECANCON Bits -----------------------------------------------------
EWIN0_BIT = 0x0000
EWIN1_BIT = 0x0001
EWIN2_BIT = 0x0002
EWIN3_BIT = 0x0003
EWIN4_BIT = 0x0004
FIFOWM_BIT = 0x0005
MDSEL0_BIT = 0x0006
MDSEL1_BIT = 0x0007

F_BIT = 0x0005


#;----- PORTA Bits -----------------------------------------------------
RA0_BIT = 0x0000
RA1_BIT = 0x0001
RA2_BIT = 0x0002
RA3_BIT = 0x0003
RA4_BIT = 0x0004
RA5_BIT = 0x0005
RA6_BIT = 0x0006
RA7_BIT = 0x0007

AN0_BIT = 0x0000
AN1_BIT = 0x0001
AN2_BIT = 0x0002
AN3_BIT = 0x0003
T0CKI_BIT = 0x0004
AN4_BIT = 0x0005
OSC2_BIT = 0x0006
OSC1_BIT = 0x0007

CVREF_BIT = 0x0000
VREFM_BIT = 0x0002
VREFP_BIT = 0x0003
LVDIN_BIT = 0x0005
CLKO_BIT = 0x0006
CLKI_BIT = 0x0007

SS_BIT = 0x0005

NOT_SS_BIT = 0x0005

HLVDIN_BIT = 0x0005


#;----- PORTB Bits -----------------------------------------------------
RB0_PORTB_BIT = 0x0000
RB1_PORTB_BIT = 0x0001
RB2_BIT = 0x0002
RB3_BIT = 0x0003
RB4_BIT = 0x0004
RB5_BIT = 0x0005
RB6_BIT = 0x0006
RB7_BIT = 0x0007

INT0_BIT = 0x0000
INT1_BIT = 0x0001
INT2_BIT = 0x0002
CANRX_BIT = 0x0003
KBI0_BIT = 0x0004
KBI1_BIT = 0x0005
KBI2_BIT = 0x0006
KBI3_BIT = 0x0007

AN10_BIT = 0x0000

FLT0_BIT = 0x0000
AN8_BIT = 0x0001
CANTX_BIT = 0x0002
AN9_BIT = 0x0004
PGM_BIT = 0x0005
PGC_BIT = 0x0006
PGD_BIT = 0x0007


#;----- PORTC Bits -----------------------------------------------------
RC0_BIT = 0x0000
RC1_BIT = 0x0001
RC2_BIT = 0x0002
RC3_BIT = 0x0003
RC4_BIT = 0x0004
RC5_BIT = 0x0005
RC6_BIT = 0x0006
RC7_BIT = 0x0007

T1OSO_BIT = 0x0000
T1OSI_BIT = 0x0001
CCP1_BIT = 0x0002
SCK_BIT = 0x0003
SDI_BIT = 0x0004
SDO_BIT = 0x0005
TX_BIT = 0x0006
RX_BIT = 0x0007

T13CKI_BIT = 0x0000
SCL_BIT = 0x0003
SDA_BIT = 0x0004
CK_BIT = 0x0006
# DT is a reserved wor
# DT               EQU  H'0007


#;----- PORTE Bits -----------------------------------------------------
RE3_BIT = 0x0003


#;----- LATA Bits -----------------------------------------------------
LATA0_BIT = 0x0000
LATA1_BIT = 0x0001
LATA2_BIT = 0x0002
LATA3_BIT = 0x0003
LATA4_BIT = 0x0004
LATA5_BIT = 0x0005
LATA6_BIT = 0x0006
LATA7_BIT = 0x0007


#;----- LATB Bits -----------------------------------------------------
LATB0_BIT = 0x0000
LATB1_BIT = 0x0001
LATB2_BIT = 0x0002
LATB3_BIT = 0x0003
LATB4_BIT = 0x0004
LATB5_BIT = 0x0005
LATB6_BIT = 0x0006
LATB7_BIT = 0x0007


#;----- LATC Bits -----------------------------------------------------
LATC0_BIT = 0x0000
LATC1_BIT = 0x0001
LATC2_BIT = 0x0002
LATC3_BIT = 0x0003
LATC4_BIT = 0x0004
LATC5_BIT = 0x0005
LATC6_BIT = 0x0006
LATC7_BIT = 0x0007


#;----- DDRA Bits -----------------------------------------------------
RA0_BIT = 0x0000
RA1_BIT = 0x0001
RA2_BIT = 0x0002
RA3_BIT = 0x0003
RA4_BIT = 0x0004
RA5_BIT = 0x0005
RA6_BIT = 0x0006


#;----- TRISA Bits -----------------------------------------------------
TRISA0_BIT = 0x0000
TRISA1_BIT = 0x0001
TRISA2_BIT = 0x0002
TRISA3_BIT = 0x0003
TRISA4_BIT = 0x0004
TRISA5_BIT = 0x0005
TRISA6_BIT = 0x0006
TRISA7_BIT = 0x0007


#;----- DDRB Bits -----------------------------------------------------
RB0_DDRB_BIT = 0x0000
RB1_DDRB_BIT = 0x0001
RB2_BIT = 0x0002
RB3_BIT = 0x0003
RB4_BIT = 0x0004
RB5_BIT = 0x0005
RB6_BIT = 0x0006
RB7_BIT = 0x0007


#;----- TRISB Bits -----------------------------------------------------
TRISB0_BIT = 0x0000
TRISB1_BIT = 0x0001
TRISB2_BIT = 0x0002
TRISB3_BIT = 0x0003
TRISB4_BIT = 0x0004
TRISB5_BIT = 0x0005
TRISB6_BIT = 0x0006
TRISB7_BIT = 0x0007


#;----- DDRC Bits -----------------------------------------------------
RC0_BIT = 0x0000
RC1_BIT = 0x0001
RC2_BIT = 0x0002
RC3_BIT = 0x0003
RC4_BIT = 0x0004
RC5_BIT = 0x0005
RC6_BIT = 0x0006
RC7_BIT = 0x0007


#;----- TRISC Bits -----------------------------------------------------
TRISC0_BIT = 0x0000
TRISC1_BIT = 0x0001
TRISC2_BIT = 0x0002
TRISC3_BIT = 0x0003
TRISC4_BIT = 0x0004
TRISC5_BIT = 0x0005
TRISC6_BIT = 0x0006
TRISC7_BIT = 0x0007


#;----- OSCTUNE Bits -----------------------------------------------------
TUN0_BIT = 0x0000
TUN1_BIT = 0x0001
TUN2_BIT = 0x0002
TUN3_BIT = 0x0003
TUN4_BIT = 0x0004
PLLEN_BIT = 0x0006
INTSCR_BIT = 0x0007

INTSRC_BIT = 0x0007


#;----- PIE1 Bits -----------------------------------------------------
TMR1IE_BIT = 0x0000
TMR2IE_BIT = 0x0001
CCP1IE_BIT = 0x0002
SSPIE_BIT = 0x0003
TXIE_BIT = 0x0004
RCIE_BIT = 0x0005
ADIE_BIT = 0x0006


#;----- PIR1 Bits -----------------------------------------------------
TMR1IF_BIT = 0x0000
TMR2IF_BIT = 0x0001
CCP1IF_BIT = 0x0002
SSPIF_BIT = 0x0003
TXIF_BIT = 0x0004
RCIF_BIT = 0x0005
ADIF_BIT = 0x0006


#;----- IPR1 Bits -----------------------------------------------------
TMR1IP_BIT = 0x0000
TMR2IP_BIT = 0x0001
CCP1IP_BIT = 0x0002
SSPIP_BIT = 0x0003
TXBIP_BIT = 0x0004
RCIP_BIT = 0x0005
ADIP_BIT = 0x0006

TXIP_BIT = 0x0004


#;----- PIE2 Bits -----------------------------------------------------
TMR3IE_BIT = 0x0001
LVDIE_BIT = 0x0002
BCLIE_BIT = 0x0003
EEIE_BIT = 0x0004
OSCFIE_BIT = 0x0007

HLVDIE_BIT = 0x0002


#;----- PIR2 Bits -----------------------------------------------------
TMR3IF_BIT = 0x0001
LVDIF_BIT = 0x0002
BCLIF_BIT = 0x0003
EEIF_BIT = 0x0004
OSCFIF_BIT = 0x0007

HLVDIF_BIT = 0x0002


#;----- IPR2 Bits -----------------------------------------------------
TMR3IP_BIT = 0x0001
LVDIP_BIT = 0x0002
BCLIP_BIT = 0x0003
EEIP_BIT = 0x0004
OSCFIP_BIT = 0x0007

HLVDIP_BIT = 0x0002


#;----- PIE3 Bits -----------------------------------------------------
RXB0IE_BIT = 0x0000
RXB1IE_BIT = 0x0001
TXB0IE_BIT = 0x0002
TXB1IE_BIT = 0x0003
TXB2IE_BIT = 0x0004
ERRIE_BIT = 0x0005
WAKIE_BIT = 0x0006
IRXIE_BIT = 0x0007

FIFOWMIE_BIT = 0x0000
RXBnIE_BIT = 0x0001
TXBnIE_BIT = 0x0004

FIFOMWIE_BIT = 0x0000


#;----- PIR3 Bits -----------------------------------------------------
RXB0IF_BIT = 0x0000
RXB1IF_BIT = 0x0001
TXB0IF_BIT = 0x0002
TXB1IF_BIT = 0x0003
TXB2IF_BIT = 0x0004
ERRIF_BIT = 0x0005
WAKIF_BIT = 0x0006
IRXIF_BIT = 0x0007

FIFOWMIF_BIT = 0x0000
RXBnIF_BIT = 0x0001
TXBnIF_BIT = 0x0004


#;----- IPR3 Bits -----------------------------------------------------
RXB0IP_BIT = 0x0000
RXB1IP_BIT = 0x0001
TXB0IP_BIT = 0x0002
TXB1IP_BIT = 0x0003
TXB2IP_BIT = 0x0004
ERRIP_BIT = 0x0005
WAKIP_BIT = 0x0006
IRXIP_BIT = 0x0007

FIFOWMIP_BIT = 0x0000
RXBnIP_BIT = 0x0001
TXBnIP_BIT = 0x0004


#;----- EECON1 Bits -----------------------------------------------------
RD_BIT = 0x0000
WR_BIT = 0x0001
WREN_BIT = 0x0002
WRERR_BIT = 0x0003
FREE_BIT = 0x0004
CFGS_BIT = 0x0006
EEPGD_BIT = 0x0007


#;----- RCSTA Bits -----------------------------------------------------
RX9D_BIT = 0x0000
OERR_BIT = 0x0001
FERR_BIT = 0x0002
ADEN_BIT = 0x0003
CREN_BIT = 0x0004
SREN_BIT = 0x0005
RX9_BIT = 0x0006
SPEN_BIT = 0x0007

ADDEN_BIT = 0x0003


#;----- TXSTA Bits -----------------------------------------------------
TX9D_BIT = 0x0000
TRMT_BIT = 0x0001
BRGH_BIT = 0x0002
SENDB_BIT = 0x0003
SYNC_BIT = 0x0004
TXEN_BIT = 0x0005
TX9_BIT = 0x0006
CSRC_BIT = 0x0007


#;----- T3CON Bits -----------------------------------------------------
TMR3ON_BIT = 0x0000
TMR3CS_BIT = 0x0001
T3SYNC_BIT = 0x0002
T3CCP1_BIT = 0x0003
T3CKPS0_BIT = 0x0004
T3CKPS1_BIT = 0x0005
T3CCP2_BIT = 0x0006
RD16_BIT = 0x0007

T3NSYNC_BIT = 0x0002

T3ECCP1_BIT = 0x0006

NOT_T3SYNC_BIT = 0x0002


#;----- BAUDCON Bits -----------------------------------------------------
ABDEN_BIT = 0x0000
WUE_BIT = 0x0001
BRG16_BIT = 0x0003
SCKP_BIT = 0x0004
RCIDL_BIT = 0x0006
ABDOVF_BIT = 0x0007


#;----- CCP1CON Bits -----------------------------------------------------
CCP1M0_BIT = 0x0000
CCP1M1_BIT = 0x0001
CCP1M2_BIT = 0x0002
CCP1M3_BIT = 0x0003
DC1B0_BIT = 0x0004
DC1B1_BIT = 0x0005


#;----- ADCON2 Bits -----------------------------------------------------
ADCS0_BIT = 0x0000
ADCS1_BIT = 0x0001
ADCS2_BIT = 0x0002
ACQT0_BIT = 0x0003
ACQT1_BIT = 0x0004
ACQT2_BIT = 0x0005
ADFM_BIT = 0x0007


#;----- ADCON1 Bits -----------------------------------------------------
PCFG0_BIT = 0x0000
PCFG1_BIT = 0x0001
PCFG2_BIT = 0x0002
PCFG3_BIT = 0x0003
VCFG0_BIT = 0x0004
VCFG1_BIT = 0x0005


#;----- ADCON0 Bits -----------------------------------------------------
ADON_BIT = 0x0000
GO_DONE_BIT = 0x0001
CHS0_BIT = 0x0002
CHS1_BIT = 0x0003
CHS2_BIT = 0x0004
CHS3_BIT = 0x0005

DONE_BIT = 0x0001

GO_BIT = 0x0001

NOT_DONE_BIT = 0x0001


#;----- SSPCON2 Bits -----------------------------------------------------
SEN_BIT = 0x0000
RSEN_BIT = 0x0001
PEN_BIT = 0x0002
RCEN_BIT = 0x0003
ACKEN_BIT = 0x0004
ACKDT_BIT = 0x0005
ACKSTAT_BIT = 0x0006
GCEN_BIT = 0x0007


#;----- SSPCON1 Bits -----------------------------------------------------
SSPM0_BIT = 0x0000
SSPM1_BIT = 0x0001
SSPM2_BIT = 0x0002
SSPM3_BIT = 0x0003
CKP_BIT = 0x0004
SSPEN_BIT = 0x0005
SSPOV_BIT = 0x0006
WCOL_BIT = 0x0007


#;----- SSPSTAT Bits -----------------------------------------------------
BF_BIT = 0x0000
UA_BIT = 0x0001
R_W_BIT = 0x0002
S_BIT = 0x0003
P_BIT = 0x0004
D_A_BIT = 0x0005
CKE_BIT = 0x0006
SMP_BIT = 0x0007

I2C_READ_BIT = 0x0002
I2C_START_BIT = 0x0003
I2C_STOP_BIT = 0x0004
I2C_DAT_BIT = 0x0005

NOT_W_BIT = 0x0002
NOT_A_BIT = 0x0005

NOT_WRITE_BIT = 0x0002
NOT_ADDRESS_BIT = 0x0005

READ_WRITE_BIT = 0x0002
DATA_ADDRESS_BIT = 0x0005

R_BIT = 0x0002
D_BIT = 0x0005


#;----- T2CON Bits -----------------------------------------------------
T2CKPS0_BIT = 0x0000
T2CKPS1_BIT = 0x0001
TMR2ON_BIT = 0x0002
T2OUTPS0_BIT = 0x0003
T2OUTPS1_BIT = 0x0004
T2OUTPS2_BIT = 0x0005
T2OUTPS3_BIT = 0x0006


#;----- T1CON Bits -----------------------------------------------------
TMR1ON_BIT = 0x0000
TMR1CS_BIT = 0x0001
T1SYNC_BIT = 0x0002
T1OSCEN_BIT = 0x0003
T1CKPS0_BIT = 0x0004
T1CKPS1_BIT = 0x0005
T1RUN_BIT = 0x0006
RD16_BIT = 0x0007

T1INSYNC_BIT = 0x0002

NOT_T1SYNC_BIT = 0x0002


#;----- RCON Bits -----------------------------------------------------
NOT_BOR_BIT = 0x0000
NOT_POR_BIT = 0x0001
NOT_PD_BIT = 0x0002
NOT_TO_BIT = 0x0003
NOT_RI_BIT = 0x0004
SBOREN_BIT = 0x0006
IPEN_BIT = 0x0007

BOR_BIT = 0x0000
POR_BIT = 0x0001
PD_BIT = 0x0002
TO_BIT = 0x0003
RI_BIT = 0x0004


#;----- WDTCON Bits -----------------------------------------------------
SWDTEN_BIT = 0x0000

SWDTE_BIT = 0x0000


#;----- HLVDCON Bits -----------------------------------------------------
LVDL0_BIT = 0x0000
LVDL1_BIT = 0x0001
LVDL2_BIT = 0x0002
LVDL3_BIT = 0x0003
LVDEN_BIT = 0x0004
IRVST_BIT = 0x0005

LVV0_BIT = 0x0000
LVV1_BIT = 0x0001
LVV2_BIT = 0x0002
LVV3_BIT = 0x0003
BGST_BIT = 0x0005

HLVDL0_BIT = 0x0000
HLVDL1_BIT = 0x0001
HLVDL2_BIT = 0x0002
HLVDL3_BIT = 0x0003
HLVDEN_BIT = 0x0004
VDIRMAG_BIT = 0x0007

IVRST_BIT = 0x0005


#;----- LVDCON Bits -----------------------------------------------------
LVDL0_BIT = 0x0000
LVDL1_BIT = 0x0001
LVDL2_BIT = 0x0002
LVDL3_BIT = 0x0003
LVDEN_BIT = 0x0004
IRVST_BIT = 0x0005

LVV0_BIT = 0x0000
LVV1_BIT = 0x0001
LVV2_BIT = 0x0002
LVV3_BIT = 0x0003
BGST_BIT = 0x0005

HLVDL0_BIT = 0x0000
HLVDL1_BIT = 0x0001
HLVDL2_BIT = 0x0002
HLVDL3_BIT = 0x0003
HLVDEN_BIT = 0x0004
VDIRMAG_BIT = 0x0007

IVRST_BIT = 0x0005


#;----- OSCCON Bits -----------------------------------------------------
SCS0_BIT = 0x0000
SCS1_BIT = 0x0001
IOFS_BIT = 0x0002
OSTS_BIT = 0x0003
IRCF0_BIT = 0x0004
IRCF1_BIT = 0x0005
IRCF2_BIT = 0x0006
IDLEN_BIT = 0x0007


#;----- T0CON Bits -----------------------------------------------------
T0PS0_BIT = 0x0000
T0PS1_BIT = 0x0001
T0PS2_BIT = 0x0002
PSA_BIT = 0x0003
T0SE_BIT = 0x0004
T0CS_BIT = 0x0005
T08BIT_BIT = 0x0006
TMR0ON_BIT = 0x0007

T0PS3_BIT = 0x0003


#;----- STATUS Bits -----------------------------------------------------
C_BIT = 0x0000
DC_BIT = 0x0001
Z_BIT = 0x0002
OV_BIT = 0x0003
N_BIT = 0x0004


#;----- INTCON3 Bits -----------------------------------------------------
INT1IF_BIT = 0x0000
INT2IF_BIT = 0x0001
INT1IE_BIT = 0x0003
INT2IE_BIT = 0x0004
INT1IP_BIT = 0x0006
INT2IP_BIT = 0x0007

INT1F_BIT = 0x0000
INT2F_BIT = 0x0001
INT1E_BIT = 0x0003
INT2E_BIT = 0x0004
INT1P_BIT = 0x0006
INT2P_BIT = 0x0007


#;----- INTCON2 Bits -----------------------------------------------------
RBIP_BIT = 0x0000
TMR0IP_BIT = 0x0002
INTEDG2_BIT = 0x0004
INTEDG1_BIT = 0x0005
INTEDG0_BIT = 0x0006
NOT_RBPU_BIT = 0x0007

T0IP_BIT = 0x0002
RBPU_BIT = 0x0007


#;----- INTCON Bits -----------------------------------------------------
RBIF_BIT = 0x0000
INT0IF_BIT = 0x0001
TMR0IF_BIT = 0x0002
RBIE_BIT = 0x0003
INT0IE_BIT = 0x0004
TMR0IE_BIT = 0x0005
PEIE_BIT = 0x0006
GIE_BIT = 0x0007

INT0F_BIT = 0x0001
T0IF_BIT = 0x0002
INT0E_BIT = 0x0004
T0IE_BIT = 0x0005
GIEL_BIT = 0x0006
GIEH_BIT = 0x0007


#;----- STKPTR Bits -----------------------------------------------------
STKPTR0_BIT = 0x0000
STKPTR1_BIT = 0x0001
STKPTR2_BIT = 0x0002
STKPTR3_BIT = 0x0003
STKPTR4_BIT = 0x0004
STKUNF_BIT = 0x0006
STKOVF_BIT = 0x0007

STKFUL_BIT = 0x0007



#=========================================================================
#
#       RAM Definition
#
#=========================================================================

#=========================================================================
#
#   IMPORTANT: For the PIC18 devices, the __CONFIG directive has bee
#              superseded by the CONFIG directive.  The following setting
#              are available for this device
#
#   Oscillator Selection bits
#     OSC = LP             LP oscillato
#     OSC = XT             XT oscillato
#     OSC = HS             HS oscillato
#     OSC = RC             External RC oscillator, CLKO function on RA
#     OSC = EC             EC oscillator, CLKO function on RA
#     OSC = ECIO           EC oscillator, port function on RA
#     OSC = HSPLL          HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1
#     OSC = RCIO           External RC oscillator, port function on RA
#     OSC = IRCIO67        Internal oscillator block, port function on RA6 and RA
#     OSC = IRCIO7         Internal oscillator block, CLKO function on RA6, port function on RA
#
#   Fail-Safe Clock Monitor Enable bit
#     FCMEN = OFF          Fail-Safe Clock Monitor disable
#     FCMEN = ON           Fail-Safe Clock Monitor enable
#
#   Internal/External Oscillator Switchover bit
#     IESO = OFF           Oscillator Switchover mode disable
#     IESO = ON            Oscillator Switchover mode enable
#
#   Power-up Timer Enable bit
#     PWRT = ON            PWRT enable
#     PWRT = OFF           PWRT disable
#
#   Brown-out Reset Enable bits
#     BOREN = OFF          Brown-out Reset disabled in hardware and softwar
#     BOREN = SBORENCTRL   Brown-out Reset enabled and controlled by software (SBOREN is enabled
#     BOREN = BOACTIVE     Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled
#     BOREN = BOHW         Brown-out Reset enabled in hardware only (SBOREN is disabled
#
#   Brown-out Reset Voltage bits
#     BORV = 0             Maximum Settin
#     BORV = 1            
#     BORV = 2            
#     BORV = 3             Minimum Settin
#
#   Watchdog Timer Enable bit
#     WDT = OFF            HW Disabled - SW Controlle
#     WDT = ON             HW Enabled - SW Disable
#
#   Watchdog Timer Postscale Select bits
#     WDTPS = 1            1:
#     WDTPS = 2            1:
#     WDTPS = 4            1:
#     WDTPS = 8            1:
#     WDTPS = 16           1:1
#     WDTPS = 32           1:3
#     WDTPS = 64           1:6
#     WDTPS = 128          1:12
#     WDTPS = 256          1:25
#     WDTPS = 512          1:51
#     WDTPS = 1024         1:102
#     WDTPS = 2048         1:204
#     WDTPS = 4096         1:409
#     WDTPS = 8192         1:819
#     WDTPS = 16384        1:1638
#     WDTPS = 32768        1:3276
#
#   MCLR Pin Enable bit
#     MCLRE = OFF          RE3 input pin enabled; MCLR disable
#     MCLRE = ON           MCLR pin enabled; RE3 input pin disable
#
#   Low-Power Timer 1 Oscillator Enable bit
#     LPT1OSC = OFF        Timer1 configured for higher power operatio
#     LPT1OSC = ON         Timer1 configured for low-power operatio
#
#   PORTB Pins Configured for A/D
#     PBADEN = OFF         PORTB<4> and PORTB<1:0> Configured as Digital I/O Pins on Rese
#     PBADEN = ON          PORTB<4> and PORTB<1:0> Configured as Analog Pins on Rese
#
#   Background Debugger Enable bit
#     DEBUG = ON           Background debugger enabled, RB6 and RB7 are dedicated to In-Circuit Debu
#     DEBUG = OFF          Background debugger disabled, RB6 and RB7 configured as general purpose I/O pin
#
#   Extended Instruction Set Enable bit
#     XINST = OFF          Instruction set extension and Indexed Addressing mode disabled (Legacy mode
#     XINST = ON           Instruction set extension and Indexed Addressing mode enable
#
#   Boot Block Size
#     BBSIZ = 1024         1K words (2K bytes) Boot Bloc
#     BBSIZ = 2048         2K words (4K bytes) Boot Bloc
#     BBSIZ = 4096         4K words (8K bytes) Boot Bloc
#
#   Single-Supply ICSP Enable bit
#     LVP = OFF            Single-Supply ICSP disable
#     LVP = ON             Single-Supply ICSP enable
#
#   Stack Full/Underflow Reset Enable bit
#     STVREN = OFF         Stack full/underflow will not cause Rese
#     STVREN = ON          Stack full/underflow will cause Rese
#
#   Code Protection Block 0
#     CP0 = ON             Block 0 (000800-003FFFh) code-protecte
#     CP0 = OFF            Block 0 (000800-003FFFh) not code-protecte
#
#   Code Protection Block 1
#     CP1 = ON             Block 1 (004000-007FFFh) code-protecte
#     CP1 = OFF            Block 1 (004000-007FFFh) not code-protecte
#
#   Code Protection Block 2
#     CP2 = ON             Block 2 (008000-00BFFFh) code-protecte
#     CP2 = OFF            Block 2 (008000-00BFFFh) not code-protecte
#
#   Code Protection Block 3
#     CP3 = ON             Block 3 (00C000-00FFFFh) code-protecte
#     CP3 = OFF            Block 3 (00C000-00FFFFh) not code-protecte
#
#   Boot Block Code Protection
#     CPB = ON             Boot block (000000-0007FFh) code-protecte
#     CPB = OFF            Boot block (000000-0007FFh) not code-protecte
#
#   Data EEPROM Code Protection
#     CPD = ON             Data EEPROM code-protecte
#     CPD = OFF            Data EEPROM not code-protecte
#
#   Write Protection Block 0
#     WRT0 = ON            Block 0 (000800-003FFFh) write-protecte
#     WRT0 = OFF           Block 0 (000800-003FFFh) not write-protecte
#
#   Write Protection Block 1
#     WRT1 = ON            Block 1 (004000-007FFFh) write-protecte
#     WRT1 = OFF           Block 1 (004000-007FFFh) not write-protecte
#
#   Write Protection Block 2
#     WRT2 = ON            Block 2 (008000-00BFFFh) write-protecte
#     WRT2 = OFF           Block 2 (008000-00BFFFh) not write-protecte
#
#   Write Protection Block 3
#     WRT3 = ON            Block 3 (00C000-00FFFFh) write-protecte
#     WRT3 = OFF           Block 3 (00C000-00FFFFh) not write-protecte
#
#   Boot Block Write Protection
#     WRTB = ON            Boot block (000000-0007FFh) write-protecte
#     WRTB = OFF           Boot block (000000-0007FFh) not write-protecte
#
#   Configuration Register Write Protection
#     WRTC = ON            Configuration registers (300000-3000FFh) write-protecte
#     WRTC = OFF           Configuration registers (300000-3000FFh) not write-protecte
#
#   Data EEPROM Write Protection
#     WRTD = ON            Data EEPROM write-protecte
#     WRTD = OFF           Data EEPROM not write-protecte
#
#   Table Read Protection Block 0
#     EBTR0 = ON           Block 0 (000800-003FFFh) protected from table reads executed in other block
#     EBTR0 = OFF          Block 0 (000800-003FFFh) not protected from table reads executed in other block
#
#   Table Read Protection Block 1
#     EBTR1 = ON           Block 1 (004000-007FFFh) protected from table reads executed in other block
#     EBTR1 = OFF          Block 1 (004000-007FFFh) not protected from table reads executed in other block
#
#   Table Read Protection Block 2
#     EBTR2 = ON           Block 2 (008000-00BFFFh) protected from table reads executed in other block
#     EBTR2 = OFF          Block 2 (008000-00BFFFh) not protected from table reads executed in other block
#
#   Table Read Protection Block 3
#     EBTR3 = ON           Block 3 (00C000-00FFFFh) protected from table reads executed in other block
#     EBTR3 = OFF          Block 3 (00C000-00FFFFh) not protected from table reads executed in other block
#
#   Boot Block Table Read Protection
#     EBTRB = ON           Boot block (000000-0007FFh) protected from table reads executed in other block
#     EBTRB = OFF          Boot block (000000-0007FFh) not protected from table reads executed in other block
#
#=========================================================================
#=========================================================================
#
#       Configuration Bit
#
#   NAME            Addres
#   CONFIG1H        300001
#   CONFIG2L        300002
#   CONFIG2H        300003
#   CONFIG3H        300005
#   CONFIG4L        300006
#   CONFIG5L        300008
#   CONFIG5H        300009
#   CONFIG6L        30000A
#   CONFIG6H        30000B
#   CONFIG7L        30000C
#   CONFIG7H        30000D
#
#=========================================================================

# The following is an assignment of address values for all of th
# configuration registers for the purpose of table read
_CONFIG1H = 0x300001
_CONFIG2L = 0x300002
_CONFIG2H = 0x300003
_CONFIG3H = 0x300005
_CONFIG4L = 0x300006
_CONFIG5L = 0x300008
_CONFIG5H = 0x300009
_CONFIG6L = 0x30000A
_CONFIG6H = 0x30000B
_CONFIG7L = 0x30000C
_CONFIG7H = 0x30000D

#----- CONFIG1H Options -------------------------------------------------
_OSC_LP_1H = 0x00F0 #  LP oscillator
_OSC_XT_1H = 0x00F1 #  XT oscillator
_OSC_HS_1H = 0x00F2 #  HS oscillator
_OSC_RC_1H = 0x00F3 #  External RC oscillator, CLKO function on RA6
_OSC_EC_1H = 0x00F4 #  EC oscillator, CLKO function on RA6
_OSC_ECIO_1H = 0x00F5 #  EC oscillator, port function on RA6
_OSC_HSPLL_1H = 0x00F6 #  HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1)
_OSC_RCIO_1H = 0x00F7 #  External RC oscillator, port function on RA6
_OSC_IRCIO67_1H = 0x00F8 #  Internal oscillator block, port function on RA6 and RA7
_OSC_IRCIO7_1H = 0x00F9 #  Internal oscillator block, CLKO function on RA6, port function on RA7

_FCMEN_OFF_1H = 0x00BF #  Fail-Safe Clock Monitor disabled
_FCMEN_ON_1H = 0x00FF #  Fail-Safe Clock Monitor enabled

_IESO_OFF_1H = 0x007F #  Oscillator Switchover mode disabled
_IESO_ON_1H = 0x00FF #  Oscillator Switchover mode enabled

#----- CONFIG2L Options -------------------------------------------------
_PWRT_ON_2L = 0x00FE #  PWRT enabled
_PWRT_OFF_2L = 0x00FF #  PWRT disabled

_BOREN_OFF_2L = 0x00F9 #  Brown-out Reset disabled in hardware and software
_BOREN_SBORENCTRL_2L = 0x00FB #  Brown-out Reset enabled and controlled by software (SBOREN is enabled)
_BOREN_BOACTIVE_2L = 0x00FD #  Brown-out Reset enabled in hardware only and disabled in Sleep mode (SBOREN is disabled)
_BOREN_BOHW_2L = 0x00FF #  Brown-out Reset enabled in hardware only (SBOREN is disabled)

_BORV_0_2L = 0x00E7 #  Maximum Setting
_BORV_1_2L = 0x00EF #  
_BORV_2_2L = 0x00F7 #  
_BORV_3_2L = 0x00FF #  Minimum Setting

#----- CONFIG2H Options -------------------------------------------------
_WDT_OFF_2H = 0x00FE #  HW Disabled - SW Controlled
_WDT_ON_2H = 0x00FF #  HW Enabled - SW Disabled

_WDTPS_1_2H = 0x00E1 #  1:1
_WDTPS_2_2H = 0x00E3 #  1:2
_WDTPS_4_2H = 0x00E5 #  1:4
_WDTPS_8_2H = 0x00E7 #  1:8
_WDTPS_16_2H = 0x00E9 #  1:16
_WDTPS_32_2H = 0x00EB #  1:32
_WDTPS_64_2H = 0x00ED #  1:64
_WDTPS_128_2H = 0x00EF #  1:128
_WDTPS_256_2H = 0x00F1 #  1:256
_WDTPS_512_2H = 0x00F3 #  1:512
_WDTPS_1024_2H = 0x00F5 #  1:1024
_WDTPS_2048_2H = 0x00F7 #  1:2048
_WDTPS_4096_2H = 0x00F9 #  1:4096
_WDTPS_8192_2H = 0x00FB #  1:8192
_WDTPS_16384_2H = 0x00FD #  1:16384
_WDTPS_32768_2H = 0x00FF #  1:32768

#----- CONFIG3H Options -------------------------------------------------
_MCLRE_OFF_3H = 0x007F #  RE3 input pin enabled; MCLR disabled
_MCLRE_ON_3H = 0x00FF #  MCLR pin enabled; RE3 input pin disabled

_LPT1OSC_OFF_3H = 0x00FB #  Timer1 configured for higher power operation
_LPT1OSC_ON_3H = 0x00FF #  Timer1 configured for low-power operation

_PBADEN_OFF_3H = 0x00FD #  PORTB<4> and PORTB<1:0> Configured as Digital I/O Pins on Reset
_PBADEN_ON_3H = 0x00FF #  PORTB<4> and PORTB<1:0> Configured as Analog Pins on Reset

#----- CONFIG4L Options -------------------------------------------------
_DEBUG_ON_4L = 0x007F #  Background debugger enabled, RB6 and RB7 are dedicated to In-Circuit Debug
_DEBUG_OFF_4L = 0x00FF #  Background debugger disabled, RB6 and RB7 configured as general purpose I/O pins

_XINST_OFF_4L = 0x00BF #  Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
_XINST_ON_4L = 0x00FF #  Instruction set extension and Indexed Addressing mode enabled

_BBSIZ_1024_4L = 0x00CF #  1K words (2K bytes) Boot Block
_BBSIZ_2048_4L = 0x00DF #  2K words (4K bytes) Boot Block
_BBSIZ_4096_4L = 0x00EF #  4K words (8K bytes) Boot Block

_LVP_OFF_4L = 0x00FB #  Single-Supply ICSP disabled
_LVP_ON_4L = 0x00FF #  Single-Supply ICSP enabled

_STVREN_OFF_4L = 0x00FE #  Stack full/underflow will not cause Reset
_STVREN_ON_4L = 0x00FF #  Stack full/underflow will cause Reset

#----- CONFIG5L Options -------------------------------------------------
_CP0_ON_5L = 0x00FE #  Block 0 (000800-003FFFh) code-protected
_CP0_OFF_5L = 0x00FF #  Block 0 (000800-003FFFh) not code-protected

_CP1_ON_5L = 0x00FD #  Block 1 (004000-007FFFh) code-protected
_CP1_OFF_5L = 0x00FF #  Block 1 (004000-007FFFh) not code-protected

_CP2_ON_5L = 0x00FB #  Block 2 (008000-00BFFFh) code-protected
_CP2_OFF_5L = 0x00FF #  Block 2 (008000-00BFFFh) not code-protected

_CP3_ON_5L = 0x00F7 #  Block 3 (00C000-00FFFFh) code-protected
_CP3_OFF_5L = 0x00FF #  Block 3 (00C000-00FFFFh) not code-protected

#----- CONFIG5H Options -------------------------------------------------
_CPB_ON_5H = 0x00BF #  Boot block (000000-0007FFh) code-protected
_CPB_OFF_5H = 0x00FF #  Boot block (000000-0007FFh) not code-protected

_CPD_ON_5H = 0x007F #  Data EEPROM code-protected
_CPD_OFF_5H = 0x00FF #  Data EEPROM not code-protected

#----- CONFIG6L Options -------------------------------------------------
_WRT0_ON_6L = 0x00FE #  Block 0 (000800-003FFFh) write-protected
_WRT0_OFF_6L = 0x00FF #  Block 0 (000800-003FFFh) not write-protected

_WRT1_ON_6L = 0x00FD #  Block 1 (004000-007FFFh) write-protected
_WRT1_OFF_6L = 0x00FF #  Block 1 (004000-007FFFh) not write-protected

_WRT2_ON_6L = 0x00FB #  Block 2 (008000-00BFFFh) write-protected
_WRT2_OFF_6L = 0x00FF #  Block 2 (008000-00BFFFh) not write-protected

_WRT3_ON_6L = 0x00F7 #  Block 3 (00C000-00FFFFh) write-protected
_WRT3_OFF_6L = 0x00FF #  Block 3 (00C000-00FFFFh) not write-protected

#----- CONFIG6H Options -------------------------------------------------
_WRTB_ON_6H = 0x00BF #  Boot block (000000-0007FFh) write-protected
_WRTB_OFF_6H = 0x00FF #  Boot block (000000-0007FFh) not write-protected

_WRTC_ON_6H = 0x00DF #  Configuration registers (300000-3000FFh) write-protected
_WRTC_OFF_6H = 0x00FF #  Configuration registers (300000-3000FFh) not write-protected

_WRTD_ON_6H = 0x007F #  Data EEPROM write-protected
_WRTD_OFF_6H = 0x00FF #  Data EEPROM not write-protected

#----- CONFIG7L Options -------------------------------------------------
_EBTR0_ON_7L = 0x00FE #  Block 0 (000800-003FFFh) protected from table reads executed in other blocks
_EBTR0_OFF_7L = 0x00FF #  Block 0 (000800-003FFFh) not protected from table reads executed in other blocks

_EBTR1_ON_7L = 0x00FD #  Block 1 (004000-007FFFh) protected from table reads executed in other blocks
_EBTR1_OFF_7L = 0x00FF #  Block 1 (004000-007FFFh) not protected from table reads executed in other blocks

_EBTR2_ON_7L = 0x00FB #  Block 2 (008000-00BFFFh) protected from table reads executed in other blocks
_EBTR2_OFF_7L = 0x00FF #  Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks

_EBTR3_ON_7L = 0x00F7 #  Block 3 (00C000-00FFFFh) protected from table reads executed in other blocks
_EBTR3_OFF_7L = 0x00FF #  Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks

#----- CONFIG7H Options -------------------------------------------------
_EBTRB_ON_7H = 0x00BF #  Boot block (000000-0007FFh) protected from table reads executed in other blocks
_EBTRB_OFF_7H = 0x00FF #  Boot block (000000-0007FFh) not protected from table reads executed in other blocks


_DEVID1 = 0x3FFFFE
_DEVID2 = 0x3FFFFF

_IDLOC0 = 0x200000
_IDLOC1 = 0x200001
_IDLOC2 = 0x200002
_IDLOC3 = 0x200003
_IDLOC4 = 0x200004
_IDLOC5 = 0x200005
_IDLOC6 = 0x200006
_IDLOC7 = 0x200007


#
#  #define-s converted to assigments:
#

