;Program compiled by Great Cow BASIC (0.99.01 2022-01-27 (Windows 64 bit) : Build 1073) for Microchip MPASM
;Need help? See the GCBASIC forums at http://sourceforge.net/projects/gcbasic/forums,
;check the documentation or email w_cholmondeley at users dot sourceforge dot net.

;********************************************************************************

;Set up the assembler options (Chip type, clock source, other bits and pieces)
 LIST p=16F18875, r=DEC
#include <P16F18875.inc>
 __CONFIG _CONFIG1, _FCMEN_ON & _CLKOUTEN_OFF & _RSTOSC_HFINT32 & _FEXTOSC_OFF
 __CONFIG _CONFIG2, _MCLRE_OFF
 __CONFIG _CONFIG3, _WDTE_OFF
 __CONFIG _CONFIG4, _LVP_OFF & _WRT_OFF
 __CONFIG _CONFIG5, _CPD_OFF & _CP_OFF

;********************************************************************************

;Set aside memory locations for variables
ADREADPORT                       EQU 32
DELAYTEMP                        EQU 112
DELAYTEMP2                       EQU 113
DISTANCEFRONT                    EQU 33
DISTANCELEFT                     EQU 34
LCDBYTE                          EQU 35
LCDCHAR                          EQU 36
LCDCOLUMN                        EQU 37
LCDLINE                          EQU 38
LCDVALUE                         EQU 39
LCDVALUEINT                      EQU 41
LCDVALUEINT_H                    EQU 42
LCDVALUETEMP                     EQU 43
LCDVALUE_H                       EQU 40
LCD_STATE                        EQU 44
LEFTCOUNTER                      EQU 45
LEFTCOUNTER_H                    EQU 46
READAD                           EQU 47
ROOMCOUNTER                      EQU 48
ROOMCOUNTER_H                    EQU 49
STRINGPOINTER                    EQU 50
SYSBYTETEMPA                     EQU 117
SYSBYTETEMPB                     EQU 121
SYSBYTETEMPX                     EQU 112
SYSCALCTEMPX                     EQU 112
SYSCALCTEMPX_H                   EQU 113
SYSDIVLOOP                       EQU 116
SYSDIVMULTA                      EQU 119
SYSDIVMULTA_H                    EQU 120
SYSDIVMULTB                      EQU 123
SYSDIVMULTB_H                    EQU 124
SYSDIVMULTX                      EQU 114
SYSDIVMULTX_H                    EQU 115
SYSINTEGERTEMPA                  EQU 117
SYSINTEGERTEMPA_H                EQU 118
SYSINTEGERTEMPB                  EQU 121
SYSINTEGERTEMPB_H                EQU 122
SYSLCDTEMP                       EQU 51
SYSREPEATTEMP1                   EQU 52
SYSSTRINGA                       EQU 119
SYSSTRINGA_H                     EQU 120
SYSTEMP1                         EQU 53
SYSTEMP1_H                       EQU 54
SYSTEMP2                         EQU 55
SYSTEMP2_H                       EQU 56
SYSWAITTEMP10US                  EQU 117
SYSWAITTEMPMS                    EQU 114
SYSWAITTEMPMS_H                  EQU 115
SYSWAITTEMPUS                    EQU 117
SYSWAITTEMPUS_H                  EQU 118
SYSWORDTEMPA                     EQU 117
SYSWORDTEMPA_H                   EQU 118
SYSWORDTEMPB                     EQU 121
SYSWORDTEMPB_H                   EQU 122
SYSWORDTEMPX                     EQU 112
SYSWORDTEMPX_H                   EQU 113
VOLTAGEFLAME                     EQU 57
VOLTAGEFRONT                     EQU 58
VOLTAGELEFT                      EQU 59
XVAR                             EQU 60
XVAR_H                           EQU 61

;********************************************************************************

;Alias variables
SYSREADADBYTE EQU 47

;********************************************************************************

;Vectors
	ORG	0
	pagesel	BASPROGRAMSTART
	goto	BASPROGRAMSTART
	ORG	4
	retfie

;********************************************************************************

;Start of program memory page 0
	ORG	5
BASPROGRAMSTART
;Call initialisation routines
	call	INITSYS
	call	INITLCD

;Start of the main program
;' FIREFIGHTER CODE: MAZE 2''
;' 16/12/2022 ''
;' Antonio Rech Santos and Guhan Iyer ''
;Chip definition
;LCD definitions
;#DEFINE LCD_IO 4
;#DEFINE LCD_RS portd.0
;#DEFINE LCD_RW portd.1
;#DEFINE LCD_ENABLE portd.2
;#DEFINE LCD_DB4 portd.4
;#DEFINE LCD_DB5 portd.5
;#DEFINE LCD_DB6 portd.6
;#DEFINE LCD_DB7 portd.7
;Sensor definitions
;#DEFINE LW porta.0
;#DEFINE FW porta.1
;#DEFINE FS porta.2
;#DEFINE LS porta.3
;Motor/Fan definitions
;#DEFINE RMF portb.0
;#DEFINE RMB portb.1
;#DEFINE LMF portb.3
;#DEFINE LMB portb.2
;#DEFINE FAN portb.4
;Input/Output port direction
;DIR porta IN
	movlw	255
	movwf	TRISA
;DIR portb OUT
	clrf	TRISB
;Variable definitions
;DIM voltage AS BYTE
;DIM distanceFront AS BYTE
;DIM voltageLeft AS BYTE
;DIM distanceLeft AS BYTE
;DIM voltageFlame AS BYTE
;DIM roomCounter AS INTEGER
;DIM xVar AS WORD
;DIM leftCounter AS INTEGER
;roomCounter = 0
	clrf	ROOMCOUNTER
	clrf	ROOMCOUNTER_H
;leftCounter = 0
	clrf	LEFTCOUNTER
	clrf	LEFTCOUNTER_H
MAIN
;gosub followWall
	call	FOLLOWWALL
;goto main
	goto	MAIN
BASPROGRAMEND
	sleep
	goto	BASPROGRAMEND

;********************************************************************************

;Source: maze2.gcb (360)
BACKWARDS
;RMF = 0
	bcf	LATB,0
;LMF = 0
	bcf	LATB,3
;RMB = 1
	bsf	LATB,1
;LMB = 1
	bsf	LATB,2
;return
	return
;Sets all motors to 'low', stopping motion
	return

;********************************************************************************

;Source: lcd.h (955)
CHECKBUSYFLAG
;Sub that waits until LCD controller busy flag goes low (ready)
;Only used by LCD_IO 4,8 and only when LCD_NO_RW is NOT Defined
;Called by sub LCDNOrmalWriteByte
;LCD_RSTemp = LCD_RS
	bcf	SYSLCDTEMP,2
	btfsc	PORTD,0
	bsf	SYSLCDTEMP,2
;DIR SCRIPT_LCD_BF  IN
	bsf	TRISD,7
;SET LCD_RS OFF
	bcf	LATD,0
;SET LCD_RW ON
	bsf	LATD,1
;Do
SysDoLoop_S13
;Set LCD_Enable ON
	bsf	LATD,2
;wait 1 us
	movlw	2
	movwf	DELAYTEMP
DelayUS14
	decfsz	DELAYTEMP,F
	goto	DelayUS14
	nop
;SysLCDTemp.7 = SCRIPT_LCD_BF
	bcf	SYSLCDTEMP,7
	btfsc	PORTD,7
	bsf	SYSLCDTEMP,7
;Set LCD_Enable OFF
	bcf	LATD,2
;Wait 1 us
	movlw	2
	movwf	DELAYTEMP
DelayUS15
	decfsz	DELAYTEMP,F
	goto	DelayUS15
	nop
;PulseOut LCD_Enable, 1 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	2
	movwf	DELAYTEMP
DelayUS16
	decfsz	DELAYTEMP,F
	goto	DelayUS16
;Set Pin Off
	bcf	LATD,2
;Wait 1 us
	movlw	2
	movwf	DELAYTEMP
DelayUS17
	decfsz	DELAYTEMP,F
	goto	DelayUS17
	nop
;Loop While SysLCDTemp.7 <> 0
	btfsc	SYSLCDTEMP,7
	goto	SysDoLoop_S13
SysDoLoop_E13
;LCD_RS = LCD_RSTemp
	bcf	LATD,0
	btfsc	SYSLCDTEMP,2
	bsf	LATD,0
	return

;********************************************************************************

;Source: maze2.gcb (400)
CLOSELEFT
;RMF = 0
	bcf	LATB,0
;LMF = 1
	bsf	LATB,3
;LMB = 0
	bcf	LATB,2
;RMB = 0
	bcf	LATB,1
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 3 ms
	movlw	3
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;Turn that is performed when the bot is too far from the left wall
	return

;********************************************************************************

;Source: maze2.gcb (446)
CLOSELEFTB
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub backwards
	call	BACKWARDS
;wait 3 ms
	movlw	3
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;Turn performed when the bot is too far to the left wall. Needed for the bot's reversal procedure in room 3.
	return

;********************************************************************************

;Source: maze2.gcb (426)
CLOSELEFTS
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 2 ms
	movlw	2
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;Turn that is performed when the bot is too far from the left wall. Sharper turn than farLeft.
	return

;********************************************************************************

;Source: lcd.h (364)
CLS
;Sub to clear the LCD
;SET LCD_RS OFF
	bcf	LATD,0
;Clear screen
;LCDWriteByte (0b00000001)
	movlw	1
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Wait 4 ms
	movlw	4
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Move to start of visible DDRAM
;LCDWriteByte(0x80)
	movlw	128
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Wait 50 us
	movlw	133
	movwf	DELAYTEMP
DelayUS1
	decfsz	DELAYTEMP,F
	goto	DelayUS1
	return

;********************************************************************************

Delay_10US
D10US_START
	movlw	25
	movwf	DELAYTEMP
DelayUS0
	decfsz	DELAYTEMP,F
	goto	DelayUS0
	nop
	decfsz	SysWaitTemp10US, F
	goto	D10US_START
	return

;********************************************************************************

Delay_MS
	incf	SysWaitTempMS_H, F
DMS_START
	movlw	14
	movwf	DELAYTEMP2
DMS_OUTER
	movlw	189
	movwf	DELAYTEMP
DMS_INNER
	decfsz	DELAYTEMP, F
	goto	DMS_INNER
	decfsz	DELAYTEMP2, F
	goto	DMS_OUTER
	decfsz	SysWaitTempMS, F
	goto	DMS_START
	decfsz	SysWaitTempMS_H, F
	goto	DMS_START
	return

;********************************************************************************

;Source: maze2.gcb (413)
FARLEFT
;RMF = 1
	bsf	LATB,0
;LMF = 0
	bcf	LATB,3
;LMB = 0
	bcf	LATB,2
;RMB = 0
	bcf	LATB,1
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 3 ms
	movlw	3
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;Turn that is preformed when the bot is too close to the left wall. Sharper turn than closeLeft.
	return

;********************************************************************************

;Source: maze2.gcb (456)
FARLEFTB
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub backwards
	call	BACKWARDS
;wait 3 ms
	movlw	3
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;A gradual left turn.
	return

;********************************************************************************

;Source: maze2.gcb (436)
FARLEFTS
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 2 ms
	movlw	2
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;Turn performed when the bot is too close to the left wall. Needed for the bot's reversal procedure in room 3.
	return

;********************************************************************************

;Source: maze2.gcb (246)
FOLLOWWALL
;Printing values for debugging
;CLS
	call	CLS
;Locate(0, 1)
	clrf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print roomCounter
	movf	ROOMCOUNTER,W
	movwf	LCDVALUEINT
	movf	ROOMCOUNTER_H,W
	movwf	LCDVALUEINT_H
	call	PRINT127
;Locate(1,1)
	movlw	1
	movwf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print distanceLeft
	movf	DISTANCELEFT,W
	movwf	LCDVALUE
	call	PRINT125
;Read sensor values before proceeding
;gosub readSensors
	call	READSENSORS
;If a line is detected
;if LS = 0 then
	btfsc	PORTA,3
	goto	ELSE1_1
;Increment the room counter
;roomCounter += 1
	incf	ROOMCOUNTER,F
	btfsc	STATUS,Z
	incf	ROOMCOUNTER_H,F
;Go forwards to move the bot off the line and prevent it from being read twice
;gosub forwards
	call	FORWARDS
;wait 100 ms
	movlw	100
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;When roomCounter = 3 for maze 2, run procedures for room 3
;if roomCounter = 3 then
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	3
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	btfsc	SysByteTempX,0
;gosub room3
	call	ROOM3
;end if
;When roomCounter = 4, the bot has exited room 3. When distanceFront <= 16, the bot is nearing the turn into room 4.
;else if roomCounter = 4 and distanceFront <= 16 then
	goto	ENDIF1
ELSE1_1
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	4
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	movf	SysByteTempX,W
	movwf	SysTemp1
	movf	DISTANCEFRONT,W
	movwf	SysBYTETempB
	movlw	16
	movwf	SysBYTETempA
	call	SYSCOMPLESSTHAN
	comf	SysByteTempX,F
	movf	SysTemp1,W
	andwf	SysByteTempX,W
	movwf	SysTemp2
	btfss	SysTemp2,0
	goto	ELSE1_2
;roomCounter += 1
	incf	ROOMCOUNTER,F
	btfsc	STATUS,Z
	incf	ROOMCOUNTER_H,F
;Go forward for alignment
;gosub forwards
	call	FORWARDS
;wait 100 ms
	movlw	100
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Turn left into room 4
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 350 ms
	movlw	94
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;Go forward until the bot crosses the line and is in room 4
;Do Until LS = 0
SysDoLoop_S1
	btfss	PORTA,3
	goto	SysDoLoop_E1
;gosub forwards
	call	FORWARDS
;gosub readSensors
	call	READSENSORS
;loop
	goto	SysDoLoop_S1
SysDoLoop_E1
;gosub room4
	call	ROOM4
;' Conditions to run turning subroutines ''
;else if voltageFlame <= 12 then
	goto	ENDIF1
ELSE1_2
	movf	VOLTAGEFLAME,W
	sublw	12
	btfss	STATUS, C
	goto	ELSE1_3
;gosub outFlame
	call	OUTFLAME
;else if distanceFront <= 15 then
	goto	ENDIF1
ELSE1_3
	movf	DISTANCEFRONT,W
	sublw	15
	btfss	STATUS, C
	goto	ELSE1_4
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;else if distanceLeft >= 23 then
	goto	ENDIF1
ELSE1_4
	movlw	23
	subwf	DISTANCELEFT,W
	btfss	STATUS, C
	goto	ELSE1_5
;gosub smoothTurnLeft
	call	SMOOTHTURNLEFT
;else if distanceLeft >= 15 then
	goto	ENDIF1
ELSE1_5
	movlw	15
	subwf	DISTANCELEFT,W
	btfss	STATUS, C
	goto	ELSE1_6
;gosub farLeftS
	call	FARLEFTS
;else if distanceLeft <= 8 then
	goto	ENDIF1
ELSE1_6
	movf	DISTANCELEFT,W
	sublw	8
	btfss	STATUS, C
	goto	ELSE1_7
;gosub closeLeftS
	call	CLOSELEFTS
;else if distanceLeft >= 13 then
	goto	ENDIF1
ELSE1_7
	movlw	13
	subwf	DISTANCELEFT,W
	btfss	STATUS, C
	goto	ELSE1_8
;gosub farLeft
	call	FARLEFT
;else if distanceLeft <= 10 then
	goto	ENDIF1
ELSE1_8
	movf	DISTANCELEFT,W
	sublw	10
	btfss	STATUS, C
	goto	ELSE1_9
;gosub closeLeft
	call	CLOSELEFT
;else
	goto	ENDIF1
ELSE1_9
;gosub forwards
	call	FORWARDS
;end if
ENDIF1
;return
	return
;'SUBROUTINES''
;Reads voltage values off wall detection sensors and linearizes them to compute distance, reads flame sensor voltage for flame detection
	return

;********************************************************************************

;Source: maze2.gcb (350)
FORWARDS
;RMF = 1
	bsf	LATB,0
;LMF = 1
	bsf	LATB,3
;RMB = 0
	bcf	LATB,1
;LMB = 0
	bcf	LATB,2
;return
	return
;Sets the backwards component of both motors to 'high'
	return

;********************************************************************************

;Source: lcd.h (437)
INITLCD
;asm showdebug  `LCD_IO selected is ` LCD_IO
;asm showdebug  `LCD_Speed is SLOW`
;asm showdebug  `OPTIMAL is set to ` OPTIMAL
;asm showdebug  `LCD_Speed is set to ` LCD_Speed
;Wait 50 ms
	movlw	50
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Dir LCD_RW OUT
	bcf	TRISD,1
;Set LCD_RW OFF
	bcf	LATD,1
;Dir LCD_DB4 OUT
	bcf	TRISD,4
;Dir LCD_DB5 OUT
	bcf	TRISD,5
;Dir LCD_DB6 OUT
	bcf	TRISD,6
;Dir LCD_DB7 OUT
	bcf	TRISD,7
;Dir LCD_RS OUT
	bcf	TRISD,0
;Dir LCD_Enable OUT
	bcf	TRISD,2
;Set LCD_RS OFF
	bcf	LATD,0
;Set LCD_Enable OFF
	bcf	LATD,2
;Wakeup (0x30 - b'0011xxxx' )
;Set LCD_DB7 OFF
	bcf	LATD,7
;Set LCD_DB6 OFF
	bcf	LATD,6
;Set LCD_DB5 ON
	bsf	LATD,5
;Set LCD_DB4 ON
	bsf	LATD,4
;Wait 2 us
	movlw	5
	movwf	DELAYTEMP
DelayUS2
	decfsz	DELAYTEMP,F
	goto	DelayUS2
;PulseOut LCD_Enable, 2 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	5
	movwf	DELAYTEMP
DelayUS3
	decfsz	DELAYTEMP,F
	goto	DelayUS3
;Set Pin Off
	bcf	LATD,2
;Wait 10 ms
	movlw	10
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Repeat 3
	movlw	3
	movwf	SysRepeatTemp1
SysRepeatLoop1
;PulseOut LCD_Enable, 2 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	5
	movwf	DELAYTEMP
DelayUS4
	decfsz	DELAYTEMP,F
	goto	DelayUS4
;Set Pin Off
	bcf	LATD,2
;Wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;End Repeat
	decfsz	SysRepeatTemp1,F
	goto	SysRepeatLoop1
SysRepeatLoopEnd1
;Set 4 bit mode (0x20 - b'0010xxxx')
;Set LCD_DB7 OFF
	bcf	LATD,7
;Set LCD_DB6 OFF
	bcf	LATD,6
;Set LCD_DB5 ON
	bsf	LATD,5
;Set LCD_DB4 OFF
	bcf	LATD,4
;Wait 2 us
	movlw	5
	movwf	DELAYTEMP
DelayUS5
	decfsz	DELAYTEMP,F
	goto	DelayUS5
;PulseOut LCD_Enable, 2 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	5
	movwf	DELAYTEMP
DelayUS6
	decfsz	DELAYTEMP,F
	goto	DelayUS6
;Set Pin Off
	bcf	LATD,2
;Wait 100 us
	movlw	1
	movwf	DELAYTEMP2
DelayUSO7
	clrf	DELAYTEMP
DelayUS7
	decfsz	DELAYTEMP,F
	goto	DelayUS7
	decfsz	DELAYTEMP2,F
	goto	DelayUSO7
	movlw	9
	movwf	DELAYTEMP
DelayUS8
	decfsz	DELAYTEMP,F
	goto	DelayUS8
;===== now in 4 bit mode =====
;LCDWriteByte 0x28    '(b'00101000')  '0x28 set 2 line mode
	movlw	40
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;LCDWriteByte 0x06    '(b'00000110')  'Set cursor movement
	movlw	6
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;LCDWriteByte 0x0C    '(b'00001100')  'Turn off cursor
	movlw	12
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Cls  'Clear the display
	call	CLS
;LCD_State = 12
	movlw	12
	movwf	LCD_STATE
	return

;********************************************************************************

;Source: system.h (156)
INITSYS
;asm showdebug This code block sets the internal oscillator to ChipMHz
;asm showdebug Default settings for microcontrollers with _OSCCON1_
;Default OSCCON1 typically, NOSC HFINTOSC; NDIV 1 - Common as this simply sets the HFINTOSC
;OSCCON1 = 0x60
	movlw	96
	banksel	OSCCON1
	movwf	OSCCON1
;Default value typically, CSWHOLD may proceed; SOSCPWR Low power
;OSCCON3 = 0x00
	clrf	OSCCON3
;Default value typically, MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled
;OSCEN = 0x00
	clrf	OSCEN
;Default value
;OSCTUNE = 0x00
	clrf	OSCTUNE
;asm showdebug The MCU is a chip family ChipFamily
;asm showdebug OSCCON type is 102
;Set OSCFRQ values for MCUs with OSCSTAT... the 16F18855 MCU family
;OSCFRQ = 0b00000110
	movlw	6
	movwf	OSCFRQ
;asm showdebug _Complete_the_chip_setup_of_BSR,ADCs,ANSEL_and_other_key_setup_registers_or_register_bits
;Ensure all ports are set for digital I/O and, turn off A/D
;SET ADFM OFF
	banksel	ADCON0
	bcf	ADCON0,ADFRM0
;Switch off A/D Var(ADCON0)
;SET ADCON0.ADON OFF
	bcf	ADCON0,ADON
;ANSELA = 0
	banksel	ANSELA
	clrf	ANSELA
;ANSELB = 0
	clrf	ANSELB
;ANSELC = 0
	clrf	ANSELC
;ANSELD = 0
	clrf	ANSELD
;ANSELE = 0
	clrf	ANSELE
;Set comparator register bits for many MCUs with register CM2CON0
;C2ON = 0
	banksel	CM2CON0
	bcf	CM2CON0,C2ON
;C1ON = 0
	bcf	CM1CON0,C1ON
;
;'Turn off all ports
;PORTA = 0
	banksel	PORTA
	clrf	PORTA
;PORTB = 0
	clrf	PORTB
;PORTC = 0
	clrf	PORTC
;PORTD = 0
	clrf	PORTD
;PORTE = 0
	clrf	PORTE
	return

;********************************************************************************

;Source: lcd.h (1006)
LCDNORMALWRITEBYTE
;Sub to write a byte to the LCD
;CheckBusyFlag         'WaitForReady
	call	CHECKBUSYFLAG
;set LCD_RW OFF
	bcf	LATD,1
;Dim Temp as Byte
;Pins must be outputs if returning from WaitForReady, or after LCDReadByte or GET subs
;DIR LCD_DB4 OUT
	bcf	TRISD,4
;DIR LCD_DB5 OUT
	bcf	TRISD,5
;DIR LCD_DB6 OUT
	bcf	TRISD,6
;DIR LCD_DB7 OUT
	bcf	TRISD,7
;Write upper nibble to output pins
;set LCD_DB4 OFF
;set LCD_DB5 OFF
;set LCD_DB6 OFF
;set LCD_DB7 OFF
;if LCDByte.7 ON THEN SET LCD_DB7 ON
;if LCDByte.6 ON THEN SET LCD_DB6 ON
;if LCDByte.5 ON THEN SET LCD_DB5 ON
;if LCDByte.4 ON THEN SET LCD_DB4 ON
;LCD_DB7 = LCDByte.7
	bcf	LATD,7
	btfsc	LCDBYTE,7
	bsf	LATD,7
;LCD_DB6 = LCDByte.6
	bcf	LATD,6
	btfsc	LCDBYTE,6
	bsf	LATD,6
;LCD_DB5 = LCDByte.5
	bcf	LATD,5
	btfsc	LCDBYTE,5
	bsf	LATD,5
;LCD_DB4 = LCDByte.4
	bcf	LATD,4
	btfsc	LCDBYTE,4
	bsf	LATD,4
;Wait 1 us
	movlw	2
	movwf	DELAYTEMP
DelayUS9
	decfsz	DELAYTEMP,F
	goto	DelayUS9
	nop
;PulseOut LCD_enable, 1 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	2
	movwf	DELAYTEMP
DelayUS10
	decfsz	DELAYTEMP,F
	goto	DelayUS10
;Set Pin Off
	bcf	LATD,2
;All data pins low
;set LCD_DB4 OFF
;set LCD_DB5 OFF
;set LCD_DB6 OFF
;set LCD_DB7 OFF
	bcf	LATD,7
;
;'Write lower nibble to output pins
;if LCDByte.3 ON THEN SET LCD_DB7 ON
	btfsc	LCDBYTE,3
	bsf	LATD,7
;if LCDByte.2 ON THEN SET LCD_DB6 ON
;if LCDByte.1 ON THEN SET LCD_DB5 ON
;if LCDByte.0 ON THEN SET LCD_DB4 ON
;LCD_DB7 = LCDByte.3
;LCD_DB6 = LCDByte.2
	bcf	LATD,6
	btfsc	LCDBYTE,2
	bsf	LATD,6
;LCD_DB5 = LCDByte.1
	bcf	LATD,5
	btfsc	LCDBYTE,1
	bsf	LATD,5
;LCD_DB4 = LCDByte.0
	bcf	LATD,4
	btfsc	LCDBYTE,0
	bsf	LATD,4
;Wait 1 us
	movlw	2
	movwf	DELAYTEMP
DelayUS11
	decfsz	DELAYTEMP,F
	goto	DelayUS11
	nop
;PulseOut LCD_enable, 1 us
;Macro Source: stdbasic.h (186)
;Set Pin On
	bsf	LATD,2
;WaitL1 Time
	movlw	2
	movwf	DELAYTEMP
DelayUS12
	decfsz	DELAYTEMP,F
	goto	DelayUS12
;Set Pin Off
	bcf	LATD,2
;Set data pins low again
;SET LCD_DB7 OFF
;SET LCD_DB6 OFF
;SET LCD_DB5 OFF
;SET LCD_DB4 OFF
;Wait SCRIPT_LCD_POSTWRITEDELAY
	movlw	226
	movwf	DELAYTEMP
DelayUS13
	decfsz	DELAYTEMP,F
	goto	DelayUS13
	nop
;If Register Select is low
;IF LCD_RS = 0 then
	btfsc	PORTD,0
	goto	ENDIF12
;IF LCDByte < 16 then
	movlw	16
	subwf	LCDBYTE,W
	btfsc	STATUS, C
	goto	ENDIF13
;if LCDByte > 7 then
	movf	LCDBYTE,W
	sublw	7
	btfsc	STATUS, C
	goto	ENDIF14
;LCD_State = LCDByte
	movf	LCDBYTE,W
	movwf	LCD_STATE
;end if
ENDIF14
;END IF
ENDIF13
;END IF
ENDIF12
	return

;********************************************************************************

;Source: lcd.h (930)
LCDWRITECHAR
;Sub to print character on the LCD
;set LCD_RS on
	bsf	LATD,0
;LCDWriteByte(LCDChar)
	movf	LCDCHAR,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;wait 5 10us
	movlw	5
	movwf	SysWaitTemp10US
	goto	Delay_10US

;********************************************************************************

;Source: lcd.h (350)
LOCATE
;Sub to locate the cursor
;Where LCDColumn is 0 to screen width-1, LCDLine is 0 to screen height-1
;Set LCD_RS Off
	bcf	LATD,0
;If LCDLine > 1 Then
	movf	LCDLINE,W
	sublw	1
	btfsc	STATUS, C
	goto	ENDIF7
;LCDLine = LCDLine - 2
	movlw	2
	subwf	LCDLINE,F
;LCDColumn = LCDColumn + LCD_WIDTH
	movlw	20
	addwf	LCDCOLUMN,F
;End If
ENDIF7
;LCDWriteByte(0x80 or 0x40 * LCDLine + LCDColumn)
	movf	LCDLINE,W
	movwf	SysBYTETempA
	movlw	64
	movwf	SysBYTETempB
	call	SYSMULTSUB
	movf	LCDCOLUMN,W
	addwf	SysBYTETempX,W
	movwf	SysTemp1
	movlw	128
	iorwf	SysTemp1,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;wait 5 10us
	movlw	5
	movwf	SysWaitTemp10US
	goto	Delay_10US

;********************************************************************************

;Source: maze2.gcb (492)
OUTFLAME
;if roomCounter = 1 then
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	1
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	btfss	SysByteTempX,0
	goto	ELSE3_1
;Approaches the flame
;gosub forwards
	call	FORWARDS
;wait 250 ms
	movlw	250
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 350 ms
	movlw	94
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 1250 ms
	movlw	226
	movwf	SysWaitTempMS
	movlw	4
	movwf	SysWaitTempMS_H
	call	Delay_MS
;else if roomCounter = 2 then
	goto	ENDIF3
ELSE3_1
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	2
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	btfss	SysByteTempX,0
	goto	ELSE3_2
;Right turn to approach the flame
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 350 ms
	movlw	94
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 1750 ms
	movlw	214
	movwf	SysWaitTempMS
	movlw	6
	movwf	SysWaitTempMS_H
	call	Delay_MS
;
;else if roomCounter = 3 then
	goto	ENDIF3
ELSE3_2
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	3
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	btfss	SysByteTempX,0
	goto	ELSE3_3
;At the point where outFlame is called in sub room3, the bot will perform a right turn for 500 ms.
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 500 ms
	movlw	244
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;The bot slowly creeps forward until it reaches the line.
;Do Until LS = 0
SysDoLoop_S2
	btfss	PORTA,3
	goto	SysDoLoop_E2
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub stopBot
	call	STOPBOT
;wait 2 ms
	movlw	2
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Correction for if the bot is facing right instead of straight forward.
;if distanceFront <= 10 then
	movf	DISTANCEFRONT,W
	sublw	10
	btfss	STATUS, C
	goto	ENDIF5
;gosub backwards
	call	BACKWARDS
;wait 100 ms
	movlw	100
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Do Until distanceFront >= 15
SysDoLoop_S3
	movlw	15
	subwf	DISTANCEFRONT,W
	btfsc	STATUS, C
	goto	SysDoLoop_E3
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;loop
	goto	SysDoLoop_S3
SysDoLoop_E3
;end if
ENDIF5
;loop
	goto	SysDoLoop_S2
SysDoLoop_E2
;When the bot has reached the line, it goes backwards a slight amount before running the general outFlame routine.
;gosub backwards
	call	BACKWARDS
;wait 250 ms
	movlw	250
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Because room 3 has two lines that the bot crosses, room 4 is reached when roomCounter = 5.
;else if roomCounter = 5 then
	goto	ENDIF3
ELSE3_3
	movf	ROOMCOUNTER,W
	movwf	SysINTEGERTempA
	movf	ROOMCOUNTER_H,W
	movwf	SysINTEGERTempA_H
	movlw	5
	movwf	SysINTEGERTempB
	clrf	SysINTEGERTempB_H
	call	SYSCOMPEQUAL16
	btfss	SysByteTempX,0
	goto	ENDIF3
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 400 ms
	movlw	144
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 250 ms
	movlw	250
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;end if
ENDIF3
;This is the beginning of the general routine for outFlame.
;FAN = 1
	bsf	LATB,4
;gosub stopBot
	call	STOPBOT
;wait 600 ms
	movlw	88
	movwf	SysWaitTempMS
	movlw	2
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 400 ms
	movlw	144
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;The bot spins from right to left in place, in order to catch the flame in a wide area.
;For xVar = 1 To 3
;Legacy method
	clrf	XVAR
	clrf	XVAR_H
SysForLoop1
	incf	XVAR,F
	btfsc	STATUS,Z
	incf	XVAR_H,F
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 800 ms
	movlw	32
	movwf	SysWaitTempMS
	movlw	3
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 800 ms
	movlw	32
	movwf	SysWaitTempMS
	movlw	3
	movwf	SysWaitTempMS_H
	call	Delay_MS
;next
	movf	XVAR,W
	movwf	SysWORDTempA
	movf	XVAR_H,W
	movwf	SysWORDTempA_H
	movlw	3
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	btfsc	SysByteTempX,0
	goto	SysForLoop1
SysForLoopEnd1
;The bot returns to its original position before the fan is put out.
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 400 ms
	movlw	144
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;FAN = 0
	bcf	LATB,4
;return
	return

;********************************************************************************

;Overloaded signature: BYTE:, Source: lcd.h (800)
PRINT125
;Sub to print a byte variable on the LCD
;LCDValueTemp = 0
	clrf	LCDVALUETEMP
;Set LCD_RS On
	bsf	LATD,0
;IF LCDValue >= 100 Then
	movlw	100
	subwf	LCDVALUE,W
	btfss	STATUS, C
	goto	ENDIF8
;LCDValueTemp = LCDValue / 100
	movf	LCDVALUE,W
	movwf	SysBYTETempA
	movlw	100
	movwf	SysBYTETempB
	call	SYSDIVSUB
	movf	SysBYTETempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;End If
ENDIF8
;If LCDValueTemp > 0 Or LCDValue >= 10 Then
	movf	LCDVALUETEMP,W
	movwf	SysBYTETempB
	clrf	SysBYTETempA
	call	SYSCOMPLESSTHAN
	movf	SysByteTempX,W
	movwf	SysTemp1
	movf	LCDVALUE,W
	movwf	SysBYTETempA
	movlw	10
	movwf	SysBYTETempB
	call	SYSCOMPLESSTHAN
	comf	SysByteTempX,F
	movf	SysTemp1,W
	iorwf	SysByteTempX,W
	movwf	SysTemp2
	btfss	SysTemp2,0
	goto	ENDIF9
;LCDValueTemp = LCDValue / 10
	movf	LCDVALUE,W
	movwf	SysBYTETempA
	movlw	10
	movwf	SysBYTETempB
	call	SYSDIVSUB
	movf	SysBYTETempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;End If
ENDIF9
;LCDWriteByte (LCDValue + 48)
	movlw	48
	addwf	LCDVALUE,W
	movwf	LCDBYTE
	goto	LCDNORMALWRITEBYTE

;********************************************************************************

;Overloaded signature: WORD:, Source: lcd.h (820)
PRINT126
;Sub to print a word variable on the LCD
;Dim SysCalcTempX As Word
;Set LCD_RS On
	bsf	LATD,0
;LCDValueTemp = 0
	clrf	LCDVALUETEMP
;If LCDValue >= 10000 then
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	16
	movwf	SysWORDTempB
	movlw	39
	movwf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	comf	SysByteTempX,F
	btfss	SysByteTempX,0
	goto	ENDIF28
;LCDValueTemp = LCDValue / 10000 [word]
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	16
	movwf	SysWORDTempB
	movlw	39
	movwf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
	movf	SYSCALCTEMPX_H,W
	movwf	LCDVALUE_H
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Goto LCDPrintWord1000
	goto	LCDPRINTWORD1000
;End If
ENDIF28
;If LCDValue >= 1000 then
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	232
	movwf	SysWORDTempB
	movlw	3
	movwf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	comf	SysByteTempX,F
	btfss	SysByteTempX,0
	goto	ENDIF29
LCDPRINTWORD1000
;LCDValueTemp = LCDValue / 1000 [word]
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	232
	movwf	SysWORDTempB
	movlw	3
	movwf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
	movf	SYSCALCTEMPX_H,W
	movwf	LCDVALUE_H
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Goto LCDPrintWord100
	goto	LCDPRINTWORD100
;End If
ENDIF29
;If LCDValue >= 100 then
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	100
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	comf	SysByteTempX,F
	btfss	SysByteTempX,0
	goto	ENDIF30
LCDPRINTWORD100
;LCDValueTemp = LCDValue / 100 [word]
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	100
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
	movf	SYSCALCTEMPX_H,W
	movwf	LCDVALUE_H
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;Goto LCDPrintWord10
	goto	LCDPRINTWORD10
;End If
ENDIF30
;If LCDValue >= 10 then
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	10
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	comf	SysByteTempX,F
	btfss	SysByteTempX,0
	goto	ENDIF31
LCDPRINTWORD10
;LCDValueTemp = LCDValue / 10 [word]
	movf	LCDVALUE,W
	movwf	SysWORDTempA
	movf	LCDVALUE_H,W
	movwf	SysWORDTempA_H
	movlw	10
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	LCDVALUETEMP
;LCDValue = SysCalcTempX
	movf	SYSCALCTEMPX,W
	movwf	LCDVALUE
	movf	SYSCALCTEMPX_H,W
	movwf	LCDVALUE_H
;LCDWriteByte(LCDValueTemp + 48)
	movlw	48
	addwf	LCDVALUETEMP,W
	movwf	LCDBYTE
	call	LCDNORMALWRITEBYTE
;End If
ENDIF31
;LCDWriteByte (LCDValue + 48)
	movlw	48
	addwf	LCDVALUE,W
	movwf	LCDBYTE
	goto	LCDNORMALWRITEBYTE

;********************************************************************************

;Overloaded signature: INTEGER:, Source: lcd.h (861)
PRINT127
;Sub to print an integer variable on the LCD
;Dim LCDValue As Word
;If sign bit is on, print - sign and then negate
;If LCDValueInt.15 = On Then
	btfss	LCDVALUEINT_H,7
	goto	ELSE10_1
;LCDWriteChar("-")
	movlw	45
	movwf	LCDCHAR
	call	LCDWRITECHAR
;LCDValue = -LCDValueInt
	comf	LCDVALUEINT,W
	movwf	LCDVALUE
	comf	LCDVALUEINT_H,W
	movwf	LCDVALUE_H
	incf	LCDVALUE,F
	btfsc	STATUS,Z
	incf	LCDVALUE_H,F
;Sign bit off, so just copy value
;Else
	goto	ENDIF10
ELSE10_1
;LCDValue = LCDValueInt
	movf	LCDVALUEINT,W
	movwf	LCDVALUE
	movf	LCDVALUEINT_H,W
	movwf	LCDVALUE_H
;End If
ENDIF10
;Use Print(word) to display value
;Print LCDValue
	goto	PRINT126

;********************************************************************************

;Overloaded signature: BYTE:, Source: a-d.h (1748)
FN_READAD22
;ADFM should configured to ensure LEFT justified
;SET ADFM OFF
	banksel	ADCON0
	bcf	ADCON0,ADFRM0
;for 16F1885x and possibly future others
;ADPCH = ADReadPort
	banksel	ADREADPORT
	movf	ADREADPORT,W
	banksel	ADPCH
	movwf	ADPCH
;***************************************
;Perform conversion
;LLReadAD 1
;Macro Source: a-d.h (373)
;***  'Special section for 16F1688x Chips ***
;'Configure ANSELA/B/C/D
;Select Case ADReadPort 'Configure ANSELA/B/C/D @DebugADC_H
;Case 0: Set ANSELA.0 On
SysSelect1Case1
	banksel	ADREADPORT
	movf	ADREADPORT,F
	btfss	STATUS, Z
	goto	SysSelect1Case2
	banksel	ANSELA
	bsf	ANSELA,0
;Case 1: Set ANSELA.1 On
	goto	SysSelectEnd1
SysSelect1Case2
	decf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case3
	banksel	ANSELA
	bsf	ANSELA,1
;Case 2: Set ANSELA.2 On
	goto	SysSelectEnd1
SysSelect1Case3
	movlw	2
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case4
	banksel	ANSELA
	bsf	ANSELA,2
;Case 3: Set ANSELA.3 On
	goto	SysSelectEnd1
SysSelect1Case4
	movlw	3
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case5
	banksel	ANSELA
	bsf	ANSELA,3
;Case 4: Set ANSELA.4 ON
	goto	SysSelectEnd1
SysSelect1Case5
	movlw	4
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case6
	banksel	ANSELA
	bsf	ANSELA,4
;Case 5: Set ANSELA.5 On
	goto	SysSelectEnd1
SysSelect1Case6
	movlw	5
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case7
	banksel	ANSELA
	bsf	ANSELA,5
;Case 6: Set ANSELA.6 On
	goto	SysSelectEnd1
SysSelect1Case7
	movlw	6
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case8
	banksel	ANSELA
	bsf	ANSELA,6
;Case 7: Set ANSELA.7 On
	goto	SysSelectEnd1
SysSelect1Case8
	movlw	7
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case9
	banksel	ANSELA
	bsf	ANSELA,7
;Case 8: Set ANSELB.0 On
	goto	SysSelectEnd1
SysSelect1Case9
	movlw	8
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case10
	banksel	ANSELB
	bsf	ANSELB,0
;Case 9: Set ANSELB.1 On
	goto	SysSelectEnd1
SysSelect1Case10
	movlw	9
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case11
	banksel	ANSELB
	bsf	ANSELB,1
;Case 10: Set ANSELB.2 On
	goto	SysSelectEnd1
SysSelect1Case11
	movlw	10
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case12
	banksel	ANSELB
	bsf	ANSELB,2
;Case 11: Set ANSELB.3 On
	goto	SysSelectEnd1
SysSelect1Case12
	movlw	11
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case13
	banksel	ANSELB
	bsf	ANSELB,3
;Case 12: Set ANSELB.4 On
	goto	SysSelectEnd1
SysSelect1Case13
	movlw	12
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case14
	banksel	ANSELB
	bsf	ANSELB,4
;Case 13: Set ANSELB.5 On
	goto	SysSelectEnd1
SysSelect1Case14
	movlw	13
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case15
	banksel	ANSELB
	bsf	ANSELB,5
;Case 14: Set ANSELB.6 On
	goto	SysSelectEnd1
SysSelect1Case15
	movlw	14
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case16
	banksel	ANSELB
	bsf	ANSELB,6
;Case 15: Set ANSELB.7 On
	goto	SysSelectEnd1
SysSelect1Case16
	movlw	15
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case17
	banksel	ANSELB
	bsf	ANSELB,7
;Case 16: Set ANSELC.0 On
	goto	SysSelectEnd1
SysSelect1Case17
	movlw	16
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case18
	banksel	ANSELC
	bsf	ANSELC,0
;Case 17: Set ANSELC.1 On
	goto	SysSelectEnd1
SysSelect1Case18
	movlw	17
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case19
	banksel	ANSELC
	bsf	ANSELC,1
;Case 18: Set ANSELC.2 On
	goto	SysSelectEnd1
SysSelect1Case19
	movlw	18
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case20
	banksel	ANSELC
	bsf	ANSELC,2
;Case 19: Set ANSELC.3 On
	goto	SysSelectEnd1
SysSelect1Case20
	movlw	19
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case21
	banksel	ANSELC
	bsf	ANSELC,3
;Case 20: Set ANSELC.4 On
	goto	SysSelectEnd1
SysSelect1Case21
	movlw	20
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case22
	banksel	ANSELC
	bsf	ANSELC,4
;Case 21: Set ANSELC.5 On
	goto	SysSelectEnd1
SysSelect1Case22
	movlw	21
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case23
	banksel	ANSELC
	bsf	ANSELC,5
;Case 22: Set ANSELC.6 On
	goto	SysSelectEnd1
SysSelect1Case23
	movlw	22
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case24
	banksel	ANSELC
	bsf	ANSELC,6
;Case 23: Set ANSELC.7 On
	goto	SysSelectEnd1
SysSelect1Case24
	movlw	23
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case25
	banksel	ANSELC
	bsf	ANSELC,7
;Case 24: Set ANSELD.0 On
	goto	SysSelectEnd1
SysSelect1Case25
	movlw	24
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case26
	banksel	ANSELD
	bsf	ANSELD,0
;Case 25: Set ANSELD.1 On
	goto	SysSelectEnd1
SysSelect1Case26
	movlw	25
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case27
	banksel	ANSELD
	bsf	ANSELD,1
;Case 26: Set ANSELD.2 On
	goto	SysSelectEnd1
SysSelect1Case27
	movlw	26
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case28
	banksel	ANSELD
	bsf	ANSELD,2
;Case 27: Set ANSELD.3 On
	goto	SysSelectEnd1
SysSelect1Case28
	movlw	27
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case29
	banksel	ANSELD
	bsf	ANSELD,3
;Case 28: Set ANSELD.4 On
	goto	SysSelectEnd1
SysSelect1Case29
	movlw	28
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case30
	banksel	ANSELD
	bsf	ANSELD,4
;Case 29: Set ANSELD.5 On
	goto	SysSelectEnd1
SysSelect1Case30
	movlw	29
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case31
	banksel	ANSELD
	bsf	ANSELD,5
;Case 30: Set ANSELD.6 On
	goto	SysSelectEnd1
SysSelect1Case31
	movlw	30
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case32
	banksel	ANSELD
	bsf	ANSELD,6
;Case 31: Set ANSELD.7 On
	goto	SysSelectEnd1
SysSelect1Case32
	movlw	31
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case33
	banksel	ANSELD
	bsf	ANSELD,7
;Case 32: Set ANSELE.0 On
	goto	SysSelectEnd1
SysSelect1Case33
	movlw	32
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case34
	banksel	ANSELE
	bsf	ANSELE,0
;Case 33: Set ANSELE.1 On
	goto	SysSelectEnd1
SysSelect1Case34
	movlw	33
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelect1Case35
	banksel	ANSELE
	bsf	ANSELE,1
;Case 34: Set ANSELE.2 On
	goto	SysSelectEnd1
SysSelect1Case35
	movlw	34
	subwf	ADREADPORT,W
	btfss	STATUS, Z
	goto	SysSelectEnd1
	banksel	ANSELE
	bsf	ANSELE,2
;End Select  '*** ANSEL Bits should now be set ***
SysSelectEnd1
;*** ANSEL Bits are now set ***
;Set voltage reference
;ADREF = 0  'Default = 0 /Vref+ = Vdd/ Vref-  = Vss
;Configure AD clock defaults
;Set ADCS off 'Clock source = FOSC/ADCLK
	banksel	ADCON0
	bcf	ADCON0,ADCS
;ADCLK = 1 ' default to FOSC/2
	movlw	1
	movwf	ADCLK
;Conversion Clock Speed
;SET ADCS OFF  'ADCON0.4
	bcf	ADCON0,ADCS
;ADCLK = 15    'FOSC/16
	movlw	15
	movwf	ADCLK
;Result formatting
;if ADLeftadjust = 0 then  '10-bit
;Set ADCON.2 off     '8-bit
;Set ADFM OFF
	bcf	ADCON0,ADFRM0
;Set ADFM0 OFF
	bcf	ADCON0,ADFM0
;End if
;Select Channel
;ADPCH = ADReadPort  'Configure AD read Channel
	banksel	ADREADPORT
	movf	ADREADPORT,W
	banksel	ADPCH
	movwf	ADPCH
;Enable A/D
;SET ADON ON
	bsf	ADCON0,ADON
;Acquisition Delay
;Wait AD_Delay
	movlw	2
	movwf	SysWaitTemp10US
	banksel	STATUS
	call	Delay_10US
;Read A/D @1
;SET GO_NOT_DONE ON
	banksel	ADCON0
	bsf	ADCON0,GO_NOT_DONE
;nop
	nop
;Wait While GO_NOT_DONE ON
SysWaitLoop1
	btfsc	ADCON0,GO_NOT_DONE
	goto	SysWaitLoop1
;Switch off A/D
;SET ADCON0.ADON OFF
	bcf	ADCON0,ADON
;ANSELA = 0
	banksel	ANSELA
	clrf	ANSELA
;ANSELB = 0
	clrf	ANSELB
;ANSELC = 0
	clrf	ANSELC
;ANSELD = 0
	clrf	ANSELD
;ANSELE = 0
	clrf	ANSELE
;ReadAD = ADRESH
	banksel	ADRESH
	movf	ADRESH,W
	banksel	READAD
	movwf	READAD
;SET ADFM OFF
	banksel	ADCON0
	bcf	ADCON0,ADFRM0
	banksel	STATUS
	return

;********************************************************************************

;Source: maze2.gcb (338)
READSENSORS
;voltageLeft = ReadAD(AN0)
	clrf	ADREADPORT
	call	FN_READAD22
	movf	SYSREADADBYTE,W
	movwf	VOLTAGELEFT
;distanceLeft = (((6787/(voltageLeft-3)))-4)/5
	movlw	3
	subwf	VOLTAGELEFT,W
	movwf	SysTemp1
	movlw	131
	movwf	SysWORDTempA
	movlw	26
	movwf	SysWORDTempA_H
	movf	SysTemp1,W
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	SysTemp2
	movf	SysWORDTempA_H,W
	movwf	SysTemp2_H
	movlw	4
	subwf	SysTemp2,W
	movwf	SysTemp1
	movlw	0
	subwfb	SysTemp2_H,W
	movwf	SysTemp1_H
	movf	SysTemp1,W
	movwf	SysWORDTempA
	movf	SysTemp1_H,W
	movwf	SysWORDTempA_H
	movlw	5
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	DISTANCELEFT
;voltageFront = ReadAD(AN1)
	movlw	1
	movwf	ADREADPORT
	call	FN_READAD22
	movf	SYSREADADBYTE,W
	movwf	VOLTAGEFRONT
;distanceFront = (((6787/(voltageFront-3)))-4)/5
	movlw	3
	subwf	VOLTAGEFRONT,W
	movwf	SysTemp1
	movlw	131
	movwf	SysWORDTempA
	movlw	26
	movwf	SysWORDTempA_H
	movf	SysTemp1,W
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	SysTemp2
	movf	SysWORDTempA_H,W
	movwf	SysTemp2_H
	movlw	4
	subwf	SysTemp2,W
	movwf	SysTemp1
	movlw	0
	subwfb	SysTemp2_H,W
	movwf	SysTemp1_H
	movf	SysTemp1,W
	movwf	SysWORDTempA
	movf	SysTemp1_H,W
	movwf	SysWORDTempA_H
	movlw	5
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSDIVSUB16
	movf	SysWORDTempA,W
	movwf	DISTANCEFRONT
;voltageFlame = ReadAD(AN2)
	movlw	2
	movwf	ADREADPORT
	call	FN_READAD22
	movf	SYSREADADBYTE,W
	movwf	VOLTAGEFLAME
;return
	return
;Sets the forward component of both motors to 'high'
	return

;********************************************************************************

;Source: maze2.gcb (54)
ROOM3
;After entering room 3 it finds the right wall
;Do Until distanceFront <= 10
SysDoLoop_S4
	movf	DISTANCEFRONT,W
	sublw	10
	btfsc	STATUS, C
	goto	SysDoLoop_E4
;gosub readSensors
	call	READSENSORS
;CLS
	call	CLS
;Locate(0, 1)
	clrf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print roomCounter
	movf	ROOMCOUNTER,W
	movwf	LCDVALUEINT
	movf	ROOMCOUNTER_H,W
	movwf	LCDVALUEINT_H
	call	PRINT127
;Locate(1,1)
	movlw	1
	movwf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print distanceFront
	movf	DISTANCEFRONT,W
	movwf	LCDVALUE
	call	PRINT125
;gosub smoothTurnRight
	call	SMOOTHTURNRIGHT
;loop
	goto	SysDoLoop_S4
SysDoLoop_E4
;Lines itself up with the wall (to to the left)
;Do Until distanceFront >= 15
SysDoLoop_S5
	movlw	15
	subwf	DISTANCEFRONT,W
	btfsc	STATUS, C
	goto	SysDoLoop_E5
;gosub readSensors
	call	READSENSORS
;CLS
	call	CLS
;Locate(0, 1)
	clrf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print roomCounter
	movf	ROOMCOUNTER,W
	movwf	LCDVALUEINT
	movf	ROOMCOUNTER_H,W
	movwf	LCDVALUEINT_H
	call	PRINT127
;Locate(1,1)
	movlw	1
	movwf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print distanceFront
	movf	DISTANCEFRONT,W
	movwf	LCDVALUE
	call	PRINT125
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;loop
	goto	SysDoLoop_S5
SysDoLoop_E5
;Fixing allignment
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 100 ms
	movlw	100
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Goes backwards until the wall ends
;Do Until distanceLeft >= 20
SysDoLoop_S6
	movlw	20
	subwf	DISTANCELEFT,W
	btfsc	STATUS, C
	goto	SysDoLoop_E6
;gosub readSensors
	call	READSENSORS
;CLS
	call	CLS
;Locate(0, 1)
	clrf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print roomCounter
	movf	ROOMCOUNTER,W
	movwf	LCDVALUEINT
	movf	ROOMCOUNTER_H,W
	movwf	LCDVALUEINT_H
	call	PRINT127
;Locate(1,1)
	movlw	1
	movwf	LCDLINE
	movlw	1
	movwf	LCDCOLUMN
	call	LOCATE
;Print distanceLeft
	movf	DISTANCELEFT,W
	movwf	LCDVALUE
	call	PRINT125
;Too far/too close corrections
;if distanceLeft >= 15 then
	movlw	15
	subwf	DISTANCELEFT,W
	btfss	STATUS, C
	goto	ELSE24_1
;gosub farLeftB
	call	FARLEFTB
;else if distanceLeft <= 10 then
	goto	ENDIF24
ELSE24_1
	movf	DISTANCELEFT,W
	sublw	10
	btfss	STATUS, C
	goto	ELSE24_2
;gosub closeLeftB
	call	CLOSELEFTB
;else
	goto	ENDIF24
ELSE24_2
;gosub backwards
	call	BACKWARDS
;end if
ENDIF24
;gosub backwards
	call	BACKWARDS
;loop
	goto	SysDoLoop_S6
SysDoLoop_E6
;gosub backwards
	call	BACKWARDS
;wait 300 ms
	movlw	44
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub stopBot
	call	STOPBOT
;Looking for the flame
;for xVar = 0 To 10
;Legacy method
	movlw	255
	movwf	XVAR
	movwf	XVAR_H
SysForLoop2
	incf	XVAR,F
	btfsc	STATUS,Z
	incf	XVAR_H,F
;gosub readSensors
	call	READSENSORS
;if voltageFlame <= 100 then
	movf	VOLTAGEFLAME,W
	sublw	100
	btfsc	STATUS, C
;gosub outFlame
	call	OUTFLAME
;end if
;wait 5 ms
	movlw	5
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;next
	movf	XVAR,W
	movwf	SysWORDTempA
	movf	XVAR_H,W
	movwf	SysWORDTempA_H
	movlw	10
	movwf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSCOMPLESSTHAN16
	btfsc	SysByteTempX,0
	goto	SysForLoop2
SysForLoopEnd2
;wait 150 ms
	movlw	150
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Fixing allignment
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 40 ms
	movlw	40
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 400 ms
	movlw	144
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub readSensors
	call	READSENSORS
;Leaves the room
;Do Until LS = 0
SysDoLoop_S7
	btfss	PORTA,3
	goto	SysDoLoop_E7
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;loop
	goto	SysDoLoop_S7
SysDoLoop_E7
;roomCounter = 4
	movlw	4
	movwf	ROOMCOUNTER
	clrf	ROOMCOUNTER_H
;Alligning itself for a good entrance into room 4
;gosub forwards
	call	FORWARDS
;wait 550 ms
	movlw	38
	movwf	SysWaitTempMS
	movlw	2
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 375 ms
	movlw	119
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub forwards
	call	FORWARDS
;wait 750 ms
	movlw	238
	movwf	SysWaitTempMS
	movlw	2
	movwf	SysWaitTempMS_H
	call	Delay_MS
;return
	return

;********************************************************************************

;Source: maze2.gcb (163)
ROOM4
;Fixing allignment
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 75 ms
	movlw	75
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Goes forwards until it finds a wall
;Do Until distanceFront <= 10
SysDoLoop_S8
	movf	DISTANCEFRONT,W
	sublw	10
	btfsc	STATUS, C
	goto	SysDoLoop_E8
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;loop
	goto	SysDoLoop_S8
SysDoLoop_E8
;gosub readSensors
	call	READSENSORS
;Turns left to go to the line
;Do Until distanceFront >= 15
SysDoLoop_S9
	movlw	15
	subwf	DISTANCEFRONT,W
	btfsc	STATUS, C
	goto	SysDoLoop_E9
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;gosub readSensors
	call	READSENSORS
;loop
	goto	SysDoLoop_S9
SysDoLoop_E9
;Fixing Allignment
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 40 ms
	movlw	40
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;Goes slowly up to the line to avoid the pit
;Do Until LS = 0
SysDoLoop_S10
	btfss	PORTA,3
	goto	SysDoLoop_E10
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;gosub stopBot
	call	STOPBOT
;wait 1 ms
	movlw	1
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;loop
	goto	SysDoLoop_S10
SysDoLoop_E10
;Backwards to not fall in
;gosub backwards
	call	BACKWARDS
;wait 350 ms
	movlw	94
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub stopBot
	call	STOPBOT
;wait 500 ms
	movlw	244
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;Turns to enter room 4
;gosub turnLeftInPlace
	call	TURNLEFTINPLACE
;wait 375 ms
	movlw	119
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub readSensors
	call	READSENSORS
;Forwards until finds a wall
;Do until distanceFront <= 10
SysDoLoop_S11
	movf	DISTANCEFRONT,W
	sublw	10
	btfsc	STATUS, C
	goto	SysDoLoop_E11
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;loop
	goto	SysDoLoop_S11
SysDoLoop_E11
;Turns to enter room 4
;gosub readSensors
	call	READSENSORS
;gosub turnRightInPlace
	call	TURNRIGHTINPLACE
;wait 325 ms
	movlw	69
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub stopBot
	call	STOPBOT
;wait 300 ms
	movlw	44
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub readSensors
	call	READSENSORS
;Enters room 4
;Do until distanceFront <= 10
SysDoLoop_S12
	movf	DISTANCEFRONT,W
	sublw	10
	btfsc	STATUS, C
	goto	SysDoLoop_E12
;gosub readSensors
	call	READSENSORS
;gosub forwards
	call	FORWARDS
;loop
	goto	SysDoLoop_S12
SysDoLoop_E12
;gosub stopBot
	call	STOPBOT
;wait 300 ms
	movlw	44
	movwf	SysWaitTempMS
	movlw	1
	movwf	SysWaitTempMS_H
	call	Delay_MS
;gosub readSensors
	call	READSENSORS
;Looks for Flame
;if voltageFlame <= 15 then
	movf	VOLTAGEFLAME,W
	sublw	15
	btfsc	STATUS, C
;gosub outFlame
	call	OUTFLAME
;end if
;return
	return
;Follows the left wall
	return

;********************************************************************************

;Source: maze2.gcb (466)
SMOOTHTURNLEFT
;gosub forwards
	call	FORWARDS
;wait 2 ms
	movlw	2
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;RMF = 1
	bsf	LATB,0
;LMF = 0
	bcf	LATB,3
;RMB = 0
	bcf	LATB,1
;LMB = 0
	bcf	LATB,2
;wait 5 ms
	movlw	5
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;A gradual right turn.
	return

;********************************************************************************

;Source: maze2.gcb (479)
SMOOTHTURNRIGHT
;gosub forwards
	call	FORWARDS
;wait 2 ms
	movlw	2
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;RMF = 0
	bcf	LATB,0
;LMF = 1
	bsf	LATB,3
;RMB = 0
	bcf	LATB,1
;LMB = 0
	bcf	LATB,2
;wait 5 ms
	movlw	5
	movwf	SysWaitTempMS
	clrf	SysWaitTempMS_H
	call	Delay_MS
;return
	return
;An extensive subroutine to extinguish the flame, depending on it's position in each room.
	return

;********************************************************************************

;Source: maze2.gcb (370)
STOPBOT
;RMF = 0
	bcf	LATB,0
;LMF = 0
	bcf	LATB,3
;RMB = 0
	bcf	LATB,1
;LMB = 0
	bcf	LATB,2
;return
	return
;Sets the right motor forward and left motor backward component to high, and all others to low
	return

;********************************************************************************

;Source: system.h (3023)
SYSCOMPEQUAL16
;dim SysWordTempA as word
;dim SysWordTempB as word
;dim SysByteTempX as byte
;clrf SysByteTempX
	clrf	SYSBYTETEMPX
;Test low, exit if false
;movf SysWordTempA, W
	movf	SYSWORDTEMPA, W
;subwf SysWordTempB, W
	subwf	SYSWORDTEMPB, W
;btfss STATUS, Z
	btfss	STATUS, Z
;return
	return
;Test high, exit if false
;movf SysWordTempA_H, W
	movf	SYSWORDTEMPA_H, W
;subwf SysWordTempB_H, W
	subwf	SYSWORDTEMPB_H, W
;btfss STATUS, Z
	btfss	STATUS, Z
;return
	return
;comf SysByteTempX,F
	comf	SYSBYTETEMPX,F
	return

;********************************************************************************

;Source: system.h (3302)
SYSCOMPLESSTHAN
;Dim SysByteTempA, SysByteTempB, SysByteTempX as byte
;clrf SysByteTempX
	clrf	SYSBYTETEMPX
;bsf STATUS, C
	bsf	STATUS, C
;movf SysByteTempB, W
	movf	SYSBYTETEMPB, W
;subwf SysByteTempA, W
	subwf	SYSBYTETEMPA, W
;btfss STATUS, C
	btfss	STATUS, C
;comf SysByteTempX,F
	comf	SYSBYTETEMPX,F
	return

;********************************************************************************

;Source: system.h (3332)
SYSCOMPLESSTHAN16
;dim SysWordTempA as word
;dim SysWordTempB as word
;dim SysByteTempX as byte
;clrf SysByteTempX
	clrf	SYSBYTETEMPX
;Test High, exit if more
;movf SysWordTempA_H,W
	movf	SYSWORDTEMPA_H,W
;subwf SysWordTempB_H,W
	subwf	SYSWORDTEMPB_H,W
;btfss STATUS,C
	btfss	STATUS,C
;return
	return
;Test high, exit true if less
;movf SysWordTempB_H,W
	movf	SYSWORDTEMPB_H,W
;subwf SysWordTempA_H,W
	subwf	SYSWORDTEMPA_H,W
;btfss STATUS,C
	btfss	STATUS,C
;goto SCLT16True
	goto	SCLT16TRUE
;Test Low, exit if more or equal
;movf SysWordTempB,W
	movf	SYSWORDTEMPB,W
;subwf SysWordTempA,W
	subwf	SYSWORDTEMPA,W
;btfsc STATUS,C
	btfsc	STATUS,C
;return
	return
SCLT16TRUE
;comf SysByteTempX,F
	comf	SYSBYTETEMPX,F
	return

;********************************************************************************

;Source: system.h (2712)
SYSDIVSUB
;dim SysByteTempA as byte
;dim SysByteTempB as byte
;dim SysByteTempX as byte
;Check for div/0
;movf SysByteTempB, F
	movf	SYSBYTETEMPB, F
;btfsc STATUS, Z
	btfsc	STATUS, Z
;return
	return
;Main calc routine
;SysByteTempX = 0
	clrf	SYSBYTETEMPX
;SysDivLoop = 8
	movlw	8
	movwf	SYSDIVLOOP
SYSDIV8START
;bcf STATUS, C
	bcf	STATUS, C
;rlf SysByteTempA, F
	rlf	SYSBYTETEMPA, F
;rlf SysByteTempX, F
	rlf	SYSBYTETEMPX, F
;movf SysByteTempB, W
	movf	SYSBYTETEMPB, W
;subwf SysByteTempX, F
	subwf	SYSBYTETEMPX, F
;bsf SysByteTempA, 0
	bsf	SYSBYTETEMPA, 0
;btfsc STATUS, C
	btfsc	STATUS, C
;goto Div8NotNeg
	goto	DIV8NOTNEG
;bcf SysByteTempA, 0
	bcf	SYSBYTETEMPA, 0
;movf SysByteTempB, W
	movf	SYSBYTETEMPB, W
;addwf SysByteTempX, F
	addwf	SYSBYTETEMPX, F
DIV8NOTNEG
;decfsz SysDivLoop, F
	decfsz	SYSDIVLOOP, F
;goto SysDiv8Start
	goto	SYSDIV8START
	return

;********************************************************************************

;Source: system.h (2780)
SYSDIVSUB16
;dim SysWordTempA as word
;dim SysWordTempB as word
;dim SysWordTempX as word
;dim SysDivMultA as word
;dim SysDivMultB as word
;dim SysDivMultX as word
;SysDivMultA = SysWordTempA
	movf	SYSWORDTEMPA,W
	movwf	SYSDIVMULTA
	movf	SYSWORDTEMPA_H,W
	movwf	SYSDIVMULTA_H
;SysDivMultB = SysWordTempB
	movf	SYSWORDTEMPB,W
	movwf	SYSDIVMULTB
	movf	SYSWORDTEMPB_H,W
	movwf	SYSDIVMULTB_H
;SysDivMultX = 0
	clrf	SYSDIVMULTX
	clrf	SYSDIVMULTX_H
;Avoid division by zero
;if SysDivMultB = 0 then
	movf	SYSDIVMULTB,W
	movwf	SysWORDTempA
	movf	SYSDIVMULTB_H,W
	movwf	SysWORDTempA_H
	clrf	SysWORDTempB
	clrf	SysWORDTempB_H
	call	SYSCOMPEQUAL16
	btfss	SysByteTempX,0
	goto	ENDIF22
;SysWordTempA = 0
	clrf	SYSWORDTEMPA
	clrf	SYSWORDTEMPA_H
;exit sub
	return
;end if
ENDIF22
;Main calc routine
;SysDivLoop = 16
	movlw	16
	movwf	SYSDIVLOOP
SYSDIV16START
;set C off
	bcf	STATUS,C
;Rotate SysDivMultA Left
	rlf	SYSDIVMULTA,F
	rlf	SYSDIVMULTA_H,F
;Rotate SysDivMultX Left
	rlf	SYSDIVMULTX,F
	rlf	SYSDIVMULTX_H,F
;SysDivMultX = SysDivMultX - SysDivMultB
	movf	SYSDIVMULTB,W
	subwf	SYSDIVMULTX,F
	movf	SYSDIVMULTB_H,W
	subwfb	SYSDIVMULTX_H,F
;Set SysDivMultA.0 On
	bsf	SYSDIVMULTA,0
;If C Off Then
	btfsc	STATUS,C
	goto	ENDIF23
;Set SysDivMultA.0 Off
	bcf	SYSDIVMULTA,0
;SysDivMultX = SysDivMultX + SysDivMultB
	movf	SYSDIVMULTB,W
	addwf	SYSDIVMULTX,F
	movf	SYSDIVMULTB_H,W
	addwfc	SYSDIVMULTX_H,F
;End If
ENDIF23
;decfsz SysDivLoop, F
	decfsz	SYSDIVLOOP, F
;goto SysDiv16Start
	goto	SYSDIV16START
;SysWordTempA = SysDivMultA
	movf	SYSDIVMULTA,W
	movwf	SYSWORDTEMPA
	movf	SYSDIVMULTA_H,W
	movwf	SYSWORDTEMPA_H
;SysWordTempX = SysDivMultX
	movf	SYSDIVMULTX,W
	movwf	SYSWORDTEMPX
	movf	SYSDIVMULTX_H,W
	movwf	SYSWORDTEMPX_H
	return

;********************************************************************************

;Source: system.h (2437)
SYSMULTSUB
;dim SysByteTempA as byte
;dim SysByteTempB as byte
;dim SysByteTempX as byte
;clrf SysByteTempX
	clrf	SYSBYTETEMPX
MUL8LOOP
;movf SysByteTempA, W
	movf	SYSBYTETEMPA, W
;btfsc SysByteTempB, 0
	btfsc	SYSBYTETEMPB, 0
;addwf SysByteTempX, F
	addwf	SYSBYTETEMPX, F
;bcf STATUS, C
	bcf	STATUS, C
;rrf SysByteTempB, F
	rrf	SYSBYTETEMPB, F
;bcf STATUS, C
	bcf	STATUS, C
;rlf SysByteTempA, F
	rlf	SYSBYTETEMPA, F
;movf SysByteTempB, F
	movf	SYSBYTETEMPB, F
;btfss STATUS, Z
	btfss	STATUS, Z
;goto MUL8LOOP
	goto	MUL8LOOP
	return

;********************************************************************************

SysStringTables
	movf	SysStringA_H,W
	movwf	PCLATH
	movf	SysStringA,W
	incf	SysStringA,F
	btfsc	STATUS,Z
	incf	SysStringA_H,F
	movwf	PCL

StringTable1
	retlw	1
	retlw	45	;-


;********************************************************************************

;Source: maze2.gcb (380)
TURNLEFTINPLACE
;RMF = 1
	bsf	LATB,0
;LMF = 0
	bcf	LATB,3
;RMB = 0
	bcf	LATB,1
;LMB = 1
	bsf	LATB,2
;return
	return
;Sets the left motor forward and right motor backward component to high, and all others to low
	return

;********************************************************************************

;Source: maze2.gcb (390)
TURNRIGHTINPLACE
;RMF = 0
	bcf	LATB,0
;LMF = 1
	bsf	LATB,3
;RMB = 1
	bsf	LATB,1
;LMB = 0
	bcf	LATB,2
;return
	return
;Turn that is preformed when the bot is too close to the left wall
	return

;********************************************************************************

;Start of program memory page 1
	ORG	2048
;Start of program memory page 2
	ORG	4096
;Start of program memory page 3
	ORG	6144

 END
