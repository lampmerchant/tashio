;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashIO: ADB I/O Device
;;;
;


;;; Connections ;;;

;;;                                                                          ;;;
;                                  .--------.                                  ;
;                          Supply -|01 \/ 14|- Ground                          ;
;                 ADB <-->    RA5 -|02    13|- RA0    <--> Port B Channel 3    ;
;    Port B Channel 1 <-->    RA4 -|03    12|- RA1    <--> Port B Channel 4    ;
;       Unit Select 0 --->    RA3 -|04    11|- RA2    <--> Port B Channel 2    ;
;       Unit Select 1 --->    RC5 -|05    10|- RC0    <--> Port A Channel 4    ;
;                 LED <---    RC4 -|06    09|- RC1    <--> Port A Channel 3    ;
;    Port A Channel 1 <-->    RC3 -|07    08|- RC2    <--> Port A Channel 2    ;
;                                  '--------'                                  ;
;;;                                                                          ;;;


;;; Assembler Directives ;;;

	list		P=PIC16F1704, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P16F1704.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PPS1WAY_OFF & _ZCDDIS_ON & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PPS1WAY_OFF	PPS can change more than once
			;_ZCDDIS_ON	Zero crossing detector disabled
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

ADBHDLR	equ	0x7A	;Handler ID for the device
ADBADDR	equ	0x07	;Default address for the device

ADBPORT	equ	PORTA	;Pin where ADB is connected
ADBTRIS	equ	TRISA	; "
ADBIOCP	equ	IOCAP	; "
ADBIOCN	equ	IOCAN	; "
ADBIOCF	equ	IOCAF	; "
ADBPIN	equ	RA5	; "

LEDPORT	equ	PORTC	;Pin where LED is connected
LEDPIN	equ	RC4	; "

U0_PORT	equ	PORTA	;Pins where unit-select jumpers are connected
U0_PIN	equ	RA3	; "
U1_PORT	equ	PORTC	; "
U1_PIN	equ	RC5	; "

PA_PORT	equ	PORTC	;Pins where port A is connected
PA_ANSL	equ	ANSELA	; "
A4_PIN	equ	RC0	; "
A3_PIN	equ	RC1	; "
A2_PIN	equ	RC2	; "
A1_PIN	equ	RC3	; "
PA_PINS	equ	(1 << A4_PIN) | (1 << A3_PIN) | (1 << A2_PIN) | (1 << A1_PIN)

PB_PORT	equ	PORTA	;Pins where port B is connected
PB_IOCP	equ	IOCAP	; "
PB_IOCN	equ	IOCAN	; "
PB_IOCF	equ	IOCAF	; "
PB_ANSL	equ	ANSELC	; "
B4_PIN	equ	RA1	; "
B3_PIN	equ	RA0	; "
B2_PIN	equ	RA2	; "
B1_PIN	equ	RA4	; "
B4_ADCN	equ	0x05	; "
B3_ADCN	equ	0x01	; "
B2_ADCN	equ	0x09	; "
B1_ADCN	equ	0x0D	; "
PB_PINS	equ	(1 << B4_PIN) | (1 << B3_PIN) | (1 << B2_PIN) | (1 << B1_PIN)

			;FLAGS:
F_PBCHG	equ	7	;Set when one of the pins on port B has changed

			;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB state machine
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	A4_TOUT	;Timeouts for port A
	A3_TOUT	; "
	A2_TOUT	; "
	A1_TOUT	; "
	PA_TCNT	;Timeout scaler counter
	ADB_R3H	;ADB register 3 high byte
	B4_ADC	;ADC values for port B
	B3_ADC	; "
	B2_ADC	; "
	B1_ADC	; "
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	movlp	0		;Copy the Timer0 flag into the carry bit so it
	bcf	STATUS,C	; doesn't change on us mid-stream
	btfsc	INTCON,TMR0IF	; "
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;If the ADB pin has had a negative or positive
	movlp	0		; edge, handle it as an event for the ADB state
	btfsc	ADBIOCF,ADBPIN	; machine
	call	IntAdbEdge	; "
	movlp	0		; "
	movlb	0		;If the timeout timer overflowed, handle it
	btfsc	PIR1,TMR2IF	; "
	call	IntTimeoutTimer	; "
	movlb	0		;If an ADC conversion finished, handle it
	btfsc	PIR1,ADIF	; "
	call	IntAdc		; "
	movlb	7		;If there's been a change on port B, raise the
	movf	PB_IOCF,W	; flag and clear the interrupt
	andlw	PB_PINS		; "
	btfsc	STATUS,Z	; "
	retfie			; "
	movlw	~PB_PINS	; "
	andwf	PB_IOCF,F	; "
	bsf	FLAGS,F_PBCHG	; "
	retfie			; "

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADBPIN	;Toggle the edge that the IOC interrupt catches
	xorwf	ADBIOCN,F	; "
	xorwf	ADBIOCP,F	; "
	bcf	ADBIOCF,ADBPIN	;Clear the interrupt flag
	btfsc	ADBIOCN,ADBPIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return

IntTimeoutTimer
	bcf	PIR1,TMR2IF	;Clear the interrupt
	incf	PA_TCNT,F	;Scale the Timer2 interrupt by 5 so the handler
	btfsc	PA_TCNT,2	; runs every 0.1 seconds
	btfss	PA_TCNT,0	; "
	return			; "
	clrf	PA_TCNT		; "
	movlw	PA_PINS		;Decrement the port A timeouts, and if any hits
	movf	A4_TOUT,F	; zero, invert it
	btfss	STATUS,Z	; "
	decfsz	A4_TOUT,F	; "
	andlw	~(1 << A4_PIN)	; "
	movf	A3_TOUT,F	; "
	btfss	STATUS,Z	; "
	decfsz	A3_TOUT,F	; "
	andlw	~(1 << A3_PIN)	; "
	movf	A2_TOUT,F	; "
	btfss	STATUS,Z	; "
	decfsz	A2_TOUT,F	; "
	andlw	~(1 << A2_PIN)	; "
	movf	A1_TOUT,F	; "
	btfss	STATUS,Z	; "
	decfsz	A1_TOUT,F	; "
	andlw	~(1 << A1_PIN)	; "
	movlb	2		; "
	xorwf	PA_PORT,F	; "
	return

IntAdc
	bcf	PIR1,ADIF	;Clear the interrupt
	movlb	2		;Turn LED off if it was on; this is a convenient
	bsf	LEDPORT,LEDPIN	; place to do it so it flashes brightly when on
	movlb	1		;Switch off by which source's conversion just
	movf	ADCON0,W	; finished
	andlw	0x7C		; "
	xorlw	B1_ADCN & 0x7C	; "
	btfsc	STATUS,Z	; "
	bra	IntAdc0		; "
	xorlw	(B1_ADCN & 0x7C) ^ (B2_ADCN & 0x7C); "
	btfsc	STATUS,Z	; "
	bra	IntAdc1		; "
	xorlw	(B3_ADCN & 0x7C) ^ (B2_ADCN & 0x7C); "
	btfsc	STATUS,Z	; "
	bra	IntAdc2		; "
	movf	ADRESH,W	;Save port B channel 4's reading
	movwf	B4_ADC		; "
	movlw	B1_ADCN		;Start acquiring channel 1's voltage
	movwf	ADCON0		; "
	return
IntAdc0	movf	ADRESH,W	;Save port B channel 1's reading
	movwf	B1_ADC		; "
	movlw	B2_ADCN		;Start acquiring channel 2's voltage
	movwf	ADCON0		; "
	return
IntAdc1	movf	ADRESH,W	;Save port B channel 2's reading
	movwf	B2_ADC		; "
	movlw	B3_ADCN		;Start acquiring channel 3's voltage
	movwf	ADCON0		; "
	return
IntAdc2	movf	ADRESH,W	;Save port B channel 3's reading
	movwf	B3_ADC		; "
	movlw	B4_ADCN		;Start acquiring channel 4's voltage
	movwf	ADCON0		; "
	return


;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	IOCAN		;ADB sets IOC on negative edge, port B sets IOC
	clrf	ADBIOCN		; on either edge
	clrf	PB_IOCN
	clrf	PB_IOCP
	movlw	1 << ADBPIN
	iorwf	ADBIOCN,F
	movlw	PB_PINS
	iorwf	PB_IOCN,F
	iorwf	PB_IOCP,F

	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG

	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON

	banksel	T2CON		;Timer2 on with 1:64 pre and 1:10 postscaler and
	movlw	B'01001111'	; period of 249, making it interrupt every 0.02
	movwf	T2CON		; seconds
	movlw	249
	movwf	PR2

	banksel	T4CON		;Timer4 on with 1:64 prescaler and maximum
	movlw	B'00000111'	; period, making it match every 2.048 ms
	movwf	T4CON
	movlw	255
	movwf	PR4

	banksel	ADCON0		;ADC is left justified since we treat it as
	movlw	B4_ADCN		; 8-bit, clock is Fosc/64, references are Vss
	movwf	ADCON0		; and Vdd to start with, starts conversion on
	movlw	B'01100000'	; Timer4 match
	movwf	ADCON1
	movlw	B'11000000'
	movwf	ADCON2

	banksel	ANSELA		;All pins digital, not analog to start with
	clrf	PA_ANSL
	clrf	PB_ANSL

	banksel	WPUA		;Weak pullups only on unit jumper pins
	clrf	WPUA
	clrf	WPUC
	movlw	1 << U0_PIN
	iorwf	U0_PORT,F
	movlw	1 << U1_PIN
	iorwf	U1_PORT,F

	banksel	LATA		;Ready to pull ADB low when output
	bcf	ADBPORT,ADBPIN

	banksel	TRISA		;All pins inputs to start with except LED
	movlw	B'00111111'
	movwf	TRISA
	movwf	TRISC
	movlw	~(1 << LEDPIN)
	andwf	LEDPORT,F

	banksel	PIE1		;Timer2 and ADC interrupts enabled
	movlw	(1 << TMR2IE) | (1 << ADIE)
	movwf	PIE1

	movlw	12		;Delay approximately 2 ms at an instruction
	movwf	AP_BUF		; clock of 2 MHz until the PLL kicks in and the
PllWait	DELAY	110		; instruction clock gears up to 8 MHz
	decfsz	AP_BUF,F
	bra	PllWait

	clrf	FLAGS		;Set initial values of key globals
	clrf	AP_FLAG
	clrf	AP_FSAP
	clrf	A4_TOUT
	clrf	A3_TOUT
	clrf	A2_TOUT
	clrf	A1_TOUT
	movlw	1
	movwf	PA_TCNT
	movlw	ADBADDR | 0x60
	movwf	ADB_R3H

	movlw	B'11001000'	;On-change interrupt, peripheral interrupts (for
	movwf	INTCON		; Timer2 and ADC) and interrupt subsystem on

AdbReset
	bcf	AP_FLAG,AP_RST	;Clear reset flag
	movlb	1		;Tristate all port pins
	bsf	PB_PORT,B4_PIN	; "
	bsf	PB_PORT,B3_PIN	; "
	bsf	PB_PORT,B2_PIN	; "
	bsf	PB_PORT,B1_PIN	; "
	bsf	PA_PORT,A4_PIN	; "
	bsf	PA_PORT,A3_PIN	; "
	bsf	PA_PORT,A2_PIN	; "
	bsf	PA_PORT,A1_PIN	; "
	movlb	3		;Switch all pins to digital mode
	clrf	PB_ANSL		; "
	movlb	7		;Enable all IOCs on port B and clear their flags
	movlw	PB_PINS		; "
	iorwf	PB_IOCN,F	; "
	iorwf	PB_IOCP,F	; "
	xorlw	0xFF		; "
	andwf	PB_IOCF,F	; "
	bcf	FLAGS,F_PBCHG	; "
	;fall through

Main
	btfsc	AP_FLAG,AP_RST	;Branch to reset if a reset was received, else
	bra	AdbReset	; wait until a command was received
	btfss	AP_FLAG,AP_RXCI	; "
	bra	Main		; "
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_RXDI	;Clear other flags too from data activity that
	bcf	AP_FLAG,AP_DONE	; might have happened while waiting for a
	bcf	AP_FLAG,AP_COL	; command
	bcf	AP_FLAG,AP_TXI	; "
	bcf	AP_FLAG,AP_SRQ	;Not calling for service just yet
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	AdbReset	; "
	swapf	ADB_R3H,W	;If the device being addressed matches our
	xorwf	AP_BUF,W	; address, handle it
	andlw	B'11110000'	; "
	btfsc	STATUS,Z	; "
	bra	AdbCmd		; "
	btfsc	FLAGS,F_PBCHG	;If SRQ is enabled and there's been a change on
	btfss	ADB_R3H,5	; one of the port B pins, send an SRQ
	bra	Main		; "
	bsf	AP_FLAG,AP_SRQ	; "
	bra	Main		; "

AdbCmd
	movlb	2		;Device has been hailed, turn LED on
	bcf	LEDPORT,LEDPIN	; "
	movf	AP_BUF,W	;Switch handler by the low four bits of the
	andlw	B'00001111'	; command
	brw			; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	AdbListen1	; "
	goto	AdbListen2	; "
	goto	AdbListen3	; "
	goto	AdbTalk0	; "
	goto	AdbTalk1	; "
	goto	AdbTalk2	; "
	goto	AdbTalk3	; "

AdbListen1
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the first data byte
	bra	AdbListen1	; "
	movlb	2		;Set high/low status of digital outputs from
	movf	PB_PORT,W	; first byte
	andlw	~PB_PINS	; "
	btfsc	AP_BUF,3	; "
	iorlw	1 << B4_PIN	; "
	btfsc	AP_BUF,2	; "
	iorlw	1 << B3_PIN	; "
	btfsc	AP_BUF,1	; "
	iorlw	1 << B2_PIN	; "
	btfsc	AP_BUF,0	; "
	iorlw	1 << B1_PIN	; "
	movwf	PB_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn10	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn10		; byte
	movlb	1		;Set TRIS of digital I/Os from second byte
	movf	PB_PORT,W	; "
	andlw	~PB_PINS	; "
	btfsc	AP_BUF,3	; "
	iorlw	1 << B4_PIN	; "
	btfsc	AP_BUF,2	; "
	iorlw	1 << B3_PIN	; "
	btfsc	AP_BUF,1	; "
	iorlw	1 << B2_PIN	; "
	btfsc	AP_BUF,0	; "
	iorlw	1 << B1_PIN	; "
	movwf	PB_PORT		; "
	swapf	AP_BUF,W	;Switch to set the analog mode depending on bits
	andlw	B'00000011'	; 4 and 5 of second byte
	brw			; "
	bra	ALstn11		; "
	bra	ALstn12		; "
	bra	ALstn13		; "
	movlb	3		;In 0-AD mode (0b11), all pins are digital
	movlw	~PB_PINS	;Set all pins to digital
	andwf	PB_ANSL,F	; "
	movlb	1		;Set ADC reference to Vdd
	bcf	ADCON1,ADPREF1	; "
	movlw	PB_PINS		;Set IOC to trigger only on the digital inputs
	andwf	PB_PORT,W	; "
	movlb	7		; "
	iorwf	PB_IOCP,F	; "
	iorwf	PB_IOCN,F	; "
	movlw	~PB_PINS	; "
	andwf	PB_IOCF,F	; "
	bcf	FLAGS,F_PBCHG	; "
	goto	Main		;Return to main
ALstn11	movlb	3		;In 4-AD mode (0b00), all pins are analog inputs
	movlw	PB_PINS		;Set all pins to analog
	iorwf	PB_ANSL,F	; "
	movlb	1		;Tristate all pins, just in case user didn't
	iorwf	PB_PORT,F	; "
	bcf	ADCON1,ADPREF1	;Set ADC reference to Vdd
	movlw	~PB_PINS	;Set IOC not to trigger
	movlb	7		; "
	andwf	PB_IOCP,F	; "
	andwf	PB_IOCN,F	; "
	andwf	PB_IOCF,F	; "
	bcf	FLAGS,F_PBCHG	; "
	goto	Main		;Return to main
ALstn12	movlb	3		;In 3-AD mode (0b01), 1-3 are analog inputs
	movlw	PB_PINS		;Set all pins to analog
	iorwf	PB_ANSL,F	; "
	movlb	1		;Tristate all pins, just in case user didn't
	iorwf	PB_PORT,F	; "
	bsf	ADCON1,ADPREF1	;Set ADC reference to channel 4
	movlw	~PB_PINS	;Set IOC not to trigger
	movlb	7		; "
	andwf	PB_IOCP,F	; "
	andwf	PB_IOCN,F	; "
	andwf	PB_IOCF,F	; "
	bcf	FLAGS,F_PBCHG	; "
	goto	Main		;Return to main
ALstn13	movlb	3		;In 2-AD mode (0b10), 1-2 analog, 3-4 digital
	movlw	~PB_PINS	;Set all pins to digital
	andwf	PB_ANSL,F	; "
	movlw	(1 << B1_PIN) | (1 << B2_PIN);Set channels 1 and 2 to analog
	iorwf	PB_ANSL,F	; "
	movlb	1		;Tristate channel 1 and 2 pins, just in case
	iorwf	PB_PORT,F	; user didn't
	bcf	ADCON1,ADPREF1	;Set ADC reference to Vdd
	movlw	~PB_PINS	;Set IOC not to trigger, except on whichever of
	movlb	7		; channels 3 and 4 are input pins
	andwf	PB_IOCP,F	; "
	andwf	PB_IOCN,F	; "
	andwf	PB_IOCF,F	; "
	movlb	1		; "
	movf	PB_PORT,W	; "
	andlw	(1 << B3_PIN) | (1 << B4_PIN); "
	movlb	7		; "
	iorwf	PB_IOCP,F	; "
	iorwf	PB_IOCN,F	; "
	bcf	FLAGS,F_PBCHG	; "
	goto	Main		;Return to main

AdbListen2
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the first data byte
	bra	AdbListen2	; "
	btfss	AP_BUF,7	;If MSB of first byte is set, format is almost
	bra	ALstn21		; the same as listen 1
	movlb	2		;Set high/low status of digital outputs from
	movf	PA_PORT,W	; first byte
	andlw	~PA_PINS	; "
	btfsc	AP_BUF,3	; "
	iorlw	1 << A4_PIN	; "
	btfsc	AP_BUF,2	; "
	iorlw	1 << A3_PIN	; "
	btfsc	AP_BUF,1	; "
	iorlw	1 << A2_PIN	; "
	btfsc	AP_BUF,0	; "
	iorlw	1 << A1_PIN	; "
	movwf	PA_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn20	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn20		; byte
	movlb	1		;Set TRIS of digital I/Os from second byte
	movf	PA_PORT,W	; "
	andlw	~PA_PINS	; "
	btfsc	AP_BUF,3	; "
	iorlw	1 << A4_PIN	; "
	btfsc	AP_BUF,2	; "
	iorlw	1 << A3_PIN	; "
	btfsc	AP_BUF,1	; "
	iorlw	1 << A2_PIN	; "
	btfsc	AP_BUF,0	; "
	iorlw	1 << A1_PIN	; "
	movwf	PA_PORT		; "
	goto	Main		;Return to main, no analog setting on port B
ALstn21	movf	AP_BUF,W	;Switch to set one of the outputs along with its
	andlw	B'00000011'	; timer
	brw			; "
	bra	ALstn23		; "
	bra	ALstn25		; "
	bra	ALstn27		; "
	clrf	A4_TOUT		;Keep pin from inverting on us while we change
	movlb	2		;Set the latch for channel 4 according to bit 6
	movf	PA_PORT,W	; of first byte
	andlw	~(1 << A4_PIN)	; "
	btfsc	AP_BUF,6	; "
	iorlw	1 << A4_PIN	; "
	movwf	PA_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn22	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn22		; byte
	movf	AP_BUF,W	;Set the timeout according to the second byte
	movwf	A4_TOUT		; "
	goto	Main		;Return to main
ALstn23	clrf	A1_TOUT		;Keep pin from inverting on us while we change
	movlb	2		;Set the latch for channel 1 according to bit 6
	movf	PA_PORT,W	; of first byte
	andlw	~(1 << A1_PIN)	; "
	btfsc	AP_BUF,6	; "
	iorlw	1 << A1_PIN	; "
	movwf	PA_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn24	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn24		; byte
	movf	AP_BUF,W	;Set the timeout according to the second byte
	movwf	A1_TOUT		; "
	goto	Main		;Return to main
ALstn25	clrf	A2_TOUT		;Keep pin from inverting on us while we change
	movlb	2		;Set the latch for channel 2 according to bit 6
	movf	PA_PORT,W	; of first byte
	andlw	~(1 << A2_PIN)	; "
	btfsc	AP_BUF,6	; "
	iorlw	1 << A2_PIN	; "
	movwf	PA_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn26	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn26		; byte
	movf	AP_BUF,W	;Set the timeout according to the second byte
	movwf	A2_TOUT		; "
	goto	Main		;Return to main
ALstn27	clrf	A3_TOUT		;Keep pin from inverting on us while we change
	movlb	2		;Set the latch for channel 3 according to bit 6
	movf	PA_PORT,W	; of first byte
	andlw	~(1 << A3_PIN)	; "
	btfsc	AP_BUF,6	; "
	iorlw	1 << A3_PIN	; "
	movwf	PA_PORT		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn28	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn28		; byte
	movf	AP_BUF,W	;Set the timeout according to the second byte
	movwf	A3_TOUT		; "
	goto	Main		;Return to main

AdbListen3
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the first data byte
	bra	AdbListen3	; "
	movf	AP_BUF,W	;Save the first byte of the listen in FSR0 for
	movwf	FSR0L		; later use (as a temp var, no dereferencing)
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
ALstn30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off receiving and return to main
	goto	Main		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for the second data
	bra	ALstn30		; byte
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	ALstn31		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfss	STATUS,Z	; device's address if a collision hasn't been
	goto	Main		; detected; ignore everything else
	btfss	ADB_R3H,7	;If a collision has not been detected, skip
	bra	ALstn32		; ahead to change the address; if one has been
	bcf	ADB_R3H,7	; detected, clear it and ignore this command
	goto	Main		; "
ALstn31	bcf	ADB_R3H,5	;Copy the state of the SRQ enable bit to the SRQ
	btfsc	FSR0L,5		; enable flag and to our copy of register 3
	bsf	ADB_R3H,5	; "
ALstn32	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	FSR0L,F		; byte as our new address and we're done
	movf	ADB_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	FSR0L,W		; "
	movwf	ADB_R3H		; "
	goto	Main		; "

AdbTalk0
	btfss	FLAGS,F_PBCHG	;Only respond on Talk 0 if there's an SRQ
	goto	Main		; "
	;fall through

AdbTalk1
	bcf	FLAGS,F_PBCHG	;Port B is being read, so clear change flag
	movlw	B'00010000'	;If the ADC is set to use an external voltage
	movlb	1		; reference, we're always in 3-AD mode
	btfsc	ADCON1,ADPREF1	; "
	bra	ATalk10		; "
	movlw	B'00000000'	;If port B channel 4 is an analog pin, we're
	movlb	3		; in 4-AD mode
	btfsc	PB_ANSL,B4_PIN	; "
	bra	ATalk10		; "
	movlw	B'00110000'	;If port B channel 2 is an analog pin, we're
	btfsc	PB_ANSL,B2_PIN	; in 2-AD mode, else we're in 0-AD mode
	movlw	B'00100000'	; "
ATalk10	movlb	0		;Bits 7 and 6 are the unit number
	btfsc	U1_PORT,U1_PIN	; "
	iorlw	B'10000000'	; "
	btfsc	U0_PORT,U0_PIN	; "
	iorlw	B'01000000'	; "
	movlb	1		;Copy the TRIS bits of port B
	btfsc	PB_PORT,B4_PIN	; "
	iorlw	B'00001000'	; "
	btfsc	PB_PORT,B3_PIN	; "
	iorlw	B'00000100'	; "
	btfsc	PB_PORT,B2_PIN	; "
	iorlw	B'00000010'	; "
	btfsc	PB_PORT,B1_PIN	; "
	iorlw	B'00000001'	; "
	movwf	AP_BUF		;Send the byte we built
	bsf	AP_FLAG,AP_TXI	; "
ATalk11	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk11		; for a byte
	movlb	0		;Do a digital read on port B channel 1, loading
	movlw	0x00		; 0xFF if it's high and 0x00 if it's low
	btfsc	PB_PORT,B1_PIN	; "
	movlw	0xFF		; "
	movlb	3		; "
	btfsc	PB_ANSL,B1_PIN	;If port B channel 1 is an analog pin, load the
	movf	B1_ADC,W	; result from the ADC instead
	movwf	AP_BUF		;Send the byte
	bsf	AP_FLAG,AP_TXI	; "
ATalk12	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk12		; for a byte
	movlb	0		;Do a digital read on port B channel 2, loading
	movlw	0x00		; 0xFF if it's high and 0x00 if it's low
	btfsc	PB_PORT,B2_PIN	; "
	movlw	0xFF		; "
	movlb	3		; "
	btfsc	PB_ANSL,B2_PIN	;If port B channel 2 is an analog pin, load the
	movf	B2_ADC,W	; result from the ADC instead
	movwf	AP_BUF		;Send the byte
	bsf	AP_FLAG,AP_TXI	; "
ATalk13	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk13		; for a byte
	movlb	0		;Do a digital read on port B channel 3, loading
	movlw	0x00		; 0xFF if it's high and 0x00 if it's low
	btfsc	PB_PORT,B3_PIN	; "
	movlw	0xFF		; "
	movlb	3		; "
	btfsc	PB_ANSL,B3_PIN	;If port B channel 3 is an analog pin, load the
	movf	B3_ADC,W	; result from the ADC instead
	movwf	AP_BUF		;Send the byte
	bsf	AP_FLAG,AP_TXI	; "
ATalk14	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk14		; for a byte
	movlb	0		;Do a digital read on port B channel 4, loading
	movlw	0x00		; 0xFF if it's high and 0x00 if it's low
	btfsc	PB_PORT,B4_PIN	; "
	movlw	0xFF		; "
	movlb	3		; "
	btfsc	PB_ANSL,B4_PIN	;If port B channel 4 is an analog pin, load the
	movf	B4_ADC,W	; result from the ADC instead
	movwf	AP_BUF		;Send the byte
	bsf	AP_FLAG,AP_TXI	; "
ATalk15	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	ATalk15		; "
	goto	Main		;Return to main when it is

AdbTalk2
	movlb	0		;Bits 5 and 4 are always 1 when reading port A,
	movlw	B'00110000'	; since it's always digital
	btfsc	U1_PORT,U1_PIN	;Bits 7 and 6 are the unit number
	iorlw	B'10000000'	; "
	btfsc	U0_PORT,U0_PIN	; "
	iorlw	B'01000000'	; "
	movlb	1		;Copy the TRIS bits of port A
	btfsc	PA_PORT,A4_PIN	; "
	iorlw	B'00001000'	; "
	btfsc	PA_PORT,A3_PIN	; "
	iorlw	B'00000100'	; "
	btfsc	PA_PORT,A2_PIN	; "
	iorlw	B'00000010'	; "
	btfsc	PA_PORT,A1_PIN	; "
	iorlw	B'00000001'	; "
	movwf	AP_BUF		;Send the byte we built
	bsf	AP_FLAG,AP_TXI	; "
ATalk20	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk20		; for a byte
	movlb	0		;Load an 0x00 or 0xFF depending on whether the
	movlw	0x00		; channel 1 signal is low or high
	btfsc	PA_PORT,A1_PIN	; "
	movlw	0xFF		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
ATalk21	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk21		; for a byte
	movlb	0		;Load an 0x00 or 0xFF depending on whether the
	movlw	0x00		; channel 2 signal is low or high
	btfsc	PA_PORT,A2_PIN	; "
	movlw	0xFF		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
ATalk22	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk22		; for a byte
	movlb	0		;Load an 0x00 or 0xFF depending on whether the
	movlw	0x00		; channel 3 signal is low or high
	btfsc	PA_PORT,A3_PIN	; "
	movlw	0xFF		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
ATalk23	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	ATalk23		; for a byte
	movlb	0		;Load an 0x00 or 0xFF depending on whether the
	movlw	0x00		; channel 4 signal is low or high
	btfsc	PA_PORT,A4_PIN	; "
	movlw	0xFF		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
AD0Tk24	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AD0Tk24		; "
	goto	Main		;Return to main when it is

AdbTalk3
	movf	ADB_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movlb	0		;Get two pseudorandom bits and put them into the
	movf	TMR1H,W		; buffer; this way we replace part of address
	xorwf	TMR1L,W		; (which the host already knows) with a random
	andlw	B'00000011'	; number, which helps with collision detection
	btfsc	U0_PORT,U0_PIN	;Copy the unit number into bits 2 and 3 of the
	iorlw	B'00000100'	; buffer; this is nonstandard but it serves as a
	btfsc	U1_PORT,U1_PIN	; way to allow four of these devices to be
	iorlw	B'00001000'	; addressed by the Mac
	iorwf	AP_BUF,F	; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
ATalk30	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be done
	bra	ATalk30		; "
	movlw	ADBHDLR		;Load the low byte of register 3 for transmit
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
ATalk31	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	goto	Main		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	ATalk32		; "
	btfsc	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	ATalk31		; "
	goto	Main		;Return to main when it is
ATalk32	bsf	ADB_R3H,7	;We collided, so set the collision flag
	goto	Main		;Return to main


;;; State Machines ;;;

AdbFsa	org	0xF00

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	ADBTRIS,ADBPIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	ADBTRIS,ADBPIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-128		;Shorten the timeout period to 512 us, which is
	movwf	TMR0		; still too long to wait for a transmission
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	bsf	INTCON,TMR0IE	;Set timer to interrupt and bring us to the
	retlw	low AdbFsaTxBitD; transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	ADBTRIS,ADBPIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	ADBTRIS,ADBPIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	ADBPORT,ADBPIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	ADBIOCP,ADBPIN	; set by releasing the pin so the timer we just
	bsf	ADBIOCN,ADBPIN	; set doesn't immediately get reset
	bcf	ADBIOCF,ADBPIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBD0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBD0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "


;;; End of Program ;;;
	end
