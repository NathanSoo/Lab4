;
; lab4.asm
;
; Created: 4/11/2024 3:12:15 PM
; Author : nsoo1
;

; set I bit in SREG with sei
; enable external interrupt with bit in eimsk

; EICRA, EICRB - interrupt trigger type (falling, rising, logic)
; EICRA - last 2 bits for INT0
; 0 0 - low level
; 0 1 - any edge (change in level)
; 1 0 - any falling edge (relevant for lab)
; 1 1 - any rising edge

.include "m2560def.inc"

.def	w = r16						; working register
.def	rot_count = r3


.cseg

.org 0x0000
	rjmp	reset
	nop
	jmp		irq0

.org OVF1addr
	jmp		timer1ovf

reset:

	rcall	INITIALISE_LCD
	ser		w
	clr		rot_count

	ldi		w, (0b10 << ISC00)
	sts		EICRA, w

	in		w, EIMSK
	ori		w, (1 << INT0)
	out		EIMSK, w

	ldi		w, 0b00000000
	sts		TCCR1A, w

	ldi		w, 0b00000100
	sts		TCCR1B, w

	ldi		w, (1 << TOIE1)
	sts		TIMSK1, w

	sei

loop:
	nop
	rjmp	loop
	

halt:
	rjmp	halt

irq0:
	push	w
	push	r17
	in		r17, SREG

	inc		rot_count

	out		SREG, r17
	pop		r17
	pop		w
	reti

timer1ovf:
	push	w
	in		r17, SREG

	lsr		rot_count
	lsr		rot_count
	mov		r24, rot_count
	rcall	INITIALISE_LCD
	rcall	decimal_conversion

	clr		rot_count

	out		SREG, r17
	pop		w
	reti

;
; lab5_output.asm
;
; Created: 22/10/2024 9:06:18 PM
; Author : Owen
;

; parameter passed from r27
; Replace with your application code
.include "m2560def.inc"
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5

.macro lcd_write_cmd		; set LCD instructions, does not wait for BF
	out PORTF, r16			; set data port
	clr r18
	out PORTA, r18			; RS = 0, RW = 0 for a command write
	nop
	sbi PORTA, LCD_E		
	nop
	nop
	nop
	cbi PORTA, LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_write_data		; write data to LCD, waits for BF
	out PORTF, r27			; set data port
	ldi r18, (1 << LCD_RS)|(0 << LCD_RW)
	out PORTA, r18			; RS = 1, RW = 0 for data write
	nop
	sbi PORTA, LCD_E		;
	nop
	nop
	nop
	cbi PORTA, LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_wait_busy		; read from LCD until BF is clear
	clr r18
	out DDRF, r17			; read from LCD
	ldi r18, (0 << LCD_RS)|(1 << LCD_RW)
	out PORTA, r18			; RS = =, RW = 1, cmd port read
busy:
	nop						
	sbi PORTA, LCD_E		; turn on enable pin
	nop						; data delay
	nop
	nop
	in r18, PINF			; read value from LCD
	cbi PORTA, LCD_E		; clear enable pin
	sbrc r18, 7			; skip next if busy flag not set
	rjmp busy				; else loop

	nop
	nop
	nop
	clr r18
	out PORTA, r18			; RS, RW = 0, IR write
	ser r18
	out DDRF, r18			; output to LCD
	nop
	nop
	nop
.endmacro

.macro delay				; delay for 1us
loop1:
	ldi r17, 3				; 1
loop2:
	dec r17				; 1
	nop						; 1
	brne loop2				; 2 taken, 1 not ----> inner loop total is 11 cycles
	subi r18, 1				; 1
	sbci r19, 0				; 1
	brne loop1				; 2 taken, each outer iteration is 11 + 1 + 1 + 1 + 2 = 16 clock cycles at 16Mhz = 1us
.endmacro

INITIALISE_LCD:
	; prologue
	push r16
	push r17
	push r18
	push r19
	push YL
	push YH
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	ser r17
	out DDRF, r17
	out DDRA, r17
	clr r17
	out PORTF, r17
	out PORTA, r17
	;
	ldi r18, low(15000)		; delay 15ms
	ldi r19, high(15000)
	delay
	ldi r16, 0b00111000	; 2 x 5 x 7 r18 = 1, 8bits | N = 1, 2-line | F = 0, 5 x 7 dots
	lcd_write_cmd			; 1st function cmd set

	ldi r18, low(4100)		; delay 4.1ms
	ldi r19, high(4100)
	delay
	lcd_write_cmd			; 2nd function cmd set

	ldi r18, low(100)		; delay 4.1ms
	ldi r19, high(100)
	delay
	lcd_write_cmd			; 3rd function cmd set
	lcd_write_cmd			; final function cmd set
	;
	lcd_wait_busy			; wait until ready

	ldi r16, 0b00001000	; LCD display off
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00000001	; LCD display clear
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00000110	; increment, no shift
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00001111	; LCD display on, cursor, blink
	lcd_write_cmd
	lcd_wait_busy

	;epilogue
	adiw Y, 1
	out SPH, YH
	out SPL, YL
	pop YH
	pop YL
	pop r19
	pop r18
	pop r17
	pop r16
	ret

decimal_conversion:
	ldi r23, '0'
    mov r26, r24             ; Load result (y) r24 into register r26.
    tst r26                   ; Test if result is zero

positive_number:
    ; Convert hundreds place
    ldi r28, 100              ; Load 100 for division
    rcall divide               ; Call divide subroutine
    mov r27, r29               ; Get the quotient (hundreds digit) in r27
    cpi r27, 0                ; Check if it's zero
    brne display_digit         ; If non-zero, display it
    rjmp check_tens_digit      ; Otherwise, move to tens place

display_digit:
    add r27, r23              ; Convert to ASCII ('0' = 0x30)
    lcd_write_data
	lcd_wait_busy

check_tens_digit:
    ; Convert tens place
    ldi r28, 10               ; Load 10 for division
    rcall divide               ; Call divide subroutine
    mov r27, r29               ; Get the quotient (tens digit) in r27
    cpi r27, 0                ; Check if it's zero
    brne display_tens_digit    ; If non-zero, display it
    rjmp display_ones_digit    ; Otherwise, move to ones place

display_tens_digit:
    add r27, r23              ; Convert to ASCII
    lcd_write_data
	lcd_wait_busy
display_ones_digit:
    ; Ones place is left in r30 (remainder of tens division)
    add r30, r23              ; Convert to ASCII
	mov r27, r30
    lcd_write_data        ; Send ones digit to LCDlcd_write_data
	lcd_wait_busy

    ret                        ; Return from subroutine

; Divide r26 by r28, result is quotient in r29 and remainder in r30
divide:
    clr r29                   ; Clear quotient register
    clr r30                   ; Clear remainder register

divide_loop:
    cp r26, r28               ; Compare r26 (dividend) with r28 (divisor)
    brlo divide_done           ; If r26 < r28, exit loop

    sub r26, r28              ; Subtract divisor from dividend
    inc r29                   ; Increment quotient
    rjmp divide_loop           ; Repeat until r26 < r28

divide_done:
    mov r30, r26              ; Store remainder in r30
    ret                        ; Return with quotient in r29 and remainder in r30