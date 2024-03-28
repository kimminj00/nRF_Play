	.cpu cortex-m4
	.arch armv7e-m
	.fpu fpv4-sp-d16
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 6
	.eabi_attribute 34, 1
	.eabi_attribute 38, 1
	.eabi_attribute 18, 4
	.file	"SEGGER_RTT.c"
	.text
.Ltext0:
	.file 1 "C:\\Users\\SJIT\\Desktop\\IET10N_Dev__git\\external\\segger_rtt\\SEGGER_RTT.c"
	.section	.data._aTerminalId,"aw"
	.align	2
	.type	_aTerminalId, %object
	.size	_aTerminalId, 16
_aTerminalId:
	.ascii	"0123456789ABCDEF"
	.global	_SEGGER_RTT
	.section	.bss._SEGGER_RTT,"aw",%nobits
	.align	2
	.type	_SEGGER_RTT, %object
	.size	_SEGGER_RTT, 120
_SEGGER_RTT:
	.space	120
	.section	.bss._acUpBuffer,"aw",%nobits
	.align	2
	.type	_acUpBuffer, %object
	.size	_acUpBuffer, 512
_acUpBuffer:
	.space	512
	.section	.bss._acDownBuffer,"aw",%nobits
	.align	2
	.type	_acDownBuffer, %object
	.size	_acDownBuffer, 16
_acDownBuffer:
	.space	16
	.section	.bss._ActiveTerminal,"aw",%nobits
	.type	_ActiveTerminal, %object
	.size	_ActiveTerminal, 1
_ActiveTerminal:
	.space	1
	.section .rodata
	.align	2
.LC0:
	.ascii	"Terminal\000"
	.align	2
.LC1:
	.ascii	"RTT\000"
	.align	2
.LC2:
	.ascii	"SEGGER\000"
	.section	.text._DoInit,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.type	_DoInit, %function
_DoInit:
.LFB145:
	.loc 1 294 27
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	sp, sp, #8
.LCFI0:
	.loc 1 299 5
	ldr	r3, .L2
	str	r3, [sp, #4]
	.loc 1 300 25
	ldr	r3, [sp, #4]
	movs	r2, #2
	str	r2, [r3, #16]
	.loc 1 301 25
	ldr	r3, [sp, #4]
	movs	r2, #2
	str	r2, [r3, #20]
	.loc 1 305 27
	ldr	r3, [sp, #4]
	ldr	r2, .L2+4
	str	r2, [r3, #24]
	.loc 1 306 27
	ldr	r3, [sp, #4]
	ldr	r2, .L2+8
	str	r2, [r3, #28]
	.loc 1 307 27
	ldr	r3, [sp, #4]
	mov	r2, #512
	str	r2, [r3, #32]
	.loc 1 308 27
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #40]
	.loc 1 309 27
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #36]
	.loc 1 310 27
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #44]
	.loc 1 314 29
	ldr	r3, [sp, #4]
	ldr	r2, .L2+4
	str	r2, [r3, #72]
	.loc 1 315 29
	ldr	r3, [sp, #4]
	ldr	r2, .L2+12
	str	r2, [r3, #76]
	.loc 1 316 29
	ldr	r3, [sp, #4]
	movs	r2, #16
	str	r2, [r3, #80]
	.loc 1 317 29
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #88]
	.loc 1 318 29
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #84]
	.loc 1 319 29
	ldr	r3, [sp, #4]
	movs	r2, #0
	str	r2, [r3, #92]
	.loc 1 325 3
	ldr	r3, [sp, #4]
	adds	r3, r3, #7
	ldr	r2, .L2+16
	ldr	r0, [r2]
	str	r0, [r3]	@ unaligned
	.loc 1 326 3
	ldr	r3, [sp, #4]
	ldr	r2, .L2+20
	ldr	r0, [r2]
	str	r0, [r3]	@ unaligned
	ldrh	r1, [r2, #4]	@ unaligned
	ldrb	r2, [r2, #6]
	strh	r1, [r3, #4]	@ unaligned
	strb	r2, [r3, #6]
	.loc 1 327 14
	ldr	r3, [sp, #4]
	movs	r2, #32
	strb	r2, [r3, #6]
	.loc 1 328 1
	nop
	add	sp, sp, #8
.LCFI1:
	@ sp needed
	bx	lr
.L3:
	.align	2
.L2:
	.word	_SEGGER_RTT
	.word	.LC0
	.word	_acUpBuffer
	.word	_acDownBuffer
	.word	.LC1
	.word	.LC2
.LFE145:
	.size	_DoInit, .-_DoInit
	.section	.text._WriteBlocking,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.type	_WriteBlocking, %function
_WriteBlocking:
.LFB146:
	.loc 1 452 101
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{lr}
.LCFI2:
	sub	sp, sp, #36
.LCFI3:
	str	r0, [sp, #12]
	str	r1, [sp, #8]
	str	r2, [sp, #4]
	.loc 1 463 19
	movs	r3, #0
	str	r3, [sp, #24]
	.loc 1 464 9
	ldr	r3, [sp, #12]
	ldr	r3, [r3, #12]
	str	r3, [sp, #20]
.L8:
	.loc 1 466 11
	ldr	r3, [sp, #12]
	ldr	r3, [r3, #16]
	str	r3, [sp, #16]
	.loc 1 467 8
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	cmp	r2, r3
	bls	.L5
	.loc 1 468 31
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	subs	r3, r2, r3
	.loc 1 468 23
	subs	r3, r3, #1
	str	r3, [sp, #28]
	b	.L6
.L5:
	.loc 1 470 30
	ldr	r3, [sp, #12]
	ldr	r2, [r3, #8]
	.loc 1 470 62
	ldr	r1, [sp, #16]
	ldr	r3, [sp, #20]
	subs	r3, r1, r3
	.loc 1 470 45
	add	r3, r3, r2
	.loc 1 470 23
	subs	r3, r3, #1
	str	r3, [sp, #28]
.L6:
	.loc 1 472 23
	ldr	r3, [sp, #12]
	ldr	r2, [r3, #8]
	ldr	r