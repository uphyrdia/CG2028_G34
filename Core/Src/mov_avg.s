/*
 * mov_avg.s
 *
 * CG2028 Assignment starter file.
 */
.syntax unified
.cpu cortex-m4
.thumb
.global ewma_filter
.type ewma_filter, %function

.text
.align 2

@ CG2028 Assignment
@ (c) ECE NUS
@ Write Student 1's Name here: ABCD (A1234567R)
@ Write Student 2's Name here: WXYZ (A0000007X)
@
@ Function prototype:
@   int ewma_filter(int new_data, int old_output, int alpha_percent);
@
@ ARM calling convention:
@   R0 = new_data       (signed integer sensor sample)
@   R1 = old_output     (previous filtered output)
@   R2 = alpha_percent  (integer from 0 to 100)
@   Return R0 = (alpha_percent * new_data
@                + (100 - alpha_percent) * old_output) / 100
@
@ Notes:
@ - Use signed integer arithmetic.
@ - Integer division must truncate towards zero, matching C integer division.
@ - Preserve all callee-saved registers that you use (R4-R11).
@ - Do not call a C helper function and do not use floating-point instructions.
@
@ Register table:
@   R0 = new_data on entry; weighted product, then weighted sum; filtered output on return
@   R1 = old_output (unchanged)
@   R2 = alpha_percent on entry; then 100 - alpha_percent; finally 100 for signed division
@   R3 = Not used
@   R4 = Not used
@
@ Write your program from here.
ewma_filter:
    @ PUSH {r4-r7, lr}

    MUL  r0, r2          	 @ new_data *= alpha_percent.
    RSB  r2, r2, #100        @ Reuse R2 for 100 - alpha_percent.
    MLA  r0, r1, r2, r0      @ Add (100 - alpha_percent) * old_output.
    MOVS r2, #100            @ Reuse R2 for the divisor. S-suffix saves 2 bytes of encoding the instruction.
    SDIV r0, r2              @ Signed division by 100 truncates towards zero.

    @ POP  {r4-r7, pc}
    BX lr

.size ewma_filter, .-ewma_filter
