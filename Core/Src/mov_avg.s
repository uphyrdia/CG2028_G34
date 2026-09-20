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
@ Write Student 2's Name here: Chen Xingtong (A0300595H)
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
@ Register usage:
@   R0 = new_data -> weighted product -> weighted sum -> filtered output
@   R1 = old_output (never modified)
@   R2 = alpha_percent -> (100 - alpha_percent) -> 100 (the divisor)
@   R3-R11 = not used

ewma_filter:
    MUL  r0, r2              @ R0 = new_data * alpha_percent
    RSB  r2, r2, #100        @ R2 = 100 - alpha_percent (alpha is finished with)
    MLA  r0, r1, r2, r0      @ R0 = old_output * (100 - alpha) + R0
    MOVS r2, #100            @ R2 = divisor. MOVS saves 2 bytes of encoding the instruction.
    SDIV r0, r2              @ R0 = R0 / 100, truncated towards zero
    BX   lr                  @ Result is already in R0, return to caller

.size ewma_filter, .-ewma_filter
