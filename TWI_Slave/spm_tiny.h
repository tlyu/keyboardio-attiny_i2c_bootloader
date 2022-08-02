#ifndef SPM_TINY_H

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared function to execute SPM commands after busy-waiting.
 * Compared to <avr/boot.h>, this saves instructions by using
 * OUT to access SPMCSR, because it's in the I/O address space
 * on the ATtiny88.
 *
 * Hardwired parameters:
 * r16=command
 * r1:r0=word for page fill
 * r31:r30=address
 */
static void __attribute__ ((__noinline__, __used__))
tspm_cmd_asm(void)
{
    /* Use r24 as a temporary register, because SPM uses the usual r0 */
    __asm__ __volatile__(
        "1: in r24, %[spmcsr]\n\t"
        "sbrc r24, %[bit]\n\t"
        "rjmp 1b\n\t"
        "out %[spmcsr], r16\n\t"
        "spm\n\t"
        "clr __zero_reg__"
        : /* no outputs */
        : [spmcsr] "I" (_SFR_IO_ADDR(SPMCSR)),
        [bit] "I" (SELFPRGEN)
    );
}

/* These inline functions interface with the hardwired registers */
static inline void __attribute__ ((__always_inline__))
tspm_cmd(uint16_t addr, uint8_t cmd)
{
    register uint8_t _cmd __asm__("r16") = cmd;

    __asm__ __volatile__(
        "rcall tspm_cmd_asm" :: "z" (addr), "r" (_cmd) : "r24"
    );
}

static inline void __attribute__ ((__always_inline__))
tspm_cmd_word(uint16_t addr, uint16_t word, uint8_t cmd)
{
    register uint16_t _word __asm__("r0") = word;
    register uint8_t _cmd __asm__("r16") = cmd;

    __asm__ __volatile__(
        "rcall tspm_cmd_asm" :: "z" (addr), "r" (_word), "r" (_cmd) : "r24"
    );
}

static inline void __attribute__ ((__always_inline__))
tspm_page_fill(uint16_t addr, uint16_t word)
{
    tspm_cmd_word(addr, word, _BV(SELFPROGEN));
}

static inline void __attribute__ ((__always_inline__))
tspm_page_erase(uint16_t addr)
{
    tspm_cmd(addr, (_BV(PGERS) | _BV(SELFPROGEN)));
}

static inline void __attribute__ ((__always_inline__))
tspm_page_write(uint16_t pageAddress) {
    tspm_cmd(pageAddress, (_BV(PGWRT) | _BV(SELFPROGEN)));
}

#ifdef __cplusplus
}
#endif

#endif /* !defined(SPM_TINY_H) */
