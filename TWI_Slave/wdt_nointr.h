/*
 * Variants of <avr/wdt.h> functions that assume interrupts are disabled
 *
 * Additional assumptions:
 *
 * - CPU clock speed is sufficiently high that there is no need to preserve
 *   the WDT prescaler when disabling the WDT, nor is function call overhead
 *   critical for the timing of changing WDT settings.
 *
 * - WDT control register name and layout is identical to ATtiny88.
 *
 * - Any necessary clearing of WDRF is already done.
 */

#ifndef WDT_NOINTR_H

#if !defined(WDTCSR) || !defined(WDCE) || !defined(WDP3)
#error "unsupported architecture"
#endif

#include <avr/io.h>
#include <avr/wdt.h>
#include <stdint.h>

static void
__attribute__ ((__noinline__))
wdt_cmd_nointr(const uint8_t value) {
    __asm__ __volatile__ (
        "wdr\n\t"
        "sts %0, %1\n\t"
        "sts %0, %2\n\t"
        : /* no outputs */
        : "n" (_SFR_MEM_ADDR(WDTCSR)),
        "r" ((uint8_t)(_BV(WDCE) | _BV(WDE))),
        "r" (value)
    );
}

static inline void
wdt_enable_nointr(uint8_t value)
{
    uint8_t tmp = (value & 0x08 ? _BV(WDP3) : 0x00);
    tmp |= _BV(WDE) | (value & 0x07);
    wdt_cmd_nointr(tmp);
}

static inline void
wdt_disable_nointr(void) {
    wdt_cmd_nointr(0);
}

#endif
