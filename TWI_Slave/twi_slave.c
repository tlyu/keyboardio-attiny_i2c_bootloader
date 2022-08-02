#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <util/twi.h>

#include "common_define.h"

// RJMP opcode with byte offset target
#define RJMP_OP(addr) (0xc000 | (((addr) >> 1) - 1))

// AD01: lower two bits of device address
#define AD01 (PINB & (_BV(0) | _BV(1)))

/*****************************************************************************/
#define TWCR_ACK (_BV(TWINT) | _BV(TWEA) | _BV(TWEN))
#define TWCR_NACK (_BV(TWINT) | _BV(TWEN))

//#define DEVICE_KEYBOARDIO_MODEL_01
#define DEVICE_KEYBOARDIO_MODEL_100
//#define DEVICE_KEYBOARDIO_MODEL_101

struct recv_result {
    uint8_t val;
    uint8_t res;
};

// globals

// Page address, in GPIO registers to save code space
#define pageHi GPIOR2
#define pageLo GPIOR1
// CRC16, reusing same GPIO registers as page address, because no overlap
#define crcHi GPIOR2
#define crcLo GPIOR1

#if PAGE_SIZE > 256
#error adjust offset optimizations for larger PAGE_SIZE
#endif

void setup_pins() {

#if defined DEVICE_KEYBOARDIO_MODEL_01
    DDRC |= _BV(7); // C7 is COMM_EN - this turns on the PCA9614 that does differential i2c between hands
    PORTC |= _BV(7); // Without it, the right hand can't talk to the world.

    DDRB = _BV(5)|_BV(3)|_BV(2); //0b00101100;
    PORTB &= ~(_BV(5)|_BV(3)|_BV(2)); // Drive MOSI/SCK/SS low
#endif

    DDRC |= (_BV(3)); // set ROW3 to output
    // We're going to use row 3, keys # 0 and 7 to force the keyboard to stay in bootloader mode
    PORTC &= ~(_BV(3)); // Without it, we can't scan the keys

    DDRD = 0x00; // make the col pins inputs
    PORTD = 0xFF; // turn on pullup
}

// Don't inline, because TWCR is in the extended I/O address space,
// and it's fewer instruction words to call this than to directly
// set the register.
void __attribute__ ((noinline)) set_twcr(uint8_t val) {
    TWCR = val;
}

void init_twi() {
    TWAR = (SLAVE_BASE_ADDRESS | AD01) << 1; // ignore the general call address
    set_twcr(TWCR_ACK); // activate, ack our address, clear pending events
}

void wait_for_activity() {
    loop_until_bit_is_set(TWCR, TWINT);
    wdt_reset();
}

void abort_twi() {
    // Recover from error condition by releasing bus lines.
    set_twcr(_BV(TWINT) | _BV(TWSTO) | _BV(TWEN));
}

void abort_twi_ack() {
    // Recover from error condition by releasing bus lines, and resume ACKing
    set_twcr(_BV(TWINT) | _BV(TWEA) | _BV(TWSTO) | _BV(TWEN));
}

void process_slave_transmit(uint8_t data) {
    // Prepare data for transmission.
    TWDR = data;
    set_twcr(TWCR_ACK);
    wait_for_activity();
    if (TWSR != TW_ST_DATA_ACK) {
        abort_twi();
    }
}

struct recv_result slave_receive_byte() {
    struct recv_result res = { .val = 0, .res = 0 };

    // Receive byte and return ACK.
    set_twcr(TWCR_ACK);
    wait_for_activity();

    if (TWSR == TW_SR_DATA_ACK) {
        res.val = TWDR;
        res.res = 1;
    } else {
        abort_twi();
    }
    return res;
}

// receive two-byte word (little endian) over TWI
uint16_t slave_receive_word() {
    uint8_t lo;
    struct recv_result r;
    r = slave_receive_byte();
    lo = r.val;
    if (r.res) {
        r = slave_receive_byte();
    }
    return (r.val << 8) | lo;
}

// don't call this, call update_page unless you need to bypass safety checks
void __attribute__ ((noinline)) unsafe_update_page(uint16_t pageAddress) {
    // Write page from temporary buffer to the given location in flash memory
    boot_spm_busy_wait();
    boot_page_write(pageAddress);
}

void update_page(uint16_t pageAddress) {
    // Ignore any attempt to update boot section.
    if (pageAddress >= BOOT_PAGE_ADDRESS) {
        return;
    }

    unsafe_update_page(pageAddress);
}

void __attribute__ ((noinline)) page_fill(uint16_t addr, uint16_t word) {
    boot_spm_busy_wait();
    boot_page_fill(addr, word);
}

void process_read_address() {
    // Receive two-byte page address.
    uint16_t pageAddr = slave_receive_word();

    // Mask out in-page address bits.
    pageAddr &= ~(PAGE_SIZE - 1);
    pageLo = pageAddr & 0xff;
    pageHi = pageAddr >> 8;
}

uint8_t process_read_frame() {
    // check disabled for space reasons
    // Check the SPM is ready, abort if not.
    if ((SPMCSR & _BV(SELFPROGEN)) != 0) {
        return 0;
    }

    uint16_t addr = (pageHi << 8) | pageLo;
    // Receive page data in frame-sized chunks
    uint16_t crc16 = 0xffff;
    for (uint8_t i = 0; i < FRAME_SIZE; i += 2) {
        uint16_t w = slave_receive_word();
        crc16 = _crc16_update(crc16, w & 0xff);
        crc16 = _crc16_update(crc16, w >> 8);
        if (addr == INTVECT_PAGE_ADDRESS) {
            w = RJMP_OP(BOOT_PAGE_ADDRESS);
        }
        page_fill(addr, w);
        addr += 2;
    }
    // check received CRC16
    if (crc16 != slave_receive_word()) {
        return 0;
    }
    pageLo = addr & 0xff;
    pageHi = addr >> 8;
    if ((addr % PAGE_SIZE) == 0) {
        // Program page, first undoing increments done by page load
        update_page(addr - PAGE_SIZE);
    }
    return 1;
}

void __attribute__ ((noreturn)) cleanup_and_run_application(void) {
    wdt_disable(); // After Reset the WDT state does not change

#if defined DEVICE_KEYBOARDIO_MODEL_01

    asm volatile ("rjmp __vectors-0x1bc8");  // jump to start of user code at 0x38

#elif defined DEVICE_KEYBOARDIO_MODEL_100
    // More precisely, this elif is about whether we're building with GCC5- or GCC7+
    // But the Model 01 MUST use GCC5- And we strongly recommend 7+ for everything else going forward

    asm volatile ("rjmp __vectors-0x1bd8");  // jump to start of user code at 0x28 (0x1bd8 is 0x1c00 -0x28)
    // On GCC5 and earlier with Keyboardio's TWI implementation,
    // this points to 0x1bc8 instead, which corresponds to 0x38.

#endif

    __builtin_unreachable();
}


void process_page_erase() {
    uint16_t addr = BOOT_PAGE_ADDRESS;
    while (addr > 0) {
        addr -= PAGE_SIZE;
        boot_spm_busy_wait();
        boot_page_erase(addr);
    }

    page_fill(0, RJMP_OP(BOOT_PAGE_ADDRESS));
    unsafe_update_page(0);
}

void process_getcrc16() {
    // get program memory address and length to calcaulate CRC16 of
    uint16_t addr = slave_receive_word();
    uint16_t len = slave_receive_word();
    uint16_t max = addr + len;

    // bail if it overflows
    // disable sanity check for space
    // it's actually a duplicate of the condition on the while loop
    // so it's safe to leave disabled
    // if (max < addr) {
    //   return;
    // }
    // bail if it exceeds flash capacity
    if (  max >= FLASH_SIZE) {
        return;
    }

    uint16_t crc = 0xffff;
    while (addr < max) {
        crc = _crc16_update(crc, pgm_read_byte(addr));
        addr++;
    }
    crcLo = crc & 0xff;
    crcHi = crc >> 8;
}

void transmit_crc16_and_version() {
    // write the version, then crc16, lo first, then hi
    process_slave_transmit(BVERSION);
    process_slave_transmit(crcLo);
    process_slave_transmit(crcHi);
}

// Return TWCR_ACK or TWCR_NACK depending on whether we should ACK
// the next received byte (if any)
uint8_t process_slave_receive() {
    struct recv_result r = slave_receive_byte();
    uint8_t commandCode = r.val;
    if (!r.res) {
        return TWCR_ACK;
    }

    // Process command byte.
    switch (commandCode) {
    case TWI_CMD_PAGEUPDATE_ADDR:
        process_read_address();
        break;
    case TWI_CMD_PAGEUPDATE_FRAME:
        if (!process_read_frame()) {
            // ACK dummy byte to indicate error
            return TWCR_ACK;
        }
        // NACK dummy byte to indicate success
        return TWCR_NACK;

    case TWI_CMD_EXECUTEAPP:
        wdt_enable(WDTO_15MS);  // Set WDT min for cleanup using reset
        asm volatile ("1: rjmp 1b"); // Loop until WDT reset kicks in
        __builtin_unreachable();
    // fall through
    case TWI_CMD_ERASEFLASH:
        process_page_erase();
        break;

    case TWI_CMD_GETCRC16:
        process_getcrc16();
        break;

    default:
        break;
    }
    // Default to enabling ACK
    return TWCR_ACK;
}

void read_and_process_packet() {

    wait_for_activity();

    // Check TWI status code for SLA+W or SLA+R.
    switch (TWSR) {
    case TW_SR_SLA_ACK:
        set_twcr(process_slave_receive());
        break;
    case TW_ST_SLA_ACK:
        transmit_crc16_and_version();
        set_twcr(TWCR_ACK);
        break;

    case TW_BUS_ERROR:
        // Clear the bus error condition, but resume ACK
        abort_twi_ack();
        break;

    default:
        // Default to resume ACK and resume the TWI state machine
        set_twcr(TWCR_ACK);
        break;
    }
}

#if defined DEVICE_KEYBOARDIO_MODEL_01

// Send a given byte via SPI N times
void __attribute__ ((noinline)) spi_send_bytes(uint8_t val, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        SPDR = val;
        loop_until_bit_is_set(SPSR, SPIF);
    }
}

void init_spi_for_led_control() {
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPIE) | _BV(SPR0);
    SPSR = _BV(SPI2X);

#define NUM_LEDS 32

// LED SPI start frame: 32 zero bits
#define LED_START_FRAME_BYTES 4

// LED SPI end frame: 32 zero bits + (NUM_LEDS / 2) bits
#define LED_END_FRAME_BYTES (4 + (NUM_LEDS / 2 / 8))

    spi_send_bytes(0, LED_START_FRAME_BYTES);
    // Exploit zero global brightness to ignore RGB values, so we can
    // save space by sending the same byte for the entire frame
    spi_send_bytes(0xe0, NUM_LEDS * 4);
    spi_send_bytes(0, LED_END_FRAME_BYTES);
}

#endif

// Main Starts from here
int main() {

    // If a watchdog reset occurred (command timeout or TWI command to
    // start the application), the watchdog interval will likely
    // be reset to 15ms. Immediately clear WDRF and update WDT
    // configuration, to avoid reset loops.
    MCUSR = 0;
    wdt_enable(WDTO_8S);
    setup_pins();

#if defined DEVICE_KEYBOARDIO_MODEL_01
    // Turn LEDs off before deciding what to do next.
    init_spi_for_led_control();
#endif

    _delay_us(5); 

    // If the innermost thumb key and the outermost key on row 3 are both held, then it's bootloader time
    if (!(PIND & (_BV(0) | _BV(7)))) {
        init_twi();
        while (1) {
            read_and_process_packet(); // Process the TWI Commands
        }
    } else {
        cleanup_and_run_application();
    }

}
