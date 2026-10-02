//! Every peripheral the mount uses, set up by writing the STM32F401's registers directly (same style as the
//! STM32Test blinky). Register addresses and bit numbers are from RM0368, the F401 reference manual.
//!
//! Nucleo-F401RE pins (Arduino header names in brackets, so the Uno wiring carries straight over):
//!   PC7  [D9]   TIM3 channel 2   AZ servo signal, 50 Hz
//!   PB6  [D10]  TIM4 channel 1   EL servo signal, 50 Hz
//!   PA0 / PA1 [A0/A1]  deliberately left alone: the Stingrays have no feedback wire to read
//!   PB8 / PB9 [D15/D14]  unused, left in their reset state (no encoder is fitted)
//!   PA2 / PA3   USART2 to the ST-LINK, which shows up on the PC as /dev/ttyACM0
//!   PA5  [D13]  LD2, the green user LED
//!
//! Clock: the chip's internal 16 MHz oscillator (HSI), untouched. Every bus runs at 16 MHz, which is plenty.
use core::ptr::{read_volatile, write_volatile};

#[inline(always)]
fn rd(addr: u32) -> u32 { unsafe { read_volatile(addr as *const u32) } }
#[inline(always)]
fn wr(addr: u32, v: u32) { unsafe { write_volatile(addr as *mut u32, v) } }
/// Read-modify-write: clear the `clear` bits, then set the `set` bits
#[inline(always)]
fn modify(addr: u32, clear: u32, set: u32) { wr(addr, rd(addr) & !clear | set) }

const CLOCK_HZ: u32 = 16_000_000;

//---------------------------------------------------------------------------------------------- addresses
const RCC: u32 = 0x4002_3800;
const RCC_AHB1ENR: u32 = RCC + 0x30;
const RCC_APB1ENR: u32 = RCC + 0x40;
const RCC_CSR: u32 = RCC + 0x74;

const GPIOA: u32 = 0x4002_0000;
const GPIOB: u32 = 0x4002_0400;
const GPIOC: u32 = 0x4002_0800;
// offsets inside a GPIO port
const MODER: u32 = 0x00;
const OTYPER: u32 = 0x04;
const OSPEEDR: u32 = 0x08;
const PUPDR: u32 = 0x0C;
const BSRR: u32 = 0x18;
const AFRL: u32 = 0x20;

const TIM2: u32 = 0x4000_0000;
const TIM3: u32 = 0x4000_0400;
const TIM4: u32 = 0x4000_0800;
// offsets inside a timer
const CR1: u32 = 0x00;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;
const CCR2: u32 = 0x38;

const USART2: u32 = 0x4000_4400;
const USART2_SR: u32 = USART2 + 0x00;
const USART2_DR: u32 = USART2 + 0x04;
const USART2_BRR: u32 = USART2 + 0x08;
const USART2_CR1: u32 = USART2 + 0x0C;
const USART2_CR3: u32 = USART2 + 0x14;

const DMA1: u32 = 0x4002_6000;
const DMA1_HISR: u32 = DMA1 + 0x04;
const DMA1_HIFCR: u32 = DMA1 + 0x0C;
/// Stream 5's transfer-complete flag in HISR / HIFCR: set each time the circular DMA wraps the ring
const TCIF5: u32 = 1 << 11;
const DMA1_S5CR: u32 = DMA1 + 0x10 + 0x18 * 5;
const DMA1_S5NDTR: u32 = DMA1_S5CR + 0x04;
const DMA1_S5PAR: u32 = DMA1_S5CR + 0x08;
const DMA1_S5M0AR: u32 = DMA1_S5CR + 0x0C;

const IWDG_KR: u32 = 0x4000_3000;
const IWDG_PR: u32 = 0x4000_3004;
const IWDG_RLR: u32 = 0x4000_3008;
const DBGMCU_APB1_FZ: u32 = 0xE004_2008;

//---------------------------------------------------------------------------------------------- setup
/// Pin modes (the two MODER bits)
const OUTPUT: u32 = 0b01;
const ALT: u32 = 0b10;

/// Configure one pin: mode, alternate function number (for ALT), open drain or push-pull, pull-up or not
fn pin(port: u32, n: u32, mode: u32, af: u32, open_drain: bool, pull_up: bool) {
    modify(port + MODER, 0b11 << (2 * n), mode << (2 * n));
    modify(port + OTYPER, 1 << n, (open_drain as u32) << n);
    modify(port + OSPEEDR, 0b11 << (2 * n), 0b10 << (2 * n));          // fast edges
    modify(port + PUPDR, 0b11 << (2 * n), (pull_up as u32) << (2 * n)); // 01 = pull-up
    if mode == ALT {
        let reg = port + AFRL + 4 * (n / 8);                               // AFRL for pins 0-7, AFRH for 8-15
        modify(reg, 0xF << (4 * (n % 8)), af << (4 * (n % 8)));
    }
}

/// Bring up everything. Servo outputs start silent (no pulses): the servos stay limp until the PC moves them.
pub fn init(baud: u32) {
    // 1. clock gates: GPIO A, B, C and DMA1 on AHB1; TIM2/3/4 and USART2 on APB1.
    //    ADC1 and I2C1 are deliberately left off: there is nothing to read (see the pin map above).
    modify(RCC_AHB1ENR, 0, 1 << 0 | 1 << 1 | 1 << 2 | 1 << 21);
    modify(RCC_APB1ENR, 0, 1 << 0 | 1 << 1 | 1 << 2 | 1 << 17);
    rd(RCC_APB1ENR); // give the clocks a moment to start

    // 2. pins
    pin(GPIOA, 5, OUTPUT, 0, false, false);       // LD2
    pin(GPIOA, 2, ALT, 7, false, false);          // USART2 TX
    pin(GPIOA, 3, ALT, 7, false, true);           // USART2 RX (pulled up so a loose wire reads idle)
    pin(GPIOC, 7, ALT, 2, false, false);          // TIM3 CH2 -> AZ servo
    pin(GPIOB, 6, ALT, 2, false, false);          // TIM4 CH1 -> EL servo

    // 3. TIM2: free-running 32-bit microsecond counter (16 MHz / 16), wraps every 71 minutes
    wr(TIM2 + PSC, CLOCK_HZ / 1_000_000 - 1);
    wr(TIM2 + ARR, 0xFFFF_FFFF);
    wr(TIM2 + EGR, 1);                            // load the prescaler now
    wr(TIM2 + CR1, 1);

    // 4. TIM3 / TIM4: 50 Hz servo frames. 16 MHz / 5 = 3.2 MHz ticks, 64000 ticks per 20 ms frame, so a
    //    pulse is set in steps of 0.3125 microseconds (about 0.07 degrees on the azimuth gearbox)
    for t in [TIM3, TIM4] {
        wr(t + PSC, 4);
        wr(t + ARR, 64_000 - 1);
        wr(t + CCR1, 0);
        wr(t + CCR2, 0);
        // PWM mode 1 (110) with preload on both channels, so a new width starts on a frame boundary
        wr(t + CCMR1, 0b110 << 4 | 1 << 3 | 0b110 << 12 | 1 << 11);
        wr(t + EGR, 1);
        wr(t + CR1, 1 << 7 | 1);                  // auto-reload preload, counter on
    }
    wr(TIM3 + CCER, 1 << 4);                      // CH2 output on
    wr(TIM4 + CCER, 1 << 0);                      // CH1 output on

    // 5. USART2 8N1, received bytes land in RX_BUF by DMA so a slow reply can never drop a command
    wr(USART2_BRR, (CLOCK_HZ + baud / 2) / baud); // 16x oversampling: BRR = clock / baud
    wr(USART2_CR3, 1 << 6);                       // DMAR: receiver requests DMA
    wr(USART2_CR1, 1 << 13 | 1 << 3 | 1 << 2);    // UE, TE, RE
    // DMA1 stream 5 channel 4 is USART2_RX: peripheral -> memory, bytes, memory increments, circular
    wr(DMA1_S5CR, 0);
    while rd(DMA1_S5CR) & 1 != 0 {}
    wr(DMA1_HIFCR, 0b111101 << 6);                // clear stream 5's flags
    wr(DMA1_S5PAR, USART2_DR);
    wr(DMA1_S5M0AR, (&raw const RX_BUF) as u32);
    wr(DMA1_S5NDTR, RX_LEN as u32);
    wr(DMA1_S5CR, 4 << 25 | 0b10 << 16 | 1 << 10 | 1 << 8 | 1);   // CHSEL 4, priority high, MINC, CIRC, EN
}

//---------------------------------------------------------------------------------------------- time
/// Microseconds since boot, wrapping (use wrapping_sub for differences)
#[inline(always)]
pub fn micros() -> u32 { rd(TIM2 + CNT) }

/// Kept for bring-up and for any peripheral that needs a settling wait; nothing uses it right now
#[allow(dead_code)]
pub fn delay_us(us: u32) {
    let t0 = micros();
    while micros().wrapping_sub(t0) < us {}
}

//---------------------------------------------------------------------------------------------- LED
pub fn led(on: bool) { wr(GPIOA + BSRR, if on { 1 << 5 } else { 1 << (5 + 16) }); }

//---------------------------------------------------------------------------------------------- servos
/// Timer ticks per microsecond of pulse (3.2 MHz)
const TICKS_PER_US: f32 = 3.2;

/// Drive a servo pulse width in microseconds; 0 turns the output off (steady low, servo goes limp)
pub fn servo_us(axis: usize, us: f32) {
    let ticks = if us <= 0.0 { 0 } else { (us * TICKS_PER_US + 0.5) as u32 };
    match axis {
        0 => wr(TIM3 + CCR2, ticks),
        _ => wr(TIM4 + CCR1, ticks),
    }
}

//---------------------------------------------------------------------------------------------- serial
const RX_LEN: usize = 1024;
static mut RX_BUF: [u8; RX_LEN] = [0; RX_LEN];

/// Reader for the DMA ring: the DMA owns the write side, `tail` is how far we have read.
///
/// The DMA's position alone (RX_LEN - NDTR) cannot tell one lap of the ring from two, so a burst that
/// arrives faster than the replies drain would silently overwrite unread commands. The reader therefore
/// also counts laps: the DMA sets its transfer-complete flag each time it wraps, `read` counts and clears
/// it, and compares bytes written with bytes read. More than a ring's worth unread is an overrun: the
/// unread input is discarded and `overrun` tells the main loop to say so. (`read` runs at least once per
/// reply, and no reply is longer than a few hundred bytes of arrival time, so a lap is never missed.)
pub struct Rx { tail: usize, laps_read: u32, laps_written: u32, pub overrun: bool }

impl Rx {
    pub const fn new() -> Self { Rx { tail: 0, laps_read: 0, laps_written: 0, overrun: false } }

    /// The DMA's write position, counting any wrap since the last call
    fn head(&mut self) -> usize {
        loop {
            // NDTR counts down from RX_LEN as bytes arrive. Read it either side of the flag: if it went
            // UP, the ring wrapped in between and the pair does not belong together, so read again.
            let n1 = rd(DMA1_S5NDTR);
            let wrapped = rd(DMA1_HISR) & TCIF5 != 0;
            let n2 = rd(DMA1_S5NDTR);
            if n2 > n1 { continue; }
            if wrapped { wr(DMA1_HIFCR, TCIF5); self.laps_written = self.laps_written.wrapping_add(1); }
            return (RX_LEN - n2 as usize) % RX_LEN;
        }
    }

    pub fn read(&mut self) -> Option<u8> {
        let head = self.head();
        let written = self.laps_written as u64 * RX_LEN as u64 + head as u64;
        let read = self.laps_read as u64 * RX_LEN as u64 + self.tail as u64;
        if written > read + RX_LEN as u64 {
            // Lapped: the oldest unread bytes are already overwritten. Drop everything and resync.
            self.tail = head; self.laps_read = self.laps_written; self.overrun = true;
            return None;
        }
        if written == read { return None; }
        let b = unsafe { read_volatile((&raw const RX_BUF as *const u8).add(self.tail)) };
        self.tail += 1;
        if self.tail == RX_LEN { self.tail = 0; self.laps_read = self.laps_read.wrapping_add(1); }
        Some(b)
    }
}

/// Blocking transmitter; `write!` into it
pub struct Tx;

impl core::fmt::Write for Tx {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            while rd(USART2_SR) & 1 << 7 == 0 {}   // TXE: the data register is free
            wr(USART2_DR, b as u32);
        }
        Ok(())
    }
}

//---------------------------------------------------------------------------------------------- watchdog
/// True if the last reset came from the watchdog. Clears the reset flags.
pub fn reset_was_watchdog() -> bool {
    let wd = rd(RCC_CSR) & 1 << 29 != 0;
    modify(RCC_CSR, 0, 1 << 24);                  // RMVF
    wd
}

/// Independent watchdog, about 1 s: if the main loop ever stalls, the chip resets and the servos go limp.
/// Frozen while a debugger has the core halted.
pub fn watchdog_start() {
    modify(DBGMCU_APB1_FZ, 0, 1 << 12);
    wr(IWDG_KR, 0xCCCC);                          // start (the LSI clock turns on by itself)
    wr(IWDG_KR, 0x5555);                          // unlock PR and RLR
    wr(IWDG_PR, 4);                               // 32 kHz / 64 = 500 Hz
    wr(IWDG_RLR, 500);                            // 500 ticks = 1 s
    wr(IWDG_KR, 0xAAAA);
}

#[inline(always)]
pub fn watchdog_feed() { wr(IWDG_KR, 0xAAAA); }
