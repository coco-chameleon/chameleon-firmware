// SPDX-FileCopyrightText: 2026 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

//! Controls the Chameleon's RGB LED.
//!
//! This driver uses the RMT peripheral to offload the work of controlling
//! the LED from the CPU. Calling any of the methods to change the LED state
//! immediately reconfigures the peripheral and returns. Nothing in this module
//! will ever block or trigger an interrupt. Animation is handled entirely in
//! hardware and does not require any CPU time after the initial setup.
//!
//! # Examples
//!
//! ```
//! let mut led = LedController::new(peripherals.GPIO5, peripherals.RMT);
//!
//! // set the LED to a steady green
//! led.solid(Color::new(0, 255, 0));
//!
//! // flash the LED in a pattern forever
//! // with one long pink flash followed by two faster cyan flashes
//! led.animate(&[
//!     Color::rgb(0x900005).for_ms(300),
//!     Color::off().for_ms(100),
//!     Color::rgb(0x006030).for_ms(100),
//!     Color::off().for_ms(100),
//!     Color::rgb(0x006030).for_ms(100),
//!     Color::off().for_ms(500),
//! ], None);
//! ```

// LED datasheet:
//   https://www.inolux-corp.com/datasheet/SMDLED/Addressable%20LED/IN-PI33TBTPRPGPB_v1.2.pdf
//
// The LED wants timing values of 300 ns +/- 150 ns and 900 ns +/- 150 ns.
// Hitting those exactly would require a timing resolution of 100 ns.
// To support longer delays in animations, we use a resolution of 200 ns
// instead with values of 400 ns and 1000 ns. The resulting tolerance is
// +50 ns -250 ns, and the RMT peripheral seems to hit +/- 5 ns reliably.
//
// The esp_hal::rmt driver insists on using interrupts and waiting for one
// transmission to finish before starting the next, so we drive the RMT
// peripheral directly at the register level.


use core::f32::math::round;
use esp_hal::gpio;
use esp_hal::gpio::Level;
use esp_hal::peripherals::{RMT, SYSTEM};
use esp_hal::rmt::PulseCode;


/// A color value which can be displayed by the LED.
///
/// For efficiency, colors are stored in the correct format to send to the LED.
/// Users should treat the binary format as opaque.
#[derive(Clone, Copy, Default, Eq, PartialEq)]
#[repr(transparent)]
pub struct Color(pub u32);

impl Color {
    /// Create from individual channel values.
    #[inline]
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self(((green as u32) << 16) | ((red as u32) << 8) | (blue as u32))
    }

    /// Create from packed RGB format (like HTML hex color codes).
    #[inline]
    pub const fn rgb(color: u32) -> Self {
        // swizzle RGB -> GRB
        Self((color & 0xFF0000) >> 8 | (color & 0xFF00) << 8 | color & 0xFF)
    }

    /// Create a color which turns off the LED.
    #[inline]
    pub const fn off() -> Self {
        Self(0)
    }

    #[inline]
    pub const fn red(self) -> u8 {
        (self.0 >> 8) as u8
    }

    #[inline]
    pub const fn green(self) -> u8 {
        (self.0 >> 16) as u8
    }

    #[inline]
    pub const fn blue(self) -> u8 {
        self.0 as u8
    }

    /// Converts this color into an animation frame by adding a duration.
    #[inline]
    pub const fn for_ms(self, duration: u16) -> Frame {
        Frame::new(self, duration)
    }
}

impl From<Color> for u32 {
    #[inline]
    fn from(color: Color) -> Self { color.0 }
}

impl defmt::Format for Color {
    fn format(&self, fmt: defmt::Formatter) {
        let grb = self.0;
        defmt::write!(
            fmt,
            "#{:06X}",
            (grb & 0xFF0000) >> 8 | (grb & 0xFF00) << 8 | (grb & 0xFF),
        );
    }
}


/// A frame of an LED animation, consisting of a color and a duration.
///
/// The public API uses milliseconds for ease of understanding, but durations
/// are actually implemented in terms of a roughly 13 ms tick. Input values
/// will be rounded to the nearest whole tick, so the maximum duration error
/// is just over 6.5 ms. The maximum representable duration is about 3,340 ms.
#[derive(Clone, Copy, Default, Eq, PartialEq)]
#[repr(transparent)]
pub struct Frame(pub u32);

const TICK_MS: f32 = (PulseCode::MAX_LEN * 2) as f32 * 0.0002;

impl Frame {
    /// Create from a color and a duration in milliseconds.
    pub const fn new(color: Color, duration_ms: u16) -> Self {
        let ticks = round(duration_ms as f32 / TICK_MS) as u32;
        assert!(ticks <= 255, "duration is too long");

        Self((ticks << 24) | (color.0 & 0xFFFFFF))
    }

    #[inline]
    pub const fn color(self) -> Color {
        Color(self.0 & 0x00FFFFFF)
    }

    #[inline]
    const fn duration_ticks(self) -> u32 {
        self.0 >> 24
    }

    #[inline]
    pub const fn duration_ms(self) -> u16 {
        round(self.duration_ticks() as f32 * TICK_MS) as u16
    }
}

impl From<Frame> for u32 {
    #[inline]
    fn from(frame: Frame) -> Self { frame.0 }
}

impl defmt::Format for Frame {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "{}/{}ms",
            self.color(),
            self.duration_ms(),
        )
    }
}


/// Controls the RGB LED using the RMT peripheral.
///
/// This uses all available RAM in the RMT peripheral, so it can't share
/// the peripheral with the [esp_hal::rmt] driver or any other consumer.
/// Creating an instance enables the peripheral's clock gate, connects
/// its output to the given GPIO pin, and sets the pin as an output.
/// Dropping the instance disables the peripheral's clock gate, disconnects
/// it from the GPIO pin, and turns off the pin's output mode.
pub struct LedController<'d> {
    pin: gpio::interconnect::OutputSignal<'d>,
}

impl Drop for LedController<'_> {
    fn drop(&mut self) {
        self.pin.set_output_enable(false);
        gpio::OutputSignal::RMT_SIG_0.disconnect_from(&self.pin);

        SYSTEM::regs().perip_clk_en0()
            .modify(|_, w| w.rmt_clk_en().clear_bit());
    }
}

impl<'d> LedController<'d> {
    pub fn new<'a>(
        gpio: impl gpio::interconnect::PeripheralOutput<'d>,
        _rmt: RMT<'d>
    ) -> Self {
        let pin: gpio::interconnect::OutputSignal = gpio.into();
        gpio::OutputSignal::RMT_SIG_0.connect_to(&pin);
        pin.apply_output_config(&gpio::OutputConfig::default());
        pin.set_output_enable(true);

        SYSTEM::regs().perip_clk_en0()
            .modify(|_, w| w.rmt_clk_en().set_bit());

        let reg = RMT::regs();

        reg.sys_conf().write(|w| unsafe {
            // set peripheral clock to 80 MHz
            w.sclk_sel().bits(1); // APB_CLK
            w.sclk_div_num().bits(0); // no division
            w.sclk_active().set_bit()
        });

        reg.ch0_tx_conf0().write(|w| unsafe {
            // set channel clock to 5 MHz (0.2 us period)
            w.div_cnt().bits(16);

            // don't apply a carrier
            w.carrier_en().clear_bit();

            // use all 384 words of available RMT RAM
            // this disables all other channels
            w.mem_size().bits(8)
        });

        LedController { pin }
    }

    #[inline]
    fn write(&self, value: PulseCode) {
        RMT::regs().ch0data()
            .write(|w| unsafe { w.bits(value.into()) });
    }

    fn encode(&self, color: u32) {
        let mut value = color;
        for _ in 0..24 {
            if value & 0x800000 != 0 {
                self.write(PulseCode::new(Level::High, 5, Level::Low, 2));
            } else {
                self.write(PulseCode::new(Level::High, 2, Level::Low, 5));
            }
            value = value << 1;
        }
    }

    fn delay(&self, ticks: u32) {
        for _ in 0..ticks {
            self.write(PulseCode::new(
                Level::Low, PulseCode::MAX_LEN,
                Level::Low, PulseCode::MAX_LEN,
            ));
        }
    }

    fn end(&self) {
        self.write(PulseCode::end_marker());

        assert!(
            RMT::regs().ch0_tx_status().read().apb_mem_wr_err().bit_is_clear(),
            "LedController: animation doesn't fit in RMT memory",
        );
    }

    #[inline]
    fn reset(&self) {
        RMT::regs().ch0_tx_conf0().modify(|_, w| {
            w.tx_stop().set_bit();
            w.apb_mem_rst().set_bit();
            w.conf_update().set_bit()
        });
    }

    #[inline]
    fn start(&self, continuous: bool) {
        RMT::regs().ch0_tx_conf0().modify(|_, w| {
            w.tx_conti_mode().bit(continuous);
            w.conf_update().set_bit();
            w.mem_rd_rst().set_bit();
            w.tx_start().set_bit()
        });
    }

    /// Turns the LED off.
    pub fn off(&mut self) {
        self.solid(Color::off());
    }

    /// Sets the LED to a single color which does not change.
    ///
    /// The LED will continue to display this color even after the controller
    /// is dropped. If you don't want that, call [off()] first.
    pub fn solid(&mut self, color: Color) {
        self.reset();

        self.encode(color.into());
        self.end();

        self.start(false);
    }

    /// Makes the LED cycle through colors in a repeating pattern.
    ///
    /// The total length of the pattern is fairly limited. It needs to be
    /// encoded into a sequence of at most 383 symbols. Each frame uses 24
    /// symbols plus one for each tick of its duration (roughly 13 ms, see
    /// [Frame] for details). This method will panic if encoding the given
    /// pattern would need more symbols than are available.
    ///
    /// If `limit` is `None`, the pattern will repeat forever.
    /// If a value is provided, the pattern will repeat that number of times,
    /// and then the LED will continue to display the color of the last frame.
    /// The maximum limit value is 1023; this method will panic if a larger
    /// limit value is given.
    ///
    /// When the controller is dropped the animation will stop and the LED
    /// will continue to display whichever color was active at that time.
    /// If you don't want that, either hold on to the controller or
    /// call [off()] before dropping it.
    pub fn animate(&mut self, frames: &[Frame], limit: Option<u16>) {
        self.reset();

        for frame in frames {
            self.encode((*frame).into());
            self.delay((*frame).duration_ticks());
        }
        self.end();

        if let Some(limit) = limit {
            assert!(limit < 1024, "limit is too large");
            RMT::regs().ch0_tx_lim().write(|w| unsafe {
                w.tx_loop_num().bits(limit);
                w.tx_loop_cnt_en().set_bit();
                w.loop_count_reset().set_bit();
                w.loop_stop_en().set_bit()
            });
        } else {
            RMT::regs().ch0_tx_lim().write(|w| {
                w.tx_loop_cnt_en().clear_bit();
                w.loop_stop_en().clear_bit()
            });
        }

        self.start(true);
    }
}
